# software/ —— 树莓派 5 大脑

Pi 5 通过 USB 连 Servo2040（跑社区 chica 固件），本包实现运动学、步态、驱动协议与吸附控制。
协议已从固件源码逐字节核实（`hexapod/driver.py` 文件头有完整协议说明）。

## 安装

```bash
# 开发机（仿真+测试）
python3 -m venv .venv && .venv/bin/pip install -e ".[sim,dev]"
# 树莓派上（实机）
pip install -e ".[pi]"
```

## 模块

| 文件 | 内容 |
|---|---|
| `hexapod/config.py` | 机器人几何/舵机标定表——默认值全部来自官方 chica-config-2040.txt（连杆 43/80/134mm、通道映射、安装偏角） |
| `hexapod/kinematics.py` | 单腿 3DOF IK/FK（有工作空间检查） |
| `hexapod/gait.py` | 相位式步态引擎：tripod（三角）/ wave（波浪）/ climb（爬墙五足支撑） |
| `hexapod/robot.py` | 身体系足端目标 → 变换 → IK → 18 路脉宽；身体姿态偏移（爬墙贴墙姿态用） |
| `hexapod/driver.py` | Servo2040 chica 协议驱动 + MockDriver；含足底开关/电压/电流读取 |
| `hexapod/adhesion.py` | 吸附状态机（RELEASED→PRESSING→SUCKING→ATTACHED→VENTING）+ 真空回路仿真；`Pi5VacuumIO` 留待 P1 台架按实际接线补全 |
| `hexapod/voice/` | 语音交互（`docs/VOICE-GUIDE.md`）：`intents.py` 识别文本→意图（纯规则）、`keywords.py` 唤醒词→KWS 关键词表、`audio.py` arecord/aplay 录放音、`tts.py` 离线合成+缓存+分句播报、`engine.py` KWS→VAD→SenseVoice 引擎线程、`voiceprint.py` 声纹锁 |

## 脚本（按上手顺序）

```bash
python scripts/sim_walk.py --gif walk.gif        # 0. 无硬件仿真，先看步态对不对
python scripts/servo_center.py                   # 1. 装配标定：全舵机回中，装舵盘
python scripts/stand_up.py                       # 2. 站立测试 + 传感器读数
python scripts/walk_teleop.py                    # 3. 键盘遥控行走 (wasd/qe)；m 键原地踏步（身体不动，按步态分组轮抬：抬起→悬空停 5s→落下→站定 5s→下一组，`--march-hold` 改秒数；空格冻结当前姿势，抬到一半也定得住；踏步全程六阀通电排气）；装气路后阀策略 --vent auto（默认）：站起/走动时六阀通电排气让吸盘通大气、站着不动断电；on 常通；off 不碰阀=脚被被动真空吸住抬不起（对照，v 键轮换）
python scripts/voice_check.py                    # 4. 语音自检：录 5 秒→回放→识别→TTS（要 ReSpeaker Lite，见 docs/VOICE-GUIDE.md）
python scripts/voice_enroll.py                   # 5. （可选）声纹注册：行走指令只听你，急停谁喊都停
python scripts/voice_teleop.py                   # 6. 语音遥控行走（“小蜘蛛，前进三秒”/“停下”；键盘照旧；阀排气同 walk_teleop）
```

全部脚本支持 `--mock` 干跑（`voice_*` 还支持 `--wav` 用录音顶替麦克风）。测试：`pytest tests/`（覆盖 IK 往返、步态约束、协议字节、吸附状态机、语音意图/关键词/回声过滤/声纹锁）。

语音一键安装（树莓派）：`bash scripts/voice_setup.sh`，依赖见 `requirements.txt` 语音段。

黑匣子与死机验尸：`climb_walk`/`body_lean` 每跑一次落一份 `logs/<tag>_时间.log`（事件+遥测，`hexapod/runlog.py`）。09-02 起启动段逐步落盘（每路阀线圈通电前、舵机继电器合闸前后母线电压），并由 `hexapod/powerlog.py` 后台线程每 0.1s 记一行 Pi 5 的 5V 输入电压与欠压标志（`TLM 电源`，需 `vcgencmd`，用户在 video 组）。启动死机（灯绿→红、SSH 失联）排查：先 `bash scripts/pi_forensics.sh setup`（内核日志持久化，一次即可）→ 复现 → 重新上电 → `bash scripts/pi_forensics.sh check`；`--startup-gap 3` 把阀线圈与舵机合闸隔开定位扳机。详见 docs/P4-GUIDE.md 常见问题。

## 上电顺序（重要）

1. Pi 5 用独立 5V/5A 降压模块供电，舵机 7.4V 经继电器（Pi GPIO17 控制，08-15 定案，原 Servo2040 A0/GPIO26）供电。
2. 软件流程：先 `set_all_pulses_us` 发好目标脉宽 → 再 `enable(True)`。固件保证舵机使能瞬间直接到设定位，不乱跳。`enable(True)` 内部拆两步：先合继电器（舵机带电不出力）→ 0.4s → 固件使能（同刻出力）；09-03 实机定案启动死机扳机就是合闸瞬间（5V 输入空载仅 4.89V 余量极薄），拆开后峰值分开、黑匣子能分清是哪一记。
3. 任何脚本退出/异常都会 `enable(False)` 断舵机电——机器人会趴下，测试时垫高机身。进程被杀时 lgpio 释放 GPIO17，继电器同样自动断电。

## 标定流程（装配后必做）

1. `servo_center.py` 让全部舵机回中，按官方视频角度装舵盘；
2. 每个关节手动摆到 -45°/+45°，记录脉宽，填入 `config.py` 的 `us_m45/us_p45`；
3. 方向相反的舵机改 `sign=-1`；零位偏差微调 `attach_deg`；
4. 换上吸盘足模块后重新量 `tibia_len`（约 +45mm）。
