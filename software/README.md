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

## 脚本（按上手顺序）

```bash
python scripts/sim_walk.py --gif walk.gif        # 0. 无硬件仿真，先看步态对不对
python scripts/servo_center.py                   # 1. 装配标定：全舵机回中，装舵盘
python scripts/stand_up.py                       # 2. 站立测试 + 传感器读数
python scripts/walk_teleop.py                    # 3. 键盘遥控行走 (wasd/qe)
```

全部脚本支持 `--mock` 干跑。测试：`pytest tests/`（18 项，覆盖 IK 往返、步态约束、协议字节、吸附状态机）。

## 上电顺序（重要）

1. Pi 5 用独立 5V/5A 降压模块供电，舵机 7.4V 经继电器（Servo2040 的 A0/GPIO26 控制）供电。
2. 软件流程：先 `set_all_pulses_us` 发好目标脉宽 → 再 `enable(True)`。固件保证舵机使能瞬间直接到设定位，不乱跳。
3. 任何脚本退出/异常都会 `enable(False)` 断舵机电——机器人会趴下，测试时垫高机身。

## 标定流程（装配后必做）

1. `servo_center.py` 让全部舵机回中，按官方视频角度装舵盘；
2. 每个关节手动摆到 -45°/+45°，记录脉宽，填入 `config.py` 的 `us_m45/us_p45`；
3. 方向相反的舵机改 `sign=-1`；零位偏差微调 `attach_deg`；
4. 换上吸盘足模块后重新量 `tibia_len`（约 +45mm）。
