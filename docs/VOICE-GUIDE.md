# 语音交互指南 · ReSpeaker Lite（USB 版）+ 4Ω3W 喇叭

目标：对着机器人说"小蜘蛛，前进三秒"它就走三秒，说"停下"立刻停，它用喇叭回话。
全部离线（树莓派本机跑唤醒词、识别、合成），不联网、不要 API key。

2026-09-01 本文最初按 ReSpeaker 2-Mics Pi HAT 写（引脚冲突、飞 9 根线、气路挪线，
见 git 72005f0）；**09-02 硬件换成 ReSpeaker Lite USB 版，那套复杂度整个消失**，本文重写。
软件部分已在开发机上用合成语音回环验证（§5）；**实机（插板 + 树莓派跑通）还没做**。

## 0. 结论先行

| 事项 | 结论 |
|---|---|
| 板子是什么 | `images/Microphone_and_speaker.jpg`：Seeed **ReSpeaker Lite 单板版 V1.1**（XMOS XU316 音频前端、2 只数字麦、板载功放 + JST PH2.0 喇叭口、3.5mm 耳机口、USB-C、Mute/USR 键、RGB 灯；右侧 XIAO 焊位空着，不用管）+ 4Ω 3W 腔体喇叭（PH2.0 插头直插板上 SPK 口） |
| 怎么接 | **两根线**：板子 USB-C → Pi 5 任一 USB-A 口（数据线）；喇叭 → 板上 SPK 口。**一根 GPIO 都不占**，现有 18 根杜邦线全不动 |
| 旧方案还剩什么 | 什么都不剩：引脚冲突表、飞线 9 根、阀 R2/泵 A/阀 R3 挪位、GPIO17/GPIO5 风险、`hardware/voice/` 覆盖层——**全部作废**（§1.3） |
| 驱动 | **不用装**。单板版出厂就是 USB 固件（UAC2 标准声卡），Linux 免驱即插即用；固件可用 dfu-util 升级（§2.3），当前最新 v2.0.7 |
| 音频前端 | XU316 片内做**回声消除（AEC）/干扰抑制（IC）/噪声抑制（NS）/自动增益（AGC）**，出来的就是处理后的人声，16kHz——正好是识别链路的采样率。比 HAT 的裸麦强一档，泵噪声好办得多 |
| 软件 | sherpa-onnx 全离线四件套（与硬件无关，未变）：KWS 流式关键词（唤醒词 + 急停词常驻）→ Silero VAD 切句 → SenseVoice 识别 → Matcha 中文合成；模型共 ≈350 MB；Pi 5 上 SenseVoice int8 单线程实时因子 ≈0.1 |
| 一键 | 树莓派上 `bash software/scripts/voice_setup.sh` → `python scripts/voice_check.py` → `python scripts/voice_teleop.py`（不用重启；只有 USB 电流上限那行 config 要重启生效） |

## 1. 硬件

### 1.1 连接与供电

- **USB**：板子 USB-C ↔ Pi 5 任一 USB-A，用带数据的线（纯充电线插上无反应）。
  与 Servo2040（`/dev/ttyACM0`）同在 USB 总线上，互不相干。
- **喇叭**：PH2.0 插头直插板边 **SPK** 座（丝印 SPK，带 +/− 标记；**不是** 3.5mm 口，
  那是耳机口）。板载功放支持 5W 喇叭，4Ω 3W 余量足；单只喇叭极性无所谓。
- **供电预算**：板子从 USB 口取电，喇叭大音量峰值 ≈0.5 A。Pi 5 用降压模块供电时
  没有 USB-PD 协商，USB 口总电流默认限 600 mA——`voice_setup.sh` 会往 `config.txt`
  追加 `usb_max_current_enable=1`（放开到 1.6 A，重启生效）。若以后 USB 口预算还是
  紧张，板上有 5V/GND 焊盘可从降压模块直接供电（USB 只走数据）。
- 音量大时 Pi 报欠压（`vcgencmd get_throttled` 非 0）就把放音音量降一档（§3.6）。

### 1.2 安装位置

- 两只麦克风在板子两端，**麦克风面朝前、远离泵和 Pi 风扇**；板子很轻，扎带/魔术贴
  固定在机身上层前部即可，不存在 HAT 的排针/散热器高度问题。
- 喇叭腔体封闭，随便固定；出声面别正对麦克风（有 AEC，但别故意为难它）。
- **Mute 键是硬件闭麦**（按下红灯亮）：闭麦时急停词也听不见——**上墙实验前检查红灯没亮**。
  USR 键/RGB 灯由固件管理，USB 模式下用不上，不接任何东西。

### 1.3 与旧 HAT 方案的关系（全部作废项）

| 旧方案条目 | 现状 |
|---|---|
| I2S 占 GPIO18~21，阀 R2（19）/泵 A（20）/阀 R3（21）挪到 22/23/24 | **不用挪了**。`adhesion.py`/`p4_mosfet_check.py`/两张接线图维持原样 |
| GPIO17（HAT 按键外接上拉）与舵机继电器冲突、GPIO5 与阀 L1 冲突 | 不存在了，Lite 不碰 GPIO |
| 飞 9 根杜邦线、I2C 并联 0x1a | 不存在了；I2C1 上仍只有 2×ADS1115 |
| `hardware/voice/` 设备树覆盖层 + `dtoverlay` 行 | 已从仓库删除；如果之前在 `config.txt` 加过 `dtoverlay=respeaker-2mic-v1_0`，删掉那行 |
| 声卡名 `seeed2micvoicec` | 变为 USB 声卡（`/proc/asound/cards` 里含 "ReSpeaker Lite"），`audio.py` 自动识别，`HEXAPOD_AUDIO_CARD` 可强制指定 |

## 2. 树莓派：系统与固件

### 2.1 一键脚本

```bash
cd ~/spider && bash software/scripts/voice_setup.sh     # 5 步，见脚本头注释
```

做的事：① apt 装 `alsa-utils` 等；② 检查声卡在不在 + `config.txt` 追加
`usb_max_current_enable=1`；③ `software/.venv` 装 `sherpa-onnx numpy pypinyin`；
④ 模型下到 `~/models/voice/`（国内慢就 `SHERPA_ONNX_MIRROR=https://ghfast.top/ bash … models`）；
⑤ 由 `hexapod/voice/keywords_raw.txt` 生成 KWS 关键词表 + 设混音器。

### 2.2 手工核对

```bash
lsusb | grep -i 2886                 # Seeed 的 USB VID，应有一行
arecord -l && aplay -l               # 应有 USB Audio: ReSpeaker Lite（录、放各一）
arecord -D plughw:CARD=Lite,DEV=0 --dump-hw-params -d 1 /dev/null 2>&1 | head
                                     # 看固件实际支持的采样率/通道数（16kHz）
bash software/scripts/voice_mixer.sh # 放音/录音音量 90% + alsactl store
arecord -D plughw:CARD=Lite,DEV=0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/t.wav
aplay   -D plughw:CARD=Lite,DEV=0 /tmp/t.wav
```

实机确认 ALSA 卡名就是 `Lite`（`/proc/asound/cards`：`0 [Lite] USB-Audio - ReSpeaker Lite`）。
TTS 合成的 22.05 kHz wav 经 `plughw` 自动重采样到 16 kHz 播放，不用管。

### 2.3 固件（一般不用动）

- 查版本：`lsusb -d 2886: -v 2>/dev/null | grep bcdDevice`（单板版出厂即 USB 固件，
  当前最新 v2.0.7）。
- 升级：`sudo apt install dfu-util`，从 <https://github.com/respeaker/ReSpeaker_Lite>
  的 `xmos_firmwares/` 拿 `respeaker_lite_usb_dfu_firmware_v2.0.7.bin`，然后
  `sudo dfu-util -R -e -a 1 -D respeaker_lite_usb_dfu_firmware_v2.0.7.bin`。
- **别刷 `_i2s_` 版固件**——那是配 XIAO ESP32S3 用的，刷了 USB 就不出声卡（可用
  dfu-util 再刷回 USB 版）。

### 2.4 与现有系统的关系

USB 总线多一个设备而已：不碰 I2C（Lite 的 I2C 从口只在 I2S 固件下有用）、不碰
GPIO/lgpio、不碰 `Pi5VacuumIO`。唯一交集是 §1.1 的 USB 供电预算。

## 3. 软件

### 3.1 组件与模型

| 环节 | 模型 | 大小 | 说明 |
|---|---|---|---|
| 唤醒 + 急停 | `sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01` | 18 MB | 流式关键词，自定义中文词不用训练（拼音 token）；CPU 单线程 |
| 切句 | `silero_vad.onnx` | 2 MB | 语音活动检测，句尾 0.4 s 静音判定 |
| 识别 | `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17` | 228 MB | 非流式，中英粤日韩；A76 单线程 RTF ≈0.1；`use_itn=True` 把"三秒"输出成"3秒" |
| 合成 | `matcha-icefall-zh-baker` + `vocos-22khz-univ.onnx` | 73 + 30 MB | 中文女声；开发机 RTF 0.12；回环识别几乎全对（§5） |
| 声纹（可选） | `3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx` | 28 MB | 声纹锁（§3.8）：指令只听注册的主人；192 维嵌入，一段 ~0.1s |

全在 `~/models/voice/`（或 `HEXAPOD_VOICE_MODELS`），`ModelPaths.discover()` 按目录名
前缀找，换更新版本只要目录名前缀不变。

### 3.2 代码

```
software/hexapod/voice/
  intents.py        识别文本 → 意图（纯规则，无三方依赖，tests/test_voice_intents.py 31 例）
  keywords.py       唤醒词/急停词 → KWS 关键词文件（只依赖 pypinyin，格式与 sherpa 官方 text2token 一致）
  keywords_raw.txt  词表源文件（改这里，重跑 python -m hexapod.voice.keywords）
  audio.py          arecord/aplay 子进程录放音（不装 PortAudio）；WavSource 用 wav 顶替麦克风
  tts.py            Speaker 线程：Matcha/VITS 合成 + 固定短语 wav 缓存（~/.cache/hexapod-voice）
  engine.py         VoiceEngine 线程：KWS 常驻 → VAD → SenseVoice → 意图 → 事件队列
  voiceprint.py     声纹锁（§3.8）：注册档案 + VoiceGate 打分
software/scripts/
  voice_setup.sh    树莓派一键安装（§2.1）
  voice_mixer.sh    混音器初始化
  voice_check.py    自检：列声卡 → 录 5 秒 → 回放 → 识别 → TTS 报结论
  voice_enroll.py   声纹注册/验证（§3.8）
  voice_teleop.py   语音遥控行走（键盘照旧可用）
```

### 3.3 工作流程

```
麦克风 16 kHz ─┬─► KWS（常驻）──► 唤醒词 → 进入听令 8 s，回一声"在"（不屏蔽麦克风）
               │                └► 急停词 → 立即发 stop 事件（不需唤醒，~0.3 s）
               └─► 听令时：VAD 切句 ─► SenseVoice ─► intents.parse ─► command 事件
                                                                   （每条有效指令把听令窗口再延 8 s）
主线程（voice_teleop）：每 20 ms 一环，取事件 → 改 vx/vy/wz/deadline → 步态引擎 → 舵机
```

两层的原因：急停要快、要随时有效，流式关键词最合适，误触代价只是多停一次；其它
指令走整句识别，准确率高得多且能自由组合（"快点往前走三秒"）。

**自听问题**（两道防线，09-02 实机教训）：

1. 默认机器人说话期间（TTS 播放 + 0.3 s 尾巴）丢弃麦克风数据；
2. **自听过滤**：引擎把每句识别结果和机器人 2.5 s 内说过的话比对，相近就丢弃
   （`engine.looks_like_echo`，容错到"电流1.0安"被听歪成"电容1.0N"；"停""在"这类
   ≤2 字短回话只认全等，用户真喊的"停下"不会被吃）。

第 2 层是 09-02 实机踩坑后加的：开 `--trust-aec` 实测发现板载 AEC **没有**压住
喇叭→麦克风回声，"电压7.4伏电流1.0安"的回答被再次识别成 status 指令，机器人
自问自答死循环。当天 `voice_check.py --echo-test` 判定：USB 固件采集就是 2 通道
16 kHz，**两通道内容相同**（有效值一致 0.1253），回声里的数字被整句识别——
即 AEC 对自听确认无效、不存在隐藏的干净通道，`HEXAPOD_AUDIO_PICK` 保持 0
（留作以后固件更新复测用）。有了自听过滤，`--trust-aec` 什么时候都能开：
坏处没了（不会自问自答），好处是机器人说话期间 KWS 仍在听，用户的急停词
有机会穿透回声被识别（命中率待实测，§6）。

另外唤醒词本身被 VAD 切成整句识别出来（"小蜘蛛。"）时，intents 返回 `ignore`，
不再回"没听懂"（09-02 实机的另一个小坑）。

**说话期间的急停**（09-02 第二轮加强）：KWS 在机器人说话期间也**常驻**——说话
期间关闭的只有整句识别，"停下"随时有效，并且会**立即打断说话**（清掉没播的、
掐断正在播的 aplay、跳过剩余句子）；说话中冒出的唤醒词（自我介绍里的"我是
小蜘蛛"）自动忽略，判定窗带 1 s 尾巴（句尾唤醒词的回声经声学+缓冲延迟到达
KWS 时已在播放结束之后——实机踩过：就绪播报"…叫我小蜘蛛"刚完它自己应了声"在"）。

**说话期间喊"停下"的命中率**（09-02 实测第一轮为 0，三个杠杆）：
1. **分句播报**：长回话按句切开、句间留 0.45 s 静音窗（`Speaker.gap_s`）——AEC
   无效的情况下，静音窗是 KWS 听清用户喊声的主要机会；顺带 VAD 也按句切回声段，
   自听过滤按句精确匹配。缓存改按句为键。
2. **急停词更灵敏**：`keywords_raw.txt` 急停阈值 0.35→**0.20** + 提升分 :2.0
   （唤醒仍 0.25——急停的误触方向是"多停"，安全）。**改完词表必须重新生成**：
   `python -m hexapod.voice.keywords --tokens <kws目录>/tokens.txt --out <kws目录>/keywords_hexapod.txt --raw hexapod/voice/keywords_raw.txt`
3. **物理与音量**：喇叭线够长——把喇叭装得离麦克风远些、出声面背对麦克风，这是
   最大的杠杆；软件侧 `--tts-gain 0.6` 压低播报音量提高信噪比。
新增"自我介绍"指令（≈15 s 长回话，措辞刻意避开急停词）作为这条链路的试金石。

### 3.4 指令表（`intents.py`）

| 说 | 效果 |
|---|---|
| **小蜘蛛** / 蜘蛛同学 | 唤醒，之后 8 s 内说指令，可连说 |
| **停下 / 停止 / 停下来 / 别动**（不用唤醒） | 急停。唤醒后说单字"停 / 站住 / 别走"也停 |
| 前进 / 往前走 / 后退 / 倒退 | 前后走，默认 3 s（`--default-secs`），到点自停 |
| 左移 / 右移 / 向左走 / 往右挪 | 平移 |
| 左转 / 右转 / 向左拐 / 顺时针 / 掉头 | 原地转 |
| … 三秒 / 5 秒 / 两步 / 半分钟 / 一直 | 时长；"一直" = 上限 10 s（`--max-secs`）；步 = 1 s |
| 快点 … / 慢点 … | 速度 ×1.5 / ×0.6 |
| 站起来 / 趴下 | 站姿 / 蹲姿（趴着时收到移动指令会先起立） |
| 三角步态 / 波浪步态 | 切步态 |
| 电压多少 / 电量 | 报"电压 7.9 伏，电流 1.2 安" |
| 你好 | "在呢" |
| 自我介绍 / 你是谁 | ≈15 秒长介绍——测"说话期间急停"用，喊"停下"应立刻闭嘴 |
| 退出 → **确定** | 退出程序（10 s 内二次确认；"取消"作罢）——退出即断舵机电，机器人会趴下 |
| 跳舞 | 不支持（dance.py 要架空机身），会说明 |

听不懂的句子不动、回"没听懂"；单字/噪声不回话。

### 3.5 运行

```bash
cd ~/spider/software && source .venv/bin/activate
python scripts/voice_check.py                     # 全套自检（说"小蜘蛛，前进三秒"）
python scripts/voice_check.py --tts "语音系统就绪"  # 只试喇叭
python scripts/voice_check.py --asr /tmp/voice_check.wav   # 只对录音跑识别

python scripts/voice_teleop.py --mock              # 先不带舵机（机器人垫高也行）
python scripts/voice_teleop.py                     # 真机，键盘 wasd/qe 仍可用
python scripts/voice_teleop.py --no-wake           # 安静环境省掉唤醒词
python scripts/voice_teleop.py --trust-aec         # AEC 实测有效后：说话期间也听急停
python scripts/voice_teleop.py --default-secs 2 --max-secs 6 --speed 30
```

无板子的开发机也能跑逻辑：`python scripts/voice_teleop.py --mock --wav 某段.wav --no-tts`
（`HEXAPOD_VOICE_MODELS` 指向模型目录）。

### 3.6 调参

| 现象 | 调 |
|---|---|
| 唤醒词老误触 | `keywords_raw.txt` 里阈值 0.25 → 0.35，或换 4 音节词；重跑关键词生成 |
| 喊不醒 | 阈值降到 0.15；`voice_check.py --record 3` 看录音峰值，太小就查 Mute 红灯/混音器；离 1 m 内说 |
| 急停词误触 | 阈值 0.35 → 0.45（误触方向是"多停"，可以容忍一点） |
| 一句话被切成两半 | `engine.py` `min_silence_duration` 0.4 → 0.6 |
| 泵一开识别变差 | XU316 的噪声抑制对稳态泵噪应该有效，先实测再说；还不行就近距离喊、只依赖急停/唤醒 |
| 怀疑麦克风取错通道 | `voice_check.py --echo-test` 判定；`HEXAPOD_AUDIO_PICK=1` 临时切到通道 1 试 |
| 说话期间喊"停下"不停 | 见 §3.3 三个杠杆：分句静音窗已内置；急停阈值 0.20（改过 `keywords_raw.txt` 要重新生成词表）；`--tts-gain 0.6` + 喇叭挪远/背对麦克风 |
| 急停后整机失聪（唤醒也没反应，KWS 日志消失） | 急停掐 aplay 偶发呛住 USB 声卡、arecord 不吐数据（09-02 实机踩到）。已加看门狗：2 s 无数据自动重启 arecord（日志"⚠ 麦克风…重启"）；仍不行=整卡挂了，拔插 USB 线 |
| 音量大 Pi 欠压 | `amixer -c <卡> sset <放音控制> 70%`（`voice_mixer.sh` 默认 90%）再 `alsactl store` |
| 说话慢半拍 | 首句合成要加载模型 ≈1.5 s，`voice_teleop` 启动时已预热常用短语；动态句（电压）每次现合成 0.1~0.3 s |

### 3.7 与爬墙脚本的关系

现在只接了地面行走（`voice_teleop.py` 与 `walk_teleop.py` 同一个环）。`climb_walk.py` /
`body_lean.py` 是带安全绳纪律的交互脚本，本次不动；以后要加语音急停，把
`VoiceEngine.events` 当另一路按键源接进它们的键盘处理即可（stop 事件 = 已有的急停键）。

### 3.8 声纹锁（可选：指令只听主人）

注册过声纹后，**行走等指令只认注册的人**，别人（或电视/它自己的 TTS）说"前进"
会被拒绝且不回话；**急停不经此闸——谁喊"停下"都停**（安全 > 便利），唤醒也不拦
（KWS 流式做不了低延迟声纹，且陌生人唤醒后说的指令照样被拒）。

```bash
python scripts/voice_enroll.py            # 读 5 段提示语注册 → voiceprint_owner.npz
python scripts/voice_enroll.py --test     # 自己验一次（✓），换别人验一次（✗）
python scripts/voice_teleop.py            # 检测到档案自动开锁；--no-voiceprint 关
```

- 档案：`<模型根>/voiceprint_owner.npz`（`$HEXAPOD_VOICEPRINT` 可改），存**整段 +
  1.5 s 子窗**两种尺度的声纹（短指令"退出"只有 0.5 s，对 4 s 整段的相似度天然低
  ——09-02 实机主人被误拒 0.44 就是这个原因，多尺度注册后同类查询升到 0.57+）+
  建议阈值（按注册自相似度 −0.15，夹在 0.35~**0.50**）；`--spk-threshold` 可覆盖。
  <0.7 s 的极短句阈值自动再降 0.05（放宽到 1 s 会让陌生人 0.8 s 指令贴线混过）。
- 误拒（自己被拒，看日志里的分数和秒数）→ 阈值调低 / 重新注册（注册环境要和
  使用时接近，泵开着用就开着泵注册几段）；误纳（别人能指挥）→ 阈值调高。
- 开发机验证（多尺度档案、两轮随机合成）：主人 5 短句 10/10 过（最低 0.57），
  三个陌生声音 30/30 拒（最高 0.48）；cmds.wav 端到端主人 5 指令全放行、
  陌生人 5 全拒且 2 个急停照常触发。
- 局限：救不了被喇叭声压住的喊话（混叠段声纹也是糊的）——说话期间的急停命中
  还是靠 §3.3 三个杠杆；带感冒、隔很远喊，分数会降。

## 4. 排障

| 症状 | 查 |
|---|---|
| `arecord -l` 没有 ReSpeaker Lite | 换带数据的 USB 线/换 USB 口；`lsusb \| grep -i 2886` 有没有；`dmesg \| tail` 看枚举报错。有 lsusb 无声卡 → 可能被刷成 I2S 固件了（§2.3 刷回 USB 版） |
| 录音全零或极小（voice_check 会提示） | **板上 Mute 键红灯亮着**（再按一下）；`voice_mixer.sh` 跑了没 |
| 放音无声 | 喇叭插的是 **SPK** 座不是 3.5mm；3.5mm 口插着耳机时喇叭可能被切走，拔掉试；放音音量（`amixer -c <卡>`） |
| `arecord: Device or resource busy` | 另一个 voice_teleop/voice_check 还在跑 |
| 大音量爆音/Pi 欠压（`get_throttled` ≠ 0） | `config.txt` 有没有 `usb_max_current_enable=1`（加了要重启）；降放音音量；终极方案板上 5V 焊盘直供 |
| 唤醒后指令没反应 | 看终端 `[asr]` 行：识别文本对但意图 unknown → 加词到 `intents.py`；没有 `[asr]` 行 → VAD 没切到句（说完停顿 0.5 s） |
| 机器人说话时喊"停下"没反应 | 默认行为（说话期间闭麦）。加 `--trust-aec` 后靠 KWS 穿透回声，命中率看 §6 的 AEC 判定结果 |
| 机器人自问自答/重复回话 | 自听过滤应该兜住（09-02 加，日志有"≈ 自己刚说的，丢弃"）；若还复现，把日志里的识别文本和回话原文发出来对比——多半是回声被听歪得太厉害（相似度 <0.7），加大 TTS 与麦克风的物理距离或降音量 |

## 5. 开发机验证记录（2026-09-01，x86 + sherpa-onnx 1.13.7）

与声卡无关，换 Lite 后仍然成立：

- 关键词生成：`keywords.py` 的 ppinyin 输出与 sherpa 官方 `text2token` 示例一致
  （"文森特卡索" → `w én s ēn t è k ǎ s uǒ`）；6 条词全在模型词表内。
- TTS 三选一（同一组 6 句，合成后喂 SenseVoice 看能否还原）：

  | 模型 | RTF | 回环识别 |
  |---|---|---|
  | `sherpa-onnx-vits-zh-ll`（5 人） | 0.36 | "前进三秒"→"田忌3秒/前击3秒"，"停下来"→"赢下来"，差 |
  | `vits-melo-tts-zh_en` | 0.52 | "前进三秒"→"眼镜3秒"，差 |
  | **`matcha-icefall-zh-baker` + vocos** | **0.12** | 6 句 5 句全对，"电压7.9伏，电流1.2安"一字不差 |

- 整链路（Matcha 合成 12 句拼成 25 s wav → `VoiceEngine`）：
  "小蜘蛛"唤醒 → "前进三秒"→walk vx=+1 3 s；"向左转两秒"→wz=+1 2 s；"蜘蛛同学"唤醒 →
  "电压多少"→status；"快点后退"→vx=−1 ×1.5；"停下来"→KWS 急停；"小蜘蛛" → "站起来"→stand；
  "别动"→急停。每句识别 60~100 ms。`voice_teleop.py --mock --wav` 按时长到点自停、急停清零均正确。
- 单元测试：`tests/test_voice_intents.py` 31 例 + `tests/test_voice_keywords.py` 4 例全绿。

**2026-09-02 树莓派实机**：插上即认卡（卡名 `Lite`），`voice_check.py` 全套通过——
录 5 s 峰值 0.771、"小蜘蛛"唤醒命中、"前进三秒"→`前进3秒。`→walk 意图（识别 0.11 s）、
TTS 出声。引擎模型加载 3.1 s、TTS 加载 2.75 s（teleop 启动时会预热）。加载时打印的
`Unknown token: shei2` ×4 无害：matcha-zh-baker 的词典里"谁"的口语读音 shei2 不在
token 表，只有合成含"谁"的句子才受影响，机器人的回话里没有这个字。

同日 `voice_teleop.py --mock --trust-aec` 实测暴露自问自答死循环（"电压…"回答被
再次识别成 status，每 3 s 一轮直到退出）→ 判定板载 AEC 没压住自听（或处理后音频
不在通道 0），当天加了自听过滤 + 唤醒词整句 ignore（§3.3），开发机复现场景验证：
合成"电压7.4伏，电流1.0安"回声喂引擎 → `≈ 自己刚说的，丢弃`，不再产生 status 事件；
cmds.wav 正常指令流回归不受影响；pytest 154 例全绿。

随后 `--echo-test` 实机判定：固件原生 2 通道 / 16 kHz / S16_LE；两通道有效值
完全一致（0.1253/0.1253，即同一路信号复制），且两路都把回声里的数字整句识别出
（`1234567891012345678910`）——**AEC 对自听无效、无干净通道**，结论已写进 §3.3。
固件升级大概率无用（v2.0.5→v2.0.7 changelog 只有 flash/WS2812 改动，未提音频算法），
不值得为此折腾 DFU。

第三轮（开发机，真实时序模拟）：用"播多久睡多久"的假播放器 + 实时喂送的自我介绍
回声 wav 模拟 `--trust-aec` 全程——自己念的"小蜘蛛"被 `[kws] 说话期间忽略唤醒`，
两段 7.2 s 回声全部 `≈ 自己刚说的，丢弃`，句尾"想让我停"未误触急停；cmds.wav
正常指令流 10 事件不变；pytest 156 例全绿。

第四轮（09-02 实机命中率第一测 + 开发机修正）：实机 `--trust-aec` 下趁自我介绍喊
"停下"**未命中**（KWS 在听——它连自己念的"小蜘蛛"都抓到了——纯声学掩蔽；且用户
喊声混进 7.2 s 回声段被一起丢弃）；就绪播报"…叫我小蜘蛛"结尾还触发了一次自唤醒
应答"在"（尾巴 0.3 s 不够）。修正=分句播报+句间 0.45 s 静音窗、急停阈值 0.20:2.0、
`--tts-gain`、唤醒抑制尾巴 1 s。开发机真实时序复测：逐句回声（含听歪的"六组机器人"）
全部丢弃、自唤醒忽略、cancel 跳过剩余句子；sherpa 正常加载带提升分的词表并照常
命中急停；pytest 157 例全绿。

第六轮（09-02 实机声纹第一测）：主人说"退出"（0.5 s）得 0.44 < 0.60 被**误拒**——
两个原因：短音频声纹向量抖 + 注册只有 4 s 整段（长短失配），且建议阈值被夹到
上限 0.60 太狠。修=注册加 1.5 s 子窗声纹（多尺度）、建议阈值上限 0.60→0.50、
<0.7 s 极短句再降 0.05（窗口不能放宽到 1 s：陌生人 0.8 s 指令会 0.43~0.46 贴线
混过，开发机实测踩到）。复测（两轮随机合成）：主人短句 10/10 过（最低 0.57）、
陌生人 30/30 拒（最高 0.48）。**改后要重新注册**（旧档案没有子窗声纹）。

第五轮（09-02 实机命中率第二测）：分句 + 阈值 0.20 后，**说话期间喊"停下"命中**
（`[kws] 急停词 停下`，自唤醒也正确被忽略）。暴露两个新问题并已修：①急停打在
"下一句正在合成"的 ~1 s 窗口里时合成完照样开播（cancel 只查句首）→ 合成后、开播前
再查一次 cancel（tests/test_voice_speaker.py 打桩验证）；②急停后整机失聪——疑为掐
aplay 呛住 USB 声卡致 arecord 停止吐数据、引擎阻塞在读麦克风 → ArecordSource 加
看门狗（2 s 无数据自动重启 arecord，连续 3 次无效才 eof），aplay 的 stop 改为
terminate+收尸。pytest 163 例全绿。

## 6. 待办

- [x] 插板跑 `voice_setup.sh` + `voice_check.py`（09-02 通过，卡名 `Lite`，见 §5）
- [ ] 日常使用中若调过 KWS 阈值/混音器音量，回填 §3.6
- [x] ~~AEC 验证第一轮~~（09-02：`--trust-aec` 下自问自答，AEC 没压住自听；已加自听
  过滤兜底，见 §3.3/§5）
- [x] ~~AEC 通道判定~~（09-02 `--echo-test`：两通道相同、回声整句可识别 → AEC 无效、
  无干净通道，接受现状，`HEXAPOD_AUDIO_PICK` 保持 0，见 §5）
- [ ] 机器人说话期间喊"停下"的命中率实测：说"小蜘蛛"→"自我介绍"，趁 15 s 长回话
  喊"停下"，应立即闭嘴 + `[voice] 急停`；多试几次记命中率，写回本节
  （KWS 现在说话期间常驻，默认模式与 `--trust-aec` 都能测）
- [ ] 声纹锁实机：`voice_enroll.py` 注册（含开泵环境段）→ `--test` 本人/他人各验
  → teleop 下别人说指令应 `[voice] 声纹不符`、喊"停下"应照停；记录实机阈值
- [ ] 舵机/泵噪声下的识别率实测；必要时换 `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`
- [ ] `climb_walk.py` 接语音急停事件

## 参考

- ReSpeaker Lite 入门（Seeed wiki）<https://wiki.seeedstudio.com/reSpeaker_usb_v3/>
- 固件与 DFU 指南 <https://github.com/respeaker/ReSpeaker_Lite>（`xmos_firmwares/`，USB 版当前 v2.0.7）
- sherpa-onnx：KWS <https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html>、
  SenseVoice <https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html>、
  TTS <https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html>
