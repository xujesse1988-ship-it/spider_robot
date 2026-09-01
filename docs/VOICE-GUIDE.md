# 语音交互指南 · ReSpeaker 2-Mics Pi HAT + 4Ω3W 喇叭

目标：对着机器人说"小蜘蛛，前进三秒"它就走三秒，说"停下"立刻停，它用喇叭回话。
全部离线（树莓派本机跑唤醒词、识别、合成），不联网、不要 API key。
本文 2026-09-01 写就；软件部分已在开发机上用合成语音回环验证（§5），**硬件接线与
树莓派实机还没做**——按 §1 接好、§2 装好后先跑 `voice_check.py`。

## 0. 结论先行

| 事项 | 结论 |
|---|---|
| 板子是什么 | `images/Microphone_and_speaker.jpg`：Seeed **ReSpeaker 2-Mics Pi HAT v1**（WM8960 编解码器、2 只模拟 MEMS 麦、3 颗 APA102 灯、1 个按键、3.5mm 耳机口、JST PH2.0 喇叭口、micro-USB 供电口）+ 4Ω 3W 腔体喇叭（PH2.0 插头，**直插 HAT 喇叭口**） |
| 能不能整板直插 Pi 5 的 40 针 | **不能**。① I2S 固定占 GPIO18~21，其中 19/20/21 正被阀 R2 / 泵 A / 阀 R3 占着；② HAT 按键脚 GPIO17 板上有外接上拉，而 GPIO17 是舵机继电器（高电平吸合）——接上去**开机瞬间舵机就上电**；③ HAT 灯电源脚 GPIO5 = 阀 L1；④ Pi 5 官方主动散热器顶着 HAT，标准排母压不到底；⑤ 排针上现有 18 根杜邦线没地方去 |
| 怎么接 | **只飞 9 根杜邦线**到 HAT 底部排母（§1.3），HAT 装机身上层前部、麦克风朝前、离泵和风扇远 |
| 必须动的现有接线 | 气路 3 根信号线挪位：阀 R2 GPIO19→22、泵 A GPIO20→23、阀 R3 GPIO21→24，并同步改 `adhesion.py`/`p4_mosfet_check.py`/两张接线图（§1.5）。**建议等论文 T4 上墙实验跑完再挪**；纯语音台架测试可先临时拔掉这 3 根线，代码不改 |
| 驱动 | Seeed 2026 版 `respeaker-2mic-v1_0` **纯设备树覆盖层**（已随仓库放 `hardware/voice/`），走内核自带 `snd-soc-wm8960` + `simple-audio-card`，Pi 5 的 6.6/6.12 内核都带这两个模块；不装 dkms。树莓派固件内置的 `wm8960-soundcard` 覆盖层是给 Waveshare 板写的（12.288 MHz / Pi 主时钟），与本板 24 MHz 晶振/编解码器主时钟不符，**别用** |
| 软件 | sherpa-onnx 全离线四件套：KWS 流式关键词（唤醒词 + 急停词常驻）→ Silero VAD 切句 → SenseVoice 识别 → Matcha 中文合成；模型共 ≈350 MB；Pi 5 上 SenseVoice int8 单线程实时因子 ≈0.1（一句 1 秒话 0.1 秒出字） |
| 一键 | 树莓派上 `bash software/scripts/voice_setup.sh` → 重启 → `python scripts/voice_check.py` → `python scripts/voice_teleop.py` |

## 1. 硬件

### 1.1 HAT 引脚占用 vs 机器人现有占用

HAT 占用按 Seeed 覆盖层源码、`respeaker/mic_hat` 例程（`button.py` 用 GPIO17 且**没开内部
上拉**→板上有外接上拉；`apa102.py` 开 SPI0 设备 1 = CE1）和 Seeed wiki；机器人占用按
`hexapod/adhesion.py`、`hexapod/driver.py`、`html/p4-pi-wiring.html`。

| BCM | 物理脚 | HAT 用途 | 机器人现用 | 处理 |
|---|---|---|---|---|
| 2 / 3 | 3 / 5 | I2C1：WM8960 @ **0x1a** | ADS1115 @ 0x48 / 0x49 | 共用总线，地址不撞，并联接（§1.3） |
| 18 | 12 | I2S BCLK（位时钟，WM8960 发） | 空 | 接 |
| **19** | 35 | I2S LRCLK（帧时钟，WM8960 发） | **阀 R2**（IN5） | **冲突**：阀 R2 挪到 GPIO22（Pin 15） |
| **20** | 38 | I2S DIN（麦克风数据 → Pi） | **泵 A**（IN7） | **冲突**：泵 A 挪到 GPIO23（Pin 16） |
| **21** | 40 | I2S DOUT（Pi → 喇叭） | **阀 R3**（IN6） | **冲突**：阀 R3 挪到 GPIO24（Pin 18） |
| **17** | 11 | 用户按键（外接上拉，按下接地） | **舵机继电器 IN1**（高电平吸合） | **绝不接**：上拉会在开机、进程未接管时把继电器拉合 |
| **5** | 29 | APA102 灯电源使能 | **阀 L1**（IN1） | 不接（放弃板上灯）；接了阀 L1 每动一次灯跟着开关 |
| 10 / 11 / 7 | 19 / 23 / 26 | SPI0 MOSI / SCLK / CE1（APA102 灯） | 空 | 不接（放弃灯）。以后要灯：这 3 根 + GPIO5，且阀 L1 得挪 |
| 12 / 13 | 32 / 33 | Grove GP12 口 | 13 = 阀 L3 | 不接 |
| 1 / 2 / 4 / 6 … | | 3V3 / 5V / GND | | 见 §1.3 |

I2S 四根在 Pi 5 上**没有替代引脚**（RP1 的 I2S0 固定在 GPIO18~21，固件 `i2s_clk_consumer`
节点只认这组），所以 19/20/21 三路气路必须挪，没有绕法。

### 1.2 为什么不整板直插（细说）

- **引脚**：§1.1 三路硬冲突 + GPIO17/GPIO5 两处上电安全问题。
- **高度**：Pi 5 官方主动散热器装上后，标准 HAT 排母（≈8.5 mm）到不了底/顶到风扇，
  论坛实测要 ≥12 mm 加长排母 + 15 mm 铜柱（[Pi 论坛](https://forums.raspberrypi.com/viewtopic.php?t=376206)、
  [element14 FAQ](https://community.element14.com/products/raspberry-pi/f/raspberry-pi-5-faq/53782/do-existing-hats-fit-on-top-of-the-raspberry-pi-5-active-cooler-and-heatsink)）。
- **噪声**：直插等于把两只麦克风架在风扇正上方、泵旁边。
- **走线**：现有 18 根杜邦线全插在 40 针上，直插后无处可去；加长排母穿过 HAT 再插
  也行，但 GPIO17/GPIO5 就被接通了。

备选（如果嫌飞线丑）：40 针排线 + "GPIO 一分二/一分三扩展板"，HAT 插一排、原有杜邦
线插另一排——**但必须把 HAT 那一排的 Pin 11（GPIO17）和 Pin 29（GPIO5）两根针拔掉或剪掉**。

### 1.3 推荐接法：杜邦飞线 9 根（+1 根可选）

HAT 底面是 2×20 排母，**公头杜邦线直接插进去**，孔位编号与 Pi 40 针完全一致
（Pin 1 在有方形焊盘、靠板边标 "1" 的一角；不确定就用万用表通断对 GND 孔）。

| # | 信号 | HAT 排母孔 | 接到 | 说明 |
|---|---|---|---|---|
| 1 | 5V | Pin 2 | **5V/5A 降压模块输出**（WAGO/并线） | 喇叭功放 + 灯供电，说话时峰值 ≈0.5 A。Pi 排针的 Pin 2/4 已被 Pi 自己的供电线占用，别再往那儿并。HAT 的 micro-USB 口和这个 5V 是同一网络，**二选一**别同时供 |
| 2 | 3V3 | Pin 1 | Pi **Pin 17**（3V3） | WM8960 数字/模拟电（Pin 1 已被 ADS1115 占） |
| 3 | GND | Pin 6 | Pi Pin 14 / 20 / 25 / 30 任一 | 与 I2S 四根扎一束走 |
| 4 | SDA | Pin 3 | 与 ADS1115 **并联**：一分二杜邦，或从分压板 J1 再引一根 | I2C 总线上多一个 0x1a |
| 5 | SCL | Pin 5 | 同上 | |
| 6 | BCLK GPIO18 | Pin 12 | Pi Pin 12 | 直连，现为空脚 |
| 7 | LRCLK GPIO19 | Pin 35 | Pi Pin 35 | 阀 R2 挪走后空出 |
| 8 | DIN GPIO20 | Pin 38 | Pi Pin 38 | 泵 A 挪走后空出 |
| 9 | DOUT GPIO21 | Pin 40 | Pi Pin 40 | 阀 R3 挪走后空出 |
| 10（可选） | GND | Pin 39 | Pi Pin 39 | 第二根地，I2S 抗干扰 |

线长 ≤20 cm；BCLK 在 16 kHz 采样时只有 1 MHz，普通杜邦线没问题，但 I2S 四根 + GND
要**扎成一束、远离 12 V 泵线和舵机线**。上电前用万用表确认 HAT 的 5V 孔与 GND 孔不短路。

### 1.4 喇叭

- PH2.0 插头直插 HAT 的 JST 喇叭口（板边、3.5mm 口旁，丝印 SPK/speaker）。**不是 3.5mm 口**
  （那是耳机口，另一路）。
- WM8960 内置 D 类功放，5 V 下 4 Ω 约 1 W，3 W 喇叭余量足；单只喇叭极性无所谓。
- 覆盖层把 "Speaker" 路由到 SPK_LP/SPK_LN，即**只出左声道**；TTS 是单声道，无影响。
- 装法：喇叭腔体本身封闭，随便固定，出声面别对着麦克风（自听回声）。

### 1.5 气路 3 根线挪位 + 同步改码（一次性，建议 T4 实验后做）

**物理**（MOSFET 板侧不动，只动 Pi 排针那头）：

| MOSFET 板端子 | 负载 | 线色 | 原 Pi 脚 | 新 Pi 脚 |
|---|---|---|---|---|
| IN5+ | 阀 R2 | 白 | Pin 35（GPIO19） | **Pin 15（GPIO22）** |
| IN7+ | 泵 A | 紫 | Pin 38（GPIO20） | **Pin 16（GPIO23）** |
| IN6+ | 阀 R3 | 白 | Pin 40（GPIO21） | **Pin 18（GPIO24）** |

Pin 15/16/18 在排针左上角连成一小片，好压一束。GPIO22~24 上电默认下拉（Pi 5 GPIO9~27
都是），和原 19/21 一样——阀/泵 MOSFET 上电全关，安全性不变。

**代码**（改完跑 `p4_mosfet_check.py --list` 对表 → 逐路点动 → `p4_sensor_check.py`）：

| 文件 | 改什么 |
|---|---|
| `software/hexapod/adhesion.py` | `VALVE_PINS = [5, 6, 13, 16, 22, 24]`，`PUMP_PIN = 23`（`PUMP_B_PIN = 26` 不变） |
| `software/scripts/p4_mosfet_check.py` | `CHANNELS` 表：通道 5 → `22, 15`；通道 6 → `24, 18`；通道 7 → `23, 16` |
| `html/p4-pi-wiring.html`、`html/p4-pneumatic-electrical.html` | 速查表同步；把 GPIO18~21 标成"I2S（语音 HAT）" |
| `software/README.md` 上电顺序、`docs/P4-GUIDE.md` 第 2 步引脚表 | 提一句 |

**为什么先别挪**：论文 T4a/T4b/T4d 上墙实验还没跑，挪线 + 改码之间机器人不能上墙，
一旦忘了同步就是阀泵错位。语音功能的台架调试**不需要气路**：把 Pin 35/38/40 上那 3 根
线暂时拔掉、插 HAT 的 LRCLK/DIN/DOUT 即可，代码一行不改；上墙实验时再插回去。

## 2. 树莓派：驱动与系统

系统：Raspberry Pi OS Lite 64-bit（Bookworm 内核 6.6 或 Trixie 6.12 都行，
`bcm2712_defconfig` 里 `CONFIG_SND_SOC_WM8960=m`、`CONFIG_SND_SIMPLE_CARD=m` 已核对）。

### 2.1 一键脚本

```bash
cd ~/spider && bash software/scripts/voice_setup.sh     # 6 步，见脚本头注释
sudo reboot
```

做的事：① apt 装 `device-tree-compiler alsa-utils i2c-tools`；② `dtc` 编译
`hardware/voice/respeaker-2mic-v1_0-overlay.dts` → `/boot/firmware/overlays/`，
`config.txt` 追加 `dtoverlay=respeaker-2mic-v1_0`；③ `software/.venv` 装
`sherpa-onnx numpy pypinyin`；④ 模型下到 `~/models/voice/`（国内慢就
`SHERPA_ONNX_MIRROR=https://ghfast.top/ bash … models`）；⑤ 由
`hexapod/voice/keywords_raw.txt` 生成 KWS 关键词表；⑥ 声卡已出现则设混音器。

### 2.2 手工等价步骤 / 逐项核对

```bash
# 覆盖层
sudo apt install -y device-tree-compiler
dtc -@ -I dts -O dtb -o respeaker-2mic-v1_0.dtbo hardware/voice/respeaker-2mic-v1_0-overlay.dts
sudo cp respeaker-2mic-v1_0.dtbo /boot/firmware/overlays/
echo dtoverlay=respeaker-2mic-v1_0 | sudo tee -a /boot/firmware/config.txt
sudo reboot
# 重启后
i2cdetect -y 1            # 应有 1a（WM8960）和 48 49（ADS1115）
aplay -l; arecord -l      # 应有 card N: seeed2micvoicec [seeed2micvoicec]
dmesg | grep -iE "wm8960|simple-card|seeed"
bash software/scripts/voice_mixer.sh          # 麦克风增益 / 喇叭音量 / 通路开关 + alsactl store
arecord -D plughw:CARD=seeed2micvoicec,DEV=0 -f S16_LE -r 16000 -c 2 -d 3 /tmp/t.wav
aplay   -D plughw:CARD=seeed2micvoicec,DEV=0 /tmp/t.wav
```

`voice_mixer.sh` 用的控制名（`Capture`、`Left/Right Input Boost Mixer LINPUT1/RINPUT1`、
`Left/Right Boost Mixer LINPUT1/RINPUT1`、`Left/Right Input Mixer Boost`、`ADC PCM`、
`Playback`、`Speaker`、`Left/Right Output Mixer PCM`）来自 wm8960 驱动，若报找不到就
`amixer -c seeed2micvoicec scontrols` 对名字。

### 2.3 与现有系统的关系

- I2C1 上多一个从机 0x1a，只在开机初始化和改音量时被内核访问，不影响
  `Pi5VacuumIO` 的 ADS1115 突发限流逻辑；`i2cdetect` 能看到三个地址即正常。
- 覆盖层不碰 GPIO17 / 阀泵引脚；`lgpio` 照旧。
- 5 V 预算：Pi 5（≈3 A 峰）+ 继电器线圈（0.1 A）+ HAT（≈0.5 A 峰）< 5 A 降压模块；
  但喇叭大音量瞬间会拉 5 V 纹波，若 Pi 报欠压（`vcgencmd get_throttled` 非 0）就把
  `Speaker` 音量从 110 降到 90。

## 3. 软件

### 3.1 组件与模型

| 环节 | 模型 | 大小 | 说明 |
|---|---|---|---|
| 唤醒 + 急停 | `sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01` | 18 MB | 流式关键词，自定义中文词不用训练（拼音 token）；CPU 单线程 |
| 切句 | `silero_vad.onnx` | 2 MB | 语音活动检测，句尾 0.4 s 静音判定 |
| 识别 | `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17` | 228 MB | 非流式，中英粤日韩；A76 单线程 RTF ≈0.1；`use_itn=True` 把"三秒"输出成"3秒" |
| 合成 | `matcha-icefall-zh-baker` + `vocos-22khz-univ.onnx` | 73 + 30 MB | 中文女声；开发机 RTF 0.12；回环识别几乎全对（§5） |

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
software/scripts/
  voice_setup.sh    树莓派一键安装（§2.1）
  voice_mixer.sh    混音器初始化
  voice_check.py    自检：列声卡 → 录 5 秒 → 回放 → 识别 → TTS 报结论
  voice_teleop.py   语音遥控行走（键盘照旧可用）
hardware/voice/     覆盖层 dts + 说明
```

### 3.3 工作流程

```
麦克风 16 kHz ─┬─► KWS（常驻）──► 唤醒词 → 进入听令 8 s，回一声"在"（不屏蔽麦克风）
               │                └► 急停词 → 立即发 stop 事件（不需唤醒，~0.3 s）
               └─► 听令时：VAD 切句 ─► SenseVoice ─► intents.parse ─► command 事件
                                                                   （每条有效指令把听令窗口再延 8 s）
主线程（voice_teleop）：每 20 ms 一环，取事件 → 改 vx/vy/wz/deadline → 步态引擎 → 舵机
机器人说话时（TTS 播放 + 0.3 s 尾巴）丢弃麦克风数据，免得听见自己说"停"
```

两层的原因：急停要快、要随时有效，流式关键词最合适，误触代价只是多停一次；其它
指令走整句识别，准确率高得多且能自由组合（"快点往前走三秒"）。

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
python scripts/voice_teleop.py --default-secs 2 --max-secs 6 --speed 30
```

无 HAT 的开发机也能跑逻辑：`python scripts/voice_teleop.py --mock --wav 某段.wav --no-tts`
（`HEXAPOD_VOICE_MODELS` 指向模型目录）。

### 3.6 调参

| 现象 | 调 |
|---|---|
| 唤醒词老误触 | `keywords_raw.txt` 里阈值 0.25 → 0.35，或换 4 音节词；重跑关键词生成 |
| 喊不醒 | 阈值降到 0.15；`voice_mixer.sh` 的 `Capture` 加到 60；离 1 m 内说 |
| 急停词误触 | 阈值 0.35 → 0.45（误触方向是"多停"，可以容忍一点） |
| 一句话被切成两半 | `engine.py` `min_silence_duration` 0.4 → 0.6 |
| 泵一开就听不见 | 预期现象（555 泵很吵）。近距离喊、只依赖急停/唤醒；`ADC High Pass Filter` 已开 |
| 说话慢半拍 | 首句合成要加载模型 ≈1.5 s，`voice_teleop` 启动时已预热常用短语；动态句（电压）每次现合成 0.1~0.3 s |

### 3.7 与爬墙脚本的关系

现在只接了地面行走（`voice_teleop.py` 与 `walk_teleop.py` 同一个环）。`climb_walk.py` /
`body_lean.py` 是带安全绳纪律的交互脚本，本次不动；以后要加语音急停，把
`VoiceEngine.events` 当另一路按键源接进它们的键盘处理即可（stop 事件 = 已有的急停键）。

## 4. 排障

| 症状 | 查 |
|---|---|
| `arecord -l` 没有 seeed2micvoicec | `config.txt` 有没有 `dtoverlay=respeaker-2mic-v1_0` 且 dtbo 在 `/boot/firmware/overlays/`；`i2cdetect -y 1` 无 0x1a → 3V3 / 5V / SDA / SCL 四根线；有 0x1a 但无声卡 → `dmesg \| grep -i wm8960` 看报错，确认没同时开 `wm8960-soundcard` |
| 录音全零或极小（voice_check 会提示） | `voice_mixer.sh` 跑了没；DIN（GPIO20/Pin 38）线；`Capture`/Boost |
| 录到爆音/嗡嗡 | LRCLK/BCLK 松、线太长、没并走 GND；`Capture` 太高 |
| 放音无声 | DOUT（GPIO21/Pin 40）；`Speaker` 音量、`Left/Right Output Mixer PCM` 开关；喇叭插的是 JST 不是 3.5mm |
| `arecord: Device or resource busy` | 另一个 voice_teleop/voice_check 还在跑 |
| ADS1115 `Errno 121` 变多 | I2C 并联那两根线接触不良；HAT 的 SDA/SCL 与 ADS 共地是否连通 |
| Pi 欠压（`get_throttled` ≠ 0） | 降 `Speaker` 音量；HAT 5V 从降压模块直接取，别串细线 |
| 唤醒后指令没反应 | 看终端 `[asr]` 行：识别文本对但意图 unknown → 加词到 `intents.py`；没有 `[asr]` 行 → VAD 没切到句（说完停顿 0.5 s） |

## 5. 开发机验证记录（2026-09-01，x86 + sherpa-onnx 1.13.7）

- 关键词生成：`keywords.py` 的 ppinyin 输出与 sherpa 官方 `text2token` 示例一致
  （"文森特卡索" → `w én s ēn t è k ǎ s uǒ`）；6 条词全在模型词表内。
- TTS 三选一（同一组 6 句，合成后喂 SenseVoice 看能否还原）：

  | 模型 | RTF | 回环识别 |
  |---|---|---|
  | `sherpa-onnx-vits-zh-ll`（5 人） | 0.36 | "前进三秒"→"田忌3秒/前击3秒"，"停下来"→"赢下来"，差 |
  | `vits-melo-tts-zh_en` | 0.52 | "前进三秒"→"眼镜3秒"，差 |
  | **`matcha-icefall-zh-baker` + vocos** | **0.12** | 6 句 5 句全对，"电压7.9伏，电流1.2安"一字不差 |

- 整链路（`engtest.py`，Matcha 合成 12 句拼成 25 s wav → `VoiceEngine`）：
  "小蜘蛛"唤醒 → "前进三秒"→walk vx=+1 3 s；"向左转两秒"→wz=+1 2 s；"蜘蛛同学"唤醒 →
  "电压多少"→status；"快点后退"→vx=−1 ×1.5；"停下来"→KWS 急停；"小蜘蛛" → "站起来"→stand；
  "别动"→急停。每句识别 60~100 ms。`voice_teleop.py --mock --wav` 按时长到点自停、急停清零均正确。
- 单元测试：`tests/test_voice_intents.py` 31 例 + `tests/test_voice_keywords.py` 4 例全绿。

## 6. 待办

- [ ] 按 §1.3 飞线、§2 装驱动，`voice_check.py` 过；记录实机 KWS 阈值与 `Capture` 增益
- [ ] T4 实验后按 §1.5 挪 3 根气路线 + 改码 + 接线图
- [ ] 板上 3 颗灯做状态指示（听令=蓝、识别到=绿）：需接 SPI0 三根 + GPIO5，阀 L1 先挪
- [ ] `climb_walk.py` 接语音急停事件
- [ ] 舵机/泵噪声下的识别率实测；必要时换 `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`

## 参考

- Seeed wiki（新版，dtoverlay 方案）<https://wiki.seeedstudio.com/ReSpeaker_2_Mics_Pi_HAT_Raspberry/>
- 覆盖层源码 <https://github.com/Seeed-Studio/seeed-linux-dtoverlays>（`overlays/rpi/respeaker-2mic-v1_0-overlay.dts`）
- HAT 例程（GPIO17 按键 / SPI 灯）<https://github.com/respeaker/mic_hat>
- 旧驱动仓库与内置覆盖层讨论 <https://github.com/respeaker/seeed-voicecard/issues/281>
- 树莓派内核配置 `arch/arm64/configs/bcm2712_defconfig`（rpi-6.6.y / rpi-6.12.y）
- sherpa-onnx：KWS <https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html>、
  SenseVoice <https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html>、
  TTS <https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html>
