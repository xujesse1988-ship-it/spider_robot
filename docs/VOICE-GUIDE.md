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
arecord -D plughw:CARD=<卡名>,DEV=0 --dump-hw-params -d 1 /dev/null 2>&1 | head
                                     # 看固件实际支持的采样率/通道数（16kHz）
bash software/scripts/voice_mixer.sh # 放音/录音音量 90% + alsactl store
arecord -D plughw:CARD=<卡名>,DEV=0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/t.wav
aplay   -D plughw:CARD=<卡名>,DEV=0 /tmp/t.wav
```

`<卡名>`看 `/proc/asound/cards` 方括号里那个。TTS 合成的 22.05 kHz wav 经 `plughw`
自动重采样到 16 kHz 播放，不用管。

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

**自听问题**：默认机器人说话期间（TTS 播放 + 0.3 s 尾巴）丢弃麦克风数据，免得听见
自己说的"停"。但喇叭就接在 Lite 板上，XU316 的 AEC 拿自己正在播的音频当参考、能把
回声消掉——实机验证有效后给 `voice_teleop.py` 加 **`--trust-aec`**，机器人说话期间
急停词照样生效（安全性更好），验证方法见 §6。

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
| 音量大 Pi 欠压 | `amixer -c <卡> sset <放音控制> 70%`（`voice_mixer.sh` 默认 90%）再 `alsactl store` |
| 说话慢半拍 | 首句合成要加载模型 ≈1.5 s，`voice_teleop` 启动时已预热常用短语；动态句（电压）每次现合成 0.1~0.3 s |

### 3.7 与爬墙脚本的关系

现在只接了地面行走（`voice_teleop.py` 与 `walk_teleop.py` 同一个环）。`climb_walk.py` /
`body_lean.py` 是带安全绳纪律的交互脚本，本次不动；以后要加语音急停，把
`VoiceEngine.events` 当另一路按键源接进它们的键盘处理即可（stop 事件 = 已有的急停键）。

## 4. 排障

| 症状 | 查 |
|---|---|
| `arecord -l` 没有 ReSpeaker Lite | 换带数据的 USB 线/换 USB 口；`lsusb \| grep -i 2886` 有没有；`dmesg \| tail` 看枚举报错。有 lsusb 无声卡 → 可能被刷成 I2S 固件了（§2.3 刷回 USB 版） |
| 录音全零或极小（voice_check 会提示） | **板上 Mute 键红灯亮着**（再按一下）；`voice_mixer.sh` 跑了没 |
| 放音无声 | 喇叭插的是 **SPK** 座不是 3.5mm；3.5mm 口插着耳机时喇叭可能被切走，拔掉试；放音音量（`amixer -c <卡>`） |
| `arecord: Device or resource busy` | 另一个 voice_teleop/voice_check 还在跑 |
| 大音量爆音/Pi 欠压（`get_throttled` ≠ 0） | `config.txt` 有没有 `usb_max_current_enable=1`（加了要重启）；降放音音量；终极方案板上 5V 焊盘直供 |
| 唤醒后指令没反应 | 看终端 `[asr]` 行：识别文本对但意图 unknown → 加词到 `intents.py`；没有 `[asr]` 行 → VAD 没切到句（说完停顿 0.5 s） |
| 机器人说话时喊"停下"没反应 | 默认行为（说话期间闭麦）。做完 §6 的 AEC 验证后加 `--trust-aec` |

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

## 6. 待办

- [ ] 插板跑 `voice_setup.sh` + `voice_check.py`；记录实机 KWS 阈值与混音器音量
- [ ] **AEC 验证**：`voice_teleop.py --mock --trust-aec`，趁机器人念长句（"电压…"）时喊
  "停下"，看 stop 事件是否触发、TTS 内容是否被误识别为指令；有效则把 `--trust-aec`
  写进日常用法（急停在机器人说话期间也生效）
- [ ] 舵机/泵噪声下的识别率实测；必要时换 `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`
- [ ] `climb_walk.py` 接语音急停事件

## 参考

- ReSpeaker Lite 入门（Seeed wiki）<https://wiki.seeedstudio.com/reSpeaker_usb_v3/>
- 固件与 DFU 指南 <https://github.com/respeaker/ReSpeaker_Lite>（`xmos_firmwares/`，USB 版当前 v2.0.7）
- sherpa-onnx：KWS <https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html>、
  SenseVoice <https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html>、
  TTS <https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html>
