> English translation of [`docs/VOICE-GUIDE.md`](../VOICE-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Voice interaction guide · ReSpeaker Lite (USB version) + 4Ω 3W speaker

Goal: say "小蜘蛛，前进三秒" (xiǎo zhīzhū, "little spider" — the wake word — then "forward three seconds") to the robot and it walks for three seconds; say "停下" ("stop") and it stops at once; it answers through the speaker.
All offline (wake-word spotting, recognition and synthesis all run on the Pi itself), no network, no API key.

2026-09-01: this document was first written around the ReSpeaker 2-Mics Pi HAT (pin conflicts, 9 flying wires, re-routing the pneumatics wiring;
see git 72005f0); **on 09-02 the hardware changed to the ReSpeaker Lite USB version and that whole pile of complexity disappeared**, so the document was rewritten.
The software side has been verified on the dev machine with a synthesized-speech loopback (§5); **the real robot (board plugged in, running on the Pi) has not been done yet**.

## 0. Conclusions first

| Item | Conclusion |
|---|---|
| What the board is | `images/Microphone_and_speaker.jpg`: Seeed **ReSpeaker Lite standalone board V1.1** (XMOS XU316 audio front end, 2 digital mics, on-board amplifier + JST PH2.0 speaker header, 3.5mm headphone jack, USB-C, Mute/USR buttons, RGB LED; the XIAO footprint on the right is empty, ignore it) + a 4Ω 3W enclosed speaker (PH2.0 plug straight into the board's SPK header) |
| How to hook it up | **Two cables**: board USB-C → any USB-A port on the Pi 5 (a data cable); speaker → the board's SPK header. **It uses no GPIO at all**, and none of the existing 18 Dupont jumpers move |
| What is left of the old design | Nothing at all: the pin-conflict table, the 9 flying wires, moving valve R2 / pump A / valve R3, the GPIO17/GPIO5 risks, the `hardware/voice/` overlay — **all void** (§1.3) |
| Driver | **None needed.** The standalone board ships with USB firmware (a standard UAC2 sound card), so Linux takes it plug-and-play; the firmware can be upgraded with dfu-util (§2.3), currently v2.0.7 |
| Audio front end | The XU316 does **echo cancellation (AEC) / interference cancellation (IC) / noise suppression (NS) / automatic gain (AGC)** on-chip, so what comes out is already-processed speech at 16kHz — exactly the sample rate of the recognition chain. A tier better than the HAT's bare mics, and far easier on pump noise |
| Software | The all-offline sherpa-onnx set of four (hardware-independent, unchanged): streaming KWS keywords (wake word + e-stop word always on) → Silero VAD sentence splitting → SenseVoice recognition → Matcha Chinese synthesis; ≈350 MB of models in total; on the Pi 5, SenseVoice int8 single-threaded runs at a real-time factor of ≈0.1 |
| One-shot | On the Pi: `bash software/scripts/voice_setup.sh` → `python scripts/voice_check.py` → `python scripts/voice_teleop.py` (no reboot needed; only the USB current-limit line in config takes a reboot to apply) |

## 1. Hardware

### 1.1 Connections and power

- **USB**: board USB-C ↔ any USB-A on the Pi 5, with a data-capable cable (a charge-only cable does nothing).
  It shares the USB bus with the Servo2040 (`/dev/ttyACM0`); they do not interfere.
- **Speaker**: the PH2.0 plug goes straight into the **SPK** header on the board edge (silkscreened SPK, with +/− marks; **not** the 3.5mm jack,
  that one is for headphones). The on-board amplifier handles 5W speakers, so 4Ω 3W has plenty of margin; polarity does not matter for a single speaker.
- **Power budget**: the board draws from the USB port, and the speaker peaks at ≈0.5 A when loud. When the Pi 5 runs off a buck converter
  there is no USB-PD negotiation, so the total USB port current defaults to 600 mA — `voice_setup.sh` appends
  `usb_max_current_enable=1` to `config.txt` (raising it to 1.6 A, effective after a reboot). If the USB budget still gets
  tight later, the board has 5V/GND pads that can be fed straight from the buck converter (leaving USB for data only).
- If the Pi reports undervoltage at high volume (`vcgencmd get_throttled` non-zero), turn the playback volume down a notch (§3.6).

### 1.2 Where to mount it

- The two mics sit at the ends of the board: **point the mic face forward, away from the pump and the Pi fan**. The board is light, so a cable tie or velcro
  at the front of the upper deck is enough; none of the HAT's header/heatsink height problems apply.
- The speaker enclosure is sealed, so mount it anywhere; just do not aim its output face straight at the mics (there is AEC, but do not go out of your way to defeat it).
- **The Mute button is a hardware mic kill** (pressed = red LED on): with the mic muted it cannot hear the emergency-stop word either — **check that the red LED is off before any wall test**.
  The USR button and RGB LED are managed by the firmware, are unused in USB mode, and nothing is wired to them.

### 1.3 Relation to the old HAT design (everything void)

| Old-design item | Now |
|---|---|
| I2S takes GPIO18–21, so valve R2 (19) / pump A (20) / valve R3 (21) move to 22/23/24 | **No move needed.** `adhesion.py` / `p4_mosfet_check.py` / both wiring diagrams stay as they are |
| GPIO17 (the HAT button's external pull-up) clashes with the servo relay, GPIO5 clashes with valve L1 | Gone; the Lite does not touch GPIO |
| 9 flying Dupont jumpers, 0x1a paralleled on I2C | Gone; I2C1 still carries only 2×ADS1115 |
| The `hardware/voice/` device-tree overlay + the `dtoverlay` line | Deleted from the repo; if you added `dtoverlay=respeaker-2mic-v1_0` to `config.txt` earlier, delete that line |
| Sound-card name `seeed2micvoicec` | Now a USB sound card ("ReSpeaker Lite" shows up in `/proc/asound/cards`); `audio.py` finds it automatically, and `HEXAPOD_AUDIO_CARD` can force it |

## 2. Raspberry Pi: system and firmware

### 2.1 One-shot script

```bash
cd ~/spider && bash software/scripts/voice_setup.sh     # 5 steps, see the header comment in the script
```

What it does: ① apt-installs `alsa-utils` and friends; ② checks the sound card is there and appends
`usb_max_current_enable=1` to `config.txt`; ③ installs `sherpa-onnx numpy pypinyin` into `software/.venv`;
④ downloads the models into `~/models/voice/` (if it is slow from inside China, use `SHERPA_ONNX_MIRROR=https://ghfast.top/ bash … models`);
⑤ generates the KWS keyword table from `hexapod/voice/keywords_raw.txt` and sets up the mixer.

### 2.2 Manual checks

```bash
lsusb | grep -i 2886                 # Seeed's USB VID, should print one line
arecord -l && aplay -l               # should show USB Audio: ReSpeaker Lite (one capture, one playback)
arecord -D plughw:CARD=Lite,DEV=0 --dump-hw-params -d 1 /dev/null 2>&1 | head
                                     # shows the sample rates / channel counts the firmware really supports (16kHz)
bash software/scripts/voice_mixer.sh # sets any control to 90% and stores it; the Lite USB firmware in practice exposes
                                     # no controls at all (scontrols empty, gain is on-chip), so it prints a note and exits
arecord -D plughw:CARD=Lite,DEV=0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/t.wav
aplay   -D plughw:CARD=Lite,DEV=0 /tmp/t.wav
```

Confirmed on the robot that the ALSA card name really is `Lite` (`/proc/asound/cards`: `0 [Lite] USB-Audio - ReSpeaker Lite`).
The 22.05 kHz wav from TTS is resampled to 16 kHz automatically by `plughw` on playback; nothing to do about it.

### 2.3 Firmware (usually leave it alone)

- Check the version: `lsusb -d 2886: -v 2>/dev/null | grep bcdDevice` (the standalone board ships with USB firmware,
  currently v2.0.7).
- Upgrade: `sudo apt install dfu-util`, take `respeaker_lite_usb_dfu_firmware_v2.0.7.bin` from the
  `xmos_firmwares/` directory at <https://github.com/respeaker/ReSpeaker_Lite>, then
  `sudo dfu-util -R -e -a 1 -D respeaker_lite_usb_dfu_firmware_v2.0.7.bin`.
- **Do not flash the `_i2s_` firmware** — that one is for the XIAO ESP32S3, and after flashing it the USB side no longer presents a sound card (you can
  flash the USB version back with dfu-util).

### 2.4 Relation to the existing system

It is just one more device on the USB bus: it does not touch I2C (the Lite's I2C slave port is only useful under the I2S firmware), does not touch
GPIO/lgpio, does not touch `Pi5VacuumIO`. The only overlap is the USB power budget of §1.1.

## 3. Software

### 3.1 Components and models

| Stage | Model | Size | Notes |
|---|---|---|---|
| Wake + e-stop | `sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01` | 18 MB | Streaming keywords; custom Chinese words need no training (pinyin tokens); single CPU thread |
| Sentence splitting | `silero_vad.onnx` | 2 MB | Voice activity detection; a sentence ends after 0.4 s of silence |
| Recognition | `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17` | 228 MB | Non-streaming; Chinese, English, Cantonese, Japanese, Korean; RTF ≈0.1 single-threaded on an A76; `use_itn=True` turns "三秒" ("three seconds") into "3秒" |
| Synthesis | `matcha-icefall-zh-baker` + `vocos-22khz-univ.onnx` | 73 + 30 MB | Chinese female voice; RTF 0.12 on the dev machine; loopback recognition is nearly perfect (§5) |
| Voiceprint (optional) | `3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx` | 28 MB | Voiceprint lock (§3.8): commands are obeyed only from the enrolled owner; 192-dim embedding, ~0.1s per clip |

They all live in `~/models/voice/` (or `HEXAPOD_VOICE_MODELS`); `ModelPaths.discover()` matches on directory-name
prefixes, so a newer version drops straight in as long as the prefix stays the same.

### 3.2 Code

```
software/hexapod/voice/
  intents.py        recognized text → intent (pure rules, no third-party deps, 31 cases in tests/test_voice_intents.py)
  keywords.py       wake / e-stop words → the KWS keyword file (depends only on pypinyin; format matches sherpa's official text2token)
  keywords_raw.txt  source word list (edit here, then re-run python -m hexapod.voice.keywords)
  audio.py          record and play through arecord/aplay subprocesses (no PortAudio to install); WavSource substitutes a wav for the mic
  tts.py            Speaker thread: Matcha/VITS synthesis + a wav cache for fixed phrases (~/.cache/hexapod-voice)
  engine.py         VoiceEngine thread: KWS always on → VAD → SenseVoice → intent → event queue
  voiceprint.py     voiceprint lock (§3.8): enrollment profile + VoiceGate scoring
software/scripts/
  voice_setup.sh    one-shot install on the Pi (§2.1)
  voice_mixer.sh    mixer initialization
  voice_check.py    self-test: list sound cards → record 5 seconds → play back → recognize → TTS the verdict
  voice_enroll.py   voiceprint enrollment / verification (§3.8)
  voice_teleop.py   voice-driven walking teleop (the keyboard still works)
```

### 3.3 How it works

```
mic 16 kHz ─┬─► KWS (always on) ─► wake word → listening for 8 s, replies "在" ("here"; mic not muted)
            │                    └► e-stop word → fire a stop event at once (no wake needed, ~0.3 s)
            └─► while listening: VAD splits sentences ─► SenseVoice ─► intents.parse ─► command event
                                                                    (each valid command extends the listen window by another 8 s)
main thread (voice_teleop): one loop every 20 ms, take events → update vx/vy/wz/deadline → gait engine → servos
```

Why two layers: the emergency stop has to be fast and has to work at any moment, which is exactly what streaming keyword spotting is best at, and the cost of a
false trigger is just one extra stop; every other command goes through whole-sentence recognition, which is far more accurate and composes freely ("快点往前走三秒", "walk forward faster for three seconds").

**The self-hearing problem** (two lines of defense, learned on the robot 09-02):

1. By default, mic data is discarded while the robot is talking (TTS playback + a 0.3 s tail);
2. **Self-hearing filter**: the engine compares every recognized sentence against what the robot said within the last 2.5 s and discards anything close
   (`engine.looks_like_echo`, tolerant enough that "电流1.0安" ("current 1.0 amps") misheard as "电容1.0N" still matches; short replies of ≤2 characters such as
   "停" ("stop") and "在" ("here") only match exactly, so a real "停下" ("stop") shouted by the user is never swallowed).

Layer 2 was added after getting burned on the robot on 09-02: with `--trust-aec` on, the on-board AEC turned out **not** to suppress the
speaker→mic echo, and the reply "电压7.4伏电流1.0安" ("voltage 7.4 volts, current 1.0 amps") was recognized again as a status command, so the robot
fell into an infinite loop asking and answering itself. That same day `voice_check.py --echo-test` settled it: the USB firmware captures 2 channels at
16 kHz and **both channels carry identical content** (RMS identical at 0.1253), and the digits in the echo were recognized as a whole sentence —
i.e. the AEC is confirmed useless against self-hearing and there is no hidden clean channel, so `HEXAPOD_AUDIO_PICK` stays 0
(kept for a re-test after some future firmware update). With the self-hearing filter in place, `--trust-aec` can be left on at any time:
the downside is gone (no more self-questioning loop) and the upside is that KWS keeps listening while the robot talks, giving the user's emergency-stop word
a chance to punch through the echo (hit rate still to be measured, §6).

Also, when the wake word itself gets split out by the VAD and recognized as a whole sentence ("小蜘蛛。"), intents returns `ignore`,
so it no longer replies "没听懂" ("didn't catch that") — another small trap found on the robot on 09-02.

**Emergency stop while it is talking** (hardened in the second round on 09-02): KWS stays **always on** even while the robot speaks — what is switched off while
it talks is only whole-sentence recognition, so "停下" ("stop") works at any moment and **interrupts the speech immediately** (clears anything unplayed,
kills the running aplay, skips the remaining sentences); a wake word out of its own mouth (the "我是小蜘蛛", "I'm little spider", in the
self-introduction) is ignored automatically, with a 1 s tail on the decision window (the echo of a sentence-final wake word reaches KWS after acoustic and buffering
delay, i.e. after playback has already ended — hit on the robot: right after the ready announcement "…叫我小蜘蛛" ("…call me little spider") it answered itself with "在" ("here")).

**Hit rate for shouting "停下" ("stop") while it talks** (first measured round on 09-02 was 0; three levers):
1. **Sentence-by-sentence announcements**: long replies are split by sentence with a 0.45 s silence window between them (`Speaker.gap_s`) — with the AEC
   useless, those silence windows are the main chance KWS gets to hear the user's shout; as a side effect the VAD also cuts the echo into sentences and
   the self-hearing filter matches sentence by sentence. The cache is now keyed by sentence.
2. **A more sensitive e-stop word**: in `keywords_raw.txt` the e-stop threshold goes 0.35→**0.20** with a boost score of :2.0
   (wake stays at 0.25 — a false e-stop only means stopping more often, which is safe). **After editing the word list you must regenerate it**:
   `python -m hexapod.voice.keywords --tokens <kws dir>/tokens.txt --out <kws dir>/keywords_hexapod.txt --raw hexapod/voice/keywords_raw.txt`
3. **Physics and volume**: the speaker cable is long enough — mount the speaker farther from the mics with its output face turned away; this is
   the biggest lever. On the software side, `--tts-gain 0.6` lowers the announcement volume and raises the signal-to-noise ratio.
A new "自我介绍" ("introduce yourself") command was added (a ≈15 s long reply, deliberately worded to avoid the e-stop words) as the touchstone for this path.

### 3.4 Command table (`intents.py`)

| Say | Effect |
|---|---|
| **小蜘蛛** (xiǎo zhīzhū, "little spider") / 蜘蛛同学 (zhīzhū tóngxué, "spider buddy") | Wake; then speak commands within 8 s, several in a row |
| **停下 / 停止 / 停下来 / 别动** ("stop" / "halt" / "stop it" / "don't move"; no wake needed) | Emergency stop. After waking, the short forms "停 / 站住 / 别走" ("stop" / "halt" / "don't go") also stop it |
| 前进 / 往前走 / 后退 / 倒退 ("forward" / "walk forward" / "back up" / "reverse") | Walk forward or back. **No duration = keep going** (the reply says "一直前进，说停就停", "going forward, say stop and I stop"), until you shout stop, give a new command, or use the keyboard |
| 左移 / 右移 / 向左走 / 往右挪 ("strafe left" / "strafe right" / "go left" / "shift right") | Strafe (same continuous semantics as above) |
| 左转 / 右转 / 向左拐 / 顺时针 / 掉头 ("turn left" / "turn right" / "bear left" / "clockwise" / "turn around") | Turn in place (same continuous semantics as above) |
| … 三秒 / 5 秒 / 两步 / 半分钟 ("three seconds" / "5 seconds" / "two steps" / "half a minute") | With a duration it stops itself when the time is up; capped at 10 s (`--max-secs`, in case the duration is misheard); one step = 1 s |
| 快点 … / 慢点 … ("faster …" / "slower …") | Speed for this sentence ×1.5 / ×0.6 ("快点前进", "forward, faster") |
| 快点 / 慢点 ("faster" / "slower", said alone) | Speed trim: a persistent multiplier ×1.5/×0.6 clamped to 0.4–2.0, effective immediately while walking and inherited by later commands; "太快了" ("too fast") = slow down; at the limit it replies "已经最快了" ("already at top speed"). Keyboard keys ignore the multiplier |
| 站起来 / 趴下 ("stand up" / "lie down") | Stand / crouch (a move command received while lying down stands it up first) |
| 三角步态 / 波浪步态 ("tripod gait" / "wave gait") | Switch gait |
| 电压多少 / 电量 ("what's the voltage" / "battery level") | Says "电压 7.9 伏，电流 1.2 安" ("voltage 7.9 volts, current 1.2 amps") |
| 你好 ("hello") | "在呢" ("I'm here") |
| 自我介绍 / 你是谁 ("introduce yourself" / "who are you") | A ≈15 second introduction — used to test "emergency stop while talking"; shouting "停下" ("stop") should shut it up instantly |
| 退出 → **确定** ("quit" → "confirm") | Quit the program (a second confirmation within 10 s; "取消", "cancel", calls it off) — quitting cuts servo power, so the robot flops down |
| 跳舞 ("dance") | Not supported (dance.py needs the body propped up); it says so |

Sentences it cannot parse do nothing and get "没听懂" ("didn't catch that"); single characters and noise get no reply at all.

### 3.5 Running it

```bash
cd ~/spider/software && source .venv/bin/activate
python scripts/voice_check.py                     # full self-test (say "小蜘蛛，前进三秒")
python scripts/voice_check.py --tts "语音系统就绪"  # speaker only ("voice system ready")
python scripts/voice_check.py --asr /tmp/voice_check.wav   # run recognition on a recording only

python scripts/voice_teleop.py --mock              # no servos yet (propping the robot up works too)
python scripts/voice_teleop.py                     # real robot; keyboard wasd/qe still works
python scripts/voice_teleop.py --no-wake           # quiet room: skip the wake word
python scripts/voice_teleop.py --trust-aec         # once the AEC is proven to work: listen for the e-stop while talking too
python scripts/voice_teleop.py --max-secs 6 --speed 30
python scripts/voice_teleop.py --vent off          # valve-policy control (default auto: while standing up or walking, the six valves are
                                                   # energized to vent so the cups see atmosphere; de-energized when standing still; on = always
                                                   # energized; off = never touch the valves, which with the pneumatics fitted means passive vacuum
                                                   # locks the feet and they cannot be lifted; the v key rotates through them at runtime, same as walk_teleop)
```

A dev machine with no board can still run the logic: `python scripts/voice_teleop.py --mock --wav some-clip.wav --no-tts`
(with `HEXAPOD_VOICE_MODELS` pointing at the model directory).

### 3.6 Tuning

| Symptom | Change |
|---|---|
| The wake word keeps triggering by itself | Threshold 0.25 → 0.35 in `keywords_raw.txt`, or switch to a 4-syllable word; re-run the keyword generation |
| It will not wake up | Drop the threshold to 0.15; check the recording peak with `voice_check.py --record 3`, and if it is tiny check the Mute red LED and the mixer; speak from within 1 m |
| The e-stop word triggers by itself | Threshold 0.35 → 0.45 (a false trigger only means an extra stop, so a bit of it is tolerable) |
| One sentence gets cut in half | `min_silence_duration` in `engine.py` 0.4 → 0.6 |
| Recognition gets worse once the pump runs | The XU316's noise suppression should handle steady pump noise; measure before theorizing. If it still fails, speak close up and rely on the e-stop / wake words only |
| Suspect the wrong mic channel is being used | Decide it with `voice_check.py --echo-test`; `HEXAPOD_AUDIO_PICK=1` temporarily switches to channel 1 to try |
| Shouting "停下" ("stop") while it talks does not stop it | See the three levers in §3.3: the sentence-end silence window is built in; e-stop threshold 0.20 (if you edited `keywords_raw.txt` you must regenerate the word list); `--tts-gain 0.6` plus moving the speaker farther away / facing it away from the mics |
| Deaf after an emergency stop (no reaction to the wake word either, KWS log lines stop) | Killing aplay on an e-stop occasionally chokes the USB sound card so arecord stops producing data (hit on the robot 09-02). A watchdog is in: no data for 2 s restarts arecord automatically (log line "⚠ 麦克风…重启", "⚠ mic … restarting"); if that still fails the whole card is hung — unplug and replug the USB cable |
| Pi undervoltage at high volume | `amixer -c <card> sset <playback control> 70%` (`voice_mixer.sh` defaults to 90%), then `alsactl store` |
| It answers a beat late | The first sentence has to load the model, ≈1.5 s; `voice_teleop` pre-warms the common phrases at startup. Dynamic sentences (voltage) are synthesized on the spot each time, 0.1–0.3 s |

### 3.7 Relation to the climbing scripts

Right now only floor walking is wired up (`voice_teleop.py` shares one loop with `walk_teleop.py`). `climb_walk.py` /
`body_lean.py` are interactive scripts with safety-rope discipline and were left alone this time; to add a voice emergency stop later,
feed `VoiceEngine.events` into their keyboard handling as a second key source (a stop event = the existing e-stop key).

### 3.8 Voiceprint lock (optional: only the owner's commands count)

Once a voiceprint is enrolled, **walk and similar commands are accepted only from the enrolled person**; anyone else (or a TV, or its own TTS) saying "前进" ("forward")
is rejected with no reply. **The emergency stop does not pass through this gate — whoever shouts "停下" ("stop"), it stops** (safety > convenience), and waking is not gated either
(streaming KWS cannot do low-latency speaker verification, and any command a stranger gives after waking it gets rejected anyway).

```bash
python scripts/voice_enroll.py            # read 5 prompt sentences + 4 short words to enroll → voiceprint_owner.npz
python scripts/voice_enroll.py --test     # verify yourself (✓), then have someone else verify (✗)
python scripts/voice_enroll.py --append   # existing profile: add only the short words (threshold untouched, no need to re-record)
python scripts/voice_teleop.py            # unlocks automatically when a profile is found; --no-voiceprint turns it off
```

- Profile: `<model root>/voiceprint_owner.npz` (changeable via `$HEXAPOD_VOICEPRINT`), storing voiceprints at two scales, **the whole clip +
  a 1.5 s sub-window** (a short command like "退出" ("quit") is only 0.5 s, and its similarity to a 4 s whole clip is naturally low
  — that is exactly why the owner was falsely rejected at 0.44 on the robot on 09-02; with multi-scale enrollment the same kind of query rises to 0.57+), plus
  a suggested threshold (enrollment self-similarity −0.15, clamped to 0.35–**0.50**); `--spk-threshold` overrides it.
  For very short utterances under 0.7 s the threshold drops another 0.05 automatically (widening that window to 1 s would let a stranger's 0.8 s command squeak past on the line).
- **Short-word anchors** (added 09-03): the voiceprint model is visibly content-dependent on audio under 1 s — the same person
  saying "确认" ("confirm") simply scores low against an enrollment clip of "前进三秒" ("forward three seconds"). At the end of the enrollment flow you now read
  确定/确认/退出/好的 ("okay" / "confirm" / "quit" / "alright"), four high-frequency short words, one at a time; the voiced span is cut out and stored as a separate anchor, so in practice
  saying one of those words has a same-word reference to score against. The e-stop words are not added (the e-stop never goes through voiceprint). An existing profile just runs `--append`,
  no full re-record. Quantified on the dev machine: owner short words 0.31–0.50 → **0.50–0.69** (all above the line);
  a stranger saying the same word also gains about 0.1 (the content now matches), but across 24 synthetic voices × 4 words = 96 trials
  the highest was only 0.412, still below the 0.45 concession line, and none got through.
- False reject (you get rejected — look at the score and the duration in the log) → first `--append` the word that was rejected /
  lower the threshold / re-enroll (enroll in conditions close to actual use; if you use it with the pump running, record a few clips with the pump running);
  false accept (someone else can command it) → raise the threshold.
- Dev-machine verification (multi-scale profile, two rounds of randomized synthesis): the owner's 5 short sentences passed 10/10 (lowest 0.57),
  three stranger voices were rejected 30/30 (highest 0.48); end to end on cmds.wav the owner's 5 commands all went through,
  the stranger's 5 were all rejected, and 2 emergency stops fired as usual.
- Limits: it cannot rescue a shout buried under the speaker (the voiceprint of an overlapped segment is mush too) — hitting the emergency stop
  while it talks still rests on the three levers of §3.3; a head cold, or shouting from far away, lowers the score.

### 3.9 Voice shell for climbing (`scripts/voice_climb.py`)

This adds voice to `climb_walk.py` (the P4 climbing bring-up), but the architecture differs from voice_teleop:
**climb_walk is not changed by a single line and runs as-is inside a pty**, and recognized commands are mapped to keystrokes written into
that pty — exactly equivalent to typing them, so all of its interlocks (single step in flight / already released / no strafing at large stride / frozen)
still apply and the keystrokes still land in the black box; the real keyboard is passed through raw (ESC×2 / o×2 / Ctrl-C keep their meaning);
its key outputs (in position / adhesion complete / frozen / hovering / single step done / released) are spoken by the shell.
If the voice engine dies, climbing is unaffected (the keyboard carries on).

| Say | Key | Notes |
|---|---|---|
| 停下 / 停止 / 别动 ("stop" / "halt" / "don't move") | space | Emergency stop; no wake needed, anyone can shout it, and it interrupts speech |
| 前进/后退/左移/右移/左转/右转 ("forward/back/strafe left/strafe right/turn left/turn right") [N 秒, N seconds] | w/s/a/d/q/e | No duration = keep going; with a duration the shell sends a space to stop when the time is up (capped by `--max-secs`, default 30); nothing is injected before adhesion is complete |
| 单步 / 抬腿；落地 / 踩下 ("single step" / "lift leg"; "set down" / "step down") | i | Both the lift and the set-down beat are i; climb_walk decides the phase |
| 解冻 / 解除冻结 ("unfreeze" / "clear the freeze"; matching on "冻结", "freeze", is enough) | f | f is a no-op when not frozen, so a loose word list is safe |
| 开始吸附 / **启动** ("start attaching" / "start") | p | Starts the full adhesion sequence after the in-position pause; "吸附" ("attach") is easily misheard (戏附/洗服/吸服 measured and folded into the word list), while "启动" ("start") has hard consonants and is the most reliable |
| 电压 ("voltage") | — | Speaks the voltage / current / worst cup pressure that the shell scrapes out of the status line |

**Voice blacklist** (keyboard only, forever — one misheard word must never be able to vent): quitting
(ESC×2, which vents, i.e. a fall if it is on the wall) and releasing the cups to take the robot off (o×2). Saying "退出/取机" ("quit" / "take it off") by voice only gets
you a pointer to the keyboard. 快点/慢点 ("faster" / "slower") do nothing (speed is fixed by `--speed`); stand / lie down / change gait are not supported.
There is also a second echo guard: an action word (walk / single step / set down / unfreeze / start) is not executed if it is contained in something the robot itself
said within the last 2.5 s — the engine-level echo filter only matches ≤2-character text exactly (to keep the e-stop faithful), so under --trust-aec the echo of
"悬停中，说落地收口" ("hovering; say 落地 to finish") could be heard as "落地" ("set down") and slip past the first guard.

Usage: `python scripts/voice_climb.py [voice options] [climb_walk options passed straight through]`
(the two option sets do not collide; `--release` is handed to climb_walk directly and does not start the voice side). The voiceprint lock turns itself on when a
profile exists (the e-stop is never gated). Wording discipline for announcements is the same as §3.3: never include a KWS e-stop word.

## 4. Troubleshooting

| Symptom | Check |
|---|---|
| No ReSpeaker Lite in `arecord -l` | Try a data-capable USB cable / another USB port; see whether `lsusb \| grep -i 2886` shows anything; check `dmesg \| tail` for enumeration errors. lsusb shows it but no sound card → it may have been flashed with the I2S firmware (§2.3, flash the USB version back) |
| Recording all zeros or tiny (voice_check warns about it) | **The board's Mute red LED is on** (press it again); did you run `voice_mixer.sh` |
| No sound on playback | The speaker must be in the **SPK** header, not the 3.5mm jack; with headphones in the 3.5mm jack the speaker may be cut off, so unplug them and retry; playback volume (`amixer -c <card>`) |
| `arecord: Device or resource busy` | Another voice_teleop / voice_check is still running |
| Clipping at high volume / Pi undervoltage (`get_throttled` ≠ 0) | Does `config.txt` have `usb_max_current_enable=1` (a reboot is needed after adding it); lower the playback volume; last resort, feed the board's 5V pads directly |
| Nothing happens after waking | Look at the `[asr]` lines in the terminal: text recognized correctly but intent unknown → add the word to `intents.py`; no `[asr]` line at all → the VAD never cut a sentence (pause 0.5 s after speaking) |
| Shouting "停下" ("stop") while the robot talks does nothing | That is the default behavior (mic closed while talking). With `--trust-aec` it relies on KWS punching through the echo; for the hit rate see the AEC verdict in §6 |
| **You have to shout to be heard while it walks** | Settled from recordings on 09-03 (§5, round nine): 98% of the walking noise is above 2kHz (gear whine), and the in-band speech SNR is only ~2dB; **the weak link is the 3.3M KWS model, since ASR usually understands fine** (on the same recording ASR produced "前进3秒" ("forward 3 seconds") perfectly, while KWS needed 3.0:0.10 to fire at all = beyond the e-stop discipline, so it is out). Filtering is disproven: a high-pass buys nothing (the low band is only 1%), and low-pass / band-limit cut the unvoiced consonants along with the noise, making ASR muddier. **Fix for a walking session: `--no-wake` + the voiceprint lock** — pure servo noise produces zero VAD sentences and no spurious commands (measured), strangers are rejected by voiceprint, and the e-stop KWS works as usual. To keep wake mode, shout "蜘蛛同学" ("spider buddy": its syllable structure survives noise better than "小蜘蛛", whose x consonant in "小" sits right in the noise band). Physical: move it away + point the mic at the person + a foam baffle between it and the servos (high frequencies are directional and die fast; vibration damping does nothing). Tools: `--noise-test` walks you through the measurement, `--noise-analyze` breaks an existing wav down by band. There is no software knob for recording gain (the firmware exposes no amixer control); the `HEXAPOD_AUDIO_HPF` high-pass is reserved for low-frequency pump noise later |
| The robot answers its own questions / repeats replies | The self-hearing filter should catch it (added 09-02; the log shows "≈ 自己刚说的，丢弃", "≈ same as what I just said, discarded"); if it still happens, post the recognized text and the original reply text from the log side by side — most likely the echo was misheard badly (similarity <0.7), so increase the physical distance between speaker and mic, or lower the volume |

## 5. Dev-machine verification log (2026-09-01, x86 + sherpa-onnx 1.13.7)

Independent of the sound card, so it still holds after the switch to the Lite:

- Keyword generation: the pypinyin output of `keywords.py` matches sherpa's official `text2token` example
  ("文森特卡索", the name Vincent Cassel → `w én s ēn t è k ǎ s uǒ`); all 6 words are inside the model's vocabulary.
- TTS, one of three (the same 6 sentences, synthesized and then fed to SenseVoice to see whether they come back):

  | Model | RTF | Loopback recognition |
  |---|---|---|
  | `sherpa-onnx-vits-zh-ll` (5 voices) | 0.36 | "前进三秒" → "田忌3秒/前击3秒", "停下来" → "赢下来"; bad |
  | `vits-melo-tts-zh_en` | 0.52 | "前进三秒" → "眼镜3秒"; bad |
  | **`matcha-icefall-zh-baker` + vocos** | **0.12** | 5 of the 6 sentences perfect, "电压7.9伏，电流1.2安" word for word |

- Full chain (12 Matcha-synthesized sentences concatenated into a 25 s wav → `VoiceEngine`):
  "小蜘蛛" wakes it → "前进三秒" → walk vx=+1 for 3 s; "向左转两秒" ("turn left two seconds") → wz=+1 for 2 s; "蜘蛛同学" wakes it →
  "电压多少" → status; "快点后退" ("back up faster") → vx=−1 ×1.5; "停下来" → KWS emergency stop; "小蜘蛛" → "站起来" → stand;
  "别动" → emergency stop. 60–100 ms to recognize each sentence. `voice_teleop.py --mock --wav` correctly stopped itself when the duration expired and zeroed everything on an emergency stop.
- Unit tests: `tests/test_voice_intents.py` 31 cases + `tests/test_voice_keywords.py` 4 cases, all green.

**2026-09-02, on the Pi**: the card was recognized as soon as it was plugged in (card name `Lite`), and `voice_check.py` passed end to end —
a 5 s recording peaking at 0.771, a "小蜘蛛" wake hit, "前进三秒" → `前进3秒。` → walk intent (0.11 s to recognize), and TTS came out of the speaker.
The engine models load in 3.1 s and TTS in 2.75 s (teleop pre-warms them at startup). The `Unknown token: shei2` ×4 printed while loading is
harmless: the colloquial reading shei2 of "谁" ("who") is missing from matcha-zh-baker's token table, so only sentences containing "谁" are affected, and no reply of the robot's uses that character.

The same day, `voice_teleop.py --mock --trust-aec` exposed the self-questioning loop on the robot (the "电压…" ("voltage…") reply was
recognized again as status, once every 3 s until it was killed) → verdict: the on-board AEC does not suppress self-hearing (or the processed audio
is not on channel 0). The self-hearing filter plus a whole-sentence ignore for the wake word were added that day (§3.3), and the scenario was reproduced on the dev machine:
feeding the engine a synthesized echo of "电压7.4伏，电流1.0安" → `≈ 自己刚说的，丢弃` ("≈ same as what I just said, discarded"), and no status event was produced;
the normal command stream of cmds.wav regressed cleanly; pytest 154 cases all green.

Then `--echo-test` settled it on the robot: the firmware natively gives 2 channels / 16 kHz / S16_LE; the two channels have exactly the same
RMS (0.1253/0.1253, i.e. one signal duplicated), and both of them recognized the digits in the echo as a whole sentence
(`1234567891012345678910`) — **the AEC is useless against self-hearing and there is no clean channel**; that conclusion is written into §3.3.
A firmware upgrade is very likely pointless (the v2.0.5→v2.0.7 changelog lists only flash/WS2812 changes and says nothing about audio algorithms),
not worth the DFU hassle.

Round three (dev machine, realistic timing simulation): a fake player that "sleeps as long as it plays" plus a self-introduction
echo wav fed in real time simulated a full `--trust-aec` run — its own "小蜘蛛" was caught by `[kws] 说话期间忽略唤醒` (wake ignored while speaking),
both 7.2 s echo segments came back `≈ 自己刚说的，丢弃`, and the sentence-final "想让我停" ("…want me to stop") did not falsely trigger the e-stop; the normal cmds.wav
command stream still produced its 10 events; pytest 156 cases all green.

Round four (first hit-rate test on the robot 09-02 + fixes on the dev machine): on the robot under `--trust-aec`, shouting
"停下" during the self-introduction **missed** (KWS was listening — it even caught its own "小蜘蛛" — so this was pure acoustic masking; and the user's
shout, mixed into the 7.2 s echo segment, was discarded along with it); the tail of the ready announcement "…叫我小蜘蛛" also triggered one self-wake
reply "在" (the 0.3 s tail was not enough). Fixes = sentence-by-sentence announcements with a 0.45 s silence window between sentences, e-stop threshold 0.20:2.0,
`--tts-gain`, and a 1 s wake-suppression tail. Re-tested with realistic timing on the dev machine: every per-sentence echo (including the misheard "六组机器人")
was discarded, self-wake was ignored, and cancel skipped the remaining sentences; sherpa loaded the boosted word list normally and hit the e-stop as usual; pytest 157 cases all green.

Round six (first voiceprint test on the robot 09-02): the owner saying "退出" (0.5 s) scored 0.44 < 0.60 and was **falsely rejected** —
two causes: the voiceprint vector is jittery on short audio, and enrollment held only the 4 s whole clip (a length mismatch); on top of that the suggested threshold was clamped
to an upper bound of 0.60, which is too harsh. Fixes = add a 1.5 s sub-window voiceprint to enrollment (multi-scale), lower the suggested-threshold cap 0.60→0.50, and drop
another 0.05 for utterances under 0.7 s (the window must not be widened to 1 s: a stranger's 0.8 s command lands at 0.43–0.46, right on the line,
and squeaks past — measured on the dev machine). Re-test (two rounds of randomized synthesis): the owner's short sentences passed 10/10 (lowest 0.57),
strangers were rejected 30/30 (highest 0.48). **You must re-enroll after this change** (old profiles have no sub-window voiceprint).

Round five (second hit-rate test on the robot 09-02): with sentence splitting and the 0.20 threshold, **shouting "停下" while it talks now hits**
(`[kws] 急停词 停下`, and self-wake was correctly ignored). Two new problems surfaced and were fixed: ① an e-stop landing inside the ~1 s window while
"the next sentence is being synthesized" would still start playing once synthesis finished (cancel was only checked at the head of a sentence) → check cancel again after synthesis
and before playback (verified with stubs in tests/test_voice_speaker.py); ② deafness after an e-stop — suspected that killing
aplay chokes the USB sound card, so arecord stops producing data and the engine blocks reading the mic → a watchdog was added to ArecordSource
(no data for 2 s restarts arecord automatically; only three failures in a row give eof), and aplay's stop was changed to terminate plus reaping. pytest 163 cases all green.

Round seven (09-03, feedback from the robot that "确认/退出 are still unreliable"): the multi-scale sub-windows only eased the length mismatch,
and the 0.5 s short words still had the "different content" layer left — the voiceprint model is visibly content-dependent under 1 s, and the same
person's "确认" simply scores low against an enrollment clip of "前进三秒". Fix = at the end of the enrollment flow, record
确定/确认/退出/好的, the four short words, one at a time (the voiced span is cut out and stored as a same-word anchor); an existing profile just runs `--append`,
no full re-record. Quantified on the dev machine (vits-zh-ll sid0 as the owner; 4 voices from the same engine plus 20 real voices from the aishell3 corpus
as strangers, with 20 dB of noise floor added to the test audio and the speed altered so it never matches the anchor waveform): the owner's short words go from tier A
0.31–0.50 to tier B **0.50–0.69**, all above the line, while short words with no anchor recorded (退下/算了, "step back" / "forget it") are unaffected;
strangers saying the same word gain about 0.1 (the content matches), but across 96 trials the highest was 0.412, still under the 0.45 concession line,
0 got through; the strangers' long commands topped out at 0.400. pytest 185 cases all green.

Round eight (09-03, the full voice_climb chain on the dev machine: real KWS + SenseVoice models plus a synthesized command
track driving climb_walk --mock): witnessed = wake → "现在启动" ("start now") → key p → adhesion sequence → the "六足
吸附完成" ("all six feet attached") announcement → "往前走" ("walk forward") → key w → status-line speed +13 (really walking) → KWS emergency stop → space → the full keyboard
ESC×2 exit sequence → exit code 0; a walk command before adhesion completes being rejected and never injected, voiceprint rejection,
the abort path during the in-position pause, and the keyboard taking over after eof were all witnessed too. Spoils along the way: ① frequent mishearings such as "电压→低压", "解除→
接出" and "吸附→戏附/洗服/吸服" have been folded into the word list; ② traps in the synthesized track (irrelevant to the real robot but
recorded for reference): vits-zh-ll distorts two-character words spoken alone ("停下" comes out as "因子"), so short words
must be embedded in a carrier sentence; the same "小蜘蛛" clip following different context scores right on the KWS threshold line,
and numerical jitter with num_threads>1 will derail it — none of this happens with real voices on the robot, where the e-stop and wake had already measured fine.
pytest 189 cases all green.

Round nine (09-03, attributing "you have to shout while it walks" from real recordings; the user uploaded three --noise-test recordings and the full
analysis ran on the dev machine): ① spectral picture — 98% of the walking noise is above 2kHz (gear whine) and only 1% below 300Hz,
so the "structure-borne low frequency" hypothesis is overturned and the high-pass disproven (0.0dB gain); inside the speech band (300–4k)
the SNR is only ~2.4dB, but below 2kHz the user's speech is clean (SNR 17dB+). ② Filtering disproven
— three variants (low-pass 3400, low-pass 2500, band-limit 250-3400) run through the whole chain: ASR actually gets muddier ("小蜘蛛
前进3秒" → "四周/这周/都前进3秒"), because unvoiced consonants share the high band with the noise, so cutting the noise must cut the
consonants; the "filter to the speech band" route is closed. ③ The real weak link is KWS: on the same noisy recording ASR came straight out with
"饺蜘蛛前进3秒" (the command entirely correct), while the KWS wake threshold had to be swept to 3.0:0.10 before it fired — past
the 2.0 discipline limit of the e-stop, so it is out. ④ The fix, demonstrated: under --no-wake, pure walking noise produces zero VAD sentences
(no spurious commands), and during the still segments wake plus commands were all correct — **for a walking session, --no-wake + the voiceprint lock is the recommendation**;
the highpass (HEXAPOD_AUDIO_HPF, off by default) is reserved for low-frequency pump noise later;
the --noise-analyze band-breakdown tool landed with readings for three shapes of noise. pytest 192 cases all green.

## 6. To-do

- [x] Plug the board in and run `voice_setup.sh` + `voice_check.py` (passed 09-02, card name `Lite`, see §5)
- [ ] If you change the KWS threshold or the mixer volume during normal use, write it back into §3.6
- [ ] The 09-03 "can't hear it while walking" issue is attributed (§5 round nine: KWS is the weak link, filtering disproven). Still to verify on the robot:
  ① a real walking session with `voice_teleop --no-wake` (the voiceprint lock guards the door; pure noise producing no spurious commands is already
  demonstrated), logging wrong actions and missed commands; ② in wake mode, compare the walking hit rate of "蜘蛛同学" vs "小蜘蛛";
  ③ after the physical changes (mic moved away / pointed at the person / foam baffle between it and the servos), re-run --noise-test
  with a target of >10dB in-band SNR and write it back. The word list is already at the sensitive 0.20:1.5, but keywords_hexapod.txt still has to be regenerated after pulling
  (it helps when still or in low noise)
- [x] ~~First round of AEC verification~~ (09-02: under `--trust-aec` it answered itself, so the AEC does not suppress self-hearing; a self-hearing
  filter was added as a backstop, see §3.3/§5)
- [x] ~~AEC channel verdict~~ (09-02 `--echo-test`: both channels identical, the echo recognizable as a whole sentence → the AEC is useless and there is
  no clean channel; accept it as is, `HEXAPOD_AUDIO_PICK` stays 0, see §5)
- [ ] Measure the hit rate for shouting "停下" while the robot is talking: say "小蜘蛛" → "自我介绍", and during the 15 s reply
  shout "停下"; it should shut up instantly and log `[voice] 急停`; try it several times, record the hit rate and write it back into this section
  (KWS is always on while speaking now, so both the default mode and `--trust-aec` can be tested)
- [ ] Voiceprint lock on the robot: enroll with `voice_enroll.py` (including a segment with the pump running) → `--test` yourself and someone else
  → under teleop, someone else's command should log `[voice] 声纹不符` (voiceprint mismatch) and shouting "停下" should still stop it; record the threshold used on the robot.
  After pulling on 09-03, first `--append` the short words (or re-enroll, the flow now includes the short-word step),
  then use `--test` on "确认" and "退出" to see whether the scores are ≥0.45
- [ ] Measure the recognition rate under servo/pump noise; if needed, switch to `sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20`
- [x] ~~Wire voice emergency-stop events into `climb_walk.py`~~ (09-03: the `voice_climb.py` voice shell landed, with the full set of
  e-stop + walking + single step + unfreeze + start, §3.9; the whole chain was witnessed on the dev machine, still to be done on the robot)

## References

- ReSpeaker Lite getting started (Seeed wiki) <https://wiki.seeedstudio.com/reSpeaker_usb_v3/>
- Firmware and DFU guide <https://github.com/respeaker/ReSpeaker_Lite> (`xmos_firmwares/`, USB version currently v2.0.7)
- sherpa-onnx: KWS <https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html>,
  SenseVoice <https://k2-fsa.github.io/sherpa/onnx/sense-voice/pretrained.html>,
  TTS <https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/vits.html>
