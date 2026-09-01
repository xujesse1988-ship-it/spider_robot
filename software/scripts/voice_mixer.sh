#!/usr/bin/env bash
# ReSpeaker 2-Mics HAT（WM8960）混音器初始化：麦克风增益 + 喇叭音量 + 通路开关，
# 然后 alsactl store 存盘，重启后仍生效。
#   bash scripts/voice_mixer.sh            # 自动找 seeed2micvoicec
#   bash scripts/voice_mixer.sh 2          # 指定声卡号
# 控制名来自 wm8960 驱动（respeaker/seeed-voicecard 的 wm8960_asound.state），
# 若某条报 "Unable to find simple control"，用 `amixer -c <卡> scontrols` 对一下名字。
set -u
CARD=${1:-}
if [ -z "$CARD" ]; then
  CARD=$(awk '/seeed2micvoicec|wm8960/ {gsub(/\[|\]|:/,"",$2); print $2; exit}' /proc/asound/cards)
fi
[ -n "$CARD" ] || { echo "没找到 HAT 声卡（/proc/asound/cards 里无 seeed2micvoicec）"; exit 1; }
echo "声卡: $CARD"
fail=0
s() { amixer -q -c "$CARD" sset "$@" 2>/dev/null || { echo "  ⚠ 设不了: $*"; fail=1; }; }

# ---- 麦克风（两只模拟 MEMS 麦 → LINPUT1 / RINPUT1）----
s 'Capture' 50                          # PGA 0..63（63=+30dB）。爆音就降到 40
s 'Left Input Boost Mixer LINPUT1' 3    # 输入升压 0..3（3=+29dB）
s 'Right Input Boost Mixer RINPUT1' 3
s 'Left Boost Mixer LINPUT1' on
s 'Right Boost Mixer RINPUT1' on
s 'Left Input Mixer Boost' on
s 'Right Input Mixer Boost' on
s 'ADC PCM' 195                         # 数字增益 0..255，195 = 0dB
s 'ADC High Pass Filter' on             # 去直流/低频轰鸣（泵振动）

# ---- 喇叭（JST 口 = 左声道 D 类功放 SPK_LP/SPK_LN）----
s 'Playback' 255                        # DAC 数字 0dB
s 'Speaker' 110                         # 0..127（127=+6dB）；4Ω 3W 小喇叭先 110，听着爆再降
s 'Headphone' 100
s 'Left Output Mixer PCM' on
s 'Right Output Mixer PCM' on

sudo alsactl store 2>/dev/null || alsactl store 2>/dev/null || echo "  ⚠ alsactl store 失败（重启后音量会回默认）"
if [ $fail = 0 ]; then echo "混音器设置完成并已存盘。"; else
  echo "有控制项名字对不上，运行: amixer -c $CARD scontrols  查看实际名字后改本脚本"; fi
echo "试听:  speaker-test -D plughw:CARD=$CARD,DEV=0 -c 1 -t sine -f 440 -l 1"
echo "录音:  arecord -D plughw:CARD=$CARD,DEV=0 -f S16_LE -r 16000 -c 2 -d 3 /tmp/t.wav && aplay -D plughw:CARD=$CARD,DEV=0 /tmp/t.wav"
