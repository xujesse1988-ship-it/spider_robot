#!/usr/bin/env bash
# ReSpeaker Lite（USB UAC2）混音器初始化：放音/录音音量 + 取消静音，
# 然后 alsactl store 存盘，重启后仍生效。
#   bash scripts/voice_mixer.sh            # 自动找 ReSpeaker Lite
#   bash scripts/voice_mixer.sh 2          # 指定声卡号
# UAC2 设备暴露哪些控制项由固件定（XU316 的增益/AEC 都在片内，主机侧通常
# 只有一两个音量），所以这里不写死控制名：枚举 scontrols，放音设 90%、
# 录音设 90%、全部取消静音。想微调先 `amixer -c <卡> scontrols` 看名字。
set -u
CARD=${1:-}
if [ -z "$CARD" ]; then
  CARD=$(awk '/^ *[0-9]+ +\[/ && tolower($0) ~ /respeaker|lite|xvf/ {gsub(/\[|\]|:/,"",$2); print $2; exit}' /proc/asound/cards)
fi
[ -n "$CARD" ] || { echo "没找到 ReSpeaker Lite（/proc/asound/cards 里无 respeaker/lite），USB 插好了吗"; exit 1; }
echo "声卡: $CARD"

amixer -c "$CARD" scontrols | sed -E "s/^Simple mixer control '([^']+)'.*/\1/" | \
while IFS= read -r name; do
  info=$(amixer -c "$CARD" sget "$name" 2>/dev/null) || continue
  if echo "$info" | grep -q 'pvolume'; then
    amixer -q -c "$CARD" sset "$name" 90% unmute 2>/dev/null \
      && echo "  放音 '$name' → 90%"
  fi
  if echo "$info" | grep -q 'cvolume'; then
    amixer -q -c "$CARD" sset "$name" 90% cap 2>/dev/null \
      && echo "  录音 '$name' → 90%"
  fi
  # 只有开关没有音量的项（如固件的 Mute 开关）：确保打开
  if ! echo "$info" | grep -qE 'pvolume|cvolume'; then
    amixer -q -c "$CARD" sset "$name" on 2>/dev/null || true
  fi
done

sudo alsactl store 2>/dev/null || alsactl store 2>/dev/null || echo "  ⚠ alsactl store 失败（重启后音量会回默认）"
echo "完成。注意：板上实体 Mute 键（按下红灯亮）是硬件静音，软件设不回来。"
echo "试听:  speaker-test -D plughw:CARD=$CARD,DEV=0 -c 1 -t sine -f 440 -l 1"
echo "录音:  arecord -D plughw:CARD=$CARD,DEV=0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/t.wav && aplay -D plughw:CARD=$CARD,DEV=0 /tmp/t.wav"
