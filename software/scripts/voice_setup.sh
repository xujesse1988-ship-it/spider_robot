#!/usr/bin/env bash
# 树莓派一次性安装语音交互（ReSpeaker Lite USB 版 + sherpa-onnx 全离线）。
# Lite 是 UAC2 标准 USB 声卡，免驱、不占 GPIO，插上就有 arecord/aplay 设备。
#   bash software/scripts/voice_setup.sh          # 全部 5 步
#   bash software/scripts/voice_setup.sh models   # 只下模型（网络断了重跑）
# 可选环境变量：
#   HEXAPOD_VOICE_MODELS  模型目录（默认 ~/models/voice）
#   VENV                  Python venv（默认 software/.venv，约定见 requirements.txt）
#   SHERPA_ONNX_MIRROR    GitHub 下载前缀（如 https://ghfast.top/），国内网慢时用
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)         # software/scripts
SW=$(dirname "$HERE")                       # software
MODELS=${HEXAPOD_VOICE_MODELS:-$HOME/models/voice}
VENV=${VENV:-$SW/.venv}
CFG=/boot/firmware/config.txt; [ -f "$CFG" ] || CFG=/boot/config.txt
ONLY=${1:-all}
step() { echo; echo "==> $*"; }

KWS=sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01
ASR=sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17
TTS=matcha-icefall-zh-baker
VOCODER=vocos-22khz-univ.onnx
SPK=3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx   # 声纹锁（tag 的拼写错误是官方原样）
GH=${SHERPA_ONNX_MIRROR:-}https://github.com/k2-fsa/sherpa-onnx/releases/download

fetch_file() {  # <URL> <目标文件>   -f：HTTP 出错(404/镜像错误页)就失败，别把错误页存成模型
  curl -fSL --retry 3 -o "$2" "$1" || { rm -f "$2"; echo "  ✗ 下载失败：$1"; return 1; }
}

fetch_tar() {  # <子目录名> <tag>
  if [ -d "$MODELS/$1" ]; then echo "  已有 $1"; return; fi
  echo "  下载 $1 …"; fetch_file "$GH/$2/$1.tar.bz2" "$MODELS/$1.tar.bz2"
  tar xjf "$MODELS/$1.tar.bz2" -C "$MODELS" && rm -f "$MODELS/$1.tar.bz2"
}

if [ "$ONLY" = all ]; then
  step "1/5 系统包（alsa-utils、venv）"
  sudo apt-get update -qq
  sudo apt-get install -y --no-install-recommends alsa-utils \
       python3-venv python3-pip bzip2 curl

  step "2/5 检查 ReSpeaker Lite 与 USB 供电"
  if grep -qiE 'respeaker|lite' /proc/asound/cards 2>/dev/null; then
    echo "  声卡已识别："; grep -iE 'respeaker|lite' /proc/asound/cards | sed 's/^/  /'
  else
    echo "  ⚠ /proc/asound/cards 里还没有 ReSpeaker Lite——USB-C 线插到 Pi 任一 USB 口"
    echo "    （要数据线，不是纯充电线）；插上即插即用，无需重启。"
  fi
  # Pi 5 用降压模块供电时没有 USB-PD 协商，USB 口总电流默认限 600mA；
  # Lite 带 3W 喇叭大音量峰值会超，放开到 1.6A（重启后生效）。
  if grep -q '^usb_max_current_enable=1' "$CFG"; then
    echo "  $CFG 已有 usb_max_current_enable=1"
  else
    echo 'usb_max_current_enable=1' | sudo tee -a "$CFG" >/dev/null
    echo "  已追加 usb_max_current_enable=1 → $CFG（USB 口总电流 600mA→1.6A，重启后生效）"
  fi

  step "3/5 Python 包 → $VENV"
  [ -d "$VENV" ] || python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q -U pip
  "$VENV/bin/pip" install -q -U sherpa-onnx numpy pypinyin
  "$VENV/bin/python" -c "import sherpa_onnx, numpy, pypinyin; print('  sherpa-onnx', sherpa_onnx.__version__)"
fi

step "4/5 模型 → $MODELS（KWS 18MB + SenseVoice 228MB + VAD 2MB + Matcha 73MB + 声码器 30MB + 声纹 28MB）"
mkdir -p "$MODELS"
fetch_tar "$KWS" kws-models
fetch_tar "$ASR" asr-models
fetch_tar "$TTS" tts-models
[ -f "$MODELS/silero_vad.onnx" ] || { echo "  下载 silero_vad.onnx …"; fetch_file "$GH/asr-models/silero_vad.onnx" "$MODELS/silero_vad.onnx"; }
[ -f "$MODELS/$VOCODER" ] || { echo "  下载 $VOCODER …"; fetch_file "$GH/vocoder-models/$VOCODER" "$MODELS/$VOCODER"; }
[ -f "$MODELS/$SPK" ] || { echo "  下载 $SPK …"; fetch_file "$GH/speaker-recongition-models/$SPK" "$MODELS/$SPK"; }
# 声纹模型完整性（应 28281138 字节；镜像错误页/断传会小很多）
if [ -f "$MODELS/$SPK" ] && [ "$(stat -c%s "$MODELS/$SPK")" != 28281138 ]; then
  echo "  ⚠ $SPK 大小不对（$(stat -c%s "$MODELS/$SPK") ≠ 28281138），已删除——"
  echo "    重跑本脚本；用了 SHERPA_ONNX_MIRROR 还失败就去掉镜像前缀直连重试"
  rm -f "$MODELS/$SPK"
fi
du -sh "$MODELS"/* | sed 's/^/  /'
[ "$ONLY" = models ] && exit 0

step "5/5 唤醒词/急停词表 + 混音器"
( cd "$SW" && PYTHONPATH="$SW" "$VENV/bin/python" -m hexapod.voice.keywords \
    --tokens "$MODELS/$KWS/tokens.txt" --out "$MODELS/$KWS/keywords_hexapod.txt" \
    --raw "$SW/hexapod/voice/keywords_raw.txt" )
if grep -qiE 'respeaker|lite' /proc/asound/cards 2>/dev/null; then
  bash "$HERE/voice_mixer.sh"
else
  echo "  声卡不在，跳过混音器。插上板子后跑: bash software/scripts/voice_mixer.sh"
fi

cat <<TXT

完成。下一步：
  1. arecord -l && aplay -l          # 应看到 ReSpeaker Lite（USB Audio）
  2. cd software && .venv/bin/python scripts/voice_check.py      # 录 5 秒→回放→识别→TTS
  3. .venv/bin/python scripts/voice_enroll.py                    # （可选）声纹注册：指令只听你
  4. .venv/bin/python scripts/voice_teleop.py --mock             # 先不带舵机试语音
（若第 2/5 步追加了 usb_max_current_enable=1，测大音量前先 sudo reboot）
TXT
