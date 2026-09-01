#!/usr/bin/env bash
# 树莓派一次性安装语音交互（ReSpeaker 2-Mics Pi HAT + sherpa-onnx 全离线）。
#   bash software/scripts/voice_setup.sh          # 全部 6 步
#   bash software/scripts/voice_setup.sh models   # 只下模型（网络断了重跑）
# 可选环境变量：
#   HEXAPOD_VOICE_MODELS  模型目录（默认 ~/models/voice）
#   VENV                  Python venv（默认 software/.venv，约定见 requirements.txt）
#   SHERPA_ONNX_MIRROR    GitHub 下载前缀（如 https://ghfast.top/），国内网慢时用
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)         # software/scripts
SW=$(dirname "$HERE")                       # software
ROOT=$(dirname "$SW")                       # 仓库根
MODELS=${HEXAPOD_VOICE_MODELS:-$HOME/models/voice}
VENV=${VENV:-$SW/.venv}
CFG=/boot/firmware/config.txt; [ -f "$CFG" ] || CFG=/boot/config.txt
OVL_DIR=/boot/firmware/overlays; [ -d "$OVL_DIR" ] || OVL_DIR=/boot/overlays
ONLY=${1:-all}
step() { echo; echo "==> $*"; }

KWS=sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01
ASR=sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17
TTS=matcha-icefall-zh-baker
VOCODER=vocos-22khz-univ.onnx
GH=${SHERPA_ONNX_MIRROR:-}https://github.com/k2-fsa/sherpa-onnx/releases/download

fetch_tar() {  # <子目录名> <tag>
  if [ -d "$MODELS/$1" ]; then echo "  已有 $1"; return; fi
  echo "  下载 $1 …"; curl -SL --retry 3 -o "$MODELS/$1.tar.bz2" "$GH/$2/$1.tar.bz2"
  tar xjf "$MODELS/$1.tar.bz2" -C "$MODELS" && rm -f "$MODELS/$1.tar.bz2"
}

if [ "$ONLY" = all ]; then
  step "1/6 系统包（dtc、alsa-utils、venv）"
  sudo apt-get update -qq
  sudo apt-get install -y --no-install-recommends device-tree-compiler alsa-utils \
       python3-venv python3-pip bzip2 curl i2c-tools

  step "2/6 设备树覆盖层 respeaker-2mic-v1_0（Seeed 官方 dts，见 hardware/voice/README.md）"
  dtc -@ -I dts -O dtb -o /tmp/respeaker-2mic-v1_0.dtbo \
      "$ROOT/hardware/voice/respeaker-2mic-v1_0-overlay.dts"
  sudo cp /tmp/respeaker-2mic-v1_0.dtbo "$OVL_DIR/"
  if grep -q '^dtoverlay=respeaker-2mic-v1_0' "$CFG"; then echo "  $CFG 已有 dtoverlay 行"
  else echo 'dtoverlay=respeaker-2mic-v1_0' | sudo tee -a "$CFG" >/dev/null; echo "  已追加 dtoverlay=respeaker-2mic-v1_0 → $CFG"; fi
  grep -q '^dtparam=i2c_arm=on' "$CFG" || echo "  ⚠ $CFG 里没有 dtparam=i2c_arm=on（ADS1115 也靠它，正常应该已开）"
  if grep -q '^dtoverlay=wm8960-soundcard' "$CFG"; then
    echo "  ⚠ $CFG 里还有 dtoverlay=wm8960-soundcard（Waveshare 板用的，时钟配置和本板不符），请删掉"; fi

  step "3/6 Python 包 → $VENV"
  [ -d "$VENV" ] || python3 -m venv "$VENV"
  "$VENV/bin/pip" install -q -U pip
  "$VENV/bin/pip" install -q -U sherpa-onnx numpy pypinyin
  "$VENV/bin/python" -c "import sherpa_onnx, numpy, pypinyin; print('  sherpa-onnx', sherpa_onnx.__version__)"
fi

step "4/6 模型 → $MODELS（KWS 18MB + SenseVoice 228MB + VAD 2MB + Matcha 73MB + 声码器 30MB）"
mkdir -p "$MODELS"
fetch_tar "$KWS" kws-models
fetch_tar "$ASR" asr-models
fetch_tar "$TTS" tts-models
[ -f "$MODELS/silero_vad.onnx" ] || { echo "  下载 silero_vad.onnx …"; curl -SL --retry 3 -o "$MODELS/silero_vad.onnx" "$GH/asr-models/silero_vad.onnx"; }
[ -f "$MODELS/$VOCODER" ] || { echo "  下载 $VOCODER …"; curl -SL --retry 3 -o "$MODELS/$VOCODER" "$GH/vocoder-models/$VOCODER"; }
du -sh "$MODELS"/* | sed 's/^/  /'
[ "$ONLY" = models ] && exit 0

step "5/6 唤醒词/急停词表 → $MODELS/$KWS/keywords_hexapod.txt"
( cd "$SW" && PYTHONPATH="$SW" "$VENV/bin/python" -m hexapod.voice.keywords \
    --tokens "$MODELS/$KWS/tokens.txt" --out "$MODELS/$KWS/keywords_hexapod.txt" \
    --raw "$SW/hexapod/voice/keywords_raw.txt" )

step "6/6 混音器"
if grep -q 'seeed2micvoicec' /proc/asound/cards 2>/dev/null; then
  bash "$HERE/voice_mixer.sh"
else
  echo "  声卡还没出现（覆盖层要重启后才生效）。重启后跑: bash software/scripts/voice_mixer.sh"
fi

cat <<TXT

完成。下一步：
  1. sudo reboot
  2. arecord -l && aplay -l          # 应看到 card N: seeed2micvoicec
     i2cdetect -y 1                  # 应看到 1a（WM8960）以及 48/49（ADS1115）
  3. bash software/scripts/voice_mixer.sh      # 若第 6 步跳过了
  4. cd software && .venv/bin/python scripts/voice_check.py      # 录 5 秒→回放→识别→TTS
  5. .venv/bin/python scripts/voice_teleop.py --mock             # 先不带舵机试语音
TXT
