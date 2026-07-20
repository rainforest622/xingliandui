#!/usr/bin/env bash
set -euo pipefail

# Installs the offline ASR runtime used by voice_service.py.  It is isolated in
# a venv so the camera/rover services keep their existing Python environment.
APP_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="${VENV_DIR:-$APP_DIR/.venv-voice}"
MODEL_DIR="${VOICE_MODEL_DIR:-$APP_DIR/models/voice}"
SENSE_ARCHIVE="sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17.tar.bz2"
SENSE_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/${SENSE_ARCHIVE}"
VAD_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx"

sudo apt update
sudo apt install -y python3-venv python3-pip python3-sounddevice libportaudio2 curl
python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install --upgrade sherpa-onnx sounddevice numpy pyserial

mkdir -p "$MODEL_DIR"
if [ ! -f "$MODEL_DIR/silero_vad.onnx" ]; then
  curl -fL --retry 3 --retry-delay 2 "$VAD_URL" -o "$MODEL_DIR/silero_vad.onnx"
fi
if [ ! -f "$MODEL_DIR/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/model.int8.onnx" ]; then
  archive_path="$MODEL_DIR/$SENSE_ARCHIVE"
  curl -fL --retry 3 --retry-delay 2 "$SENSE_URL" -o "$archive_path"
  tar -xjf "$archive_path" -C "$MODEL_DIR"
  rm -f "$archive_path"
fi

echo "Voice runtime ready."
echo "SenseVoice: $MODEL_DIR/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/model.int8.onnx"
echo "Tokens:     $MODEL_DIR/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/tokens.txt"
echo "Silero VAD: $MODEL_DIR/silero_vad.onnx"
