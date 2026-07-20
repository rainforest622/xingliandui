#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-nearlink-voice}"
APP_DIR="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_APP_USER="$(id -un)"
if [ "$(id -u)" = "0" ] && [ -n "${SUDO_USER:-}" ]; then
  DEFAULT_APP_USER="${SUDO_USER}"
fi
APP_USER="${APP_USER:-${DEFAULT_APP_USER}}"
APP_HOME="${APP_HOME:-$(getent passwd "$APP_USER" | cut -d: -f6)}"
PYTHON_BIN="${PYTHON_BIN:-$APP_DIR/.venv-voice/bin/python}"
VOICE_MODE="${VOICE_MODE:-serial}"
VOICE_SERIAL_PORT="${VOICE_SERIAL_PORT:-/dev/nearlink-asr}"
VOICE_BAUDRATE="${VOICE_BAUDRATE:-9600}"
VOICE_SERIAL_PROTOCOL="${VOICE_SERIAL_PROTOCOL:-asrpro-byte}"
VOICE_MODEL_DIR="${VOICE_MODEL_DIR:-$APP_DIR/models/voice}"
ARBITER_URL="${ARBITER_URL:-http://127.0.0.1:8090}"
SERVICE="/etc/systemd/system/${SERVICE_NAME}.service"

if [ ! -x "$PYTHON_BIN" ]; then
  echo "Voice virtual environment missing: $PYTHON_BIN" >&2
  echo "Run ./install_voice_runtime.sh first." >&2
  exit 2
fi

if [ "$VOICE_MODE" = "microphone" ]; then
  EXEC_ARGS="--microphone --sense-voice-model $VOICE_MODEL_DIR/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/model.int8.onnx --tokens $VOICE_MODEL_DIR/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-int8-2024-07-17/tokens.txt --silero-vad-model $VOICE_MODEL_DIR/silero_vad.onnx"
else
  EXEC_ARGS="--serial-port $VOICE_SERIAL_PORT --baudrate $VOICE_BAUDRATE --serial-protocol $VOICE_SERIAL_PROTOCOL"
fi

sudo tee "$SERVICE" >/dev/null <<EOF
[Unit]
Description=NearLink offline voice gateway
After=network.target nearlink-rover-arbiter.service
Wants=nearlink-rover-arbiter.service

[Service]
Type=simple
User=${APP_USER}
WorkingDirectory=${APP_DIR}
Environment=HOME=${APP_HOME}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PYTHON_BIN} ${APP_DIR}/voice_service.py --arbiter-url ${ARBITER_URL} ${EXEC_ARGS}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

chmod +x "$APP_DIR/voice_service.py"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"
sudo systemctl status "$SERVICE_NAME" --no-pager
