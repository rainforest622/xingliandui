#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-nearlink-rover-arbiter}"
BRIDGE_SERVICE_NAME="${BRIDGE_SERVICE_NAME:-nearlink-rover-bridge}"
APP_DIR="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_APP_USER="$(id -un)"
if [ "$(id -u)" = "0" ] && [ -n "${SUDO_USER:-}" ]; then
  DEFAULT_APP_USER="${SUDO_USER}"
fi
APP_USER="${APP_USER:-${DEFAULT_APP_USER}}"
APP_HOME="${APP_HOME:-$(getent passwd "$APP_USER" | cut -d: -f6)}"
PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"
WS63_PORT="${WS63_PORT:-auto}"
ROVER_PORT="${ROVER_PORT:-auto}"
HTTP_HOST="${HTTP_HOST:-0.0.0.0}"
HTTP_PORT="${HTTP_PORT:-8090}"
AUTO_SPEED="${AUTO_SPEED:-0.18}"
TURN_SPEED="${TURN_SPEED:-0.18}"
SQUARE_FORWARD="${SQUARE_FORWARD:-3.0}"
SQUARE_TURN="${SQUARE_TURN:-0.65}"
CAMERA_STREAM_URL="${CAMERA_STREAM_URL:-http://127.0.0.1:8080/stream.mjpg}"
VISION_SPEED="${VISION_SPEED:-0.16}"
VISION_GAIN="${VISION_GAIN:-0.14}"
VISION_MIN_AREA="${VISION_MIN_AREA:-600}"
VISION_COLOR="${VISION_COLOR:-yellow}"
MAP_PATH="${MAP_PATH:-$APP_DIR/patrol_map.json}"
SERVICE="/etc/systemd/system/${SERVICE_NAME}.service"

sudo tee "$SERVICE" >/dev/null <<EOF
[Unit]
Description=NearLink robot safety arbiter
After=network.target

[Service]
Type=simple
User=${APP_USER}
WorkingDirectory=$APP_DIR
Environment=HOME=${APP_HOME}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PYTHON_BIN} $APP_DIR/rover_arbiter.py --ws63-port ${WS63_PORT} --rover-port ${ROVER_PORT} --http-host ${HTTP_HOST} --http-port ${HTTP_PORT} --auto-speed ${AUTO_SPEED} --turn-speed ${TURN_SPEED} --square-forward ${SQUARE_FORWARD} --square-turn ${SQUARE_TURN} --camera-stream-url ${CAMERA_STREAM_URL} --vision-speed ${VISION_SPEED} --vision-gain ${VISION_GAIN} --vision-min-area ${VISION_MIN_AREA} --vision-color ${VISION_COLOR} --map-path ${MAP_PATH}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

chmod +x "$APP_DIR/rover_arbiter.py"
if systemctl list-unit-files "${BRIDGE_SERVICE_NAME}.service" >/dev/null 2>&1; then
  sudo systemctl stop "${BRIDGE_SERVICE_NAME}.service" || true
  sudo systemctl disable "${BRIDGE_SERVICE_NAME}.service" || true
fi
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"
sudo systemctl restart "${SERVICE_NAME}.service"
sudo systemctl status "${SERVICE_NAME}.service" --no-pager
