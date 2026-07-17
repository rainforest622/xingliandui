#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-nearlink-rover-bridge}"
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
SERVICE="/etc/systemd/system/${SERVICE_NAME}.service"

sudo tee "$SERVICE" >/dev/null <<EOF
[Unit]
Description=WS63 to WAVE ROVER serial bridge
After=network.target

[Service]
Type=simple
User=${APP_USER}
WorkingDirectory=$APP_DIR
Environment=HOME=${APP_HOME}
Environment=PYTHONUNBUFFERED=1
ExecStart=${PYTHON_BIN} $APP_DIR/rover_bridge.py --ws63-port ${WS63_PORT} --rover-port ${ROVER_PORT}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

chmod +x "$APP_DIR/start_rover_bridge.sh"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"
sudo systemctl restart "${SERVICE_NAME}.service"
sudo systemctl status "${SERVICE_NAME}.service" --no-pager
