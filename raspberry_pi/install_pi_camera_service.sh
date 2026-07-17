#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="${SERVICE_NAME:-nearlink-pi-camera}"
APP_DIR="$(cd "$(dirname "$0")" && pwd)"
DEFAULT_APP_USER="$(id -un)"
if [ "$(id -u)" = "0" ] && [ -n "${SUDO_USER:-}" ]; then
  DEFAULT_APP_USER="${SUDO_USER}"
fi
APP_USER="${APP_USER:-${DEFAULT_APP_USER}}"
APP_HOME="${APP_HOME:-$(getent passwd "$APP_USER" | cut -d: -f6)}"
PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"
PYTHON_USER_SITE="$(HOME="$APP_HOME" "$PYTHON_BIN" -c 'import site; print(site.getusersitepackages())')"
PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
PYTHONPATH_VALUE="${PYTHONPATH_VALUE:-${PYTHON_USER_SITE}:/usr/local/lib/python3.13/dist-packages:/usr/lib/python3/dist-packages}"

sudo tee "/etc/systemd/system/${SERVICE_NAME}.service" >/dev/null <<SERVICE
[Unit]
Description=NearLink robot Raspberry Pi camera service
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=${APP_USER}
WorkingDirectory=${APP_DIR}
Environment=PYTHONUNBUFFERED=${PYTHONUNBUFFERED}
Environment=HOME=${APP_HOME}
Environment=PYTHON=${PYTHON_BIN}
Environment=PYTHONPATH=${PYTHONPATH_VALUE}
Environment=HOST=0.0.0.0
Environment=PORT=8080
Environment=WIDTH=640
Environment=HEIGHT=480
Environment=FPS=15
Environment=BACKEND=auto
Environment=CAMERA=0
Environment=DETECT_COOLDOWN=1.0
ExecStart=${APP_DIR}/start_pi_camera.sh
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
SERVICE

chmod +x "${APP_DIR}/start_pi_camera.sh"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"
sudo systemctl restart "${SERVICE_NAME}.service"
sudo systemctl --no-pager --full status "${SERVICE_NAME}.service"
