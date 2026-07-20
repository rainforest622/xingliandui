#!/usr/bin/env bash
set -euo pipefail

# The deployed WS63 and ASRPRO boards are both CH340 devices.  ttyUSB indexes
# can change after reconnecting USB, so services use stable aliases instead.
ASR_VENDOR_ID="${ASR_VENDOR_ID:-1a86}"
ASR_PRODUCT_ID="${ASR_PRODUCT_ID:-7522}"
WS63_VENDOR_ID="${WS63_VENDOR_ID:-1a86}"
WS63_PRODUCT_ID="${WS63_PRODUCT_ID:-7523}"
RULES_FILE="/etc/udev/rules.d/99-nearlink-robot-serial.rules"

sudo tee "$RULES_FILE" >/dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="${ASR_VENDOR_ID}", ATTRS{idProduct}=="${ASR_PRODUCT_ID}", SYMLINK+="nearlink-asr", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="${WS63_VENDOR_ID}", ATTRS{idProduct}=="${WS63_PRODUCT_ID}", SYMLINK+="nearlink-ws63", GROUP="dialout", MODE="0660"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
sleep 1
ls -l /dev/nearlink-asr /dev/nearlink-ws63
