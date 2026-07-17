#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-15}"
BACKEND="${BACKEND:-auto}"
CAMERA="${CAMERA:-0}"
DETECT_COOLDOWN="${DETECT_COOLDOWN:-1.0}"
NO_DETECT="${NO_DETECT:-0}"
PYTHON="${PYTHON:-python3}"

args=(
  "$PYTHON" run.py
  --host "$HOST"
  --port "$PORT"
  --camera "$CAMERA"
  --width "$WIDTH"
  --height "$HEIGHT"
  --fps "$FPS"
  --camera-backend "$BACKEND"
  --detect-cooldown "$DETECT_COOLDOWN"
)

if [ "$NO_DETECT" = "1" ]; then
  args+=(--no-detect)
fi

exec "${args[@]}"
