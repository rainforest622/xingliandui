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
PERSON_DETECT="${PERSON_DETECT:-0}"
NO_DETECT="${NO_DETECT:-0}"
FIRE_SMOKE_DETECT="${FIRE_SMOKE_DETECT:-0}"
FIRE_SMOKE_INTERVAL_MS="${FIRE_SMOKE_INTERVAL_MS:-1000}"
VOICE_ARBITER_URL="${VOICE_ARBITER_URL:-http://127.0.0.1:8090}"
VOICE_ALERT_COOLDOWN="${VOICE_ALERT_COOLDOWN:-8.0}"
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
  --fire-smoke-interval-ms "$FIRE_SMOKE_INTERVAL_MS"
  --voice-arbiter-url "$VOICE_ARBITER_URL"
  --voice-alert-cooldown "$VOICE_ALERT_COOLDOWN"
)

if [ "$PERSON_DETECT" = "1" ]; then
  args+=(--person-detect)
fi
if [ "$NO_DETECT" = "1" ]; then
  args+=(--no-detect)
fi
if [ "$FIRE_SMOKE_DETECT" = "1" ]; then
  args+=(--fire-smoke-detect)
fi

exec "${args[@]}"
