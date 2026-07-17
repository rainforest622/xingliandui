from __future__ import annotations

import argparse
import glob
import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError as exc:  # pragma: no cover - only hit on a Pi without pyserial
    raise SystemExit("pyserial is required: sudo apt install -y python3-serial") from exc


STOP_COMMAND = b'{"T":1,"L":0,"R":0}\n'
DEFAULT_WS63_CANDIDATES = (
    "/dev/serial/by-id/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
)
DEFAULT_ROVER_CANDIDATES = (
    "/dev/ttyAMA0",
    "/dev/serial0",
    "/dev/ttyS0",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward WS63 USB-serial JSON commands to the WAVE ROVER 40PIN UART."
    )
    parser.add_argument(
        "--ws63-port",
        default="auto",
        help="WS63 USB serial device, or 'auto' to scan /dev/ttyUSB* and /dev/ttyACM*.",
    )
    parser.add_argument(
        "--rover-port",
        default="auto",
        help="WAVE ROVER UART device. Use 'auto' to prefer the Pi 5 GPIO14/15 UART0 device.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--read-timeout", type=float, default=0.02)
    parser.add_argument(
        "--stop-timeout",
        type=float,
        default=0.30,
        help="Send a stop command when no motion JSON arrives for this many seconds.",
    )
    parser.add_argument("--no-start-stop", action="store_true", help="Do not send stop when the bridge starts.")
    parser.add_argument("--no-exit-stop", action="store_true", help="Do not send stop when the bridge exits.")
    parser.add_argument("--quiet-rover", action="store_true", help="Do not print WAVE ROVER UART feedback.")
    parser.add_argument(
        "--allow-non-motion",
        action="store_true",
        help="Forward JSON without a T field. By default those lines are ignored.",
    )
    return parser.parse_args()


def resolve_ws63_port(requested: str, rover_port: str) -> str:
    if requested != "auto":
        return requested

    rover_real = str(Path(rover_port).resolve()) if Path(rover_port).exists() else rover_port
    candidates: list[str] = []
    for pattern in DEFAULT_WS63_CANDIDATES:
        candidates.extend(glob.glob(pattern))

    unique_candidates = []
    seen = set()
    for candidate in candidates:
        real = str(Path(candidate).resolve()) if Path(candidate).exists() else candidate
        if real == rover_real or candidate == rover_port or real in seen:
            continue
        seen.add(real)
        unique_candidates.append(candidate)

    if not unique_candidates:
        raise SystemExit(
            "WS63 USB serial not found. Connect the WS63 Type-C data cable, then check: ls /dev/ttyUSB* /dev/ttyACM*"
        )
    return unique_candidates[0]


def gpio_uart0_on_header() -> bool:
    try:
        pin14 = subprocess.run(
            ["pinctrl", "get", "14"],
            check=False,
            capture_output=True,
            text=True,
            timeout=0.5,
        ).stdout
        pin15 = subprocess.run(
            ["pinctrl", "get", "15"],
            check=False,
            capture_output=True,
            text=True,
            timeout=0.5,
        ).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    return "TXD0" in pin14 and "RXD0" in pin15


def resolve_rover_port(requested: str) -> str:
    if requested != "auto":
        return requested

    # Raspberry Pi 5 exposes the 40PIN GPIO14/GPIO15 UART0 pins as /dev/ttyAMA0
    # after dtparam=uart0=on, while /dev/serial0 may still point at ttyAMA10.
    if Path("/dev/ttyAMA0").exists() and gpio_uart0_on_header():
        return "/dev/ttyAMA0"

    for candidate in DEFAULT_ROVER_CANDIDATES:
        if Path(candidate).exists():
            return candidate

    raise SystemExit("WAVE ROVER UART not found. Enable the Raspberry Pi 40PIN UART first.")


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, dsrdtr=False)


def is_motion_command(payload: dict[str, Any]) -> bool:
    return payload.get("T") == 1 and ("L" in payload or "R" in payload)


def is_active_motion_command(payload: dict[str, Any]) -> bool:
    if not is_motion_command(payload):
        return False
    try:
        return float(payload.get("L", 0)) != 0.0 or float(payload.get("R", 0)) != 0.0
    except (TypeError, ValueError):
        return True


def normalize_json_line(line: bytes, allow_non_motion: bool) -> tuple[bytes | None, bool, bool]:
    stripped = line.strip()
    if not stripped.startswith(b"{") or not stripped.endswith(b"}"):
        return None, False, False

    try:
        payload = json.loads(stripped.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None, False, False

    if not isinstance(payload, dict):
        return None, False, False

    motion = is_motion_command(payload)
    active_motion = is_active_motion_command(payload)
    if "T" not in payload and not allow_non_motion:
        return None, False, False

    encoded = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"
    return encoded, motion, active_motion


def rover_reader(rover: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = rover.readline()
        except serial.SerialException as exc:
            print(f"[ROVER] read failed: {exc}", file=sys.stderr)
            stop_event.set()
            return
        if line:
            print("[ROVER]", line.decode("utf-8", "replace").rstrip())


def safe_stop(rover: serial.Serial) -> None:
    try:
        rover.write(STOP_COMMAND)
        rover.flush()
    except serial.SerialException as exc:
        print(f"[BRIDGE] stop failed: {exc}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()

    def handle_signal(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rover_port = resolve_rover_port(args.rover_port)
    ws63_port = resolve_ws63_port(args.ws63_port, rover_port)
    ws63 = open_serial(ws63_port, args.baudrate, args.read_timeout)
    rover = open_serial(rover_port, args.baudrate, args.read_timeout)

    if not args.no_start_stop:
        safe_stop(rover)

    reader_thread: threading.Thread | None = None
    if not args.quiet_rover:
        reader_thread = threading.Thread(target=rover_reader, args=(rover, stop_event), daemon=True)
        reader_thread.start()

    print(
        f"[BRIDGE] WS63 {ws63_port} -> WAVE ROVER {rover_port} @ {args.baudrate}. "
        f"stop_timeout={args.stop_timeout:.2f}s"
    )

    last_motion_at = time.monotonic()
    motion_active = False
    forwarded = 0
    ignored = 0

    try:
        while not stop_event.is_set():
            try:
                line = ws63.readline()
            except serial.SerialException as exc:
                print(f"[WS63] read failed: {exc}", file=sys.stderr)
                return 2

            if line:
                command, motion, active_motion = normalize_json_line(line, args.allow_non_motion)
                if command is None:
                    ignored += 1
                else:
                    try:
                        rover.write(command)
                        rover.flush()
                    except serial.SerialException as exc:
                        print(f"[ROVER] write failed: {exc}", file=sys.stderr)
                        return 3
                    forwarded += 1
                    print("[BRIDGE]", command.decode("utf-8", "replace").rstrip())
                    if motion:
                        last_motion_at = time.monotonic()
                        motion_active = active_motion

            if motion_active and (time.monotonic() - last_motion_at) >= args.stop_timeout:
                safe_stop(rover)
                print("[BRIDGE] watchdog stop")
                motion_active = False
                last_motion_at = time.monotonic()

    finally:
        if not args.no_exit_stop:
            safe_stop(rover)
        stop_event.set()
        if reader_thread is not None:
            reader_thread.join(timeout=0.5)
        ws63.close()
        rover.close()
        print(f"[BRIDGE] closed forwarded={forwarded} ignored={ignored}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
