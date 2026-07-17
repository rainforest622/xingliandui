from __future__ import annotations

import argparse
import glob
import json
import subprocess
import time
from pathlib import Path
from typing import Iterable

try:
    import serial
except ImportError as exc:  # pragma: no cover - only hit on a Pi without pyserial
    raise SystemExit("pyserial is required: sudo apt install -y python3-serial") from exc


STOP_COMMAND = {"T": 1, "L": 0, "R": 0}
PORT_PATTERNS = (
    "/dev/serial0",
    "/dev/ttyAMA*",
    "/dev/ttyS*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
    "/dev/serial/by-id/*",
)
DEFAULT_ROVER_CANDIDATES = (
    "/dev/ttyAMA0",
    "/dev/serial0",
    "/dev/ttyS0",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safely check the Raspberry Pi UART path to the WAVE ROVER."
    )
    parser.add_argument(
        "--rover-port",
        default="auto",
        help="WAVE ROVER UART device. Use 'auto' to prefer the Pi 5 GPIO14/15 UART0 device.",
    )
    parser.add_argument(
        "--ws63-port",
        default="auto",
        help="WS63 USB serial device, or 'auto' to scan /dev/ttyUSB* and /dev/ttyACM*.",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.2)
    parser.add_argument("--listen-seconds", type=float, default=2.0)
    parser.add_argument(
        "--ws63-listen",
        action="store_true",
        help="Listen to the WS63 USB serial port and print JSON lines without forwarding them.",
    )
    parser.add_argument(
        "--move-test",
        action="store_true",
        help="Run a low-speed forward pulse after the stop-only check.",
    )
    parser.add_argument("--speed", type=float, default=0.10, help="Move-test speed, max 0.25.")
    parser.add_argument("--duration", type=float, default=0.25, help="Move-test duration in seconds.")
    return parser.parse_args()


def encode_command(payload: dict[str, object]) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("ascii") + b"\n"


def existing_ports() -> list[str]:
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(dict.fromkeys(ports))


def print_ports() -> None:
    print("[PORTS]")
    ports = existing_ports()
    if not ports:
        print("  no matching /dev serial devices found")
        return
    for port in ports:
        target = ""
        path = Path(port)
        if path.is_symlink():
            target = f" -> {path.resolve()}"
        print(f"  {port}{target}")


def resolve_ws63_port(requested: str, rover_port: str) -> str:
    if requested != "auto":
        return requested

    rover_real = str(Path(rover_port).resolve()) if Path(rover_port).exists() else rover_port
    candidates: list[str] = []
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        candidates.extend(glob.glob(pattern))

    for candidate in sorted(dict.fromkeys(candidates)):
        real = str(Path(candidate).resolve()) if Path(candidate).exists() else candidate
        if candidate != rover_port and real != rover_real:
            return candidate

    raise SystemExit("WS63 USB serial not found. Plug the WS63 Type-C data cable into the Raspberry Pi.")


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


def open_port(port: str, baudrate: int, timeout: float) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, dsrdtr=False)


def read_available(ser: serial.Serial, seconds: float) -> list[bytes]:
    deadline = time.monotonic() + seconds
    lines: list[bytes] = []
    while time.monotonic() < deadline:
        line = ser.readline()
        if line:
            lines.append(line.rstrip())
    return lines


def print_lines(prefix: str, lines: Iterable[bytes]) -> None:
    count = 0
    for count, line in enumerate(lines, start=1):
        print(f"{prefix} {line.decode('utf-8', 'replace')}")
    if count == 0:
        print(f"{prefix} no feedback")


def rover_stop_check(args: argparse.Namespace) -> None:
    print(f"[ROVER] opening {args.rover_port} @ {args.baudrate}")
    with open_port(args.rover_port, args.baudrate, args.timeout) as rover:
        rover.write(encode_command(STOP_COMMAND))
        rover.flush()
        print("[ROVER] sent stop")
        print_lines("[ROVER]", read_available(rover, args.listen_seconds))

        if not args.move_test:
            return

        if not (0 < args.speed <= 0.25):
            raise SystemExit("--speed must be > 0 and <= 0.25 for this safety check")
        if not (0 < args.duration <= 1.0):
            raise SystemExit("--duration must be > 0 and <= 1.0 for this safety check")

        move = {"T": 1, "L": round(args.speed, 3), "R": round(args.speed, 3)}
        print(f"[ROVER] move-test {move} for {args.duration:.2f}s")
        rover.write(encode_command(move))
        rover.flush()
        time.sleep(args.duration)
        rover.write(encode_command(STOP_COMMAND))
        rover.flush()
        print("[ROVER] sent stop after move-test")
        print_lines("[ROVER]", read_available(rover, args.listen_seconds))


def ws63_listen_check(args: argparse.Namespace) -> None:
    ws63_port = resolve_ws63_port(args.ws63_port, args.rover_port)
    print(f"[WS63] opening {ws63_port} @ {args.baudrate}")
    with open_port(ws63_port, args.baudrate, args.timeout) as ws63:
        lines = read_available(ws63, args.listen_seconds)
    for line in lines:
        stripped = line.strip()
        if stripped.startswith(b"{") and stripped.endswith(b"}"):
            try:
                payload = json.loads(stripped.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                print(f"[WS63] invalid-json {stripped!r}")
            else:
                print(f"[WS63] json {payload}")
        elif stripped:
            print(f"[WS63] non-json {stripped.decode('utf-8', 'replace')}")
    if not lines:
        print("[WS63] no data during listen window")


def main() -> int:
    args = parse_args()
    args.rover_port = resolve_rover_port(args.rover_port)
    print_ports()
    rover_stop_check(args)
    if args.ws63_listen:
        ws63_listen_check(args)
    print("[CHECK] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
