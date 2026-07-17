from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from upper_client.robot_client import SerialAtRobotClient  # noqa: E402


COMMAND_RE = re.compile(r"RobotSleClient: write command ([TFBLRSIADEO]) success")
DEFAULT_HDC = (
    r"D:\devecostudio-windows-6.1.1.290\DevEco Studio\sdk\default"
    r"\openharmony\toolchains\hdc.exe"
)


def run_hdc(hdc: str, target: str, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [hdc, "-t", target, *args],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Relay Harmony NearLink app button logs to the robot serial AT port."
    )
    parser.add_argument("--hdc", default=DEFAULT_HDC)
    parser.add_argument("--target", default="4CGBB24C13200213")
    parser.add_argument("--serial-port", default="COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.5)
    parser.add_argument("--init", action="store_true", help="send I once when the bridge starts")
    parser.add_argument(
        "--dedupe-ms",
        type=int,
        default=120,
        help="ignore repeated identical non-stop commands inside this window",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    hdc_path = Path(args.hdc)
    if not hdc_path.exists():
        print(f"hdc not found: {hdc_path}", file=sys.stderr)
        return 2

    target_result = run_hdc(str(hdc_path), args.target, "shell", "echo", "ok")
    if target_result.returncode != 0:
        print(target_result.stdout.strip(), file=sys.stderr)
        return target_result.returncode

    run_hdc(str(hdc_path), args.target, "shell", "hilog", "-r")
    print(
        f"Bridge ready: phone={args.target} -> serial={args.serial_port}. "
        "Press app buttons; Ctrl+C stops and sends S."
    )

    last_command = ""
    last_sent_at = 0.0
    hilog: subprocess.Popen[str] | None = None

    with SerialAtRobotClient(args.serial_port, args.baudrate, args.timeout) as robot:
        if args.init:
            try:
                result = robot.send("I")
                print(f"[SERIAL] I -> status={result.status.name} ready={int(result.ready)}")
            except Exception as exc:  # noqa: BLE001
                print(f"[SERIAL] I failed: {exc}")

        hilog = subprocess.Popen(
            [str(hdc_path), "-t", args.target, "shell", "hilog"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            errors="replace",
        )

        try:
            assert hilog.stdout is not None
            for line in hilog.stdout:
                match = COMMAND_RE.search(line)
                if not match:
                    continue

                command = match.group(1)
                now = time.monotonic()
                if (
                    command != "S"
                    and command == last_command
                    and (now - last_sent_at) * 1000 < args.dedupe_ms
                ):
                    continue

                last_command = command
                last_sent_at = now
                try:
                    result = robot.send(command)
                    print(
                        f"[RELAY] {command} -> status={result.status.name} "
                        f"moving={int(result.moving)} ready={int(result.ready)} "
                        f"rtt={result.rtt_ms:.1f}ms"
                    )
                except Exception as exc:  # noqa: BLE001
                    print(f"[RELAY] {command} failed: {exc}")
        except KeyboardInterrupt:
            print("Stopping bridge; sending S.")
        finally:
            try:
                robot.send("S")
            except Exception as exc:  # noqa: BLE001
                print(f"[SERIAL] stop failed: {exc}")
            if hilog is not None:
                hilog.terminate()
                try:
                    hilog.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    hilog.kill()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
