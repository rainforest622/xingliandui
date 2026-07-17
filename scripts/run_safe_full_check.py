from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from upper_client.robot_client import OBSTACLE_REASON_NAMES, SerialAtRobotClient  # noqa: E402


def reason_text(result) -> str:
    if result.obstacle_reason is None:
        return ""
    name = OBSTACLE_REASON_NAMES.get(result.obstacle_reason, f"unknown_{result.obstacle_reason}")
    return f" reason={result.obstacle_reason}/{name}"


def env_text(result) -> str:
    if result.temperature_deci_c is None:
        return ""
    return f" temp={result.temperature_deci_c / 10:.1f}C hum={result.humidity_deci_percent / 10:.1f}%"


def dist_text(result) -> str:
    if result.distance_mm is None:
        return ""
    return f" dist={result.distance_mm}mm threshold={result.obstacle_threshold_mm}mm"


def state_text(result) -> str:
    if result.uptime_ms is None:
        return ""
    return (
        f" ready={int(result.ready)} moving={int(result.moving)} "
        f"uptime={result.uptime_ms}ms last_cmd={result.last_command}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run non-dangerous full MVP checks: status, OLED, env, ultrasonic, stop."
    )
    parser.add_argument("--serial-port", default="COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument(
        "--avoid-non-motion",
        action="store_true",
        help="send A only when motor is not initialized, verifying the non-motion error path",
    )
    args = parser.parse_args()

    print(f"SAFE_FULL_CHECK port={args.serial_port}")
    results = []
    with SerialAtRobotClient(args.serial_port, args.baudrate, args.timeout) as client:
        for key in ["T", "O", "E", "D", "D", "D"]:
            result = client.send(key)
            results.append(result)
            print(
                f"cmd={key} status={result.status.name} moving={int(result.moving)} "
                f"oled={int(result.oled)} env={int(result.env)} "
                f"obs_valid={int(result.obstacle_valid)} block={int(result.obstacle_blocked)} "
                f"rtt={result.rtt_ms:.3f}ms"
                f"{env_text(result)}{dist_text(result)}{reason_text(result)}{state_text(result)}"
            )

        last_state = next((result for result in results if result.command == "T"), None)
        if args.avoid_non_motion and last_state is not None and not last_state.ready:
            print("SAFE_AVOID_CHECK: motor_not_ready, sending A once to verify non-motion command path")
            result = client.send("A")
            print(
                f"cmd=A status={result.status.name} moving={int(result.moving)} "
                f"obs_valid={int(result.obstacle_valid)} block={int(result.obstacle_blocked)} "
                f"avoid={int(result.avoid_active)} phase={result.avoid_phase} "
                f"rtt={result.rtt_ms:.3f}ms{dist_text(result)}{reason_text(result)}"
            )
        else:
            print("SAFE_AVOID_CHECK: skipped motion-capable A path")

        result = client.send("S")
        print(f"cmd=S status={result.status.name} moving={int(result.moving)} rtt={result.rtt_ms:.3f}ms")

    print("SUMMARY: safe full check finished")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
