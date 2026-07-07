from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from upper_client.robot_client import OBSTACLE_REASON_NAMES, SerialAtRobotClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Continuously read WS63 robot ultrasonic distance over AT. "
            "This script only sends AT+ROBOTOBS; it never starts the motors."
        )
    )
    parser.add_argument("--serial-port", default="COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument(
        "--require-valid",
        action="store_true",
        help="return exit code 1 if no valid distance sample is observed",
    )
    args = parser.parse_args()

    valid_distances: list[int] = []
    reasons: Counter[str] = Counter()

    print(
        f"reading ultrasonic distance on {args.serial_port}, "
        f"samples={args.samples}, interval={args.interval}s"
    )
    print("idx valid blocked distance_mm threshold_mm reason")

    with SerialAtRobotClient(args.serial_port, args.baudrate, args.timeout) as client:
        for index in range(1, args.samples + 1):
            result = client.send("D")
            reason_name = (
                "none"
                if result.obstacle_reason is None
                else OBSTACLE_REASON_NAMES.get(result.obstacle_reason, f"unknown_{result.obstacle_reason}")
            )
            reasons[reason_name] += 1
            if result.obstacle_valid and result.distance_mm is not None:
                valid_distances.append(result.distance_mm)
            print(
                f"{index:03d} "
                f"{int(result.obstacle_valid)} "
                f"{int(result.obstacle_blocked)} "
                f"{'' if result.distance_mm is None else result.distance_mm} "
                f"{'' if result.obstacle_threshold_mm is None else result.obstacle_threshold_mm} "
                f"{reason_name}"
            )
            time.sleep(args.interval)

    print("summary")
    print("  reasons=" + ", ".join(f"{name}:{count}" for name, count in sorted(reasons.items())))
    if valid_distances:
        print(
            "  valid_distance_mm="
            f"min:{min(valid_distances)} "
            f"max:{max(valid_distances)} "
            f"last:{valid_distances[-1]}"
        )
        return 0

    print("  valid_distance_mm=none")
    if args.require_valid:
        print(
            "  diagnosis=no valid ECHO pulse; check module power, VCC/TRIG/ECHO/GND order, "
            "3.3V compatibility, and connector contact",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
