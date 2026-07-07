from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import socket
import statistics
import sys
import time
from pathlib import Path

from .robot_protocol import (
    ACK_SIZE,
    AckPacket,
    CommandPacket,
    Status,
    decode_ack,
    encode_command,
)
from .robot_profile import AT_COMMANDS, MOTION_COMMAND_KEYS, normalize_robot_key

OBSTACLE_REASON_NAMES = {
    0: "ok",
    1: "not_ready",
    2: "echo_idle_high",
    3: "no_echo_rise",
    4: "no_echo_fall",
    5: "invalid_pulse",
}

ROBOT_AT_RESPONSE_MARKERS = (
    "+ROBOT:ACK,",
    "+ROBOT:MOTOR,",
    "+ROBOT:OLED,",
    "+ROBOT:ENV,",
    "+ROBOT:OBS,",
    "+ROBOT:AVOID,",
    "+ROBOT:STATE,",
)


def has_robot_at_response(response: str) -> bool:
    return any(marker in response for marker in ROBOT_AT_RESPONSE_MARKERS)


def is_plain_at_error(response: str) -> bool:
    lines = [line.strip() for line in response.splitlines() if line.strip()]
    return "ERROR" in lines and not has_robot_at_response(response)


@dataclass(frozen=True)
class Result:
    sequence: int
    command: str
    status: Status
    rtt_ms: float
    moving: bool = False
    raw_response: str = ""
    ready: bool = False
    oled: bool = False
    env: bool = False
    temperature_deci_c: int | None = None
    humidity_deci_percent: int | None = None
    obstacle_enabled: bool = False
    obstacle_valid: bool = False
    obstacle_blocked: bool = False
    distance_mm: int | None = None
    obstacle_threshold_mm: int | None = None
    obstacle_reason: int | None = None
    avoid_active: bool = False
    avoid_phase: int | None = None
    uptime_ms: int | None = None
    last_command: int | None = None
    last_sequence: int | None = None
    age_ms: int | None = None


class RobotClient:
    def __init__(self, host: str, port: int, timeout: float = 1.0):
        self._socket = socket.create_connection((host, port), timeout=timeout)
        self._socket.settimeout(timeout)
        self._sequence = 0

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "RobotClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _receive_exactly(self, size: int) -> bytes:
        data = bytearray()
        while len(data) < size:
            chunk = self._socket.recv(size - len(data))
            if not chunk:
                raise ConnectionError("robot closed the connection")
            data.extend(chunk)
        return bytes(data)

    def send(self, key: str, speed: int = 45) -> Result:
        normalized = key.strip().upper()
        if normalized not in MOTION_COMMAND_KEYS:
            raise ValueError(f"unknown command key: {key}")
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        packet = CommandPacket(MOTION_COMMAND_KEYS[normalized], speed, speed, sequence)
        started = time.perf_counter_ns()
        self._socket.sendall(encode_command(packet))
        ack: AckPacket = decode_ack(self._receive_exactly(ACK_SIZE))
        elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000
        if ack.sequence != sequence:
            raise RuntimeError(
                f"ACK sequence mismatch: expected {sequence}, got {ack.sequence}"
            )
        return Result(sequence, normalized, ack.status, elapsed_ms)


def parse_at_ack(response: str) -> tuple[int, Status, bool]:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:ACK,"):
            continue
        fields = line.split(",")
        if len(fields) != 5:
            raise ValueError(f"bad robot AT ACK line: {line!r}")
        try:
            sequence = int(fields[1])
            status = Status(int(fields[3]))
            moving = int(fields[4]) != 0
        except ValueError as exc:
            raise ValueError(f"bad robot AT ACK values: {line!r}") from exc
        return sequence, status, moving
    raise TimeoutError(f"robot AT ACK not found in response: {response!r}")


def parse_at_motor(response: str) -> bool:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:MOTOR,"):
            continue
        fields = line.split(",")
        if len(fields) != 2:
            raise ValueError(f"bad robot motor line: {line!r}")
        return int(fields[1]) != 0
    raise TimeoutError(f"robot motor result not found in response: {response!r}")


def parse_at_oled(response: str) -> bool:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:OLED,"):
            continue
        fields = line.split(",")
        if len(fields) != 2:
            raise ValueError(f"bad robot OLED line: {line!r}")
        return int(fields[1]) != 0
    raise TimeoutError(f"robot OLED result not found in response: {response!r}")


def parse_at_env(response: str) -> tuple[bool, int, int]:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:ENV,"):
            continue
        fields = line.split(",")
        if len(fields) != 4:
            raise ValueError(f"bad robot env line: {line!r}")
        try:
            ok = int(fields[1]) != 0
            temperature_deci_c = int(fields[2])
            humidity_deci_percent = int(fields[3])
        except ValueError as exc:
            raise ValueError(f"bad robot env values: {line!r}") from exc
        return ok, temperature_deci_c, humidity_deci_percent
    raise TimeoutError(f"robot env result not found in response: {response!r}")


def parse_at_obstacle(response: str) -> tuple[bool, bool, bool, int, int, int | None]:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:OBS,"):
            continue
        fields = line.split(",")
        if len(fields) not in (6, 7):
            raise ValueError(f"bad robot obstacle line: {line!r}")
        try:
            enabled = int(fields[1]) != 0
            valid = int(fields[2]) != 0
            blocked = int(fields[3]) != 0
            distance_mm = int(fields[4])
            threshold_mm = int(fields[5])
            reason = int(fields[6]) if len(fields) == 7 else None
        except ValueError as exc:
            raise ValueError(f"bad robot obstacle values: {line!r}") from exc
        return enabled, valid, blocked, distance_mm, threshold_mm, reason
    raise TimeoutError(f"robot obstacle result not found in response: {response!r}")


def parse_at_avoid(response: str) -> tuple[bool, int, Status, bool, bool, bool, int, int, int | None]:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:AVOID,"):
            continue
        fields = line.split(",")
        if len(fields) not in (9, 10):
            raise ValueError(f"bad robot avoid line: {line!r}")
        try:
            active = int(fields[1]) != 0
            phase = int(fields[2])
            status = Status(int(fields[3]))
            enabled = int(fields[4]) != 0
            valid = int(fields[5]) != 0
            blocked = int(fields[6]) != 0
            distance_mm = int(fields[7])
            threshold_mm = int(fields[8])
            reason = int(fields[9]) if len(fields) == 10 else None
        except ValueError as exc:
            raise ValueError(f"bad robot avoid values: {line!r}") from exc
        return active, phase, status, enabled, valid, blocked, distance_mm, threshold_mm, reason
    raise TimeoutError(f"robot avoid result not found in response: {response!r}")


def parse_at_state(response: str) -> tuple[int, bool, bool, int, int, int | None]:
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line.startswith("+ROBOT:STATE,"):
            continue
        fields = line.split(",")
        if len(fields) != 7:
            raise ValueError(f"bad robot state line: {line!r}")
        try:
            uptime_ms = int(fields[1])
            ready = int(fields[2]) != 0
            moving = int(fields[3]) != 0
            last_command = int(fields[4])
            last_sequence = int(fields[5])
            age_ms = int(fields[6])
        except ValueError as exc:
            raise ValueError(f"bad robot state values: {line!r}") from exc
        if age_ms == 0xFFFFFFFF:
            age_ms = None
        return uptime_ms, ready, moving, last_command, last_sequence, age_ms
    raise TimeoutError(f"robot state not found in response: {response!r}")


class SerialAtRobotClient:
    def __init__(self, port: str = "COM5", baudrate: int = 115200, timeout: float = 1.0):
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "serial-at transport requires pyserial. Install it with: "
                "python -m pip install pyserial"
            ) from exc

        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baudrate
        self._serial.bytesize = 8
        self._serial.parity = "N"
        self._serial.stopbits = 1
        self._serial.timeout = timeout
        self._serial.write_timeout = timeout
        self._serial.rtscts = False
        self._serial.dsrdtr = False
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()
        self._serial.dtr = False
        self._serial.rts = False
        self._timeout = timeout

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "SerialAtRobotClient":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def send(self, key: str, speed: int = 45) -> Result:
        del speed  # The current board-side AT MVP uses fixed safe speed.
        normalized = normalize_robot_key(key)

        command = f"{AT_COMMANDS[normalized]}\r\n".encode("ascii")
        started = time.perf_counter_ns()
        self._serial.reset_input_buffer()
        self._serial.write(command)
        self._serial.flush()

        response = ""
        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if raw:
                response += raw.decode("ascii", errors="replace")
                if has_robot_at_response(response) and ("\nOK" in response or response.endswith("OK\r\n")):
                    break
                if is_plain_at_error(response) and time.monotonic() < deadline:
                    # Right after reset/burn the base AT task can answer before
                    # robot_mvp registers its custom command table. Retry within
                    # the caller's timeout instead of surfacing a Python traceback.
                    time.sleep(0.12)
                    self._serial.reset_input_buffer()
                    self._serial.write(command)
                    self._serial.flush()
                    response = ""
            else:
                time.sleep(0.01)

        ready = False
        oled = False
        env = False
        temperature_deci_c = None
        humidity_deci_percent = None
        obstacle_enabled = False
        obstacle_valid = False
        obstacle_blocked = False
        distance_mm = None
        obstacle_threshold_mm = None
        obstacle_reason = None
        avoid_active = False
        avoid_phase = None
        uptime_ms = None
        last_command = None
        last_sequence = None
        age_ms = None

        if normalized == "I":
            ready = parse_at_motor(response)
            sequence = 0
            status = Status.OK if ready else Status.MOTOR_ERROR
            moving = False
        elif normalized == "O":
            oled = parse_at_oled(response)
            sequence = 0
            status = Status.OK if oled else Status.MOTOR_ERROR
            moving = False
        elif normalized == "E":
            env, temperature_deci_c, humidity_deci_percent = parse_at_env(response)
            sequence = 0
            status = Status.OK if env else Status.MOTOR_ERROR
            moving = False
        elif normalized == "D":
            (
                obstacle_enabled,
                obstacle_valid,
                obstacle_blocked,
                distance_mm,
                obstacle_threshold_mm,
                obstacle_reason,
            ) = parse_at_obstacle(response)
            sequence = 0
            status = Status.OBSTACLE_STOP if obstacle_blocked else Status.OK
            moving = False
        elif normalized == "A":
            (
                avoid_active,
                avoid_phase,
                status,
                obstacle_enabled,
                obstacle_valid,
                obstacle_blocked,
                distance_mm,
                obstacle_threshold_mm,
                obstacle_reason,
            ) = parse_at_avoid(response)
            sequence = 0
            moving = avoid_active
        elif normalized == "T":
            uptime_ms, ready, moving, last_command, last_sequence, age_ms = parse_at_state(response)
            sequence = last_sequence
            status = Status.OK
        else:
            sequence, status, moving = parse_at_ack(response)
        elapsed_ms = (time.perf_counter_ns() - started) / 1_000_000
        return Result(
            sequence=sequence,
            command=normalized,
            status=status,
            rtt_ms=elapsed_ms,
            moving=moving,
            raw_response=response,
            ready=ready,
            oled=oled,
            env=env,
            temperature_deci_c=temperature_deci_c,
            humidity_deci_percent=humidity_deci_percent,
            obstacle_enabled=obstacle_enabled,
            obstacle_valid=obstacle_valid,
            obstacle_blocked=obstacle_blocked,
            distance_mm=distance_mm,
            obstacle_threshold_mm=obstacle_threshold_mm,
            obstacle_reason=obstacle_reason,
            avoid_active=avoid_active,
            avoid_phase=avoid_phase,
            uptime_ms=uptime_ms,
            last_command=last_command,
            last_sequence=last_sequence,
            age_ms=age_ms,
        )


def write_csv(path: Path, results: list[Result]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "sequence",
            "command",
            "status",
            "rtt_ms",
            "moving",
            "ready",
            "oled",
            "env",
            "temperature_c",
            "humidity_percent",
            "obstacle_enabled",
            "obstacle_valid",
            "obstacle_blocked",
            "distance_mm",
            "obstacle_threshold_mm",
            "obstacle_reason",
            "obstacle_reason_name",
            "avoid_active",
            "avoid_phase",
            "uptime_ms",
            "last_command",
            "last_sequence",
            "age_ms",
        ))
        for result in results:
            writer.writerow(
                (
                    result.sequence,
                    result.command,
                    result.status.name,
                    f"{result.rtt_ms:.3f}",
                    int(result.moving),
                    int(result.ready),
                    int(result.oled),
                    int(result.env),
                    "" if result.temperature_deci_c is None else f"{result.temperature_deci_c / 10:.1f}",
                    "" if result.humidity_deci_percent is None else f"{result.humidity_deci_percent / 10:.1f}",
                    int(result.obstacle_enabled),
                    int(result.obstacle_valid),
                    int(result.obstacle_blocked),
                    "" if result.distance_mm is None else result.distance_mm,
                    "" if result.obstacle_threshold_mm is None else result.obstacle_threshold_mm,
                    "" if result.obstacle_reason is None else result.obstacle_reason,
                    "" if result.obstacle_reason is None else OBSTACLE_REASON_NAMES.get(
                        result.obstacle_reason, f"unknown_{result.obstacle_reason}"
                    ),
                    int(result.avoid_active),
                    "" if result.avoid_phase is None else result.avoid_phase,
                    "" if result.uptime_ms is None else result.uptime_ms,
                    "" if result.last_command is None else result.last_command,
                    "" if result.last_sequence is None else result.last_sequence,
                    "" if result.age_ms is None else result.age_ms,
                )
            )


def summarize(results: list[Result]) -> dict[str, float]:
    values = sorted(result.rtt_ms for result in results)
    if not values:
        return {}

    def percentile(percent: float) -> float:
        index = max(0, math.ceil(len(values) * percent) - 1)
        return values[index]

    return {
        "mean_ms": statistics.fmean(values),
        "p95_ms": percentile(0.95),
        "p99_ms": percentile(0.99),
        "max_ms": values[-1],
        "jitter_ms": statistics.pstdev(values),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="WS63 robot MVP control client")
    parser.add_argument(
        "--transport",
        choices=("tcp", "serial-at"),
        default="tcp",
        help="tcp uses the simulator bridge; serial-at talks to the WS63 board over COM",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--serial-port", default="COM5")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--speed", type=int, default=45)
    parser.add_argument(
        "--commands",
        default="F,S",
        help=(
            "comma-separated commands. tcp: F/B/L/R/S; serial-at also supports "
            "I=motor init, O=OLED init, E=environment, D=obstacle distance, "
            "A=auto obstacle avoidance, T=status"
        ),
    )
    parser.add_argument("--interval", type=float, default=0.15)
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="repeat the command sequence N times",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    keys = [item.strip().upper() for item in args.commands.split(",") if item.strip()]
    if args.repeat < 1:
        parser.error("--repeat must be at least 1")
    keys *= args.repeat
    results: list[Result] = []
    if args.transport == "serial-at":
        client_factory = lambda: SerialAtRobotClient(args.serial_port, args.baudrate, args.timeout)
    else:
        client_factory = lambda: RobotClient(args.host, args.port, args.timeout)

    try:
        with client_factory() as client:
            for key in keys:
                result = client.send(key, args.speed)
                results.append(result)
                print(
                    f"seq={result.sequence:03d} cmd={result.command} "
                    f"status={result.status.name} moving={int(result.moving)} "
                    f"oled={int(result.oled)} env={int(result.env)} "
                    f"obs_valid={int(result.obstacle_valid)} block={int(result.obstacle_blocked)} "
                    f"rtt={result.rtt_ms:.3f} ms"
                    + (
                        ""
                        if result.temperature_deci_c is None
                        else (
                            f" temp={result.temperature_deci_c / 10:.1f}C"
                            f" hum={result.humidity_deci_percent / 10:.1f}%"
                        )
                    )
                    + (
                        ""
                        if result.distance_mm is None
                        else (
                            f" dist={result.distance_mm}mm"
                            f" threshold={result.obstacle_threshold_mm}mm"
                        )
                    )
                    + (
                        ""
                        if result.obstacle_reason is None
                        else (
                            f" reason={result.obstacle_reason}"
                            f"/{OBSTACLE_REASON_NAMES.get(result.obstacle_reason, 'unknown')}"
                        )
                    )
                    + (
                        ""
                        if result.avoid_phase is None
                        else (
                            f" avoid={int(result.avoid_active)}"
                            f" phase={result.avoid_phase}"
                        )
                    )
                    + (
                        ""
                        if result.uptime_ms is None
                        else (
                            f" ready={int(result.ready)} uptime={result.uptime_ms}ms "
                            f"last_cmd={result.last_command} "
                            f"age={'none' if result.age_ms is None else str(result.age_ms) + 'ms'}"
                        )
                    )
                )
                time.sleep(args.interval)
    except (TimeoutError, ValueError, ConnectionError, OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        if args.transport == "serial-at":
            print(
                "诊断: 串口已打开，但没有拿到 robot_mvp 自定义 AT 响应；"
                "请确认烧录的是 ws63-liteos-app_mvp_real_hcsr04_load_only.fwpkg，"
                "reset 后等待 3-5 秒再测，且没有 HiSpark Studio 串口监视器占用 COM 口。",
                file=sys.stderr,
            )
        raise SystemExit(2) from None
    summary = summarize(results)
    print(
        "summary "
        + " ".join(f"{name}={value:.3f}" for name, value in summary.items())
    )
    if args.output:
        write_csv(args.output, results)
        print(f"wrote {len(results)} rows to {args.output}")


if __name__ == "__main__":
    main()
