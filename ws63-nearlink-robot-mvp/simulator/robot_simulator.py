from __future__ import annotations

import argparse
from dataclasses import dataclass
import socket
import socketserver
import threading
import time

from upper_client.robot_protocol import (
    COMMAND_HEADER,
    COMMAND_SIZE,
    AckPacket,
    Command,
    CommandPacket,
    ProtocolError,
    Status,
    decode_command,
    encode_ack,
)


@dataclass(frozen=True)
class MotorSnapshot:
    left: int
    right: int
    last_command: Command
    watchdog_stops: int


class RobotController:
    def __init__(self, watchdog_seconds: float = 0.5):
        self.watchdog_seconds = watchdog_seconds
        self._left = 0
        self._right = 0
        self._last_command = Command.STOP
        self._last_received = time.monotonic()
        self._watchdog_stops = 0
        self._lock = threading.Lock()

    def apply(self, packet: CommandPacket, obstacle: bool = False) -> Status:
        with self._lock:
            self._last_received = time.monotonic()
            if obstacle and packet.command == Command.FORWARD:
                self._stop()
                return Status.OBSTACLE_STOP
            left, right = packet.speed_left, packet.speed_right
            actions = {
                Command.FORWARD: (left, right),
                Command.BACKWARD: (-left, -right),
                Command.LEFT: (-left, right),
                Command.RIGHT: (left, -right),
                Command.STOP: (0, 0),
            }
            self._left, self._right = actions[packet.command]
            self._last_command = packet.command
            return Status.OK

    def tick(self) -> bool:
        with self._lock:
            moving = self._left != 0 or self._right != 0
            expired = time.monotonic() - self._last_received >= self.watchdog_seconds
            if moving and expired:
                self._stop()
                self._watchdog_stops += 1
                return True
            return False

    def _stop(self) -> None:
        self._left = 0
        self._right = 0
        self._last_command = Command.STOP

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            return MotorSnapshot(
                self._left,
                self._right,
                self._last_command,
                self._watchdog_stops,
            )


class RobotRequestHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        server: RobotTCPServer = self.server  # type: ignore[assignment]
        self.request.settimeout(0.05)
        buffer = bytearray()
        while True:
            try:
                chunk = self.request.recv(256)
                if not chunk:
                    return
                buffer.extend(chunk)
            except socket.timeout:
                if server.controller.tick() and not server.quiet:
                    print("watchdog: STOP")
                continue

            while len(buffer) >= COMMAND_SIZE:
                if buffer[0] != COMMAND_HEADER:
                    del buffer[0]
                    continue
                raw = bytes(buffer[:COMMAND_SIZE])
                del buffer[:COMMAND_SIZE]
                sequence = raw[4]
                try:
                    packet = decode_command(raw)
                    status = server.controller.apply(packet, server.obstacle)
                except ProtocolError as exc:
                    status = (
                        Status.CHECKSUM_ERROR
                        if "checksum" in str(exc)
                        else Status.INVALID_COMMAND
                    )
                self.request.sendall(encode_ack(AckPacket(sequence, status)))
                if not server.quiet:
                    state = server.controller.snapshot()
                    print(
                        f"seq={sequence:03d} status={status.name:<15} "
                        f"motor=({state.left:+d},{state.right:+d})"
                    )


class RobotTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        controller: RobotController | None = None,
        obstacle: bool = False,
        quiet: bool = False,
    ):
        self.controller = controller or RobotController()
        self.obstacle = obstacle
        self.quiet = quiet
        super().__init__(address, RobotRequestHandler)


def main() -> None:
    parser = argparse.ArgumentParser(description="WS63 robot TCP simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--obstacle", action="store_true")
    args = parser.parse_args()
    with RobotTCPServer((args.host, args.port), obstacle=args.obstacle) as server:
        print(f"robot simulator listening on {args.host}:{args.port}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()

