from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

COMMAND_HEADER = 0xAA
ACK_HEADER = 0x55
STATE_HEADER = 0x5A

COMMAND_SIZE = 6
ACK_SIZE = 4
STATE_SIZE = 8


class ProtocolError(ValueError):
    pass


class Command(IntEnum):
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    STOP = 5


class Status(IntEnum):
    OK = 0
    OBSTACLE_STOP = 1
    LOW_BATTERY = 2
    CHECKSUM_ERROR = 3
    INVALID_COMMAND = 4
    MOTOR_ERROR = 5


@dataclass(frozen=True)
class CommandPacket:
    command: Command
    speed_left: int
    speed_right: int
    sequence: int


@dataclass(frozen=True)
class AckPacket:
    sequence: int
    status: Status


@dataclass(frozen=True)
class StatePacket:
    sequence: int
    battery: int
    humidity: int
    distance_mm: int
    motor_state: int


def checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def _byte(name: str, value: int, upper: int = 255) -> int:
    if not 0 <= value <= upper:
        raise ProtocolError(f"{name} must be in 0..{upper}, got {value}")
    return value


def encode_command(packet: CommandPacket) -> bytes:
    speed_left = _byte("speed_left", packet.speed_left, 100)
    speed_right = _byte("speed_right", packet.speed_right, 100)
    sequence = _byte("sequence", packet.sequence)
    body = bytes(
        (COMMAND_HEADER, int(packet.command), speed_left, speed_right, sequence)
    )
    return body + bytes((checksum(body),))


def decode_command(data: bytes) -> CommandPacket:
    if len(data) != COMMAND_SIZE:
        raise ProtocolError(f"command packet must be {COMMAND_SIZE} bytes")
    if data[0] != COMMAND_HEADER:
        raise ProtocolError("bad command header")
    if checksum(data[:-1]) != data[-1]:
        raise ProtocolError("bad command checksum")
    try:
        command = Command(data[1])
    except ValueError as exc:
        raise ProtocolError(f"unknown command {data[1]}") from exc
    return CommandPacket(
        command,
        _byte("speed_left", data[2], 100),
        _byte("speed_right", data[3], 100),
        data[4],
    )


def encode_ack(packet: AckPacket) -> bytes:
    sequence = _byte("sequence", packet.sequence)
    body = bytes((ACK_HEADER, sequence, int(packet.status)))
    return body + bytes((checksum(body),))


def decode_ack(data: bytes) -> AckPacket:
    if len(data) != ACK_SIZE:
        raise ProtocolError(f"ACK packet must be {ACK_SIZE} bytes")
    if data[0] != ACK_HEADER:
        raise ProtocolError("bad ACK header")
    if checksum(data[:-1]) != data[-1]:
        raise ProtocolError("bad ACK checksum")
    try:
        status = Status(data[2])
    except ValueError as exc:
        raise ProtocolError(f"unknown status {data[2]}") from exc
    return AckPacket(data[1], status)


def encode_state(packet: StatePacket) -> bytes:
    battery = _byte("battery", packet.battery, 100)
    humidity = _byte("humidity", packet.humidity, 100)
    distance = packet.distance_mm
    if not 0 <= distance <= 65535:
        raise ProtocolError("distance_mm must be in 0..65535")
    body = struct.pack(
        "<BBBBHB",
        STATE_HEADER,
        _byte("sequence", packet.sequence),
        battery,
        humidity,
        distance,
        _byte("motor_state", packet.motor_state, 4),
    )
    return body + bytes((checksum(body),))


def decode_state(data: bytes) -> StatePacket:
    if len(data) != STATE_SIZE:
        raise ProtocolError(f"state packet must be {STATE_SIZE} bytes")
    if data[0] != STATE_HEADER or checksum(data[:-1]) != data[-1]:
        raise ProtocolError("invalid state packet")
    _, sequence, battery, humidity, distance, motor_state = struct.unpack(
        "<BBBBHB", data[:-1]
    )
    return StatePacket(sequence, battery, humidity, distance, motor_state)
