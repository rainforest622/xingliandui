from __future__ import annotations

from .robot_protocol import Command

MOTION_COMMAND_KEYS = {
    "F": Command.FORWARD,
    "B": Command.BACKWARD,
    "L": Command.LEFT,
    "R": Command.RIGHT,
    "S": Command.STOP,
}

AT_COMMANDS = {
    "I": "AT+ROBOTMI",
    "O": "AT+ROBOTOLED",
    "E": "AT+ROBOTENV",
    "D": "AT+ROBOTOBS",
    "A": "AT+ROBOTAVOID",
    "P": "AT+ROBOTPATROL",
    "G": "AT+ROBOTPATROLST",
    "T": "AT+ROBOTST",
    "M": "AT+ROBOTMON",
    "F": "AT+ROBOTF",
    "B": "AT+ROBOTB",
    "L": "AT+ROBOTL",
    "R": "AT+ROBOTR",
    "S": "AT+ROBOTS",
}

SLE_ASCII_COMMAND_KEYS = tuple(AT_COMMANDS.keys())


def normalize_robot_key(key: str) -> str:
    normalized = key.strip().upper()
    if normalized not in AT_COMMANDS:
        raise ValueError(f"unknown robot command key: {key}")
    return normalized


def encode_sle_ascii_command(key: str) -> bytes:
    return normalize_robot_key(key).encode("ascii")
