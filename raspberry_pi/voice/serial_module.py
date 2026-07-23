"""Adapter for UART voice modules that report recognised text or JSON lines."""

from __future__ import annotations

import json
from typing import Any

from .intents import VoiceObservation, infer_observation, observation_for_event


TEXT_KEYS = ("text", "transcript", "result", "recognition", "command")
CONFIDENCE_KEYS = ("confidence", "score", "probability")
EVENT_KEYS = ("event", "sound_event", "alarm")

# ASRPRO firmware sends these one-byte values after a recognised command.  A
# fixed binary vocabulary is less error-prone than transmitting UTF-8 across a
# noisy robot chassis link and makes command handling deterministic.
ASRPRO_EVENTS: dict[int, VoiceObservation] = {
    0xA0: VoiceObservation("小星小星"),
    0xA1: VoiceObservation("开始巡检", intent="patrol_start"),
    0xA2: VoiceObservation("暂停巡检", intent="patrol_pause"),
    0xA3: VoiceObservation("继续巡检", intent="patrol_resume"),
    0xA4: VoiceObservation("停止巡检", intent="patrol_stop"),
    0xA5: VoiceObservation("报告状态", intent="status_report"),
    0xA6: VoiceObservation("检测到求助", event="help"),
    0xA7: VoiceObservation("解除报警", intent="alarm_clear_request"),
    0xA8: VoiceObservation("报告温湿度", intent="environment_report"),
    0xA9: VoiceObservation("前方距离", intent="distance_report"),
    0xAA: VoiceObservation("巡检进度", intent="patrol_report"),
    0xAB: VoiceObservation("报告电量", intent="battery_report"),
}

# Returned by the Raspberry Pi to ASRPRO to request a locally stored response.
ASRPRO_REPLY_STARTED = 0xF1
ASRPRO_REPLY_PAUSED = 0xF2
ASRPRO_REPLY_STOPPED = 0xF3
ASRPRO_REPLY_STATUS = 0xF4
ASRPRO_REPLY_SLE_CONFIRM = 0xF5
ASRPRO_REPLY_CRITICAL = 0xF6
ASRPRO_REPLY_OBSTACLE_AVOID = 0xF7
ASRPRO_REPLY_TEMP_ALARM = 0xF8
ASRPRO_REPLY_HUMIDITY_ALARM = 0xF9
ASRPRO_REPLY_PATROL_COMPLETE_NORMAL = 0xFA
ASRPRO_REPLY_PATROL_COMPLETE_ALERT = 0xFB
ASRPRO_REPLY_STATUS_DETAIL = 0xFC
ASRPRO_REPLY_BATTERY_UNAVAILABLE = 0xFD
ASRPRO_REPLY_SENSOR_UNAVAILABLE = 0xFE
ASRPRO_REPLY_PERSON_ALERT = 0xEF
ASRPRO_REPLY_FIRE_SMOKE_ALERT = 0xEE

# Dynamic telemetry packet sent from the Pi to ASRPRO.  The first byte is a
# frame marker and the following six bytes are: kind, value_a (big endian),
# value_b (big endian), XOR checksum.  Values use the same deci-units as the
# WS63 monitor frame, so no rounding occurs between sensing and speaking.
ASRPRO_DYNAMIC_FRAME = 0xE0
ASRPRO_DYNAMIC_ENVIRONMENT = 0x01
ASRPRO_DYNAMIC_DISTANCE = 0x02

ASRPRO_ANNOUNCEMENT_CODES = {
    "obstacle_avoid": ASRPRO_REPLY_OBSTACLE_AVOID,
    "temperature_alarm": ASRPRO_REPLY_TEMP_ALARM,
    "humidity_alarm": ASRPRO_REPLY_HUMIDITY_ALARM,
    "patrol_complete_normal": ASRPRO_REPLY_PATROL_COMPLETE_NORMAL,
    "patrol_complete_alert": ASRPRO_REPLY_PATROL_COMPLETE_ALERT,
    "status_detail": ASRPRO_REPLY_STATUS_DETAIL,
    "battery_unavailable": ASRPRO_REPLY_BATTERY_UNAVAILABLE,
    "person_alert": ASRPRO_REPLY_PERSON_ALERT,
    "fire_smoke_alert": ASRPRO_REPLY_FIRE_SMOKE_ALERT,
}


def _number(value: Any, default: float = 1.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_module_line(raw: bytes | str) -> VoiceObservation:
    """Parse common UTF-8/JSON UART result formats from a voice board.

    The physical module is deliberately kept behind this adapter.  It accepts
    formats such as ``ASR: 开始巡检`` and ``{"text":"开始巡检"}``, while an
    eventual vendor-specific binary protocol only needs a small parser here.
    """

    line = raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw
    line = line.strip()
    if not line:
        return VoiceObservation("")

    try:
        payload = json.loads(line)
    except json.JSONDecodeError:
        payload = None

    if isinstance(payload, dict):
        confidence = next((_number(payload.get(key)) for key in CONFIDENCE_KEYS if key in payload), 1.0)
        for key in EVENT_KEYS:
            value = payload.get(key)
            if isinstance(value, str):
                event = observation_for_event(value, confidence=confidence)
                if event.is_actionable:
                    return event
        for key in TEXT_KEYS:
            value = payload.get(key)
            if isinstance(value, str):
                return infer_observation(value, confidence)
        return VoiceObservation("")

    text = line
    for prefix in ("ASR:", "ASR：", "识别结果:", "识别结果：", "VOICE:", "VOICE："):
        if text.upper().startswith(prefix.upper()):
            text = text[len(prefix):].strip()
            break
    return infer_observation(text)


def parse_asrpro_byte(raw: bytes | int) -> VoiceObservation:
    """Decode the custom ASRPRO robot command byte sent by its firmware."""

    value = raw if isinstance(raw, int) else (raw[0] if raw else -1)
    observation = ASRPRO_EVENTS.get(value)
    if observation is None:
        return VoiceObservation("")
    return observation


def asrpro_reply_code(observation: VoiceObservation, reply: dict[str, object]) -> int | None:
    """Translate arbiter acknowledgement to a short ASRPRO speech prompt."""

    if observation.event:
        return ASRPRO_REPLY_CRITICAL
    if not reply.get("ok", False):
        return ASRPRO_REPLY_SLE_CONFIRM
    action = str(reply.get("action", ""))
    if action in ("map patrol started", "map patrol resumed"):
        return ASRPRO_REPLY_STARTED
    if action == "map patrol paused":
        return ASRPRO_REPLY_PAUSED
    if action == "map patrol stopped":
        return ASRPRO_REPLY_STOPPED
    # Query replies are emitted by the announcement queue as a telemetry frame
    # so the speaker reads the measured value rather than a fixed sentence.
    if action in ("status reported", "environment reported", "distance reported", "patrol reported", "battery reported"):
        return None
    return None


def asrpro_announcement_code(announcement: object) -> int | None:
    """Map a Pi-side proactive announcement to a stored ASRPRO prompt."""

    if not isinstance(announcement, dict):
        return None
    return ASRPRO_ANNOUNCEMENT_CODES.get(str(announcement.get("kind", "")))


def _u16_bytes(value: object, signed: bool = False) -> tuple[int, int]:
    """Clamp a telemetry value and return its on-wire big-endian bytes."""

    try:
        number = int(value)
    except (TypeError, ValueError):
        number = 0
    lower, upper = (-32768, 32767) if signed else (0, 65535)
    number = max(lower, min(upper, number))
    if signed and number < 0:
        number += 1 << 16
    return (number >> 8) & 0xFF, number & 0xFF


def _dynamic_frame(kind: int, value_a: object, value_b: object, *, signed_a: bool = False) -> bytes:
    a_hi, a_lo = _u16_bytes(value_a, signed=signed_a)
    b_hi, b_lo = _u16_bytes(value_b)
    body = (kind, a_hi, a_lo, b_hi, b_lo)
    checksum = ASRPRO_DYNAMIC_FRAME
    for value in body:
        checksum ^= value
    return bytes((ASRPRO_DYNAMIC_FRAME, *body, checksum))


def asrpro_announcement_frames(announcement: object) -> list[bytes]:
    """Encode a queued Pi announcement for the ASRPRO UART writer.

    Fixed alerts continue to use one-byte prompt codes.  Live query answers
    use compact framed values, which lets the board compose locally stored
    number prompts without relying on cloud TTS.
    """

    if not isinstance(announcement, dict):
        return []
    kind = str(announcement.get("kind", ""))
    telemetry = announcement.get("telemetry")
    if not isinstance(telemetry, dict):
        telemetry = {}

    environment_valid = bool(telemetry.get("environment_valid", False))
    obstacle_valid = bool(telemetry.get("obstacle_valid", False))
    if kind in ("environment_report", "status_report"):
        if not environment_valid:
            return [bytes((ASRPRO_REPLY_SENSOR_UNAVAILABLE,))]
        return [
            _dynamic_frame(
                ASRPRO_DYNAMIC_ENVIRONMENT,
                telemetry.get("temperature_deci_c", 0),
                telemetry.get("humidity_deci_percent", 0),
                signed_a=True,
            )
        ]
    if kind == "distance_report":
        if not obstacle_valid:
            return [bytes((ASRPRO_REPLY_SENSOR_UNAVAILABLE,))]
        return [
            _dynamic_frame(
                ASRPRO_DYNAMIC_DISTANCE,
                telemetry.get("distance_mm", 0),
                telemetry.get("threshold_mm", 0),
            )
        ]

    code = asrpro_announcement_code(announcement)
    return [] if code is None else [bytes((code,))]
