from __future__ import annotations

import argparse
from collections import deque
import glob
import json
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import serial
except ImportError as exc:  # pragma: no cover - only hit on a Pi without pyserial
    raise SystemExit("pyserial is required: sudo apt install -y python3-serial") from exc


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
MODE_MANUAL = "manual"
MODE_AUTO_SQUARE = "auto_square"
MODE_AUTO_VISION = "auto_vision"
MODE_AUTO_MAP = "auto_map"
MODE_ESTOP = "estop"
VOICE_INTENT_START = "patrol_start"
VOICE_INTENT_PAUSE = "patrol_pause"
VOICE_INTENT_RESUME = "patrol_resume"
VOICE_INTENT_STOP = "patrol_stop"
VOICE_INTENT_STATUS = "status_report"
VOICE_INTENT_ENVIRONMENT = "environment_report"
VOICE_INTENT_DISTANCE = "distance_report"
VOICE_INTENT_PATROL_REPORT = "patrol_report"
VOICE_INTENT_BATTERY = "battery_report"
VOICE_INTENT_CLEAR_ALARM = "alarm_clear_request"
VOICE_EVENTS_STOP = {"help", "alarm_sound", "impact"}
VOICE_INTENTS = {
    VOICE_INTENT_START,
    VOICE_INTENT_PAUSE,
    VOICE_INTENT_RESUME,
    VOICE_INTENT_STOP,
    VOICE_INTENT_STATUS,
    VOICE_INTENT_ENVIRONMENT,
    VOICE_INTENT_DISTANCE,
    VOICE_INTENT_PATROL_REPORT,
    VOICE_INTENT_BATTERY,
    VOICE_INTENT_CLEAR_ALARM,
}
STOP_PAYLOAD = {"T": 1, "L": 0.0, "R": 0.0}
ALLOWED_ROUTE_ACTIONS = {"move", "turn", "rotate", "wait", "inspect", "stop", "pause"}
ROBOT_ALARM_OBSTACLE_BLOCKED = 1 << 2
ROBOT_ALARM_TEMP_HIGH = 1 << 3
ROBOT_ALARM_HUMIDITY_HIGH = 1 << 4
VOICE_ANNOUNCEMENT_COOLDOWNS = {
    "obstacle_avoid": 4.0,
    "temperature_alarm": 12.0,
    "humidity_alarm": 12.0,
}
VOICE_ANNOUNCEMENT_MAX_AGE_S = 30.0
RANGE_SAFETY_WINDOW = 3
RANGE_SAFETY_HISTORY = 12
RANGE_SAFETY_BLOCK_CONFIRMATIONS = 2
RANGE_SAFETY_CLEAR_CONFIRMATIONS = 2
RANGE_SAFETY_STALE_S = 1.5
RANGE_SAFETY_MAX_MEASUREMENT_AGE_MS = 350


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Safety arbiter for WS63 SLE manual commands, auto patrol, and WAVE ROVER UART output."
    )
    parser.add_argument("--ws63-port", default="auto")
    parser.add_argument("--rover-port", default="auto")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--read-timeout", type=float, default=0.02)
    parser.add_argument("--manual-timeout", type=float, default=0.35)
    parser.add_argument("--stop-repeat", type=float, default=0.25)
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", type=int, default=8090)
    parser.add_argument("--auto-speed", type=float, default=0.18)
    parser.add_argument("--turn-speed", type=float, default=0.18)
    parser.add_argument("--square-forward", type=float, default=3.0)
    parser.add_argument("--square-turn", type=float, default=0.65)
    parser.add_argument("--auto-period", type=float, default=0.10)
    parser.add_argument(
        "--start-mode",
        choices=(MODE_MANUAL, MODE_AUTO_SQUARE, MODE_AUTO_VISION, MODE_AUTO_MAP),
        default=MODE_MANUAL,
    )
    parser.add_argument("--camera-stream-url", default="http://127.0.0.1:8080/stream.mjpg")
    parser.add_argument("--vision-speed", type=float, default=0.16)
    parser.add_argument("--vision-gain", type=float, default=0.14)
    parser.add_argument("--vision-min-area", type=float, default=600.0)
    parser.add_argument("--vision-lost-timeout", type=float, default=0.5)
    parser.add_argument("--vision-color", choices=("yellow", "black", "both"), default="yellow")
    parser.add_argument("--map-path", default="patrol_map.json")
    parser.add_argument("--avoid-emergency-mm", type=int, default=120)
    parser.add_argument("--avoid-block-mm", type=int, default=280)
    parser.add_argument("--avoid-caution-mm", type=int, default=380)
    parser.add_argument("--avoid-turn-speed", type=float, default=0.50)
    parser.add_argument("--avoid-turn-s", type=float, default=0.686)
    parser.add_argument("--avoid-side-m", type=float, default=0.65)
    parser.add_argument("--avoid-pass-m", type=float, default=1.20)
    parser.add_argument("--avoid-backtrack-m", type=float, default=0.15)
    parser.add_argument("--avoid-scan-angle-deg", type=float, default=45.0)
    parser.add_argument("--avoid-scan-settle-s", type=float, default=0.40)
    parser.add_argument("--avoid-scan-timeout-s", type=float, default=1.50)
    parser.add_argument("--avoid-scan-samples", type=int, default=2)
    parser.add_argument("--avoid-max-expansions", type=int, default=2)
    parser.add_argument("--camera-status-url", default="")
    parser.add_argument("--quiet-rover", action="store_true")
    return parser.parse_args()


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
    if Path("/dev/ttyAMA0").exists() and gpio_uart0_on_header():
        return "/dev/ttyAMA0"
    for candidate in DEFAULT_ROVER_CANDIDATES:
        if Path(candidate).exists():
            return candidate
    raise SystemExit("WAVE ROVER UART not found. Enable the Raspberry Pi 40PIN UART first.")


def resolve_ws63_port(requested: str, rover_port: str) -> str:
    if requested != "auto":
        return requested

    rover_real = str(Path(rover_port).resolve()) if Path(rover_port).exists() else rover_port
    # ASRPRO and WS63 both identify as CH340, so /dev/serial/by-id can point
    # at the ASRPRO.  This alias is created from the WS63 product ID by udev.
    stable_ws63 = Path("/dev/nearlink-ws63")
    if stable_ws63.exists() and str(stable_ws63.resolve()) != rover_real:
        return str(stable_ws63)

    candidates: list[str] = []
    for pattern in DEFAULT_WS63_CANDIDATES:
        candidates.extend(glob.glob(pattern))

    seen = set()
    for candidate in sorted(candidates):
        real = str(Path(candidate).resolve()) if Path(candidate).exists() else candidate
        if candidate == rover_port or real == rover_real or real in seen:
            continue
        seen.add(real)
        return candidate

    raise SystemExit("WS63 USB serial not found. Connect the WS63 Type-C data cable to the Raspberry Pi.")


def open_serial(port: str, baudrate: int, timeout: float) -> serial.Serial:
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, dsrdtr=False)


def clamp_speed(value: Any, limit: float = 0.5) -> float:
    try:
        speed = float(value)
    except (TypeError, ValueError):
        return 0.0
    if speed > limit:
        return limit
    if speed < -limit:
        return -limit
    return speed


def clean_motion_payload(payload: dict[str, Any]) -> dict[str, float | int] | None:
    if payload.get("T") != 1 or ("L" not in payload and "R" not in payload):
        return None
    return {
        "T": 1,
        "L": round(clamp_speed(payload.get("L", 0.0)), 3),
        "R": round(clamp_speed(payload.get("R", 0.0)), 3),
    }


def adapt_rover_motion(payload: dict[str, Any]) -> dict[str, Any]:
    """Use open-loop PWM whenever the field rover must drive in reverse."""

    motion = clean_motion_payload(payload)
    if motion is None:
        return dict(payload)
    left = float(motion["L"])
    right = float(motion["R"])
    if left < 0.0 < right:
        return {"T": 11, "L": -255, "R": 255}
    if right < 0.0 < left:
        return {"T": 11, "L": 255, "R": -255}
    if left < 0.0 and right < 0.0:
        return {"T": 11, "L": round(left * 510), "R": round(right * 510)}
    return motion


def encode_payload(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8") + b"\n"


def is_nonzero_motion(payload: dict[str, Any]) -> bool:
    return abs(float(payload.get("L", 0.0))) > 0.0001 or abs(float(payload.get("R", 0.0))) > 0.0001


def safe_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def query_from_dict(data: dict[str, Any]) -> dict[str, list[str]]:
    query: dict[str, list[str]] = {}
    for key, value in data.items():
        if value is None:
            continue
        query[str(key)] = [str(value)]
    return query


def scale_motion_payload(payload: dict[str, Any], factor: float) -> dict[str, Any]:
    """Reduce a differential-drive command without changing its turn ratio."""

    motion = clean_motion_payload(payload)
    if motion is None:
        return dict(payload)
    scale = max(0.0, min(1.0, float(factor)))
    return {
        "T": 1,
        "L": round(float(motion["L"]) * scale, 3),
        "R": round(float(motion["R"]) * scale, 3),
    }


class RangeSafetyGuard:
    """Filter fresh HC-SR04 samples and expose conservative auto-patrol states."""

    def __init__(self, emergency_mm: int, block_mm: int, caution_mm: int) -> None:
        self.emergency_mm = max(80, int(emergency_mm))
        self.block_mm = max(self.emergency_mm + 20, int(block_mm))
        self.caution_mm = max(self.block_mm + 20, int(caution_mm))
        self.samples: deque[int] = deque(maxlen=RANGE_SAFETY_WINDOW)
        self.history: deque[tuple[int, int, float]] = deque(maxlen=RANGE_SAFETY_HISTORY)
        self.last_sample_at = 0.0
        self.last_raw_mm = 0
        self.filtered_mm = 0
        self.sequence = 0
        self.state = "sensor_wait"
        self.block_count = 0
        self.clear_count = 0
        self.last_monitor_sample_count: int | None = None
        self.last_measurement_age_ms: int | None = None
        self.ignored_duplicate_frames = 0
        self.ignored_stale_frames = 0

    def observe(self, telemetry: dict[str, Any], now: float) -> None:
        # Textual "ROBOT OBSTACLE STOP" logs are useful diagnostics but are not
        # a new physical range sample.  Only the documented MON frame may move
        # the autonomous state machine.
        if bool(telemetry.get("_range_event_only", False)):
            return
        if not bool(telemetry.get("obstacle_enabled", False)) or not bool(telemetry.get("obstacle_valid", False)):
            return
        try:
            distance_mm = int(telemetry.get("distance_mm", 0))
        except (TypeError, ValueError):
            return
        if distance_mm <= 0:
            return

        sample_count = telemetry.get("sample_count")
        if sample_count is not None:
            try:
                monitor_sample_count = int(sample_count)
            except (TypeError, ValueError):
                monitor_sample_count = None
            if monitor_sample_count is not None:
                if self.last_monitor_sample_count == monitor_sample_count:
                    self.ignored_duplicate_frames += 1
                    return
                self.last_monitor_sample_count = monitor_sample_count

        measurement_age = telemetry.get("obstacle_age_ms")
        if measurement_age is not None:
            try:
                self.last_measurement_age_ms = max(0, int(measurement_age))
            except (TypeError, ValueError):
                self.last_measurement_age_ms = None
            if self.last_measurement_age_ms is not None and self.last_measurement_age_ms > RANGE_SAFETY_MAX_MEASUREMENT_AGE_MS:
                self.ignored_stale_frames += 1
                return

        self.samples.append(distance_mm)
        self.sequence += 1
        self.history.append((self.sequence, distance_mm, now))
        self.last_raw_mm = distance_mm
        self.last_sample_at = now
        ordered = sorted(self.samples)
        self.filtered_mm = ordered[len(ordered) // 2]

        if self.filtered_mm <= self.emergency_mm:
            self.state = "emergency_stop"
            self.block_count = RANGE_SAFETY_BLOCK_CONFIRMATIONS
            self.clear_count = 0
            return

        if self.filtered_mm <= self.block_mm:
            self.block_count += 1
            self.clear_count = 0
            self.state = "blocked" if self.block_count >= RANGE_SAFETY_BLOCK_CONFIRMATIONS else "caution"
            return

        self.block_count = 0
        if self.filtered_mm <= self.caution_mm:
            self.clear_count = 0
            self.state = "caution"
            return

        self.clear_count += 1
        if self.clear_count >= RANGE_SAFETY_CLEAR_CONFIRMATIONS:
            self.state = "clear"

    def fresh_samples_after(self, sequence: int) -> list[int]:
        """Return only range samples captured after a chassis turn completed."""

        return [distance_mm for sample_sequence, distance_mm, _sample_at in self.history if sample_sequence > sequence]

    def decision(self, now: float) -> dict[str, Any]:
        age_s = None if self.last_sample_at <= 0 else round(now - self.last_sample_at, 3)
        state = self.state
        if self.last_sample_at <= 0:
            state = "sensor_wait"
        elif (now - self.last_sample_at) > RANGE_SAFETY_STALE_S:
            state = "sensor_stale"
        return {
            "state": state,
            "blocking": state in ("sensor_wait", "sensor_stale", "blocked", "emergency_stop"),
            "caution": state == "caution",
            "last_raw_mm": self.last_raw_mm,
            "filtered_mm": self.filtered_mm,
            "sample_count": len(self.samples),
            "sequence": self.sequence,
            "sample_age_s": age_s,
            "measurement_age_ms": self.last_measurement_age_ms,
            "ignored_duplicate_frames": self.ignored_duplicate_frames,
            "ignored_stale_frames": self.ignored_stale_frames,
            "emergency_mm": self.emergency_mm,
            "block_mm": self.block_mm,
            "caution_mm": self.caution_mm,
        }


def camera_status_url_from_stream(stream_url: str) -> str:
    """Build the local camera-service status endpoint from its MJPEG URL."""

    parsed = urlparse(stream_url)
    if not parsed.scheme or not parsed.netloc:
        return ""
    return parsed._replace(path="/status", params="", query="", fragment="").geturl()


class CameraSafetyMonitor:
    """Poll camera-side person detection without blocking the motion loop."""

    def __init__(self, status_url: str, poll_s: float = 0.40) -> None:
        self.status_url = status_url
        self.poll_s = max(0.15, float(poll_s))
        self.lock = threading.Lock()
        self.running = False
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.last_update_at = 0.0
        self.last_error = ""
        self.person_detected = False
        self.alarm_active = False
        self.frames_total = 0
        self.camera_frame_ready = False
        self.camera_frame_age_s: float | None = None

    def start(self) -> None:
        if not self.status_url or self.running:
            return
        self.running = True
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.thread = None

    def _run(self) -> None:
        while not self.stop_event.is_set():
            try:
                with urllib.request.urlopen(self.status_url, timeout=0.30) as response:
                    payload = json.loads(response.read().decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("camera status root is not an object")
                frame_age = safe_float(payload.get("last_frame_age_sec"), 9999.0)
                frame_ready = bool(payload.get("frame_ready", False)) and frame_age <= 1.0
                with self.lock:
                    self.person_detected = bool(payload.get("person_detected", False)) if frame_ready else False
                    self.alarm_active = bool(payload.get("alarm_active", False))
                    self.frames_total = int(payload.get("frames_total", 0))
                    self.camera_frame_ready = frame_ready
                    self.camera_frame_age_s = round(frame_age, 3)
                    self.last_error = ""
                    self.last_update_at = time.monotonic()
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                with self.lock:
                    self.last_error = str(exc)[:160]
            self.stop_event.wait(self.poll_s)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            age_s = None if self.last_update_at <= 0 else round(now - self.last_update_at, 3)
            return {
                "available": self.camera_frame_ready
                and self.last_update_at > 0
                and (now - self.last_update_at) <= max(1.5, self.poll_s * 4.0),
                "person_detected": self.person_detected,
                "alarm_active": self.alarm_active,
                "frames_total": self.frames_total,
                "frame_age_s": self.camera_frame_age_s,
                "age_s": age_s,
                "last_error": self.last_error,
                "status_url": self.status_url,
            }


def validate_route_data(data: Any) -> tuple[bool, str]:
    if not isinstance(data, dict):
        return False, "route root must be an object"
    steps = data.get("steps")
    if not isinstance(steps, list) or not steps:
        return False, "route steps must be a non-empty list"
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            return False, f"step {index + 1} must be an object"
        action = str(step.get("action", step.get("type", "move"))).lower()
        if action not in ALLOWED_ROUTE_ACTIONS:
            return False, f"step {index + 1} unsupported action: {action}"
        duration = safe_float(step.get("duration_s", step.get("duration")), -1.0)
        if duration <= 0:
            return False, f"step {index + 1} duration_s must be positive"
        if action in ("turn", "rotate"):
            direction = str(step.get("direction", "right")).lower()
            if direction not in ("right", "cw", "clockwise", "left", "ccw", "counterclockwise"):
                return False, f"step {index + 1} unsupported turn direction: {direction}"
    return True, ""


class VisionTracker:
    def __init__(
        self,
        stream_url: str,
        speed: float,
        gain: float,
        min_area: float,
        lost_timeout: float,
        color_mode: str,
    ) -> None:
        self.stream_url = stream_url
        self.speed = clamp_speed(speed)
        self.gain = max(0.0, float(gain))
        self.min_area = max(1.0, float(min_area))
        self.lost_timeout = max(0.1, float(lost_timeout))
        self.color_mode = color_mode if color_mode in ("yellow", "black", "both") else "yellow"
        self.lock = threading.Lock()
        self.running = False
        self.thread: threading.Thread | None = None
        self.last_error = ""
        self.last_seen_at = 0.0
        self.last_frame_at = 0.0
        self.last_error_value = 0.0
        self.last_area = 0.0
        self.last_color = "none"
        self.frames = 0

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def configure(self, query: dict[str, list[str]]) -> None:
        with self.lock:
            if "speed" in query:
                self.speed = clamp_speed(query["speed"][0])
            if "gain" in query:
                self.gain = max(0.0, float(query["gain"][0]))
            if "min_area" in query:
                self.min_area = max(1.0, float(query["min_area"][0]))
            if "color" in query and query["color"][0] in ("yellow", "black", "both"):
                self.color_mode = query["color"][0]

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            return {
                "stream_url": self.stream_url,
                "speed": self.speed,
                "gain": self.gain,
                "min_area": self.min_area,
                "color_mode": self.color_mode,
                "last_error": self.last_error,
                "last_color": self.last_color,
                "line_error": round(self.last_error_value, 3),
                "line_area": round(self.last_area, 1),
                "last_frame_age_s": None if self.last_frame_at <= 0 else round(now - self.last_frame_at, 3),
                "last_seen_age_s": None if self.last_seen_at <= 0 else round(now - self.last_seen_at, 3),
                "frames": self.frames,
            }

    def next_payload(self) -> tuple[dict[str, Any], str]:
        now = time.monotonic()
        with self.lock:
            if self.last_seen_at <= 0 or (now - self.last_seen_at) > self.lost_timeout:
                return dict(STOP_PAYLOAD), "line lost"
            turn = self.gain * self.last_error_value
            left = clamp_speed(self.speed + turn)
            right = clamp_speed(self.speed - turn)
            return {"T": 1, "L": round(left, 3), "R": round(right, 3)}, f"line {self.last_color}"

    def _run(self) -> None:
        try:
            import cv2
            import numpy as np
        except ImportError as exc:
            with self.lock:
                self.last_error = f"vision dependencies missing: {exc}"
            return

        retry_delay = 2.0
        while self.running:
            try:
                with urllib.request.urlopen(self.stream_url, timeout=5) as response:
                    buffer = b""
                    while self.running:
                        chunk = response.read(4096)
                        if not chunk:
                            raise RuntimeError("camera stream ended")
                        buffer += chunk
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2)
                        if start < 0 or end < 0:
                            buffer = buffer[-65536:]
                            continue
                        jpg = buffer[start : end + 2]
                        buffer = buffer[end + 2 :]
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue
                        self._process_frame(frame, cv2, np)
            except Exception as exc:
                with self.lock:
                    self.last_error = str(exc)
                if self.running:
                    time.sleep(retry_delay)

    def _process_frame(self, frame: Any, cv2: Any, np: Any) -> None:
        height, width = frame.shape[:2]
        roi = frame[int(height * 0.45) :, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, np.array([18, 60, 60]), np.array([42, 255, 255]))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        black = cv2.inRange(gray, 0, 55)
        kernel = np.ones((5, 5), np.uint8)

        candidates: list[tuple[str, float, float]] = []
        masks: list[tuple[str, Any]] = []
        if self.color_mode in ("yellow", "both"):
            masks.append(("yellow", yellow))
        if self.color_mode in ("black", "both"):
            masks.append(("black", black))

        for color, mask in masks:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < self.min_area:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            cx = float(moments["m10"] / moments["m00"])
            error = (cx - (width / 2.0)) / max(1.0, width / 2.0)
            candidates.append((color, area, error))

        now = time.monotonic()
        with self.lock:
            self.frames += 1
            self.last_frame_at = now
            if candidates:
                color, area, error = max(candidates, key=lambda item: item[1])
                self.last_seen_at = now
                self.last_color = color
                self.last_area = area
                self.last_error_value = max(-1.0, min(1.0, error))
                self.last_error = ""
            else:
                self.last_color = "none"
                self.last_area = 0.0


class MapBrain:
    def __init__(self, map_path: str) -> None:
        configured_path = Path(map_path)
        active_route_path = configured_path.parent / "active_route.json"
        # A phone-imported route must survive an arbiter service restart.  The
        # configured map remains the fallback for a fresh installation.
        self.map_path = active_route_path if active_route_path.is_file() else configured_path
        self.lock = threading.Lock()
        self.name = ""
        self.description = ""
        self.area_m2 = 0.0
        self.default_speed = 0.16
        self.default_turn_speed = 0.16
        self.default_loops = 1
        self.seconds_per_meter = 2.80
        self.ninety_degree_turn_s = 0.686
        self.steps: list[dict[str, Any]] = []
        self.active = False
        self.finished = False
        self.paused = False
        self.paused_at = 0.0
        self.speed_scale = 1.0
        self.max_loops = 1
        self.step_index = 0
        self.loop_index = 0
        self.step_started_at = 0.0
        self.step_until = 0.0
        self.last_resume_progress_s = 0.0
        self.last_loaded_at = 0.0
        self.last_error = ""
        with self.lock:
            self._load_unlocked()

    def import_route(self, data: dict[str, Any]) -> tuple[bool, str, dict[str, Any]]:
        ok, error = validate_route_data(data)
        now = time.monotonic()
        with self.lock:
            if not ok:
                self.last_error = error
                return False, error, self.snapshot_unlocked(now)

            target = self.map_path.parent / "active_route.json"
            try:
                with target.open("w", encoding="utf-8") as handle:
                    json.dump(data, handle, ensure_ascii=False, indent=2)
                    handle.write("\n")
            except OSError as exc:
                self.last_error = f"route save failed: {exc}"
                return False, self.last_error, self.snapshot_unlocked(now)

            self.map_path = target
            self.active = False
            self.finished = False
            self.paused = False
            self.paused_at = 0.0
            self.step_index = 0
            self.loop_index = 0
            self.step_started_at = now
            self.step_until = now
            self._load_unlocked()
            return True, "", self.snapshot_unlocked(now)

    def current_route(self) -> dict[str, Any]:
        with self.lock:
            try:
                with self.map_path.open("r", encoding="utf-8") as handle:
                    route = json.load(handle)
            except (OSError, json.JSONDecodeError):
                route = None
            snapshot = self.snapshot_unlocked(time.monotonic())
        return {"map_brain": snapshot, "route": route}

    def avoidance_calibration(self) -> dict[str, float]:
        """Return the route's measured open-loop movement calibration."""

        with self.lock:
            speed_scale = max(0.10, self.speed_scale)
            return {
                "seconds_per_meter": self.seconds_per_meter / speed_scale,
                "ninety_degree_turn_s": self.ninety_degree_turn_s,
                "linear_speed": self.default_speed * speed_scale,
                "turn_speed": self.default_turn_speed,
            }

    def _load_unlocked(self) -> None:
        try:
            with self.map_path.open("r", encoding="utf-8") as handle:
                data = json.load(handle)
        except OSError as exc:
            self.last_error = f"map load failed: {exc}"
            self.steps = []
            return
        except json.JSONDecodeError as exc:
            self.last_error = f"map json invalid: {exc}"
            self.steps = []
            return

        if not isinstance(data, dict):
            self.last_error = "map root must be an object"
            self.steps = []
            return

        raw_steps = data.get("steps", [])
        if not isinstance(raw_steps, list):
            self.last_error = "map steps must be a list"
            self.steps = []
            return

        self.name = str(data.get("name", self.map_path.stem))
        self.description = str(data.get("description", ""))
        self.area_m2 = max(0.0, safe_float(data.get("area_m2"), 0.0))
        self.default_speed = clamp_speed(data.get("default_speed", 0.16))
        self.default_turn_speed = clamp_speed(data.get("default_turn_speed", 0.16))
        self.default_loops = max(0, int(safe_float(data.get("default_loops"), 1)))
        calibration = data.get("calibration") if isinstance(data.get("calibration"), dict) else {}
        self.seconds_per_meter = max(0.25, safe_float(calibration.get("seconds_per_meter"), 2.80))
        self.ninety_degree_turn_s = max(
            0.10,
            safe_float(calibration.get("ninety_degree_turn_ms"), 686.0) / 1000.0,
        )
        self.steps = [step for step in raw_steps if isinstance(step, dict)]
        self.last_loaded_at = time.monotonic()
        self.last_error = "" if self.steps else "map has no valid steps"

    def configure_and_start(self, query: dict[str, list[str]]) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            if "path" in query:
                requested = Path(query["path"][0])
                self.map_path = requested if requested.is_absolute() else self.map_path.parent / requested
            self._load_unlocked()
            self.speed_scale = max(0.0, min(2.0, safe_float(query.get("speed_scale", ["1.0"])[0], 1.0)))
            self.max_loops = self.default_loops
            if "loops" in query:
                self.max_loops = max(0, int(safe_float(query["loops"][0], self.default_loops)))
            if "loop" in query:
                value = query["loop"][0].strip().lower()
                if value in ("1", "true", "yes", "on"):
                    self.max_loops = 0
                elif value in ("0", "false", "no", "off"):
                    self.max_loops = 1
            self.active = bool(self.steps)
            self.finished = False
            self.paused = False
            self.paused_at = 0.0
            self.step_index = 0
            self.loop_index = 0
            self.step_started_at = now
            self.step_until = now + self._current_duration_unlocked()
            return self.snapshot_unlocked(now)

    def stop(self) -> None:
        with self.lock:
            self.active = False
            self.paused = False
            self.paused_at = 0.0

    def pause(self) -> None:
        with self.lock:
            if self.active and not self.paused:
                self.paused = True
                self.paused_at = time.monotonic()

    def resume(self, projected_progress_s: float = 0.0) -> None:
        now = time.monotonic()
        with self.lock:
            if not self.paused:
                return
            paused_for = max(0.0, now - self.paused_at)
            progress_s = 0.0
            if self.steps and self.step_index < len(self.steps):
                step = self.steps[self.step_index]
                action = str(step.get("action", step.get("type", "move"))).lower()
                if action == "move":
                    remaining_at_pause = max(0.0, self.step_until - self.paused_at)
                    progress_s = min(max(0.0, projected_progress_s), remaining_at_pause)
            clock_shift = paused_for - progress_s
            self.step_started_at += clock_shift
            self.step_until += clock_shift
            self.last_resume_progress_s = progress_s
            self.paused = False
            self.paused_at = 0.0

    def next_payload(self) -> tuple[dict[str, Any], str]:
        now = time.monotonic()
        with self.lock:
            if not self.steps:
                return dict(STOP_PAYLOAD), self.last_error or "map unavailable"
            if not self.active:
                return dict(STOP_PAYLOAD), "map inactive"
            if self.paused:
                return dict(STOP_PAYLOAD), "map paused for safety"

            while self.active and now >= self.step_until:
                self.step_index += 1
                if self.step_index >= len(self.steps):
                    self.loop_index += 1
                    if self.max_loops != 0 and self.loop_index >= self.max_loops:
                        self.active = False
                        self.finished = True
                        return dict(STOP_PAYLOAD), "map complete"
                    self.step_index = 0
                self.step_started_at = now
                self.step_until = now + self._current_duration_unlocked()

            step = self.steps[self.step_index]
            payload = self._payload_for_step_unlocked(step)
            return payload, self._reason_for_step_unlocked(step, now)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            return self.snapshot_unlocked(now)

    def snapshot_unlocked(self, now: float) -> dict[str, Any]:
        current = self.steps[self.step_index] if self.steps and self.step_index < len(self.steps) else {}
        return {
            "path": str(self.map_path),
            "name": self.name,
            "description": self.description,
            "area_m2": self.area_m2,
            "active": self.active,
            "finished": self.finished,
            "paused": self.paused,
            "paused_for_s": round(max(0.0, now - self.paused_at), 3) if self.paused else 0.0,
            "speed_scale": self.speed_scale,
            "calibration": {
                "seconds_per_meter": self.seconds_per_meter,
                "ninety_degree_turn_s": self.ninety_degree_turn_s,
            },
            "loop": self.loop_index,
            "max_loops": self.max_loops,
            "step_index": self.step_index,
            "step_count": len(self.steps),
            "step_remaining_s": round(max(0.0, self.step_until - now), 3) if self.active else 0.0,
            "last_resume_progress_s": round(self.last_resume_progress_s, 3),
            "current_step": {
                "id": current.get("id", ""),
                "name": current.get("name", ""),
                "action": current.get("action", current.get("type", "")),
            },
            "last_error": self.last_error,
            "last_loaded_age_s": None if self.last_loaded_at <= 0 else round(now - self.last_loaded_at, 3),
        }

    def _current_duration_unlocked(self) -> float:
        if not self.steps:
            return 0.1
        step = self.steps[self.step_index]
        return max(0.05, safe_float(step.get("duration_s", step.get("duration", 0.5)), 0.5))

    def _payload_for_step_unlocked(self, step: dict[str, Any]) -> dict[str, Any]:
        action = str(step.get("action", step.get("type", "move"))).lower()
        if action in ("wait", "inspect", "stop", "pause"):
            return dict(STOP_PAYLOAD)

        if action in ("turn", "rotate"):
            speed = abs(safe_float(step.get("speed"), self.default_turn_speed)) * self.speed_scale
            direction = str(step.get("direction", "right")).lower()
            if direction in ("left", "ccw", "counterclockwise"):
                left = -speed
                right = speed
            else:
                left = speed
                right = -speed
            return {"T": 1, "L": round(clamp_speed(left), 3), "R": round(clamp_speed(right), 3)}

        speed = safe_float(step.get("speed"), self.default_speed)
        left = safe_float(step.get("left"), speed) * self.speed_scale
        right = safe_float(step.get("right"), speed) * self.speed_scale
        return {"T": 1, "L": round(clamp_speed(left), 3), "R": round(clamp_speed(right), 3)}

    def _reason_for_step_unlocked(self, step: dict[str, Any], now: float) -> str:
        label = str(step.get("name", step.get("id", f"step-{self.step_index + 1}")))
        loops = "inf" if self.max_loops == 0 else str(self.max_loops)
        remaining = max(0.0, self.step_until - now)
        return f"{self.name} loop {self.loop_index + 1}/{loops} step {self.step_index + 1}/{len(self.steps)} {label} {remaining:.1f}s"


class AvoidanceSupervisor:
    """Execute a time-balanced detour while the route clock is frozen.

    The rover has one forward ultrasonic sensor and no pan mechanism.  A pair
    of small calibrated chassis turns therefore turns that sensor into a left/right
    scan.  The return leg mirrors the selected lateral move so the vehicle
    reaches the original patrol line before MapBrain resumes its frozen step.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        self.default_turn_speed = clamp_speed(getattr(args, "avoid_turn_speed", 0.50))
        self.default_turn_s = max(0.10, float(getattr(args, "avoid_turn_s", 0.686)))
        self.side_m = max(0.20, float(getattr(args, "avoid_side_m", 0.65)))
        self.pass_m = max(0.30, float(getattr(args, "avoid_pass_m", 1.20)))
        self.backtrack_m = max(0.05, float(getattr(args, "avoid_backtrack_m", 0.15)))
        self.scan_angle_deg = max(25.0, min(60.0, float(getattr(args, "avoid_scan_angle_deg", 45.0))))
        self.scan_settle_s = max(0.20, float(getattr(args, "avoid_scan_settle_s", 0.40)))
        self.scan_timeout_s = max(self.scan_settle_s + 0.20, float(getattr(args, "avoid_scan_timeout_s", 1.50)))
        self.scan_samples_required = max(1, min(4, int(getattr(args, "avoid_scan_samples", 2))))
        self.max_expansions = max(0, int(getattr(args, "avoid_max_expansions", 2)))
        self.next_scan_side = "left"
        self.next_preferred_side = "left"
        self.reset("idle")

    def reset(self, reason: str = "reset") -> None:
        self.active = False
        self.phase = "idle"
        self.phase_until = 0.0
        self.started_at = 0.0
        self.completed_at = 0.0
        self.last_reason = reason
        self.selected_side = ""
        self.right_scan_mm = 0
        self.left_scan_mm = 0
        self.drive_speed = 0.18
        self.turn_speed = self.default_turn_speed
        self.turn_s = self.default_turn_s
        self.scan_turn_s = self.default_turn_s * self.scan_angle_deg / 90.0
        self.seconds_per_meter = 2.80
        self.backtrack_s = 0.42
        self.side_total_s = 0.0
        self.pass_s = 3.36
        self.projected_route_progress_s = 0.0
        self.expand_s = 0.98
        self.expansions = 0
        self.last_camera: dict[str, Any] = {"available": False, "person_detected": False}
        self.scan_first_side = ""
        self.scan_second_side = ""
        self.measurement_reference_sequence = 0
        self.measurement_ready_at = 0.0
        self.last_fresh_measurement_mm = 0
        self.last_fresh_measurement_samples = 0

    def start(self, now: float, map_brain: MapBrain | None) -> None:
        calibration = map_brain.avoidance_calibration() if map_brain is not None else {}
        self.seconds_per_meter = max(0.25, safe_float(calibration.get("seconds_per_meter"), 2.80))
        self.turn_s = max(0.10, safe_float(calibration.get("ninety_degree_turn_s"), self.default_turn_s))
        self.scan_turn_s = self.turn_s * self.scan_angle_deg / 90.0
        self.drive_speed = clamp_speed(safe_float(calibration.get("linear_speed"), 0.18))
        self.turn_speed = self.default_turn_speed
        self.backtrack_s = max(0.18, self.backtrack_m * self.seconds_per_meter)
        self.side_total_s = self.side_m * self.seconds_per_meter
        self.expand_s = max(0.18, min(self.side_total_s, 0.35 * self.seconds_per_meter))
        # Include the initial reverse distance so the rejoined route position
        # remains ahead of the obstacle instead of returning to its front face.
        self.pass_s = (self.pass_m + self.backtrack_m) * self.seconds_per_meter
        # Net displacement projected onto the original route is pass_m:
        # the extra forward backtrack_m merely cancels the initial reverse.
        self.projected_route_progress_s = self.pass_m * self.seconds_per_meter
        self.active = True
        self.phase = "halt"
        self.phase_until = now + 0.35
        self.started_at = now
        self.completed_at = 0.0
        self.last_reason = "obstacle confirmed; stopping before scan"
        self.selected_side = ""
        self.right_scan_mm = 0
        self.left_scan_mm = 0
        self.expansions = 0
        self.scan_first_side = self.next_scan_side
        self.scan_second_side = self._opposite_side(self.scan_first_side)
        self.next_scan_side = self.scan_second_side

    def snapshot(self, now: float) -> dict[str, Any]:
        return {
            "active": self.active,
            "phase": self.phase,
            "phase_remaining_s": round(max(0.0, self.phase_until - now), 3) if self.active else 0.0,
            "selected_side": self.selected_side,
            "right_scan_mm": self.right_scan_mm,
            "left_scan_mm": self.left_scan_mm,
            "scan_first_side": self.scan_first_side,
            "scan_second_side": self.scan_second_side,
            "fresh_measurement_mm": self.last_fresh_measurement_mm,
            "fresh_measurement_samples": self.last_fresh_measurement_samples,
            "measurement_reference_sequence": self.measurement_reference_sequence,
            "expansions": self.expansions,
            "max_expansions": self.max_expansions,
            "last_reason": self.last_reason,
            "camera": dict(self.last_camera),
            "elapsed_s": round(max(0.0, now - self.started_at), 3) if self.started_at else 0.0,
            "calibration": {
                "seconds_per_meter": round(self.seconds_per_meter, 3),
                "turn_s": round(self.turn_s, 3),
                "scan_angle_deg": round(self.scan_angle_deg, 1),
                "scan_turn_s": round(self.scan_turn_s, 3),
                "scan_settle_s": round(self.scan_settle_s, 3),
                "scan_timeout_s": round(self.scan_timeout_s, 3),
                "scan_samples_required": self.scan_samples_required,
                "drive_speed": round(self.drive_speed, 3),
                "turn_speed": round(self.turn_speed, 3),
                "side_total_s": round(self.side_total_s, 3),
                "pass_s": round(self.pass_s, 3),
                "projected_route_progress_s": round(self.projected_route_progress_s, 3),
            },
        }

    def _set_phase(self, phase: str, now: float, duration_s: float, reason: str) -> None:
        self.phase = phase
        self.phase_until = now + max(0.0, duration_s)
        self.last_reason = reason

    def _begin_fresh_measurement_wait(self, phase: str, now: float, safety: dict[str, Any], reason: str) -> None:
        """Wait for samples that were physically captured after the turn stopped."""

        self.measurement_reference_sequence = max(0, int(safe_float(safety.get("sequence"), 0.0)))
        self.measurement_ready_at = now + self.scan_settle_s
        self.last_fresh_measurement_mm = 0
        self.last_fresh_measurement_samples = 0
        self._set_phase(phase, now, self.scan_timeout_s, reason)

    def _fresh_measurement(
        self,
        now: float,
        safety: dict[str, Any],
        range_guard: RangeSafetyGuard | None,
    ) -> int | None:
        if now < self.measurement_ready_at:
            return None

        samples: list[int]
        if range_guard is not None:
            samples = range_guard.fresh_samples_after(self.measurement_reference_sequence)
        elif "sequence" not in safety:
            # Unit tests and compatible external callers can still exercise the
            # state machine with one explicit stable measurement.
            samples = [int(safe_float(safety.get("filtered_mm"), 0.0))] * self.scan_samples_required
        elif int(safe_float(safety.get("sequence"), 0.0)) > self.measurement_reference_sequence:
            samples = [int(safe_float(safety.get("filtered_mm"), 0.0))]
        else:
            samples = []

        samples = [sample for sample in samples if sample > 0]
        self.last_fresh_measurement_samples = len(samples)
        if len(samples) < self.scan_samples_required:
            return None

        # The most recent samples belong to the current chassis orientation;
        # their median rejects a single HC-SR04 echo outlier without reusing a
        # pre-turn distance.
        recent = sorted(samples[-self.scan_samples_required:])
        measurement = recent[len(recent) // 2]
        self.last_fresh_measurement_mm = measurement
        return measurement

    def _fresh_measurement_or_timeout(
        self,
        now: float,
        safety: dict[str, Any],
        range_guard: RangeSafetyGuard | None,
        label: str,
    ) -> tuple[int | None, tuple[dict[str, Any], str, bool] | None]:
        measurement = self._fresh_measurement(now, safety, range_guard)
        if measurement is not None:
            return measurement, None
        if now >= self.phase_until:
            return None, self._fail(f"{label} timed out waiting for fresh range samples")
        return None, (dict(STOP_PAYLOAD), self.last_reason, False)

    def _turn_payload(self, direction: str) -> dict[str, Any]:
        speed = self.turn_speed
        if direction == "left":
            return {"T": 1, "L": round(-speed, 3), "R": round(speed, 3)}
        return {"T": 1, "L": round(speed, 3), "R": round(-speed, 3)}

    def _forward_payload(self, reverse: bool = False) -> dict[str, Any]:
        speed = -self.drive_speed if reverse else self.drive_speed
        return {"T": 1, "L": round(speed, 3), "R": round(speed, 3)}

    @staticmethod
    def _side_turn(side: str) -> str:
        return "right" if side == "right" else "left"

    @staticmethod
    def _opposite_side(side: str) -> str:
        return "left" if side == "right" else "right"

    @staticmethod
    def _opposite_turn(side: str) -> str:
        return "left" if side == "right" else "right"

    @staticmethod
    def _is_blocking(safety: dict[str, Any]) -> bool:
        return bool(safety.get("blocking", False))

    def _fail(self, reason: str) -> tuple[dict[str, Any], str, bool]:
        self.phase = "manual_required"
        self.phase_until = 0.0
        self.last_reason = reason
        return dict(STOP_PAYLOAD), reason, False

    def next_payload(
        self,
        now: float,
        safety: dict[str, Any],
        camera: dict[str, Any],
        range_guard: RangeSafetyGuard | None = None,
    ) -> tuple[dict[str, Any], str, bool]:
        """Return a safe detour command, state text, and completion flag."""

        if not self.active:
            return dict(STOP_PAYLOAD), "avoidance inactive", False

        self.last_camera = dict(camera)

        if safety.get("state") in ("sensor_wait", "sensor_stale"):
            return self._fail(f"range sensor unavailable: {safety.get('state')}")

        if self.phase in ("manual_required", "sensor_fault"):
            return dict(STOP_PAYLOAD), self.last_reason, False

        # A new obstacle while travelling on a detour is not safely solvable
        # with a single forward range sensor.  Stop and expose the exact state
        # instead of guessing a second manoeuvre into a possible collision.
        if self.phase in ("lateral_out", "expand_out", "pass_forward", "return_lateral") and self._is_blocking(safety):
            return self._fail(f"detour path blocked during {self.phase}")

        for _ in range(3):
            if self.phase == "halt":
                if now < self.phase_until:
                    return dict(STOP_PAYLOAD), self.last_reason, False
                self._set_phase("backtrack", now, self.backtrack_s, "backtracking before side scan")
                continue
            if self.phase == "backtrack":
                if now < self.phase_until:
                    return self._forward_payload(reverse=True), self.last_reason, False
                self._set_phase(
                    "scan_first_turn",
                    now,
                    self.scan_turn_s,
                    f"turning {self.scan_first_side} {self.scan_angle_deg:.0f} degrees for ultrasonic scan",
                )
                continue
            if self.phase == "scan_first_turn":
                if now < self.phase_until:
                    return self._turn_payload(self._side_turn(self.scan_first_side)), self.last_reason, False
                self._begin_fresh_measurement_wait(
                    "scan_first_wait", now, safety, f"sampling fresh {self.scan_first_side} clearance"
                )
                continue
            if self.phase == "scan_first_wait":
                measurement, pending = self._fresh_measurement_or_timeout(
                    now, safety, range_guard, f"{self.scan_first_side} scan"
                )
                if pending is not None:
                    return pending
                if self.scan_first_side == "right":
                    self.right_scan_mm = measurement or 0
                else:
                    self.left_scan_mm = measurement or 0
                self._set_phase(
                    "scan_second_turn",
                    now,
                    self.scan_turn_s * 2.0,
                    f"turning {self.scan_second_side} {self.scan_angle_deg * 2.0:.0f} degrees for ultrasonic scan",
                )
                continue
            if self.phase == "scan_second_turn":
                if now < self.phase_until:
                    return self._turn_payload(self._side_turn(self.scan_second_side)), self.last_reason, False
                self._begin_fresh_measurement_wait(
                    "scan_second_wait", now, safety, f"sampling fresh {self.scan_second_side} clearance"
                )
                continue
            if self.phase == "scan_second_wait":
                measurement, pending = self._fresh_measurement_or_timeout(
                    now, safety, range_guard, f"{self.scan_second_side} scan"
                )
                if pending is not None:
                    return pending
                if self.scan_second_side == "right":
                    self.right_scan_mm = measurement or 0
                else:
                    self.left_scan_mm = measurement or 0
                required_mm = int(safety.get("caution_mm", 650))
                right_clear = self.right_scan_mm >= required_mm
                left_clear = self.left_scan_mm >= required_mm
                if not right_clear and not left_clear:
                    return self._fail("no side clearance; manual assistance required")
                if right_clear and left_clear and abs(self.right_scan_mm - self.left_scan_mm) < max(80, required_mm // 7):
                    self.selected_side = self.next_preferred_side
                elif right_clear and (not left_clear or self.right_scan_mm > self.left_scan_mm):
                    self.selected_side = "right"
                else:
                    self.selected_side = "left"
                self.next_preferred_side = self._opposite_side(self.selected_side)
                orient_turn_s = self.scan_turn_s if self.selected_side == self.scan_second_side else self.scan_turn_s * 3.0
                self._set_phase(
                    "orient_selected",
                    now,
                    orient_turn_s,
                    f"selecting wider {self.selected_side} passage",
                )
                continue
            if self.phase == "orient_selected":
                if now < self.phase_until:
                    return self._turn_payload(self._side_turn(self.selected_side)), self.last_reason, False
                self._begin_fresh_measurement_wait(
                    "side_confirm_wait", now, safety, f"confirming fresh {self.selected_side} passage"
                )
                continue
            if self.phase == "side_confirm_wait":
                measurement, pending = self._fresh_measurement_or_timeout(
                    now, safety, range_guard, f"{self.selected_side} confirmation"
                )
                if pending is not None:
                    return pending
                if (measurement or 0) < int(safety.get("caution_mm", 650)):
                    return self._fail("selected side passage is no longer clear")
                self._set_phase("lateral_out", now, self.side_total_s, "moving laterally around obstacle")
                continue
            if self.phase == "lateral_out":
                if now < self.phase_until:
                    return self._forward_payload(), self.last_reason, False
                self._set_phase(
                    "restore_heading",
                    now,
                    self.turn_s,
                    "turning back to original route heading",
                )
                continue
            if self.phase == "restore_heading":
                if now < self.phase_until:
                    return self._turn_payload(self._opposite_turn(self.selected_side)), self.last_reason, False
                self._begin_fresh_measurement_wait("pass_probe_wait", now, safety, "checking fresh forward passage")
                continue
            if self.phase == "pass_probe_wait":
                measurement, pending = self._fresh_measurement_or_timeout(now, safety, range_guard, "forward probe")
                if pending is not None:
                    return pending
                if (measurement or 0) < int(safety.get("caution_mm", 650)):
                    if self.expansions >= self.max_expansions:
                        return self._fail("forward passage remains blocked after lateral expansion")
                    self.expansions += 1
                    self._set_phase("expand_turn", now, self.turn_s, "widening lateral clearance")
                    continue
                self._set_phase("pass_forward", now, self.pass_s, "passing obstacle on selected side")
                continue
            if self.phase == "expand_turn":
                if now < self.phase_until:
                    return self._turn_payload(self._side_turn(self.selected_side)), self.last_reason, False
                self._set_phase("expand_out", now, self.expand_s, "increasing side clearance")
                continue
            if self.phase == "expand_out":
                if now < self.phase_until:
                    return self._forward_payload(), self.last_reason, False
                self.side_total_s += self.expand_s
                self._set_phase("expand_restore", now, self.turn_s, "restoring route heading after expansion")
                continue
            if self.phase == "expand_restore":
                if now < self.phase_until:
                    return self._turn_payload(self._opposite_turn(self.selected_side)), self.last_reason, False
                self._set_phase("pass_probe_wait", now, self.scan_settle_s, "rechecking forward passage")
                continue
            if self.phase == "pass_forward":
                if now < self.phase_until:
                    return self._forward_payload(), self.last_reason, False
                self._set_phase("return_turn", now, self.turn_s, "turning toward original patrol line")
                continue
            if self.phase == "return_turn":
                if now < self.phase_until:
                    return self._turn_payload(self._opposite_turn(self.selected_side)), self.last_reason, False
                self._set_phase("return_lateral", now, self.side_total_s, "returning to original patrol line")
                continue
            if self.phase == "return_lateral":
                if now < self.phase_until:
                    return self._forward_payload(), self.last_reason, False
                self._set_phase("final_heading", now, self.turn_s, "aligning with original route heading")
                continue
            if self.phase == "final_heading":
                if now < self.phase_until:
                    return self._turn_payload(self._side_turn(self.selected_side)), self.last_reason, False
                self.active = False
                self.phase = "complete"
                self.completed_at = now
                self.last_reason = "detour complete; original route line rejoined"
                return dict(STOP_PAYLOAD), self.last_reason, True
            return self._fail(f"unknown avoidance phase: {self.phase}")

        return dict(STOP_PAYLOAD), self.last_reason, False


class ArbiterState:
    def __init__(self, args: argparse.Namespace) -> None:
        now = time.monotonic()
        self.lock = threading.Lock()
        self.mode = args.start_mode
        self.last_source = "boot"
        self.last_reason = "startup"
        self.last_manual_at = 0.0
        self.last_ws63_line = ""
        self.last_sent_payload: dict[str, Any] = dict(STOP_PAYLOAD)
        self.last_sent_at = 0.0
        self.last_motion_was_nonzero = False
        self.force_stop = True
        self.forwarded = 0
        self.ignored = 0
        self.rover_feedback = 0
        self.auto_speed = clamp_speed(args.auto_speed)
        self.turn_speed = clamp_speed(args.turn_speed)
        self.square_forward = max(0.2, float(args.square_forward))
        self.square_turn = max(0.1, float(args.square_turn))
        self.auto_period = max(0.05, float(args.auto_period))
        self.manual_timeout = max(0.05, float(args.manual_timeout))
        self.stop_repeat = max(0.05, float(args.stop_repeat))
        self.range_safety = RangeSafetyGuard(
            args.avoid_emergency_mm,
            args.avoid_block_mm,
            args.avoid_caution_mm,
        )
        self.avoidance = AvoidanceSupervisor(args)
        self.auto_phase = "forward"
        self.auto_leg = 0
        self.auto_loop = 0
        self.auto_until = now + self.square_forward
        self.auto_last_emit = 0.0
        self.auto_clock_paused = False
        self.auto_clock_paused_at = 0.0
        self.voice = {
            "last_intent": "",
            "last_event": "",
            "last_transcript": "",
            "confidence": 0.0,
            "source": "",
            "last_action": "",
            "updated_at": 0.0,
            "accepted": 0,
            "rejected": 0,
        }
        self.voice_emergency = {
            "active": False,
            "sequence": 0,
            "event": "",
            "message": "",
            "transcript": "",
            "source": "",
            "raised_at": 0.0,
            "acknowledged_at": 0.0,
        }
        self.telemetry: dict[str, Any] = {
            "available": False,
            "source": "",
            "updated_at": 0.0,
            "uptime_ms": 0,
            "motor_ready": False,
            "moving": False,
            "environment_valid": False,
            "temperature_deci_c": 0,
            "humidity_deci_percent": 0,
            "obstacle_enabled": False,
            "obstacle_valid": False,
            "obstacle_blocked": False,
            "distance_mm": 0,
            "threshold_mm": 0,
            "obstacle_reason": 0,
            "alarm_flags": 0,
            "sample_count": 0,
            "environment_age_ms": 0,
            "obstacle_age_ms": 0,
        }
        self.camera_safety: dict[str, Any] = {
            "available": False,
            "person_detected": False,
            "alarm_active": False,
            "frames_total": 0,
            "age_s": None,
            "last_error": "",
            "status_url": "",
        }
        self.voice_announcements: deque[dict[str, Any]] = deque(maxlen=24)
        self.last_voice_announcement_at: dict[str, float] = {}
        self.next_voice_announcement_id = 1
        self.patrol_alert_seen = False
        self.patrol_episode = 0

    def set_mode(self, mode: str, reason: str) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            if mode not in (MODE_MANUAL, MODE_AUTO_SQUARE, MODE_AUTO_VISION, MODE_AUTO_MAP, MODE_ESTOP):
                raise ValueError(f"unsupported mode: {mode}")
            self.mode = mode
            self.last_source = "control"
            self.last_reason = reason
            if mode == MODE_AUTO_SQUARE:
                self.auto_phase = "forward"
                self.auto_leg = 0
                self.auto_until = now + self.square_forward
                self.auto_last_emit = 0.0
                self.auto_clock_paused = False
                self.auto_clock_paused_at = 0.0
                self.avoidance.reset("new square patrol")
            else:
                self.force_stop = True
                self.auto_clock_paused = False
                self.auto_clock_paused_at = 0.0
                self.avoidance.reset(f"mode changed to {mode}")
            return self.snapshot_locked(now)

    def note_rover_feedback(self) -> None:
        with self.lock:
            self.rover_feedback += 1

    def note_ignored(self) -> None:
        with self.lock:
            self.ignored += 1

    def note_manual(self, line: str, payload: dict[str, Any]) -> None:
        now = time.monotonic()
        with self.lock:
            self.last_manual_at = now
            self.last_ws63_line = line
            self.last_source = "ws63_manual"
            self.last_reason = "manual motion"
            self.last_sent_payload = payload
            self.last_sent_at = now
            self.last_motion_was_nonzero = is_nonzero_motion(payload)
            self.forwarded += 1

    def note_sent(self, payload: dict[str, Any], source: str, reason: str) -> None:
        now = time.monotonic()
        with self.lock:
            self.last_sent_payload = payload
            self.last_sent_at = now
            self.last_motion_was_nonzero = is_nonzero_motion(payload)
            self.last_source = source
            self.last_reason = reason
            self.forwarded += 1

    def note_voice(
        self,
        intent: str,
        event: str,
        transcript: str,
        confidence: float,
        source: str,
        action: str,
        accepted: bool,
    ) -> None:
        now = time.monotonic()
        with self.lock:
            self.voice.update(
                {
                    "last_intent": intent,
                    "last_event": event,
                    "last_transcript": transcript[:160],
                    "confidence": round(max(0.0, min(1.0, confidence)), 3),
                    "source": source[:80],
                    "last_action": action,
                    "updated_at": now,
                }
            )
            self.voice["accepted" if accepted else "rejected"] += 1

    def raise_voice_emergency(self, event: str, transcript: str, source: str) -> None:
        """Expose a safety voice event to mobile clients until it is confirmed."""

        message_by_event = {
            "help": "检测到紧急语音事件（求助/火情），巡检已停止，请人工接管",
            "alarm_sound": "检测到声学报警，巡检已停止，请人工接管",
            "impact": "检测到碰撞事件，巡检已停止，请人工接管",
        }
        now = time.monotonic()
        with self.lock:
            is_duplicate = (
                bool(self.voice_emergency["active"])
                and self.voice_emergency["event"] == event
                and (now - float(self.voice_emergency["raised_at"])) < 3.0
            )
            if is_duplicate:
                return
            self.voice_emergency.update(
                {
                    "active": True,
                    "sequence": int(self.voice_emergency["sequence"]) + 1,
                    "event": event,
                    "message": message_by_event.get(event, "检测到语音紧急事件，请人工接管"),
                    "transcript": transcript[:160],
                    "source": source[:80],
                    "raised_at": now,
                }
            )
            self.patrol_alert_seen = True

    def acknowledge_voice_emergency(self) -> dict[str, Any]:
        """Acknowledge mobile visibility without releasing any motion safety state."""

        now = time.monotonic()
        with self.lock:
            self.voice_emergency["active"] = False
            self.voice_emergency["acknowledged_at"] = now
            return dict(self.voice_emergency)

    def start_patrol_episode(self) -> None:
        """Reset the route-local abnormality state before a new map run."""

        with self.lock:
            self.patrol_episode += 1
            self.patrol_alert_seen = False

    def _queue_voice_announcement_locked(
        self,
        kind: str,
        message: str,
        *,
        cooldown_s: float | None = None,
        force: bool = False,
        telemetry: dict[str, Any] | None = None,
    ) -> bool:
        now = time.monotonic()
        cooldown = VOICE_ANNOUNCEMENT_COOLDOWNS.get(kind, 0.0) if cooldown_s is None else cooldown_s
        last_at = self.last_voice_announcement_at.get(kind, 0.0)
        if not force and cooldown > 0.0 and (now - last_at) < cooldown:
            return False
        announcement = {
            "id": self.next_voice_announcement_id,
            "kind": kind,
            "message": message,
            "created_at": now,
            "telemetry": dict(self.telemetry if telemetry is None else telemetry),
        }
        self.next_voice_announcement_id += 1
        self.last_voice_announcement_at[kind] = now
        self.voice_announcements.append(announcement)
        self.voice["last_announcement"] = kind
        self.voice["last_announcement_at"] = now
        return True

    def queue_voice_announcement(
        self,
        kind: str,
        message: str,
        *,
        cooldown_s: float | None = None,
        force: bool = False,
        telemetry: dict[str, Any] | None = None,
    ) -> bool:
        with self.lock:
            return self._queue_voice_announcement_locked(
                kind,
                message,
                cooldown_s=cooldown_s,
                force=force,
                telemetry=telemetry,
            )

    def pop_voice_announcements(self) -> list[dict[str, Any]]:
        """Return-and-clear prompts for the single ASRPRO UART writer."""

        with self.lock:
            now = time.monotonic()
            announcements = [
                announcement
                for announcement in self.voice_announcements
                if (now - float(announcement["created_at"])) <= VOICE_ANNOUNCEMENT_MAX_AGE_S
            ]
            self.voice_announcements.clear()
            return announcements

    def note_ws63_telemetry(self, telemetry: dict[str, Any], source: str) -> None:
        """Store WS63 monitor data and convert meaningful changes to speech."""

        now = time.monotonic()
        with self.lock:
            self.telemetry.update(telemetry)
            self.telemetry["available"] = True
            self.telemetry["source"] = source[:80]
            self.telemetry["updated_at"] = now
            # Use the incoming frame rather than the accumulated status object:
            # an environment-only frame must not be mistaken for a second copy
            # of the previous range sample.
            self.range_safety.observe(telemetry, now)
            alarm_flags = int(self.telemetry.get("alarm_flags", 0))
            temperature = int(self.telemetry.get("temperature_deci_c", 0))
            humidity = int(self.telemetry.get("humidity_deci_percent", 0))
            blocked = bool(self.telemetry.get("obstacle_blocked", False))

            temperature_alarm = bool(alarm_flags & ROBOT_ALARM_TEMP_HIGH) or temperature >= 350
            humidity_alarm = bool(alarm_flags & ROBOT_ALARM_HUMIDITY_HIGH) or humidity >= 850
            obstacle_alarm = bool(alarm_flags & ROBOT_ALARM_OBSTACLE_BLOCKED) or blocked

            if temperature_alarm:
                self.patrol_alert_seen = True
                self._queue_voice_announcement_locked("temperature_alarm", "temperature threshold exceeded")
            if humidity_alarm:
                self.patrol_alert_seen = True
                self._queue_voice_announcement_locked("humidity_alarm", "humidity threshold exceeded")
            # Avoidance speech is emitted only when the arbiter starts a
            # confirmed detour.  This prevents a raw WS63 threshold event from
            # announcing a manoeuvre that the Pi has not actually authorised.

    def manual_override_active_locked(self, now: float) -> bool:
        return (now - self.last_manual_at) < self.manual_timeout

    def pause_auto_clock_locked(self, now: float) -> None:
        if self.mode == MODE_AUTO_SQUARE and not self.auto_clock_paused:
            self.auto_clock_paused = True
            self.auto_clock_paused_at = now

    def resume_auto_clock_locked(self, now: float) -> None:
        if self.mode == MODE_AUTO_SQUARE and self.auto_clock_paused:
            self.auto_until += max(0.0, now - self.auto_clock_paused_at)
            self.auto_clock_paused = False
            self.auto_clock_paused_at = 0.0

    def next_auto_payload_locked(self, now: float) -> dict[str, Any]:
        if self.auto_clock_paused:
            return dict(STOP_PAYLOAD)
        if now >= self.auto_until:
            if self.auto_phase == "forward":
                self.auto_phase = "turn"
                self.auto_until = now + self.square_turn
            else:
                self.auto_phase = "forward"
                self.auto_leg = (self.auto_leg + 1) % 4
                if self.auto_leg == 0:
                    self.auto_loop += 1
                self.auto_until = now + self.square_forward

        if self.auto_phase == "forward":
            return {"T": 1, "L": round(self.auto_speed, 3), "R": round(self.auto_speed, 3)}
        return {"T": 1, "L": round(self.turn_speed, 3), "R": round(-self.turn_speed, 3)}

    def next_background_payload(
        self,
        vision: VisionTracker | None = None,
        map_brain: MapBrain | None = None,
        camera_safety: CameraSafetyMonitor | None = None,
    ) -> tuple[dict[str, Any] | None, str, str]:
        now = time.monotonic()
        with self.lock:
            if self.force_stop:
                self.force_stop = False
                return dict(STOP_PAYLOAD), self.mode, "forced stop"

            if self.mode == MODE_ESTOP:
                if (now - self.last_sent_at) >= self.stop_repeat or self.last_motion_was_nonzero:
                    return dict(STOP_PAYLOAD), "estop", "estop repeat stop"
                return None, "", ""

            if self.manual_override_active_locked(now):
                return None, "", ""

            safety = self.range_safety.decision(now)
            camera = camera_safety.snapshot() if camera_safety is not None else {"available": False, "person_detected": False}
            self.camera_safety = dict(camera)
            if self.mode in (MODE_AUTO_SQUARE, MODE_AUTO_VISION, MODE_AUTO_MAP):
                if self.avoidance.active:
                    payload, reason, completed = self.avoidance.next_payload(now, safety, camera, self.range_safety)
                    if completed:
                        if self.mode == MODE_AUTO_MAP and map_brain is not None:
                            map_brain.resume(self.avoidance.projected_route_progress_s)
                        self.resume_auto_clock_locked(now)
                        self._queue_voice_announcement_locked(
                            "obstacle_avoid",
                            "detour complete; patrol route resumed",
                            force=True,
                        )
                    return payload, "avoidance", reason

                if safety["blocking"]:
                    if self.mode == MODE_AUTO_MAP and map_brain is not None:
                        map_brain.pause()
                    self.pause_auto_clock_locked(now)
                    if safety["state"] in ("blocked", "emergency_stop"):
                        self.avoidance.start(now, map_brain)
                        self._queue_voice_announcement_locked(
                            "obstacle_avoid",
                            f"obstacle at {safety['filtered_mm']}mm; starting autonomous detour",
                            telemetry={
                                **self.telemetry,
                                "distance_mm": safety["filtered_mm"],
                                "threshold_mm": safety["block_mm"],
                                "measurement_age_ms": safety.get("measurement_age_ms"),
                            },
                        )
                        return dict(STOP_PAYLOAD), "avoidance", "obstacle confirmed; halt before detour"
                    if self.last_motion_was_nonzero or (now - self.last_sent_at) >= self.stop_repeat:
                        return dict(STOP_PAYLOAD), "range_safety", str(safety["state"])
                    return None, "", ""
                if self.mode == MODE_AUTO_MAP and map_brain is not None:
                    map_brain.resume()

            if self.mode == MODE_AUTO_SQUARE:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    payload = self.next_auto_payload_locked(now)
                    if safety["caution"]:
                        return scale_motion_payload(payload, 0.5), "range_safety", "caution"
                    return payload, "auto_square", self.auto_phase
                return None, "", ""

            if self.mode == MODE_AUTO_VISION:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    if vision is None:
                        return dict(STOP_PAYLOAD), "auto_vision", "vision unavailable"
                    payload, reason = vision.next_payload()
                    if safety["caution"]:
                        return scale_motion_payload(payload, 0.5), "range_safety", "caution"
                    return payload, "auto_vision", reason
                return None, "", ""

            if self.mode == MODE_AUTO_MAP:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    if map_brain is None:
                        return dict(STOP_PAYLOAD), "auto_map", "map brain unavailable"
                    payload, reason = map_brain.next_payload()
                    if reason in ("map complete", "map unavailable"):
                        self.mode = MODE_MANUAL
                        self.last_source = "auto_map"
                        self.last_reason = reason
                        if reason == "map complete":
                            if self.patrol_alert_seen:
                                self._queue_voice_announcement_locked(
                                    "patrol_complete_alert",
                                    "patrol completed with environment alerts",
                                    force=True,
                                )
                            else:
                                self._queue_voice_announcement_locked(
                                    "patrol_complete_normal",
                                    "patrol completed without environment alerts",
                                    force=True,
                                )
                    if safety["caution"]:
                        return scale_motion_payload(payload, 0.5), "range_safety", "caution"
                    return payload, "auto_map", reason
                return None, "", ""

            if self.last_motion_was_nonzero and (now - self.last_sent_at) >= self.stop_repeat:
                return dict(STOP_PAYLOAD), "idle", "manual timeout stop"
            return None, "", ""

    def snapshot_locked(self, now: float) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "last_source": self.last_source,
            "last_reason": self.last_reason,
            "last_manual_age_s": None if self.last_manual_at <= 0 else round(now - self.last_manual_at, 3),
            "last_sent_age_s": None if self.last_sent_at <= 0 else round(now - self.last_sent_at, 3),
            "last_sent_payload": self.last_sent_payload,
            "last_ws63_line": self.last_ws63_line,
            "forwarded": self.forwarded,
            "ignored": self.ignored,
            "rover_feedback": self.rover_feedback,
            "auto": {
                "speed": self.auto_speed,
                "turn_speed": self.turn_speed,
                "phase": self.auto_phase,
                "leg": self.auto_leg,
                "loop": self.auto_loop,
                "phase_remaining_s": round(max(0.0, self.auto_until - now), 3),
                "forward_s": self.square_forward,
                "turn_s": self.square_turn,
                "paused": self.auto_clock_paused,
                "paused_for_s": round(max(0.0, now - self.auto_clock_paused_at), 3)
                if self.auto_clock_paused
                else 0.0,
            },
            "voice": {
                **self.voice,
                "last_age_s": None if self.voice["updated_at"] <= 0 else round(now - self.voice["updated_at"], 3),
            },
            "voice_emergency": {
                **self.voice_emergency,
                "age_s": None
                if self.voice_emergency["raised_at"] <= 0
                else round(now - float(self.voice_emergency["raised_at"]), 3),
            },
            "telemetry": {
                **self.telemetry,
                "age_s": None
                if self.telemetry["updated_at"] <= 0
                else round(now - float(self.telemetry["updated_at"]), 3),
            },
            "range_safety": self.range_safety.decision(now),
            "camera_safety": dict(self.camera_safety),
            "avoidance": self.avoidance.snapshot(now),
            "voice_announcements": {
                "queued": len(self.voice_announcements),
                "patrol_episode": self.patrol_episode,
                "patrol_alert_seen": self.patrol_alert_seen,
            },
        }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            return self.snapshot_locked(now)


def apply_voice_intent(
    state: ArbiterState,
    map_brain: MapBrain | None,
    body: Any,
) -> tuple[int, dict[str, Any]]:
    """Apply an allow-listed voice request without bypassing WS63 safety rules."""

    if not isinstance(body, dict):
        return 400, {"ok": False, "error": "voice body must be an object"}

    intent = str(body.get("intent", "")).strip()
    event = str(body.get("event", "")).strip()
    transcript = str(body.get("transcript", "")).strip()
    source = str(body.get("source", "voice")).strip() or "voice"
    confidence = max(0.0, min(1.0, safe_float(body.get("confidence"), 0.0)))

    if intent not in VOICE_INTENTS and event not in VOICE_EVENTS_STOP:
        state.note_voice(intent, event, transcript, confidence, source, "unsupported", False)
        return 400, {"ok": False, "error": "unsupported voice intent or event", "state": state.snapshot()}
    if confidence < 0.72:
        state.note_voice(intent, event, transcript, confidence, source, "low_confidence", False)
        return 409, {"ok": False, "message": "voice confidence too low", "state": state.snapshot()}

    current_mode = state.snapshot()["mode"]
    if event in VOICE_EVENTS_STOP:
        if map_brain is not None:
            map_brain.stop()
        if current_mode != MODE_ESTOP:
            state.set_mode(MODE_MANUAL, f"voice safety event {event}")
            action = "auto patrol stopped; SLE manual control remains available"
        else:
            action = "estop remains locked; voice cannot release it"
        state.raise_voice_emergency(event, transcript, source)
        state.note_voice(intent, event, transcript, confidence, source, action, True)
        return 200, {
            "ok": True,
            "action": action,
            "message": "critical voice event handled",
            "state": state.snapshot(),
            "map_brain": map_brain.snapshot() if map_brain is not None else None,
        }

    if intent in (
        VOICE_INTENT_STATUS,
        VOICE_INTENT_ENVIRONMENT,
        VOICE_INTENT_DISTANCE,
        VOICE_INTENT_PATROL_REPORT,
        VOICE_INTENT_BATTERY,
    ):
        action_by_intent = {
            VOICE_INTENT_STATUS: "status reported",
            VOICE_INTENT_ENVIRONMENT: "environment reported",
            VOICE_INTENT_DISTANCE: "distance reported",
            VOICE_INTENT_PATROL_REPORT: "patrol reported",
            VOICE_INTENT_BATTERY: "battery reported",
        }
        action = action_by_intent[intent]
        state.note_voice(intent, event, transcript, confidence, source, action, True)
        telemetry = state.snapshot()["telemetry"]
        if intent == VOICE_INTENT_BATTERY:
            # The current chassis has no BMS/current-sense hardware.  Report
            # this explicitly instead of inventing a battery percentage.
            state.queue_voice_announcement(
                "battery_unavailable",
                "battery telemetry is not installed on this chassis",
                cooldown_s=1.0,
            )
        else:
            announcement_by_intent = {
                VOICE_INTENT_STATUS: "status_report",
                VOICE_INTENT_ENVIRONMENT: "environment_report",
                VOICE_INTENT_DISTANCE: "distance_report",
                # Route progress remains a stored prompt because it is not a
                # WS63 sensor value.  The map state is still present in the
                # returned HTTP payload for the mobile client.
                VOICE_INTENT_PATROL_REPORT: "status_detail",
            }
            if bool(telemetry.get("environment_valid", False)):
                temperature = int(telemetry.get("temperature_deci_c", 0)) / 10.0
                humidity = int(telemetry.get("humidity_deci_percent", 0)) / 10.0
                message = f"live telemetry temperature={temperature:.1f}C humidity={humidity:.1f}%"
            elif intent == VOICE_INTENT_DISTANCE and bool(telemetry.get("obstacle_valid", False)):
                message = f"live telemetry distance={int(telemetry.get('distance_mm', 0))}mm"
            else:
                message = "live telemetry is not available yet"
            state.queue_voice_announcement(
                announcement_by_intent[intent],
                message,
                cooldown_s=0.5,
            )
        return 200, {
            "ok": True,
            "action": action,
            "message": "live WS63 telemetry queued for the voice and HarmonyOS clients",
            "state": state.snapshot(),
            "map_brain": map_brain.snapshot() if map_brain is not None else None,
        }

    if intent == VOICE_INTENT_CLEAR_ALARM:
        action = "requires SLE confirmation"
        state.note_voice(intent, event, transcript, confidence, source, action, False)
        return 409, {
            "ok": False,
            "message": "voice cannot release an alarm or emergency lock; confirm on SLE",
            "state": state.snapshot(),
        }

    if current_mode == MODE_ESTOP:
        action = "blocked by estop"
        state.note_voice(intent, event, transcript, confidence, source, action, False)
        return 409, {
            "ok": False,
            "message": "estop is locked; release it on the SLE client first",
            "state": state.snapshot(),
        }

    if intent in (VOICE_INTENT_START, VOICE_INTENT_RESUME):
        if map_brain is None:
            action = "map brain unavailable"
            state.note_voice(intent, event, transcript, confidence, source, action, False)
            return 503, {"ok": False, "message": action, "state": state.snapshot()}
        map_payload = map_brain.configure_and_start({})
        if not map_payload["active"]:
            action = "route unavailable"
            state.note_voice(intent, event, transcript, confidence, source, action, False)
            return 409, {"ok": False, "message": action, "state": state.snapshot(), "map_brain": map_payload}
        state.start_patrol_episode()
        state.set_mode(MODE_AUTO_MAP, f"voice {intent}")
        action = "map patrol started" if intent == VOICE_INTENT_START else "map patrol resumed"
        state.note_voice(intent, event, transcript, confidence, source, action, True)
        return 200, {"ok": True, "action": action, "state": state.snapshot(), "map_brain": map_payload}

    if intent in (VOICE_INTENT_PAUSE, VOICE_INTENT_STOP):
        if map_brain is not None:
            map_brain.stop()
        state.set_mode(MODE_MANUAL, f"voice {intent}")
        action = "map patrol paused" if intent == VOICE_INTENT_PAUSE else "map patrol stopped"
        state.note_voice(intent, event, transcript, confidence, source, action, True)
        return 200, {
            "ok": True,
            "action": action,
            "state": state.snapshot(),
            "map_brain": map_brain.snapshot() if map_brain is not None else None,
        }

    state.note_voice(intent, event, transcript, confidence, source, "unsupported", False)
    return 400, {"ok": False, "error": "unsupported voice request", "state": state.snapshot()}


def make_http_handler(
    state: ArbiterState,
    vision: VisionTracker | None = None,
    map_brain: MapBrain | None = None,
) -> type[BaseHTTPRequestHandler]:
    class ControlHandler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: Any) -> None:
            return

        def send_json(self, status: int, payload: dict[str, Any]) -> None:
            body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 - stdlib callback name
            parsed = urlparse(self.path)
            if parsed.path in ("/", "/status"):
                payload = state.snapshot()
                if vision is not None:
                    payload["vision"] = vision.snapshot()
                if map_brain is not None:
                    payload["map_brain"] = map_brain.snapshot()
                self.send_json(200, payload)
                return
            if parsed.path == "/voice/announcements":
                self.send_json(200, {"announcements": state.pop_voice_announcements()})
                return
            if parsed.path == "/voice/latest":
                snapshot = state.snapshot()
                self.send_json(
                    200,
                    {
                        "voice": snapshot["voice"],
                        "emergency": snapshot["voice_emergency"],
                        "telemetry": snapshot["telemetry"],
                        "voice_announcements": snapshot["voice_announcements"],
                    },
                )
                return
            if parsed.path == "/route/current":
                if map_brain is None:
                    self.send_json(503, {"error": "map brain unavailable"})
                    return
                self.send_json(200, map_brain.current_route())
                return
            if parsed.path in ("/manual", "/auto/start", "/auto/vision", "/auto/map", "/auto/stop", "/estop", "/release"):
                self.handle_control(parsed.path, parse_qs(parsed.query))
                return
            self.send_json(404, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802 - stdlib callback name
            parsed = urlparse(self.path)
            length = int(self.headers.get("Content-Length", "0") or "0")
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                body = json.loads(raw.decode("utf-8")) if raw else {}
            except json.JSONDecodeError:
                self.send_json(400, {"error": "invalid json"})
                return
            if parsed.path == "/mode":
                mode = str(body.get("mode", ""))
                try:
                    self.send_json(200, state.set_mode(mode, "http mode"))
                except ValueError as exc:
                    self.send_json(400, {"error": str(exc)})
                return
            if parsed.path == "/voice/intent":
                status, payload = apply_voice_intent(state, map_brain, body)
                self.send_json(status, payload)
                return
            if parsed.path == "/voice/emergency/ack":
                self.send_json(
                    200,
                    {
                        "ok": True,
                        "message": "voice emergency acknowledged; motion remains stopped",
                        "emergency": state.acknowledge_voice_emergency(),
                    },
                )
                return
            if parsed.path == "/route/import":
                if map_brain is None:
                    self.send_json(503, {"error": "map brain unavailable"})
                    return
                ok, error, snapshot = map_brain.import_route(body)
                if ok:
                    state.set_mode(MODE_MANUAL, "route imported")
                    self.send_json(200, {"ok": True, "map_brain": snapshot})
                else:
                    self.send_json(400, {"ok": False, "error": error, "map_brain": snapshot})
                return
            if parsed.path == "/route/start":
                if map_brain is None:
                    self.send_json(503, {"error": "map brain unavailable"})
                    return
                map_payload = map_brain.configure_and_start(query_from_dict(body))
                state.start_patrol_episode()
                state_payload = state.set_mode(MODE_AUTO_MAP, "http route start")
                state_payload["map_brain"] = map_payload
                self.send_json(200, state_payload)
                return
            if parsed.path == "/route/stop":
                if map_brain is not None:
                    map_brain.stop()
                self.send_json(200, state.set_mode(MODE_MANUAL, "http route stop"))
                return
            self.send_json(404, {"error": "not found"})

        def handle_control(self, path: str, query: dict[str, list[str]]) -> None:
            if path in ("/manual", "/auto/stop", "/release"):
                if map_brain is not None:
                    map_brain.stop()
                self.send_json(200, state.set_mode(MODE_MANUAL, f"http {path}"))
                return
            if path == "/estop":
                if map_brain is not None:
                    map_brain.stop()
                self.send_json(200, state.set_mode(MODE_ESTOP, "http estop"))
                return
            if path == "/auto/start":
                if map_brain is not None:
                    map_brain.stop()
                with state.lock:
                    if "speed" in query:
                        state.auto_speed = clamp_speed(query["speed"][0])
                    if "turn" in query:
                        state.turn_speed = clamp_speed(query["turn"][0])
                    if "forward" in query:
                        state.square_forward = max(0.2, float(query["forward"][0]))
                    if "turn_s" in query:
                        state.square_turn = max(0.1, float(query["turn_s"][0]))
                self.send_json(200, state.set_mode(MODE_AUTO_SQUARE, "http auto start"))
                return
            if path == "/auto/vision":
                if map_brain is not None:
                    map_brain.stop()
                if vision is not None:
                    vision.configure(query)
                self.send_json(200, state.set_mode(MODE_AUTO_VISION, "http vision start"))
                return
            if path == "/auto/map":
                if map_brain is None:
                    self.send_json(503, {"error": "map brain unavailable"})
                    return
                map_payload = map_brain.configure_and_start(query)
                state.start_patrol_episode()
                state_payload = state.set_mode(MODE_AUTO_MAP, "http map start")
                state_payload["map_brain"] = map_payload
                self.send_json(200, state_payload)
                return
            self.send_json(404, {"error": "not found"})

    return ControlHandler


def start_http_server(
    state: ArbiterState,
    vision: VisionTracker | None,
    map_brain: MapBrain | None,
    host: str,
    port: int,
    stop_event: threading.Event,
) -> threading.Thread:
    server = ThreadingHTTPServer((host, port), make_http_handler(state, vision, map_brain))
    server.timeout = 0.5

    def run() -> None:
        print(f"[HTTP] listening on http://{host}:{port}/status", flush=True)
        while not stop_event.is_set():
            server.handle_request()
        server.server_close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def rover_reader(rover: serial.Serial, state: ArbiterState, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = rover.readline()
        except serial.SerialException as exc:
            print(f"[ROVER] read failed: {exc}", file=sys.stderr)
            stop_event.set()
            return
        if line:
            state.note_rover_feedback()
            print("[ROVER]", line.decode("utf-8", "replace").rstrip())


def write_rover(rover: serial.Serial, payload: dict[str, Any], source: str, reason: str, state: ArbiterState) -> bool:
    output_payload = adapt_rover_motion(payload)
    try:
        encoded = encode_payload(output_payload)
        rover.write(encoded)
        rover.flush()
    except serial.SerialException as exc:
        print(f"[ROVER] write failed: {exc}", file=sys.stderr)
        return False
    print(f"[ARBITER] {source} {reason} {encoded.decode('utf-8', 'replace').rstrip()}", flush=True)
    state.note_sent(output_payload, source, reason)
    return True


def parse_ws63_telemetry_line(line: str) -> dict[str, Any] | None:
    """Extract a monitor frame from raw WS63 debug/SLE output.

    The Type-C bridge carries both clean motion JSON and debug lines such as
    ``ROBOT SLE cmd=M response=+ROBOT:MON,...``.  Only the documented
    ``+ROBOT`` payload is trusted as telemetry; other logs remain diagnostic.
    """

    monitor = re.search(r"\+ROBOT:MON,([0-9,\-]+)", line)
    if monitor is not None:
        values = [int(value) for value in monitor.group(1).split(",")]
        if len(values) >= 16:
            return {
                "uptime_ms": values[0],
                "motor_ready": values[1] == 1,
                "moving": values[2] == 1,
                "environment_valid": values[3] == 1,
                "temperature_deci_c": values[4],
                "humidity_deci_percent": values[5],
                "obstacle_enabled": values[6] == 1,
                "obstacle_valid": values[7] == 1,
                "obstacle_blocked": values[8] == 1,
                "distance_mm": values[9],
                "threshold_mm": values[10],
                "obstacle_reason": values[11],
                "alarm_flags": values[12],
                "sample_count": values[13],
                "environment_age_ms": values[14],
                "obstacle_age_ms": values[15],
            }

    environment = re.search(r"\+ROBOT:ENV,([0-9]+),(-?[0-9]+),([0-9]+)", line)
    if environment is not None:
        return {
            "environment_valid": environment.group(1) == "1",
            "temperature_deci_c": int(environment.group(2)),
            "humidity_deci_percent": int(environment.group(3)),
        }

    obstacle = re.search(r"\+ROBOT:OBS,([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+)", line)
    if obstacle is not None:
        return {
            "obstacle_enabled": obstacle.group(1) == "1",
            "obstacle_valid": obstacle.group(2) == "1",
            "obstacle_blocked": obstacle.group(3) == "1",
            "distance_mm": int(obstacle.group(4)),
            "threshold_mm": int(obstacle.group(5)),
            "obstacle_reason": int(obstacle.group(6)),
        }

    patrol = re.search(r"\+ROBOT:PATROL,([0-9,]+)", line)
    if patrol is not None:
        values = [int(value) for value in patrol.group(1).split(",")]
        if len(values) >= 12:
            return {
                "alarm_flags": values[5],
                "obstacle_enabled": values[6] == 1,
                "obstacle_valid": values[7] == 1,
                "obstacle_blocked": values[8] == 1,
                "distance_mm": values[9],
                "threshold_mm": values[10],
                "obstacle_reason": values[11],
            }

    obstacle_stop = re.search(r"ROBOT OBSTACLE STOP distance=([0-9]+) threshold=([0-9]+)", line)
    if obstacle_stop is not None:
        return {
            "obstacle_enabled": True,
            "obstacle_valid": True,
            "obstacle_blocked": True,
            "distance_mm": int(obstacle_stop.group(1)),
            "threshold_mm": int(obstacle_stop.group(2)),
            "alarm_flags": ROBOT_ALARM_OBSTACLE_BLOCKED,
            "_range_event_only": True,
        }
    return None


def process_ws63_line(line: bytes, rover: serial.Serial, state: ArbiterState) -> bool:
    stripped = line.strip()
    if not stripped:
        return True
    line_text = stripped.decode("utf-8", "replace")
    telemetry = parse_ws63_telemetry_line(line_text)
    if telemetry is not None:
        state.note_ws63_telemetry(telemetry, "ws63 Type-C")
        return True
    try:
        payload = json.loads(line_text)
    except (UnicodeDecodeError, json.JSONDecodeError):
        state.note_ignored()
        return True
    if not isinstance(payload, dict):
        state.note_ignored()
        return True

    if payload.get("estop") is True or payload.get("mode") == MODE_ESTOP:
        state.set_mode(MODE_ESTOP, "ws63 estop")
        return write_rover(rover, dict(STOP_PAYLOAD), "estop", "ws63 estop", state)
    if payload.get("mode") in (MODE_MANUAL, MODE_AUTO_SQUARE, MODE_AUTO_VISION, MODE_AUTO_MAP):
        state.set_mode(str(payload["mode"]), "ws63 mode")
        return True

    motion = clean_motion_payload(payload)
    if motion is None:
        state.note_ignored()
        return True

    with state.lock:
        if state.mode == MODE_ESTOP:
            pass
        else:
            line_text = stripped.decode("utf-8", "replace")
            state.last_manual_at = time.monotonic()
            state.last_ws63_line = line_text

    if state.snapshot()["mode"] == MODE_ESTOP:
        return write_rover(rover, dict(STOP_PAYLOAD), "estop", "manual ignored", state)

    ok = write_rover(rover, motion, "ws63_manual", "manual motion", state)
    with state.lock:
        state.last_manual_at = time.monotonic()
        state.last_ws63_line = stripped.decode("utf-8", "replace")
    return ok


def main() -> int:
    args = parse_args()
    stop_event = threading.Event()
    state = ArbiterState(args)
    vision = VisionTracker(
        args.camera_stream_url,
        args.vision_speed,
        args.vision_gain,
        args.vision_min_area,
        args.vision_lost_timeout,
        args.vision_color,
    )
    camera_safety = CameraSafetyMonitor(args.camera_status_url or camera_status_url_from_stream(args.camera_stream_url))
    map_path = Path(args.map_path)
    if not map_path.is_absolute():
        map_path = Path(__file__).resolve().parent / map_path
    map_brain = MapBrain(str(map_path))

    def handle_signal(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    rover_port = resolve_rover_port(args.rover_port)
    ws63_port = resolve_ws63_port(args.ws63_port, rover_port)
    ws63 = open_serial(ws63_port, args.baudrate, args.read_timeout)
    rover = open_serial(rover_port, args.baudrate, args.read_timeout)

    reader_thread: threading.Thread | None = None
    if not args.quiet_rover:
        reader_thread = threading.Thread(target=rover_reader, args=(rover, state, stop_event), daemon=True)
        reader_thread.start()
    vision.start()
    camera_safety.start()
    http_thread = start_http_server(state, vision, map_brain, args.http_host, args.http_port, stop_event)

    print(
        f"[ARBITER] WS63 {ws63_port} -> WAVE ROVER {rover_port} @ {args.baudrate}. "
        f"mode={state.mode} manual_timeout={state.manual_timeout:.2f}s",
        flush=True,
    )
    write_rover(rover, dict(STOP_PAYLOAD), "startup", "safe stop", state)

    try:
        while not stop_event.is_set():
            try:
                line = ws63.readline()
            except serial.SerialException as exc:
                print(f"[WS63] read failed: {exc}", file=sys.stderr)
                return 2

            if line and not process_ws63_line(line, rover, state):
                return 3

            payload, source, reason = state.next_background_payload(vision, map_brain, camera_safety)
            if payload is not None and not write_rover(rover, payload, source, reason, state):
                return 4

    finally:
        write_rover(rover, dict(STOP_PAYLOAD), "shutdown", "safe stop", state)
        stop_event.set()
        vision.stop()
        camera_safety.stop()
        http_thread.join(timeout=1.0)
        if reader_thread is not None:
            reader_thread.join(timeout=0.5)
        ws63.close()
        rover.close()
        print(
            f"[ARBITER] closed forwarded={state.snapshot()['forwarded']} "
            f"ignored={state.snapshot()['ignored']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
