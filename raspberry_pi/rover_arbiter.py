from __future__ import annotations

import argparse
import glob
import json
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
STOP_PAYLOAD = {"T": 1, "L": 0.0, "R": 0.0}
ALLOWED_ROUTE_ACTIONS = {"move", "turn", "rotate", "wait", "inspect", "stop", "pause"}


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
        self.map_path = Path(map_path)
        self.lock = threading.Lock()
        self.name = ""
        self.description = ""
        self.area_m2 = 0.0
        self.default_speed = 0.16
        self.default_turn_speed = 0.16
        self.default_loops = 1
        self.steps: list[dict[str, Any]] = []
        self.active = False
        self.finished = False
        self.speed_scale = 1.0
        self.max_loops = 1
        self.step_index = 0
        self.loop_index = 0
        self.step_started_at = 0.0
        self.step_until = 0.0
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
            self.step_index = 0
            self.loop_index = 0
            self.step_started_at = now
            self.step_until = now + self._current_duration_unlocked()
            return self.snapshot_unlocked(now)

    def stop(self) -> None:
        with self.lock:
            self.active = False

    def next_payload(self) -> tuple[dict[str, Any], str]:
        now = time.monotonic()
        with self.lock:
            if not self.steps:
                return dict(STOP_PAYLOAD), self.last_error or "map unavailable"
            if not self.active:
                return dict(STOP_PAYLOAD), "map inactive"

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
            "speed_scale": self.speed_scale,
            "loop": self.loop_index,
            "max_loops": self.max_loops,
            "step_index": self.step_index,
            "step_count": len(self.steps),
            "step_remaining_s": round(max(0.0, self.step_until - now), 3) if self.active else 0.0,
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
        self.auto_phase = "forward"
        self.auto_leg = 0
        self.auto_loop = 0
        self.auto_until = now + self.square_forward
        self.auto_last_emit = 0.0

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
            else:
                self.force_stop = True
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

    def manual_override_active_locked(self, now: float) -> bool:
        return (now - self.last_manual_at) < self.manual_timeout

    def next_auto_payload_locked(self, now: float) -> dict[str, Any]:
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

            if self.mode == MODE_AUTO_SQUARE:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    return self.next_auto_payload_locked(now), "auto_square", self.auto_phase
                return None, "", ""

            if self.mode == MODE_AUTO_VISION:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    if vision is None:
                        return dict(STOP_PAYLOAD), "auto_vision", "vision unavailable"
                    payload, reason = vision.next_payload()
                    return payload, "auto_vision", reason
                return None, "", ""

            if self.mode == MODE_AUTO_MAP:
                if (now - self.auto_last_emit) >= self.auto_period:
                    self.auto_last_emit = now
                    if map_brain is None:
                        return dict(STOP_PAYLOAD), "auto_map", "map brain unavailable"
                    payload, reason = map_brain.next_payload()
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
            },
        }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            return self.snapshot_locked(now)


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
    try:
        encoded = encode_payload(payload)
        rover.write(encoded)
        rover.flush()
    except serial.SerialException as exc:
        print(f"[ROVER] write failed: {exc}", file=sys.stderr)
        return False
    print(f"[ARBITER] {source} {reason} {encoded.decode('utf-8', 'replace').rstrip()}", flush=True)
    state.note_sent(payload, source, reason)
    return True


def process_ws63_line(line: bytes, rover: serial.Serial, state: ArbiterState) -> bool:
    stripped = line.strip()
    if not stripped:
        return True
    try:
        payload = json.loads(stripped.decode("utf-8"))
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

            payload, source, reason = state.next_background_payload(vision, map_brain)
            if payload is not None and not write_rover(rover, payload, source, reason, state):
                return 4

    finally:
        write_rover(rover, dict(STOP_PAYLOAD), "shutdown", "safe stop", state)
        stop_event.set()
        vision.stop()
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
