from __future__ import annotations

import os
import threading
import time

try:
    import psutil

    _HAS_PSUTIL = True
except ImportError:
    _HAS_PSUTIL = False

_THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"


class MetricsCollector:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames_captured = 0
        self._frames_encoded = 0
        self._bytes_sent = 0
        self._bandwidth_window: list[tuple[float, int]] = []
        self._process = psutil.Process() if _HAS_PSUTIL else None

    def tick_captured(self) -> None:
        with self._lock:
            self._frames_captured += 1

    def tick_encoded(self, byte_count: int) -> None:
        now = time.monotonic()
        with self._lock:
            self._frames_encoded += 1
            self._bytes_sent += byte_count
            self._bandwidth_window.append((now, byte_count))
            self._bandwidth_window = [
                (ts, n) for (ts, n) in self._bandwidth_window
                if now - ts <= 1.0
            ]

    def snapshot(self) -> dict:
        with self._lock:
            captured = self._frames_captured
            encoded = self._frames_encoded
            drop_rate = (captured - encoded) / max(1, captured)
            now = time.monotonic()
            self._bandwidth_window = [
                (ts, n) for (ts, n) in self._bandwidth_window
                if now - ts <= 1.0
            ]
            bw_bytes = sum(n for _, n in self._bandwidth_window)
            cpu = 0.0
            if self._process is not None:
                try:
                    cpu = self._process.cpu_percent()
                except Exception:
                    cpu = 0.0
            temp_c = self._read_temp()
            return {
                "frames_captured": captured,
                "frames_encoded": encoded,
                "drop_rate": round(drop_rate, 4),
                "bandwidth_kbps": round(bw_bytes * 8 / 1000, 1),
                "cpu_percent": round(cpu, 1),
                "cpu_temp_c": temp_c,
            }

    @staticmethod
    def _read_temp() -> float | None:
        try:
            if os.path.exists(_THERMAL_PATH):
                millideg = int(open(_THERMAL_PATH).read().strip())
                return round(millideg / 1000.0, 1)
        except Exception:
            pass
        return None
