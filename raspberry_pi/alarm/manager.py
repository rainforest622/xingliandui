from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np


class AlarmManager:
    def __init__(
        self,
        snapshot_dir: str = "snapshots",
        cooldown_sec: float = 5.0,
        max_history: int = 50,
    ):
        self._dir = Path(snapshot_dir)
        self._dir.mkdir(parents=True, exist_ok=True)
        self.cooldown_sec = cooldown_sec
        self.max_history = max_history
        self._lock = threading.Lock()
        self._active = False
        self._last_triggered: float = 0.0
        self._history: list[dict] = []
        self._load_history()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def clear(self) -> None:
        with self._lock:
            self._active = False

    def feed(
        self,
        alarm_active: bool,
        frame: np.ndarray | None = None,
        person_count: int = 0,
        kind: str = "person",
        detail: str = "",
    ) -> bool:
        now = time.monotonic()
        with self._lock:
            self._active = alarm_active
            if not alarm_active:
                return False
            if now - self._last_triggered < self.cooldown_sec:
                return False
            self._last_triggered = now

        if frame is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            filename = f"alarm_{ts}.jpg"
            path = self._dir / filename
            cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

            entry = {
                "time": ts,
                "filename": filename,
                "kind": kind,
                "detail": detail,
            }
            if person_count > 0:
                entry["person_count"] = person_count
            with self._lock:
                self._history.append(entry)
                if len(self._history) > self.max_history:
                    self._history = self._history[-self.max_history:]
                self._save_history()
            return True
        return False

    def get_alarms(self) -> list[dict]:
        with self._lock:
            return list(self._history)

    def _history_path(self) -> Path:
        return self._dir / "alarm_history.json"

    def _load_history(self) -> None:
        path = self._history_path()
        if path.exists():
            try:
                self._history = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                self._history = []

    def _save_history(self) -> None:
        try:
            self._history_path().write_text(
                json.dumps(self._history, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception:
            pass
