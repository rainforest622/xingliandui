from __future__ import annotations

import threading
import time

import cv2
import numpy as np


class CameraCapture:
    def __init__(
        self,
        camera_id: int = 0,
        width: int = 640,
        height: int = 480,
        target_fps: float = 15,
        synthetic: bool = False,
    ):
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.synthetic = synthetic
        self.camera_id = camera_id
        self._frame: np.ndarray | None = None
        self._fps_actual: float = 0.0
        self._frames_total: int = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)

    def read(self) -> tuple[np.ndarray | None, float]:
        with self._lock:
            frame = self._frame
            fps = self._fps_actual
        return frame, fps

    @property
    def frames_total(self) -> int:
        with self._lock:
            return self._frames_total

    def _run(self) -> None:
        cap: cv2.VideoCapture | None = None
        if not self.synthetic:
            cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            if not cap.isOpened():
                raise RuntimeError(f"cannot open camera {self.camera_id}")

        period = 1.0 / max(1, self.target_fps)
        tick_times: list[float] = []
        started = time.monotonic()

        try:
            while self._running:
                loop_start = time.monotonic()

                if self.synthetic:
                    frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                    t = time.monotonic() - started
                    x = int(t * 60) % max(1, self.width)
                    cv2.circle(frame, (x, self.height // 2), 24, (0, 180, 255), -1)
                    cv2.putText(
                        frame, "SYNTHETIC", (16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2,
                    )
                else:
                    assert cap is not None
                    ok, frame = cap.read()
                    if not ok:
                        time.sleep(0.01)
                        continue

                now = time.monotonic()
                tick_times.append(now)
                tick_times = [t for t in tick_times if now - t <= 1.0]
                fps = len(tick_times)

                with self._lock:
                    self._frame = frame
                    self._fps_actual = fps
                    self._frames_total += 1

                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, period - elapsed))
        finally:
            if cap is not None:
                cap.release()
