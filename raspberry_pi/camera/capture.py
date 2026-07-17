from __future__ import annotations

import threading
import time
from glob import glob
from typing import Any, Iterator

import cv2
import numpy as np

try:
    from picamera2 import Picamera2

    _HAS_PICAMERA2 = True
except ImportError:
    Picamera2 = None
    _HAS_PICAMERA2 = False


class CameraCapture:
    def __init__(
        self,
        camera_id: int = 0,
        width: int = 640,
        height: int = 480,
        target_fps: float = 15,
        synthetic: bool = False,
        backend: str = "auto",
    ):
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.backend = "synthetic" if synthetic else backend
        self.camera_id = camera_id
        self.active_backend = "idle"
        self.last_error = ""
        self._started_at = 0.0
        self._last_frame_at = 0.0
        self._frame: np.ndarray | None = None
        self._fps_actual: float = 0.0
        self._frames_total: int = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self.last_error = ""
        self.active_backend = "starting"
        self._started_at = time.monotonic()
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

    @property
    def last_frame_age_sec(self) -> float | None:
        with self._lock:
            last_frame_at = self._last_frame_at
        if last_frame_at <= 0:
            return None
        return max(0.0, time.monotonic() - last_frame_at)

    def wait_for_frame(self, timeout_sec: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            frame, _fps = self.read()
            if frame is not None:
                return True
            if self.active_backend == "error" and not self._running:
                return False
            time.sleep(0.05)
        return False

    def status(self) -> dict[str, object]:
        frame, fps = self.read()
        return {
            "camera_requested_backend": self.backend,
            "camera_backend": self.active_backend,
            "camera_error": self.last_error,
            "camera_id": self.camera_id,
            "width": self.width,
            "height": self.height,
            "fps_actual": fps,
            "fps_target": self.target_fps,
            "frames_total": self.frames_total,
            "frame_ready": frame is not None,
            "last_frame_age_sec": self.last_frame_age_sec,
        }

    def _run(self) -> None:
        started = time.monotonic()
        retry_delay = 2.0
        while self._running:
            tick_times: list[float] = []
            try:
                for frame in self._frame_source(started):
                    if not self._running:
                        break
                    now = time.monotonic()
                    tick_times.append(now)
                    tick_times = [t for t in tick_times if now - t <= 1.0]
                    with self._lock:
                        self._frame = frame
                        self._fps_actual = len(tick_times)
                        self._frames_total += 1
                        self._last_frame_at = now
                    if self.last_error:
                        self.last_error = ""
                if self._running:
                    self.last_error = "camera stream ended"
                    self.active_backend = "error"
                    with self._lock:
                        self._frame = None
                        self._fps_actual = 0.0
            except Exception as exc:
                self.last_error = str(exc)
                self.active_backend = "error"
                with self._lock:
                    self._frame = None
                    self._fps_actual = 0.0
                print(f"[camera] {self.last_error}; retrying in {retry_delay:.1f}s")
            if self._running:
                time.sleep(retry_delay)

    def _frame_source(self, started: float) -> Iterator[np.ndarray]:
        if self.backend == "synthetic":
            yield from self._synthetic_frames(started)
            return

        errors: list[str] = []
        if self.backend in ("auto", "picamera2") and _HAS_PICAMERA2:
            try:
                yield from self._picamera2_frames()
                return
            except Exception as exc:
                errors.append(f"picamera2 failed: {exc}")
                self.last_error = "; ".join(errors)
                if self.backend == "picamera2":
                    raise
        elif self.backend == "picamera2" and not _HAS_PICAMERA2:
            raise RuntimeError("picamera2 is not installed; run sudo apt install -y python3-picamera2")

        if self.backend in ("auto", "v4l2"):
            try:
                yield from self._v4l2_frames()
                return
            except Exception as exc:
                errors.append(f"v4l2 failed: {exc}")
                self.last_error = "; ".join(errors)
                if self.backend == "v4l2":
                    raise

        if errors:
            raise RuntimeError("; ".join(errors))
        raise RuntimeError(f"unsupported camera backend: {self.backend}")

    def _synthetic_frames(self, started: float) -> Iterator[np.ndarray]:
        self.active_backend = "synthetic"
        period = 1.0 / max(1, self.target_fps)
        while self._running:
            loop_start = time.monotonic()
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            t = time.monotonic() - started
            x = int(t * 60) % max(1, self.width)
            cv2.circle(frame, (x, self.height // 2), 24, (0, 180, 255), -1)
            cv2.putText(
                frame,
                "SYNTHETIC",
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (200, 200, 200),
                2,
            )
            yield frame
            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, period - elapsed))

    def _picamera2_frames(self) -> Iterator[np.ndarray]:
        if Picamera2 is None:
            raise RuntimeError("picamera2 is not available")

        self.active_backend = "picamera2"
        picam2: Any = Picamera2(self.camera_id)
        config = picam2.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            controls={"FrameRate": float(self.target_fps)},
        )
        picam2.configure(config)
        picam2.start()
        period = 1.0 / max(1, self.target_fps)
        try:
            while self._running:
                loop_start = time.monotonic()
                rgb = picam2.capture_array()
                if rgb.ndim != 3:
                    raise RuntimeError(f"unexpected picamera2 frame shape: {rgb.shape}")
                if rgb.shape[2] == 4:
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGR)
                else:
                    frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                yield frame
                elapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, period - elapsed))
        finally:
            try:
                picam2.stop()
            finally:
                close = getattr(picam2, "close", None)
                if callable(close):
                    close()

    def _v4l2_frames(self) -> Iterator[np.ndarray]:
        period = 1.0 / max(1, self.target_fps)
        candidates: list[int | str] = [self.camera_id]
        for device in sorted(glob("/dev/video*")):
            if device not in candidates:
                candidates.append(device)

        errors: list[str] = []
        for candidate in candidates:
            self.active_backend = f"v4l2:{candidate}"
            cap: cv2.VideoCapture | None = None
            try:
                cap = cv2.VideoCapture(candidate, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.target_fps)
                if not cap.isOpened():
                    errors.append(f"{candidate}: open failed")
                    continue

                failed_reads = 0
                while self._running:
                    loop_start = time.monotonic()
                    ok, frame = cap.read()
                    if not ok:
                        failed_reads += 1
                        if failed_reads >= 30:
                            raise RuntimeError(f"{candidate}: repeated empty frames")
                        time.sleep(0.01)
                        continue
                    failed_reads = 0
                    yield frame
                    elapsed = time.monotonic() - loop_start
                    time.sleep(max(0.0, period - elapsed))
                return
            except Exception as exc:
                errors.append(f"{candidate}: {exc}")
            finally:
                if cap is not None:
                    cap.release()

        raise RuntimeError("cannot open V4L2 camera; tried " + ", ".join(errors))
