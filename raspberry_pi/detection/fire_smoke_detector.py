from __future__ import annotations

import os
import time
from dataclasses import dataclass

import cv2
import numpy as np


MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "models")
DEFAULT_MODEL_PATH = os.path.join(MODEL_DIR, "fire_smoke_yolov5s.onnx")
CLASS_NAMES = ("smoke", "fire")


@dataclass(frozen=True)
class HazardDetection:
    label: str
    confidence: float
    x: int
    y: int
    width: int
    height: int

    @property
    def area(self) -> int:
        return self.width * self.height

    def snapshot(self) -> dict[str, object]:
        return {
            "label": self.label,
            "confidence": round(self.confidence, 3),
            "box": {"x": self.x, "y": self.y, "width": self.width, "height": self.height},
        }


@dataclass(frozen=True)
class FireSmokeResult:
    fire: bool
    smoke: bool
    fire_area_ratio: float
    smoke_area_ratio: float
    checked_at: float
    detections: tuple[HazardDetection, ...] = ()

    @property
    def active(self) -> bool:
        return self.fire or self.smoke

    def snapshot(self) -> dict[str, object]:
        return {
            "fire_detected": self.fire,
            "smoke_detected": self.smoke,
            "fire_smoke_detected": self.active,
            "fire_area_ratio": round(self.fire_area_ratio, 4),
            "smoke_area_ratio": round(self.smoke_area_ratio, 4),
            "fire_smoke_boxes": [detection.snapshot() for detection in self.detections],
            "checked_at": self.checked_at,
        }


class FireSmokeDetector:
    """YOLOv5 fire/smoke detector backed by OpenCV DNN.

    The model performs semantic object detection rather than classifying colors.
    It runs at a throttled interval and requires consecutive detections to keep
    the Raspberry Pi responsive while rejecting single-frame false positives.
    """

    def __init__(
        self,
        cooldown_sec: float = 1.0,
        confidence_threshold: float = 0.45,
        nms_threshold: float = 0.45,
        consecutive_hits: int = 2,
        model_path: str = DEFAULT_MODEL_PATH,
        input_size: int = 416,
    ):
        self.cooldown_sec = max(0.25, cooldown_sec)
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.consecutive_hits = max(1, consecutive_hits)
        self.model_path = model_path
        self.input_size = input_size
        self._net: cv2.dnn.Net | None = None
        self._model_loaded = False
        self._model_error = ""
        self._last_checked_at = 0.0
        self._hits = {label: 0 for label in CLASS_NAMES}
        self._last = FireSmokeResult(False, False, 0.0, 0.0, 0.0)

    @property
    def model_loaded(self) -> bool:
        return self._model_loaded

    @property
    def model_error(self) -> str:
        return self._model_error

    @property
    def last_result(self) -> FireSmokeResult:
        return self._last

    def start(self) -> None:
        if self._model_loaded or self._model_error:
            return
        if not os.path.isfile(self.model_path):
            self._model_error = f"fire/smoke model not found: {self.model_path}"
            print(f"[fire-smoke] {self._model_error}")
            return
        try:
            cv2.setNumThreads(1)
            self._net = cv2.dnn.readNetFromONNX(self.model_path)
            self._model_loaded = True
            print(f"[fire-smoke] YOLO model loaded: {self.model_path}")
        except cv2.error as error:
            self._model_error = f"failed to load fire/smoke model: {error}"
            print(f"[fire-smoke] {self._model_error}")

    def reset(self) -> None:
        self._hits = {label: 0 for label in CLASS_NAMES}
        self._last = FireSmokeResult(False, False, 0.0, 0.0, time.monotonic())

    def feed(self, frame: np.ndarray, now: float | None = None) -> FireSmokeResult:
        now = time.monotonic() if now is None else now
        if now - self._last_checked_at < self.cooldown_sec:
            return self._last
        self._last_checked_at = now
        if not self._model_loaded:
            self.start()
        if not self._model_loaded or frame.size == 0:
            return self._last

        candidates = self._infer(frame)
        for label in CLASS_NAMES:
            self._hits[label] = self._hits[label] + 1 if any(
                detection.label == label for detection in candidates
            ) else 0
        confirmed = tuple(
            detection
            for detection in candidates
            if self._hits[detection.label] >= self.consecutive_hits
        )
        height, width = frame.shape[:2]
        total = float(max(1, height * width))
        fire_area = sum(detection.area for detection in confirmed if detection.label == "fire") / total
        smoke_area = sum(detection.area for detection in confirmed if detection.label == "smoke") / total
        self._last = FireSmokeResult(
            fire=any(detection.label == "fire" for detection in confirmed),
            smoke=any(detection.label == "smoke" for detection in confirmed),
            fire_area_ratio=float(fire_area),
            smoke_area_ratio=float(smoke_area),
            checked_at=now,
            detections=confirmed,
        )
        return self._last

    def _infer(self, frame: np.ndarray) -> list[HazardDetection]:
        assert self._net is not None
        source_height, source_width = frame.shape[:2]
        scale = min(self.input_size / source_width, self.input_size / source_height)
        resized_width = max(1, int(round(source_width * scale)))
        resized_height = max(1, int(round(source_height * scale)))
        resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        pad_x = (self.input_size - resized_width) // 2
        pad_y = (self.input_size - resized_height) // 2
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, (self.input_size, self.input_size), swapRB=True)
        self._net.setInput(blob)
        return self._decode(self._net.forward(), scale, pad_x, pad_y, source_width, source_height)

    def _decode(
        self,
        output: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
        source_width: int,
        source_height: int,
    ) -> list[HazardDetection]:
        rows = output[0] if output.ndim == 3 else output
        boxes_by_label: dict[str, list[list[int]]] = {label: [] for label in CLASS_NAMES}
        confidences_by_label: dict[str, list[float]] = {label: [] for label in CLASS_NAMES}
        for row in rows:
            if len(row) < 5 + len(CLASS_NAMES):
                continue
            objectness = float(row[4])
            if objectness <= 0.0:
                continue
            class_id = int(np.argmax(row[5:5 + len(CLASS_NAMES)]))
            confidence = objectness * float(row[5 + class_id])
            if confidence < self.confidence_threshold:
                continue
            label = CLASS_NAMES[class_id]
            center_x, center_y, box_width, box_height = (float(value) for value in row[:4])
            x1 = int(round((center_x - box_width / 2 - pad_x) / scale))
            y1 = int(round((center_y - box_height / 2 - pad_y) / scale))
            x2 = int(round((center_x + box_width / 2 - pad_x) / scale))
            y2 = int(round((center_y + box_height / 2 - pad_y) / scale))
            x1 = max(0, min(source_width - 1, x1))
            y1 = max(0, min(source_height - 1, y1))
            x2 = max(0, min(source_width, x2))
            y2 = max(0, min(source_height, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            boxes_by_label[label].append([x1, y1, x2 - x1, y2 - y1])
            confidences_by_label[label].append(confidence)

        detections: list[HazardDetection] = []
        for label in CLASS_NAMES:
            boxes = boxes_by_label[label]
            confidences = confidences_by_label[label]
            if not boxes:
                continue
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)
            for index in np.asarray(indices).reshape(-1):
                x, y, width, height = boxes[int(index)]
                detections.append(HazardDetection(label, confidences[int(index)], x, y, width, height))
        return detections

    def annotate(self, frame: np.ndarray) -> np.ndarray:
        result = self.last_result
        colors = {"fire": (0, 0, 255), "smoke": (180, 180, 40)}
        for detection in result.detections:
            color = colors[detection.label]
            cv2.rectangle(
                frame,
                (detection.x, detection.y),
                (detection.x + detection.width, detection.y + detection.height),
                color,
                2,
            )
            label = f"{detection.label.upper()} {detection.confidence:.0%}"
            cv2.putText(
                frame,
                label,
                (detection.x, max(20, detection.y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
        if not result.detections:
            label = "FIRE/SMOKE AI: CLEAR" if self.model_loaded else "FIRE/SMOKE AI: NO MODEL"
            color = (0, 180, 0) if self.model_loaded else (0, 165, 255)
            cv2.putText(frame, label, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        return frame
