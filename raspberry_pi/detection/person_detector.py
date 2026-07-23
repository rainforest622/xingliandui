from __future__ import annotations

import os
import re
import threading
import time

import cv2
import numpy as np


MODEL_DIR = os.path.join(os.path.dirname(__file__), "..", "models")
PROTOTXT = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.prototxt")
PROTOTXT_PATCHED = os.path.join(MODEL_DIR, "MobileNetSSD_deploy_patched.prototxt")
CAFFEMODEL = os.path.join(MODEL_DIR, "MobileNetSSD_deploy.caffemodel")


def _patch_prototxt(src: str, dst: str) -> None:
    """Strip BatchNorm + Scale layers from a Caffe prototxt and rewire
    connections so that OpenCV >= 4.x can load the fused caffemodel."""
    with open(src) as f:
        text = f.read()

    # Find all BatchNorm and Scale layer names
    bn_names: set[str] = set()
    scale_names: set[str] = set()
    layer_blocks = re.split(r"(?=^layer\s*\{)", text, flags=re.MULTILINE)
    keep: list[str] = []
    for block in layer_blocks:
        m_type = re.search(r'type:\s*"(\w+)"', block)
        m_name = re.search(r'name:\s*"([^"]+)"', block)
        if not m_type or not m_name:
            keep.append(block)
            continue
        ltype = m_type.group(1)
        lname = m_name.group(1)
        if ltype == "BatchNorm":
            bn_names.add(lname)
            continue  # remove
        if ltype == "Scale":
            scale_names.add(lname)
            continue  # remove
        keep.append(block)

    text = "".join(keep)

    # Rewire: layers that took input from a removed BN/Scale now take
    # input from whatever that BN/Scale took input from. Example:
    #   Conv1 -> BN1 -> Scale1 -> ReLU1
    # becomes:
    #   Conv1 -> ReLU1
    removed = bn_names | scale_names
    for name in removed:
        # find the bottom that produced this layer
        pattern = rf'(bottom:\s*)"{re.escape(name)}"'
        # we need to figure out what the removed layer's bottom was.
        # Instead of tracing, just strip BN/Scale tops as bottoms.
        # The original text already had these references; we need to
        # find what the BN took as bottom and replace all references
        # to the BN/Scale tops with that bottom.
        pass  # done in a second pass below

    # Simpler approach: find each removed layer's bottom, then replace
    # all references to that layer's top with its bottom.
    original = text
    # Re-split to get original blocks for dependency tracking
    orig_blocks = re.split(r"(?=^layer\s*\{)", open(src).read(), flags=re.MULTILINE)
    rewrites: dict[str, str] = {}  # layer_name -> its bottom
    for block in orig_blocks:
        m_type = re.search(r'type:\s*"(\w+)"', block)
        m_name = re.search(r'name:\s*"([^"]+)"', block)
        m_bottom = re.search(r'bottom:\s*"([^"]+)"', block)
        if m_type and m_name and m_bottom:
            if m_type.group(1) in ("BatchNorm", "Scale"):
                rewrites[m_name.group(1)] = m_bottom.group(1)

    # Apply rewrites: replace references to removed tops with their bottoms
    # A removed layer might chain: Conv1 -> BN1 -> Scale1
    # So BN1's top = "conv1_bn", bottom = "conv1"
    #    Scale1's top = "conv1_scale", bottom = "conv1_bn"
    # We first map: BN1->conv1, Scale1->conv1_bn
    # Then resolve chains: Scale1->conv1_bn->conv1
    for _ in range(10):  # resolve chains (at most 3 deep)
        changed = False
        for name, bottom in list(rewrites.items()):
            if bottom in rewrites:
                rewrites[name] = rewrites[bottom]
                changed = True
        if not changed:
            break

    for old, new in rewrites.items():
        text = re.sub(rf'(bottom:\s*)"{re.escape(old)}"', rf'\1"{new}"', text)
        text = re.sub(rf'(top:\s*)"{re.escape(old)}"', rf'\1"{new}"', text)

    with open(dst, "w") as f:
        f.write(text)

# COCO classes MobileNet-SSD was trained on
_PERSON_CLASS = 15
_CONFIDENCE_THRESHOLD = 0.4


class PersonDetector:
    """Async person detector using OpenCV DNN MobileNet-SSD + motion trigger.

    Motion detection acts as a cheap gate: only run the full-frame SSD when
    the scene has changed, avoiding continuous high CPU load.
    """

    def __init__(
        self,
        cooldown_sec: float = 0.5,
        motion_threshold: float = 0.01,
    ):
        self._cooldown = cooldown_sec
        self._motion_threshold = motion_threshold
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._frame_seq = 0
        self._processed_seq = -1
        self._boxes: list[tuple[int, int, int, int]] = []
        self._present = False
        self._last_infer_time = 0.0
        self._running = False
        self._thread: threading.Thread | None = None
        # Motion detection state
        self._prev_gray: np.ndarray | None = None
        self._motion_detected = False
        self._net: cv2.dnn.Net | None = None
        self._model_loaded = False

    @property
    def last_boxes(self) -> list[tuple[int, int, int, int]]:
        with self._lock:
            return list(self._boxes)

    @property
    def last_present(self) -> bool:
        with self._lock:
            return self._present

    @property
    def model_loaded(self) -> bool:
        return self._model_loaded

    def clear(self) -> None:
        with self._lock:
            self._latest_frame = None
            self._boxes = []
            self._present = False

    def start(self) -> None:
        if self._running or self._model_loaded:
            return
        if not os.path.isfile(PROTOTXT) or not os.path.isfile(CAFFEMODEL):
            print(f"[detector] Model files not found in {MODEL_DIR}")
            print("[detector] Run: bash download_models.sh")
            print("[detector] Detection disabled — run with --no-detect to suppress")
            self._model_loaded = False
            return
        # Auto-patch prototxt for OpenCV 4.x compatibility (strip BN/Scale)
        if not os.path.isfile(PROTOTXT_PATCHED):
            try:
                _patch_prototxt(PROTOTXT, PROTOTXT_PATCHED)
                print("[detector] Patched prototxt for OpenCV 4.x")
            except Exception as e:
                print(f"[detector] Failed to patch prototxt: {e}")
                self._model_loaded = False
                return
        self._net = cv2.dnn.readNetFromCaffe(PROTOTXT_PATCHED, CAFFEMODEL)
        cv2.setNumThreads(1)  # single-core DNN to avoid under-voltage on Pi 5
        self._model_loaded = True
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2)

    def feed(self, frame: np.ndarray) -> None:
        if not self._model_loaded:
            return
        # Motion detection: run inline (cheap, ~0.5ms) to decide if we
        # should queue this frame for DNN inference.
        self._motion_detected = self._motion_check(frame)
        if self._motion_detected:
            with self._lock:
                self._latest_frame = frame  # reference only
                self._frame_seq += 1

    def _motion_check(self, frame: np.ndarray) -> bool:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        motion = False
        if self._prev_gray is not None:
            h, w = gray.shape[:2]
            delta = cv2.absdiff(self._prev_gray, gray)
            changed = cv2.countNonZero(cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1])
            motion = changed > (w * h * self._motion_threshold)
        self._prev_gray = gray
        return motion

    def _run(self) -> None:
        while self._running:
            with self._lock:
                seq = self._frame_seq
                if seq == self._processed_seq or self._latest_frame is None:
                    frame = None
                else:
                    frame = self._latest_frame
                    self._processed_seq = seq

            if frame is not None:
                now = time.monotonic()
                if now - self._last_infer_time >= self._cooldown:
                    time.sleep(0.03)  # let the CPU breathe before spike
                    boxes = self._infer(frame)
                    self._last_infer_time = time.monotonic()
                    with self._lock:
                        self._boxes = boxes
                        self._present = len(boxes) > 0
                    time.sleep(0.03)  # let the CPU breathe after spike

            time.sleep(0.15)

    def _infer(self, frame: np.ndarray) -> list[tuple[int, int, int, int]]:
        assert self._net is not None
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
        self._net.setInput(blob)
        detections = self._net.forward()

        boxes: list[tuple[int, int, int, int]] = []
        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence < _CONFIDENCE_THRESHOLD:
                continue
            class_id = int(detections[0, 0, i, 1])
            if class_id != _PERSON_CLASS:
                continue
            x1 = int(detections[0, 0, i, 3] * w)
            y1 = int(detections[0, 0, i, 4] * h)
            x2 = int(detections[0, 0, i, 5] * w)
            y2 = int(detections[0, 0, i, 6] * h)
            boxes.append((x1, y1, x2 - x1, y2 - y1))
        return boxes

    def annotate(self, frame: np.ndarray) -> np.ndarray:
        """Annotate in-place."""
        boxes, present = self.last_boxes, self.last_present
        color = (0, 0, 255) if present else (0, 180, 0)
        label = f"PERSON ({len(boxes)})" if present else ("CLEAR" if self.model_loaded else "NO-MODEL")
        cv2.putText(frame, label, (16, frame.shape[0] - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        return frame
