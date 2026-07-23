from __future__ import annotations

import sys
import json
import threading
import unittest
import urllib.request
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "raspberry_pi"))

from detection.fire_smoke_detector import FireSmokeDetector, HazardDetection
from run import AiRuntimeConfig, AppServer, RequestHandler


class _ClearableStub:
    def __init__(self) -> None:
        self.cleared = False

    def clear(self) -> None:
        self.cleared = True


class _ResettableStub:
    def __init__(self) -> None:
        self.reset_called = False
        self.start_called = False
        self.cooldown_sec = 1.0

    def start(self) -> None:
        self.start_called = True

    def reset(self) -> None:
        self.reset_called = True


class _AlarmStub:
    def __init__(self) -> None:
        self.cleared = False

    def clear(self) -> None:
        self.cleared = True


class FireSmokeDetectorTests(unittest.TestCase):
    def test_clear_frame_has_no_fire_or_smoke(self) -> None:
        detector = FireSmokeDetector(cooldown_sec=0.0, model_path="")
        frame = np.zeros((120, 160, 3), dtype=np.uint8)

        result = detector.feed(frame, now=1.0)

        self.assertFalse(result.fire)
        self.assertFalse(result.smoke)
        self.assertFalse(result.active)

    def test_semantic_model_does_not_use_orange_pixels_as_fire(self) -> None:
        detector = FireSmokeDetector(cooldown_sec=0.0, model_path="")
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[30:70, 45:105] = (0, 140, 255)  # BGR orange flame-like region.

        result = detector.feed(frame, now=1.0)

        self.assertFalse(result.fire)
        self.assertFalse(result.smoke)

    def test_requires_consecutive_semantic_fire_detections(self) -> None:
        detector = FireSmokeDetector(cooldown_sec=0.0, consecutive_hits=2)
        detector._model_loaded = True
        detector._infer = lambda _frame: [HazardDetection("fire", 0.91, 25, 20, 40, 35)]  # type: ignore[method-assign]
        frame = np.zeros((120, 160, 3), dtype=np.uint8)

        first = detector.feed(frame, now=1.0)
        result = detector.feed(frame, now=2.0)

        self.assertFalse(first.fire)
        self.assertTrue(result.fire)
        self.assertEqual(len(result.detections), 1)
        self.assertEqual(result.detections[0].label, "fire")
        self.assertGreater(result.fire_area_ratio, 0.07)

    def test_confirms_smoke_and_draws_a_box(self) -> None:
        detector = FireSmokeDetector(cooldown_sec=0.0, consecutive_hits=2)
        detector._model_loaded = True
        detector._infer = lambda _frame: [HazardDetection("smoke", 0.88, 30, 25, 60, 45)]  # type: ignore[method-assign]
        frame = np.zeros((120, 160, 3), dtype=np.uint8)

        detector.feed(frame, now=1.0)
        result = detector.feed(frame, now=2.0)
        annotated = detector.annotate(frame.copy())

        self.assertTrue(result.smoke)
        self.assertGreater(np.count_nonzero(annotated), 0)

    def test_decodes_yolov5_smoke_and_fire_boxes(self) -> None:
        detector = FireSmokeDetector(cooldown_sec=0.0)
        output = np.zeros((1, 3, 7), dtype=np.float32)
        output[0, 0] = [80, 60, 40, 30, 0.9, 0.8, 0.1]
        output[0, 1] = [130, 100, 50, 50, 0.85, 0.1, 0.9]
        output[0, 2] = [132, 102, 50, 50, 0.8, 0.1, 0.85]

        detections = detector._decode(output, 1.0, 0, 0, 160, 120)

        self.assertEqual({detection.label for detection in detections}, {"smoke", "fire"})
        self.assertEqual(len(detections), 2)

    def test_ai_config_http_endpoint_updates_runtime_switches(self) -> None:
        server = AppServer(("127.0.0.1", 0), RequestHandler)
        detector = _ClearableStub()
        fire_smoke = _ResettableStub()
        alarm = _AlarmStub()
        server.ai_config = AiRuntimeConfig()
        server.detector = detector
        server.fire_smoke = fire_smoke
        server.alarm = alarm
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/ai/config", timeout=2) as response:
                current = json.loads(response.read().decode("utf-8"))
            self.assertFalse(current["ai"]["person_detection"])
            self.assertFalse(current["ai"]["fire_smoke_detection"])

            body = json.dumps({"person_detection": True, "fire_smoke_detection": True}).encode("utf-8")
            request = urllib.request.Request(
                f"http://127.0.0.1:{server.server_port}/ai/config",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=2) as response:
                updated = json.loads(response.read().decode("utf-8"))

            self.assertTrue(updated["ok"])
            self.assertTrue(updated["ai"]["person_detection"])
            self.assertTrue(updated["ai"]["fire_smoke_detection"])
            self.assertFalse(detector.cleared)
            self.assertTrue(fire_smoke.start_called)
            self.assertFalse(fire_smoke.reset_called)
            self.assertTrue(alarm.cleared)
        finally:
            server.shutdown()
            thread.join(timeout=2)
            server.server_close()


if __name__ == "__main__":
    unittest.main()
