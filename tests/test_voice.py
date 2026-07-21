from __future__ import annotations

import argparse
import json
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "raspberry_pi"))

from rover_arbiter import (
    AvoidanceSupervisor,
    ArbiterState,
    MapBrain,
    MODE_AUTO_MAP,
    MODE_MANUAL,
    RangeSafetyGuard,
    adapt_rover_motion,
    apply_voice_intent,
    make_http_handler,
    parse_ws63_telemetry_line,
)
from voice.intents import (
    EVENT_HELP,
    INTENT_BATTERY_REPORT,
    INTENT_DISTANCE_REPORT,
    INTENT_ENVIRONMENT_REPORT,
    INTENT_PATROL_PAUSE,
    INTENT_PATROL_REPORT,
    INTENT_PATROL_START,
    INTENT_PATROL_STOP,
    infer_observation,
)
from voice.serial_module import (
    ASRPRO_DYNAMIC_DISTANCE,
    ASRPRO_DYNAMIC_ENVIRONMENT,
    ASRPRO_DYNAMIC_FRAME,
    ASRPRO_REPLY_BATTERY_UNAVAILABLE,
    ASRPRO_REPLY_CRITICAL,
    ASRPRO_REPLY_STARTED,
    ASRPRO_REPLY_STATUS_DETAIL,
    ASRPRO_REPLY_TEMP_ALARM,
    asrpro_announcement_code,
    asrpro_announcement_frames,
    asrpro_reply_code,
    parse_asrpro_byte,
    parse_module_line,
)
from voice_service import should_ignore_asrpro_event


def arbiter_args() -> argparse.Namespace:
    return argparse.Namespace(
        start_mode=MODE_MANUAL,
        auto_speed=0.18,
        turn_speed=0.18,
        square_forward=3.0,
        square_turn=0.8,
        auto_period=0.05,
        manual_timeout=0.35,
        stop_repeat=0.25,
        avoid_emergency_mm=120,
        avoid_block_mm=280,
        avoid_caution_mm=380,
        avoid_turn_speed=0.5,
        avoid_turn_s=0.686,
        avoid_side_m=0.65,
        avoid_pass_m=1.2,
        avoid_backtrack_m=0.15,
        avoid_scan_angle_deg=45.0,
        avoid_scan_settle_s=0.85,
        avoid_max_expansions=2,
    )


class VoiceIntentTests(unittest.TestCase):
    def test_manual_pivot_turn_uses_opposite_full_pwm(self) -> None:
        self.assertEqual(
            adapt_rover_motion({"T": 1, "L": -0.5, "R": 0.5}),
            {"T": 11, "L": -255, "R": 255},
        )
        self.assertEqual(
            adapt_rover_motion({"T": 1, "L": 0.5, "R": -0.5}),
            {"T": 11, "L": 255, "R": -255},
        )

    def test_reverse_avoidance_motion_uses_reverse_pwm(self) -> None:
        self.assertEqual(
            adapt_rover_motion({"T": 1, "L": -0.18, "R": -0.18}),
            {"T": 11, "L": -92, "R": -92},
        )

    def test_avoidance_supervisor_executes_symmetric_detour(self) -> None:
        supervisor = AvoidanceSupervisor(arbiter_args())
        clear = {"state": "clear", "blocking": False, "filtered_mm": 1200, "caution_mm": 650}
        supervisor.start(0.0, None)

        phases: list[str] = []
        pivot_commands: list[tuple[float, float]] = []
        now = 0.0
        complete = False
        for _ in range(32):
            payload, _reason, complete = supervisor.next_payload(now, clear, {"person_detected": False})
            phases.append(supervisor.phase)
            if float(payload["L"]) * float(payload["R"]) < 0:
                pivot_commands.append((float(payload["L"]), float(payload["R"])))
            if complete:
                self.assertEqual(payload, {"T": 1, "L": 0.0, "R": 0.0})
                break
            now = max(now + 0.01, supervisor.phase_until + 0.01)

        self.assertTrue(complete)
        self.assertEqual(supervisor.selected_side, "left")
        self.assertIn("left_turn_out", phases)
        self.assertIn("left_offset", phases)
        self.assertIn("right_turn_out", phases)
        self.assertIn("right_turn_return", phases)
        self.assertIn("return_left_offset", phases)
        self.assertIn("left_turn_restore", phases)
        self.assertEqual(pivot_commands, [(-0.5, 0.5), (0.5, -0.5), (0.5, -0.5), (-0.5, 0.5)])
        self.assertEqual(supervisor.snapshot(now)["strategy"], "fixed_left_box")
        self.assertEqual(supervisor.phase, "complete")

    def test_camera_person_is_not_an_avoidance_motion_gate(self) -> None:
        supervisor = AvoidanceSupervisor(arbiter_args())
        clear = {"state": "clear", "blocking": False, "filtered_mm": 1200, "caution_mm": 650}
        supervisor.start(0.0, None)
        payload, reason, complete = supervisor.next_payload(0.36, clear, {"person_detected": True})

        self.assertEqual(payload, {"T": 1, "L": -0.18, "R": -0.18})
        self.assertFalse(complete)
        self.assertEqual(supervisor.phase, "backtrack")
        self.assertIn("backtracking", reason)

    def test_fixed_left_detour_never_switches_to_right_side(self) -> None:
        supervisor = AvoidanceSupervisor(arbiter_args())
        clear = {"state": "clear", "blocking": False, "filtered_mm": 1200, "caution_mm": 380}

        def complete_detour(start_at: float) -> str:
            supervisor.start(start_at, None)
            now = start_at
            for _ in range(32):
                supervisor.next_payload(now, clear, {"person_detected": False})
                if supervisor.phase == "complete":
                    return supervisor.selected_side
                now = max(now + 0.01, supervisor.phase_until + 0.01)
            self.fail("avoidance did not complete")

        self.assertEqual(complete_detour(0.0), "left")
        self.assertEqual(complete_detour(20.0), "left")
        snapshot = supervisor.snapshot(20.0)
        self.assertEqual(snapshot["scan_first_side"], "left")
        self.assertEqual(snapshot["scan_second_side"], "")
        self.assertEqual(snapshot["right_scan_mm"], 0)

    def test_blocked_map_patrol_starts_detour_and_freezes_route(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "m1", "action": "move", "duration_s": 10.0}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            map_brain = MapBrain(str(route_path))
            map_brain.configure_and_start({})
            state.set_mode(MODE_AUTO_MAP, "test")
            for _ in range(3):
                state.note_ws63_telemetry(
                    {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200},
                    "test",
                )
            state.next_background_payload(map_brain=map_brain)
            for _ in range(3):
                state.note_ws63_telemetry(
                    {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 260},
                    "test",
                )

            payload, source, reason = state.next_background_payload(map_brain=map_brain)
            self.assertEqual(payload, {"T": 1, "L": 0.0, "R": 0.0})
            self.assertEqual(source, "avoidance")
            self.assertIn("halt", reason)
            self.assertTrue(map_brain.snapshot()["paused"])
            self.assertTrue(state.snapshot()["avoidance"]["active"])

    def test_completed_detour_resumes_same_route_step_with_remaining_time(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "original-leg", "action": "move", "duration_s": 10.0}]}),
                encoding="utf-8",
            )
            with patch("rover_arbiter.time.monotonic", return_value=100.0):
                state = ArbiterState(arbiter_args())
                map_brain = MapBrain(str(route_path))
                map_brain.configure_and_start({})
                state.set_mode(MODE_AUTO_MAP, "test")
                state.next_background_payload(map_brain=map_brain)  # consume forced stop
            with patch("rover_arbiter.time.monotonic", return_value=102.0):
                map_brain.pause()
                remaining_before = map_brain.snapshot()["step_remaining_s"]
                state.avoidance.start(102.0, map_brain)
            state.avoidance.phase = "left_turn_restore"
            state.avoidance.phase_until = 102.0
            with patch("rover_arbiter.time.monotonic", return_value=110.0):
                for _ in range(3):
                    state.note_ws63_telemetry(
                        {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200}, "test"
                    )
                payload, source, completed_reason = state.next_background_payload(map_brain=map_brain)
                resumed = map_brain.snapshot()

            self.assertEqual(payload, {"T": 1, "L": 0.0, "R": 0.0})
            self.assertEqual(source, "avoidance")
            self.assertIn("rejoined", completed_reason)
            self.assertFalse(resumed["paused"])
            self.assertEqual(resumed["step_index"], 0)
            self.assertEqual(resumed["current_step"]["id"], "original-leg")
            projected_s = state.avoidance.pass_m * state.avoidance.seconds_per_meter
            self.assertAlmostEqual(resumed["last_resume_progress_s"], projected_s, places=2)
            self.assertAlmostEqual(resumed["step_remaining_s"], remaining_before - projected_s, places=2)

    def test_camera_person_does_not_pause_map_patrol(self) -> None:
        class CameraStub:
            def __init__(self, person_detected: bool) -> None:
                self.person_detected = person_detected

            def snapshot(self) -> dict[str, object]:
                return {"available": True, "person_detected": self.person_detected}

        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "m1", "action": "move", "duration_s": 10.0}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            map_brain = MapBrain(str(route_path))
            map_brain.configure_and_start({})
            state.set_mode(MODE_AUTO_MAP, "test")
            for _ in range(3):
                state.note_ws63_telemetry(
                    {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200},
                    "test",
                )
            camera = CameraStub(True)
            state.next_background_payload(map_brain=map_brain, camera_safety=camera)
            payload, source, reason = state.next_background_payload(map_brain=map_brain, camera_safety=camera)
            self.assertEqual(payload, {"T": 1, "L": 0.16, "R": 0.16})
            self.assertEqual(source, "auto_map")
            self.assertIn("step", reason)
            self.assertFalse(map_brain.snapshot()["paused"])
            self.assertFalse(state.snapshot()["avoidance"]["active"])
            self.assertTrue(state.snapshot()["camera_safety"]["person_detected"])

    def test_range_safety_filters_outlier_and_rearms_after_clearance(self) -> None:
        guard = RangeSafetyGuard(emergency_mm=180, block_mm=400, caution_mm=650)
        guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200}, 1.0)
        guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 150}, 1.1)
        guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200}, 1.2)
        self.assertEqual(guard.decision(1.2)["state"], "clear")

        guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 320}, 1.3)
        guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 300}, 1.4)
        blocked = guard.decision(1.4)
        self.assertTrue(blocked["blocking"])
        self.assertEqual(blocked["state"], "blocked")

        for timestamp in (1.5, 1.6, 1.7, 1.8):
            guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1000}, timestamp)
        self.assertEqual(guard.decision(1.8)["state"], "clear")

    def test_range_safety_ignores_non_authoritative_obstacle_log(self) -> None:
        guard = RangeSafetyGuard(emergency_mm=120, block_mm=280, caution_mm=380)
        guard.observe(
            {
                "_range_event_only": True,
                "obstacle_enabled": True,
                "obstacle_valid": True,
                "obstacle_blocked": True,
                "distance_mm": 150,
            },
            1.0,
        )
        self.assertEqual(guard.decision(1.0)["state"], "sensor_wait")

    def test_left_clearance_waits_for_post_turn_range_samples(self) -> None:
        args = arbiter_args()
        args.avoid_scan_settle_s = 0.20
        args.avoid_scan_timeout_s = 1.0
        args.avoid_scan_samples = 2
        guard = RangeSafetyGuard(emergency_mm=120, block_mm=280, caution_mm=380)
        for timestamp in (0.0, 0.1, 0.2):
            guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200}, timestamp)

        supervisor = AvoidanceSupervisor(args)
        supervisor.start(0.0, None)
        safety = guard.decision(0.0)
        supervisor.next_payload(0.36, safety, {}, guard)
        supervisor.next_payload(0.80, safety, {}, guard)
        supervisor.next_payload(1.20, safety, {}, guard)

        # The pre-turn 1200 mm values must not be accepted as left clearance.
        payload, reason, complete = supervisor.next_payload(1.55, safety, {}, guard)
        self.assertEqual(payload, {"T": 1, "L": 0.0, "R": 0.0})
        self.assertFalse(complete)
        self.assertIn("sampling fresh", reason)
        self.assertEqual(supervisor.left_scan_mm, 0)

        for timestamp in (1.56, 1.66):
            guard.observe({"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 4000}, timestamp)
        supervisor.next_payload(1.76, guard.decision(1.76), {}, guard)
        self.assertEqual(supervisor.left_scan_mm, 4000)

    def test_chinese_command_matching(self) -> None:
        self.assertEqual(infer_observation("小车，开始自动巡检").intent, INTENT_PATROL_START)
        self.assertEqual(infer_observation("请暂停巡检").intent, INTENT_PATROL_PAUSE)
        self.assertEqual(infer_observation("小车停止").intent, INTENT_PATROL_STOP)
        self.assertEqual(infer_observation("有人吗，救命").event, EVENT_HELP)

    def test_uart_json_and_text_formats(self) -> None:
        json_result = parse_module_line('{"text":"开始巡检", "confidence":0.91}')
        self.assertEqual(json_result.intent, INTENT_PATROL_START)
        self.assertAlmostEqual(json_result.confidence, 0.91)
        text_result = parse_module_line("ASR: 停止巡检\r\n")
        self.assertEqual(text_result.intent, INTENT_PATROL_STOP)

    def test_asrpro_binary_protocol(self) -> None:
        start = parse_asrpro_byte(b"\xA1")
        self.assertEqual(start.intent, INTENT_PATROL_START)
        self.assertEqual(asrpro_reply_code(start, {"ok": True, "action": "map patrol started"}), ASRPRO_REPLY_STARTED)
        help_event = parse_asrpro_byte(0xA6)
        self.assertEqual(help_event.event, EVENT_HELP)
        self.assertEqual(asrpro_reply_code(help_event, {"ok": True}), ASRPRO_REPLY_CRITICAL)

    def test_asrpro_extended_queries_and_announcements(self) -> None:
        self.assertEqual(parse_asrpro_byte(0xA8).intent, INTENT_ENVIRONMENT_REPORT)
        self.assertEqual(parse_asrpro_byte(0xA9).intent, INTENT_DISTANCE_REPORT)
        self.assertEqual(parse_asrpro_byte(0xAA).intent, INTENT_PATROL_REPORT)
        self.assertEqual(parse_asrpro_byte(0xAB).intent, INTENT_BATTERY_REPORT)
        self.assertEqual(asrpro_announcement_code({"kind": "temperature_alarm"}), ASRPRO_REPLY_TEMP_ALARM)
        self.assertEqual(asrpro_announcement_code({"kind": "status_detail"}), ASRPRO_REPLY_STATUS_DETAIL)
        self.assertEqual(
            asrpro_announcement_code({"kind": "battery_unavailable"}),
            ASRPRO_REPLY_BATTERY_UNAVAILABLE,
        )

    def test_asrpro_live_telemetry_frames_preserve_deci_values(self) -> None:
        environment = asrpro_announcement_frames(
            {
                "kind": "environment_report",
                "telemetry": {
                    "environment_valid": True,
                    "temperature_deci_c": 238,
                    "humidity_deci_percent": 652,
                },
            }
        )
        self.assertEqual(len(environment), 1)
        self.assertEqual(environment[0][:2], bytes((ASRPRO_DYNAMIC_FRAME, ASRPRO_DYNAMIC_ENVIRONMENT)))
        self.assertEqual(environment[0][2:6], bytes((0x00, 0xEE, 0x02, 0x8C)))
        self.assertEqual(
            environment[0][-1],
            ASRPRO_DYNAMIC_FRAME ^ ASRPRO_DYNAMIC_ENVIRONMENT ^ 0x00 ^ 0xEE ^ 0x02 ^ 0x8C,
        )

        distance = asrpro_announcement_frames(
            {
                "kind": "distance_report",
                "telemetry": {"obstacle_valid": True, "distance_mm": 4000, "threshold_mm": 250},
            }
        )
        self.assertEqual(distance[0][:2], bytes((ASRPRO_DYNAMIC_FRAME, ASRPRO_DYNAMIC_DISTANCE)))
        self.assertEqual(distance[0][2:6], bytes((0x0F, 0xA0, 0x00, 0xFA)))

    def test_ws63_monitor_frame_creates_deduplicated_environment_alerts(self) -> None:
        telemetry = parse_ws63_telemetry_line(
            "ROBOT SLE cmd=M response=+ROBOT:MON,1234,1,1,1,360,860,1,1,1,180,250,0,28,9,100,80"
        )
        self.assertIsNotNone(telemetry)
        state = ArbiterState(arbiter_args())
        state.set_mode(MODE_AUTO_MAP, "test patrol")
        state.note_ws63_telemetry(telemetry or {}, "test")
        kinds = [item["kind"] for item in state.pop_voice_announcements()]
        self.assertEqual(kinds, ["temperature_alarm", "humidity_alarm"])
        state.note_ws63_telemetry(telemetry or {}, "test")
        self.assertEqual(state.pop_voice_announcements(), [])

    def test_voice_status_and_battery_queries_never_invent_battery_data(self) -> None:
        state = ArbiterState(arbiter_args())
        status, reply = apply_voice_intent(
            state,
            None,
            {"intent": INTENT_DISTANCE_REPORT, "transcript": "前方距离", "confidence": 0.99},
        )
        self.assertEqual(status, 200)
        self.assertEqual(reply["action"], "distance reported")
        self.assertEqual(state.pop_voice_announcements()[0]["kind"], "distance_report")

        status, reply = apply_voice_intent(
            state,
            None,
            {"intent": INTENT_BATTERY_REPORT, "transcript": "报告电量", "confidence": 0.99},
        )
        self.assertEqual(status, 200)
        self.assertEqual(reply["action"], "battery reported")
        self.assertEqual(state.pop_voice_announcements()[0]["kind"], "battery_unavailable")

    def test_unverified_asrpro_safety_event_is_quarantined(self) -> None:
        help_event = parse_asrpro_byte(0xA6)
        stop = parse_asrpro_byte(0xA4)
        self.assertTrue(should_ignore_asrpro_event(help_event, False))
        self.assertFalse(should_ignore_asrpro_event(help_event, True))
        self.assertFalse(should_ignore_asrpro_event(stop, False))

    def test_voice_start_uses_map_brain(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"name": "voice-test", "steps": [{"id": "m1", "action": "move", "duration_s": 1.0}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            for _ in range(3):
                state.note_ws63_telemetry(
                    {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200},
                    "test",
                )
            map_brain = MapBrain(str(route_path))
            status, reply = apply_voice_intent(
                state,
                map_brain,
                {"intent": INTENT_PATROL_START, "transcript": "开始巡检", "confidence": 0.95},
            )
            self.assertEqual(status, 200)
            self.assertTrue(reply["ok"])
            self.assertEqual(state.snapshot()["mode"], MODE_AUTO_MAP)
            self.assertTrue(map_brain.snapshot()["active"])

    def test_map_brain_restores_imported_route_after_restart(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            map_path = Path(temp_dir) / "patrol_map.json"
            active_path = Path(temp_dir) / "active_route.json"
            map_path.write_text(
                json.dumps({"name": "fallback", "steps": [{"id": "fallback", "action": "move", "duration_s": 1.0}]}),
                encoding="utf-8",
            )
            active_path.write_text(
                json.dumps({"name": "phone-route", "steps": [{"id": "phone", "action": "move", "duration_s": 2.8}]}),
                encoding="utf-8",
            )
            map_brain = MapBrain(str(map_path))
            self.assertEqual(map_brain.snapshot()["name"], "phone-route")
            self.assertEqual(map_brain.map_path, active_path)

    def test_avoidance_calibration_tracks_route_speed_scale(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps(
                    {
                        "default_speed": 0.24,
                        "calibration": {"seconds_per_meter": 2.8, "ninety_degree_turn_ms": 686},
                        "steps": [{"id": "m1", "action": "move", "duration_s": 1.0}],
                    }
                ),
                encoding="utf-8",
            )
            map_brain = MapBrain(str(route_path))
            map_brain.configure_and_start({"speed_scale": ["0.5"]})
            calibration = map_brain.avoidance_calibration()
            self.assertAlmostEqual(calibration["linear_speed"], 0.12)
            self.assertAlmostEqual(calibration["seconds_per_meter"], 5.6)

    def test_completed_map_patrol_returns_arbiter_to_manual(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "m1", "action": "move", "duration_s": 0.05}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            for _ in range(3):
                state.note_ws63_telemetry(
                    {"obstacle_enabled": True, "obstacle_valid": True, "distance_mm": 1200},
                    "test",
                )
            map_brain = MapBrain(str(route_path))
            map_brain.configure_and_start({})
            state.set_mode(MODE_AUTO_MAP, "test")
            state.next_background_payload(map_brain=map_brain)
            time.sleep(0.08)
            payload, source, reason = state.next_background_payload(map_brain=map_brain)
            self.assertEqual(payload, {"T": 1, "L": 0.0, "R": 0.0})
            self.assertEqual(source, "auto_map")
            self.assertEqual(reason, "map complete")
            self.assertEqual(state.snapshot()["mode"], MODE_MANUAL)
            self.assertEqual(state.pop_voice_announcements()[0]["kind"], "patrol_complete_normal")

    def test_voice_help_stops_auto_patrol_without_unlocking_estop(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "m1", "action": "move", "duration_s": 1.0}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            map_brain = MapBrain(str(route_path))
            map_brain.configure_and_start({})
            state.set_mode(MODE_AUTO_MAP, "test")
            status, reply = apply_voice_intent(
                state,
                map_brain,
                {"event": EVENT_HELP, "transcript": "救命", "confidence": 0.99},
            )
            self.assertEqual(status, 200)
            self.assertTrue(reply["ok"])
            self.assertEqual(state.snapshot()["mode"], MODE_MANUAL)
            self.assertFalse(map_brain.snapshot()["active"])

            emergency = state.snapshot()["voice_emergency"]
            self.assertTrue(emergency["active"])
            self.assertEqual(emergency["event"], EVENT_HELP)
            self.assertEqual(emergency["sequence"], 1)
            self.assertIn("人工接管", emergency["message"])

    def test_voice_latest_exposes_emergency_for_mobile_broadcast(self) -> None:
        state = ArbiterState(arbiter_args())
        status, reply = apply_voice_intent(
            state,
            None,
            {"event": EVENT_HELP, "transcript": "救命", "confidence": 0.99, "source": "test"},
        )
        self.assertEqual(status, 200)
        self.assertTrue(reply["ok"])

        server = ThreadingHTTPServer(("127.0.0.1", 0), make_http_handler(state))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/voice/latest", timeout=2) as response:
                latest = json.loads(response.read().decode("utf-8"))
            self.assertTrue(latest["emergency"]["active"])
            self.assertEqual(latest["emergency"]["event"], EVENT_HELP)
            self.assertEqual(latest["emergency"]["sequence"], 1)

            acknowledgement = urllib.request.Request(
                f"http://127.0.0.1:{server.server_port}/voice/emergency/ack",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(acknowledgement, timeout=2) as response:
                acknowledged = json.loads(response.read().decode("utf-8"))
            self.assertTrue(acknowledged["ok"])
            self.assertFalse(acknowledged["emergency"]["active"])
        finally:
            server.shutdown()
            thread.join(timeout=2)
            server.server_close()

    def test_http_voice_endpoint_starts_map_patrol(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            route_path = Path(temp_dir) / "route.json"
            route_path.write_text(
                json.dumps({"steps": [{"id": "m1", "action": "move", "duration_s": 1.0}]}),
                encoding="utf-8",
            )
            state = ArbiterState(arbiter_args())
            map_brain = MapBrain(str(route_path))
            server = ThreadingHTTPServer(("127.0.0.1", 0), make_http_handler(state, map_brain=map_brain))
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                payload = json.dumps(
                    {"intent": INTENT_PATROL_START, "transcript": "开始巡检", "confidence": 0.96}
                ).encode("utf-8")
                request = urllib.request.Request(
                    f"http://127.0.0.1:{server.server_port}/voice/intent",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(request, timeout=2) as response:
                    reply = json.loads(response.read().decode("utf-8"))
                self.assertTrue(reply["ok"])
                self.assertEqual(state.snapshot()["mode"], MODE_AUTO_MAP)
            finally:
                server.shutdown()
                thread.join(timeout=2)
                server.server_close()


if __name__ == "__main__":
    unittest.main()
