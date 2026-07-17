import unittest

from upper_client.robot_client import (
    Result,
    parse_at_ack,
    parse_at_avoid,
    parse_at_env,
    parse_at_monitor,
    parse_at_motor,
    parse_at_obstacle,
    parse_at_oled,
    parse_at_patrol,
    parse_at_state,
    summarize,
)
from upper_client.robot_profile import (
    SLE_ASCII_COMMAND_KEYS,
    encode_sle_ascii_command,
    normalize_robot_key,
)
from upper_client.robot_protocol import (
    AckPacket,
    Command,
    CommandPacket,
    ProtocolError,
    StatePacket,
    Status,
    decode_ack,
    decode_command,
    decode_state,
    encode_ack,
    encode_command,
    encode_state,
)


class ProtocolTests(unittest.TestCase):
    def test_command_round_trip(self) -> None:
        packet = CommandPacket(Command.FORWARD, 45, 46, 7)
        self.assertEqual(decode_command(encode_command(packet)), packet)

    def test_ack_round_trip(self) -> None:
        packet = AckPacket(255, Status.OBSTACLE_STOP)
        self.assertEqual(decode_ack(encode_ack(packet)), packet)

    def test_state_round_trip(self) -> None:
        packet = StatePacket(9, 80, 55, 1234, 1)
        self.assertEqual(decode_state(encode_state(packet)), packet)

    def test_bad_checksum_is_rejected(self) -> None:
        raw = bytearray(encode_command(CommandPacket(Command.STOP, 0, 0, 1)))
        raw[-1] ^= 0x01
        with self.assertRaises(ProtocolError):
            decode_command(bytes(raw))

    def test_speed_above_100_is_rejected(self) -> None:
        with self.assertRaises(ProtocolError):
            encode_command(CommandPacket(Command.FORWARD, 101, 50, 1))

    def test_latency_summary(self) -> None:
        results = [
            Result(0, "F", Status.OK, 1.0),
            Result(1, "S", Status.OK, 2.0),
            Result(2, "F", Status.OK, 3.0),
            Result(3, "S", Status.OK, 4.0),
        ]
        summary = summarize(results)
        self.assertEqual(summary["mean_ms"], 2.5)
        self.assertEqual(summary["p95_ms"], 4.0)
        self.assertEqual(summary["p99_ms"], 4.0)
        self.assertEqual(summary["max_ms"], 4.0)

    def test_parse_at_ack(self) -> None:
        response = "AT+ROBOTF\r\n+ROBOT:ACK,2,1,5,0\r\nOK\r\n"
        sequence, status, moving = parse_at_ack(response)
        self.assertEqual(sequence, 2)
        self.assertEqual(status, Status.MOTOR_ERROR)
        self.assertFalse(moving)

    def test_parse_at_motor(self) -> None:
        self.assertTrue(parse_at_motor("AT+ROBOTMI\r\n+ROBOT:MOTOR,1\r\nOK\r\n"))

    def test_parse_at_oled(self) -> None:
        self.assertTrue(parse_at_oled("AT+ROBOTOLED\r\n+ROBOT:OLED,1\r\nOK\r\n"))

    def test_parse_at_env(self) -> None:
        response = "AT+ROBOTENV\r\n+ROBOT:ENV,1,253,582\r\nOK\r\n"
        ok, temperature_deci_c, humidity_deci_percent = parse_at_env(response)
        self.assertTrue(ok)
        self.assertEqual(temperature_deci_c, 253)
        self.assertEqual(humidity_deci_percent, 582)

    def test_parse_at_obstacle(self) -> None:
        response = "AT+ROBOTOBS\r\n+ROBOT:OBS,1,1,1,180,250\r\nOK\r\n"
        enabled, valid, blocked, distance_mm, threshold_mm, reason = parse_at_obstacle(response)
        self.assertTrue(enabled)
        self.assertTrue(valid)
        self.assertTrue(blocked)
        self.assertEqual(distance_mm, 180)
        self.assertEqual(threshold_mm, 250)
        self.assertIsNone(reason)

    def test_parse_at_obstacle_with_reason(self) -> None:
        response = "AT+ROBOTOBS\r\n+ROBOT:OBS,1,0,0,0,250,2\r\nOK\r\n"
        enabled, valid, blocked, distance_mm, threshold_mm, reason = parse_at_obstacle(response)
        self.assertTrue(enabled)
        self.assertFalse(valid)
        self.assertFalse(blocked)
        self.assertEqual(distance_mm, 0)
        self.assertEqual(threshold_mm, 250)
        self.assertEqual(reason, 2)

    def test_parse_at_obstacle_disabled(self) -> None:
        response = "AT+ROBOTOBS\r\n+ROBOT:OBS,0,0,0,0,250\r\nOK\r\n"
        enabled, valid, blocked, distance_mm, threshold_mm, reason = parse_at_obstacle(response)
        self.assertFalse(enabled)
        self.assertFalse(valid)
        self.assertFalse(blocked)
        self.assertEqual(distance_mm, 0)
        self.assertEqual(threshold_mm, 250)
        self.assertIsNone(reason)

    def test_parse_at_avoid(self) -> None:
        response = "AT+ROBOTAVOID\r\n+ROBOT:AVOID,1,1,0,1,1,0,420,250\r\nOK\r\n"
        active, phase, status, enabled, valid, blocked, distance_mm, threshold_mm, reason = parse_at_avoid(response)
        self.assertTrue(active)
        self.assertEqual(phase, 1)
        self.assertEqual(status, Status.OK)
        self.assertTrue(enabled)
        self.assertTrue(valid)
        self.assertFalse(blocked)
        self.assertEqual(distance_mm, 420)
        self.assertEqual(threshold_mm, 250)
        self.assertIsNone(reason)

    def test_parse_at_avoid_with_reason(self) -> None:
        response = "AT+ROBOTAVOID\r\n+ROBOT:AVOID,0,4,1,1,0,0,0,250,3\r\nOK\r\n"
        active, phase, status, enabled, valid, blocked, distance_mm, threshold_mm, reason = parse_at_avoid(response)
        self.assertFalse(active)
        self.assertEqual(phase, 4)
        self.assertEqual(status, Status.OBSTACLE_STOP)
        self.assertTrue(enabled)
        self.assertFalse(valid)
        self.assertFalse(blocked)
        self.assertEqual(distance_mm, 0)
        self.assertEqual(threshold_mm, 250)
        self.assertEqual(reason, 3)

    def test_parse_at_state(self) -> None:
        response = "AT+ROBOTST\r\n+ROBOT:STATE,1234,1,0,5,9,42\r\nOK\r\n"
        uptime_ms, ready, moving, last_command, last_sequence, age_ms = parse_at_state(response)
        self.assertEqual(uptime_ms, 1234)
        self.assertTrue(ready)
        self.assertFalse(moving)
        self.assertEqual(last_command, 5)
        self.assertEqual(last_sequence, 9)
        self.assertEqual(age_ms, 42)

    def test_parse_at_state_without_last_command_age(self) -> None:
        response = "AT+ROBOTST\r\n+ROBOT:STATE,1234,0,0,0,0,4294967295\r\nOK\r\n"
        *_, age_ms = parse_at_state(response)
        self.assertIsNone(age_ms)

    def test_parse_at_monitor(self) -> None:
        response = (
            "AT+ROBOTMON\r\n"
            "+ROBOT:MON,1234,1,0,1,253,582,1,1,0,420,250,0,0,9,120,80\r\n"
            "OK\r\n"
        )
        (
            uptime_ms,
            ready,
            moving,
            env_ok,
            temperature_deci_c,
            humidity_deci_percent,
            obstacle_enabled,
            obstacle_valid,
            obstacle_blocked,
            distance_mm,
            threshold_mm,
            reason,
            alarm_flags,
            sample_count,
            env_age_ms,
            obstacle_age_ms,
        ) = parse_at_monitor(response)
        self.assertEqual(uptime_ms, 1234)
        self.assertTrue(ready)
        self.assertFalse(moving)
        self.assertTrue(env_ok)
        self.assertEqual(temperature_deci_c, 253)
        self.assertEqual(humidity_deci_percent, 582)
        self.assertTrue(obstacle_enabled)
        self.assertTrue(obstacle_valid)
        self.assertFalse(obstacle_blocked)
        self.assertEqual(distance_mm, 420)
        self.assertEqual(threshold_mm, 250)
        self.assertEqual(reason, 0)
        self.assertEqual(alarm_flags, 0)
        self.assertEqual(sample_count, 9)
        self.assertEqual(env_age_ms, 120)
        self.assertEqual(obstacle_age_ms, 80)

    def test_parse_at_monitor_log_hex_alarm(self) -> None:
        response = (
            "ROBOT MON uptime=1234 ready=1 moving=0 env=0 temp=0 hum=0 "
            "obs=1 valid=0 blocked=0 distance=0 threshold=250 reason=3 "
            "alarm=0x2 samples=9 env_age=120 obs_age=80\r\n"
            "OK\r\n"
        )
        (
            _uptime_ms,
            _ready,
            _moving,
            env_ok,
            _temperature_deci_c,
            _humidity_deci_percent,
            obstacle_enabled,
            obstacle_valid,
            _obstacle_blocked,
            _distance_mm,
            _threshold_mm,
            reason,
            alarm_flags,
            sample_count,
            env_age_ms,
            obstacle_age_ms,
        ) = parse_at_monitor(response)
        self.assertFalse(env_ok)
        self.assertTrue(obstacle_enabled)
        self.assertFalse(obstacle_valid)
        self.assertEqual(reason, 3)
        self.assertEqual(alarm_flags, 0x2)
        self.assertEqual(sample_count, 9)
        self.assertEqual(env_age_ms, 120)
        self.assertEqual(obstacle_age_ms, 80)

    def test_parse_at_patrol_log(self) -> None:
        response = (
            "AT+ROBOTPATROL\r\n"
            "ROBOT PATROL active=1 phase=1 status=0 leg=0 loop=0 "
            "alarm=0x0 distance=4000 threshold=250 reason=6\r\n"
            "OK\r\n"
        )
        (
            active,
            phase,
            status,
            leg,
            loop,
            alarm_flags,
            enabled,
            valid,
            blocked,
            distance_mm,
            threshold_mm,
            reason,
        ) = parse_at_patrol(response)
        self.assertTrue(active)
        self.assertEqual(phase, 1)
        self.assertEqual(status, Status.OK)
        self.assertEqual(leg, 0)
        self.assertEqual(loop, 0)
        self.assertEqual(alarm_flags, 0)
        self.assertTrue(enabled)
        self.assertTrue(valid)
        self.assertFalse(blocked)
        self.assertEqual(distance_mm, 4000)
        self.assertEqual(threshold_mm, 250)
        self.assertEqual(reason, 6)

    def test_sle_ascii_command_profile(self) -> None:
        self.assertNotIn("X", SLE_ASCII_COMMAND_KEYS)
        self.assertIn("A", SLE_ASCII_COMMAND_KEYS)
        self.assertIn("D", SLE_ASCII_COMMAND_KEYS)
        self.assertIn("G", SLE_ASCII_COMMAND_KEYS)
        self.assertIn("M", SLE_ASCII_COMMAND_KEYS)
        self.assertIn("P", SLE_ASCII_COMMAND_KEYS)
        self.assertEqual(normalize_robot_key(" t "), "T")
        self.assertEqual(normalize_robot_key(" m "), "M")
        self.assertEqual(normalize_robot_key(" p "), "P")
        self.assertEqual(normalize_robot_key(" g "), "G")
        self.assertEqual(encode_sle_ascii_command("f"), b"F")

    def test_sle_ascii_rejects_unknown_key(self) -> None:
        with self.assertRaises(ValueError):
            encode_sle_ascii_command("Q")


if __name__ == "__main__":
    unittest.main()
