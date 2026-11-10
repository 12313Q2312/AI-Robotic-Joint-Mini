"""Header stamps and the degraded cap on both assist paths. No rclpy."""

import json
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.bus import TopicBus
from knee_stair_l1.config import load_config
from knee_stair_l1.controller import ControllerCommand
from knee_stair_l1.loop import run_trial
from knee_stair_l1.safety import DEGRADED, NORMAL, JointSafetyCore
from knee_stair_l1.stamp import packet_time_from_header, stamped_packet_time
from knee_stair_l1.topics import TOPIC_TELEMETRY
import knee_stair_l1.loop as loop_module


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, stamp):
        self.stamp = stamp


class _Message:
    def __init__(self, header):
        self.header = header


class HeaderStampTests(unittest.TestCase):
    def test_future_zero_missing_and_normal_stamps(self):
        now = 1.0
        tolerance = 0.02
        future, future_reason = packet_time_from_header(
            _Message(_Header(_Stamp(1, 50_000_000))), now, tolerance,
        )
        self.assertIsNone(future)
        self.assertEqual(future_reason, "future_stamp")
        zero, zero_reason = packet_time_from_header(_Message(_Header(_Stamp(0, 0))), now, tolerance)
        self.assertIsNone(zero)
        self.assertEqual(zero_reason, "zero_stamp")
        missing, missing_reason = packet_time_from_header(_Message(None), now, tolerance)
        self.assertIsNone(missing)
        self.assertEqual(missing_reason, "missing_stamp")
        bare, bare_reason = packet_time_from_header(object(), now, tolerance)
        self.assertIsNone(bare)
        self.assertEqual(bare_reason, "missing_stamp")
        packet, reason = packet_time_from_header(
            _Message(_Header(_Stamp(1, 0))), now, tolerance,
        )
        self.assertIsNone(reason)
        self.assertAlmostEqual(packet, 1.0)

    def test_node_source_uses_header_stamp_not_receipt_time(self):
        source = (
            pathlib.Path(__file__).resolve().parents[1]
            / "ros2_ws/src/ai_robotic_joint_ros2/scripts/joint_safety_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("stamped_packet_time", source)
        self.assertIn("packet_time=packet_time", source)
        self.assertNotIn("observe_sensor(message.position[0], message.velocity[0], now)", source)

    def test_bad_stamps_are_over_age_and_a_normal_stamp_is_fresh(self):
        config = load_config()
        tolerance = config["safety"]["stamp_future_tolerance_s"]
        now = 1.0
        cases = (
            (_Message(None), DEGRADED),
            (_Message(_Header(_Stamp(0, 0))), DEGRADED),
            (_Message(_Header(_Stamp(2, 0))), DEGRADED),
            (_Message(_Header(_Stamp(1, 0))), NORMAL),
        )
        for message, expected in cases:
            core = JointSafetyCore(config, legacy_enabled=False)
            core.reset(now)
            packet_time, _reason = stamped_packet_time(core, message, now, tolerance)
            core.observe_sensor(1.0, 0.0, now, packet_time=packet_time)
            core.observe_intent(packet_time, now)
            core.observe_command(3.0, now)
            self.assertEqual(core.evaluate(now).state, expected)


class DegradedPathTests(unittest.TestCase):
    def test_both_paths_stay_at_or_below_12_nm_when_degraded(self):
        config = load_config()
        config["duration_s"] = 0.16
        original = loop_module.AssistController.command

        def injected(self, *args, **kwargs):
            command = original(self, *args, **kwargs)
            return ControllerCommand(
                30.0,
                command.desired_flexion_rad,
                command.desired_flexion_velocity_rad_s,
                command.pid_saturated,
                command.policy_saturated,
                command.intent_time_s,
                command.source_label,
            )

        loop_module.AssistController.command = injected
        try:
            for condition in ("admittance", "assist_policy"):
                bus = TopicBus()
                run_trial(config, seed=0, condition=condition, signal_delay_s=0.06, bus=bus)
                rows = [json.loads(payload) for topic, payload in bus.messages if topic == TOPIC_TELEMETRY]
                degraded = [row for row in rows if row["state"] == "DEGRADED"]
                self.assertGreater(len(degraded), 0)
                applied = [row["applied_torque_nm"] for row in degraded]
                self.assertTrue(all(abs(value) <= 12.0 + 1e-9 for value in applied))
                self.assertGreater(max(applied), 11.0)
                self.assertTrue(all(abs(row["safe_extension_torque_nm"]) <= 12.0 + 1e-9 for row in degraded))
        finally:
            loop_module.AssistController.command = original
