"""Merged supervisor: legacy angle numbers stay, knee faults use one state machine."""

import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.safety import DAMPING_ONLY, E_STOP, LIMITED_ASSIST, NORMAL, JointSafetyCore


class LegacyAnglePortTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        safety = self.config["safety"]
        self.assertAlmostEqual(safety["legacy_min_angle_rad"], math.radians(-90.0))
        self.assertAlmostEqual(safety["legacy_max_angle_rad"], math.radians(90.0))
        self.assertEqual(safety["legacy_timeout_s"], 1.0)

    def test_clamp_reject_and_timeout_match_the_historical_node(self):
        core = JointSafetyCore(self.config, legacy_enabled=True)
        core.reset(0.0)
        self.assertAlmostEqual(core.evaluate(0.0).legacy_safe_target_rad, 0.0)
        self.assertTrue(core.observe_legacy_target(0.4, 0.0))
        self.assertAlmostEqual(core.evaluate(0.0).legacy_safe_target_rad, 0.4)
        self.assertEqual(core.evaluate(0.0).state, NORMAL)
        self.assertFalse(core.observe_legacy_target(float("nan"), 0.1))
        self.assertAlmostEqual(core.evaluate(0.1).legacy_safe_target_rad, 0.4)
        held = core.evaluate(1.0)
        self.assertAlmostEqual(held.legacy_safe_target_rad, 0.4)
        self.assertEqual(held.state, NORMAL)
        expired = core.evaluate(1.0 + 1e-6)
        self.assertAlmostEqual(expired.legacy_safe_target_rad, 0.0)
        self.assertEqual(expired.state, DAMPING_ONLY)
        self.assertIn("legacy_command_timeout", expired.reasons)

    def test_out_of_range_angle_is_clamped_and_marked_limited(self):
        core = JointSafetyCore(self.config, legacy_enabled=True)
        core.reset(0.0)
        self.assertTrue(core.observe_legacy_target(2.0, 0.0))
        decision = core.evaluate(0.0)
        self.assertAlmostEqual(decision.legacy_safe_target_rad, self.config["safety"]["legacy_max_angle_rad"])
        self.assertEqual(decision.state, LIMITED_ASSIST)
        self.assertIn("legacy_command_out_of_range", decision.reasons)
        self.assertTrue(core.observe_legacy_target(-2.0, 0.1))
        low = core.evaluate(0.1)
        self.assertAlmostEqual(low.legacy_safe_target_rad, self.config["safety"]["legacy_min_angle_rad"])

    def test_knee_estop_does_not_rewrite_the_legacy_angle_number(self):
        # Preserves /joint/safe_target_angle. Forcing that topic to 0 on knee
        # E_STOP would change the existing numeric contract.
        core = JointSafetyCore(self.config, legacy_enabled=True)
        core.reset(0.0)
        core.observe_legacy_target(0.3, 0.0)
        core.observe_sensor(float("nan"), 0.0, 0.0)
        decision = core.evaluate(0.0)
        self.assertEqual(decision.state, E_STOP)
        self.assertAlmostEqual(decision.legacy_safe_target_rad, 0.3)
        self.assertEqual(decision.safe_extension_torque_nm, 0.0)


class KneePortTests(unittest.TestCase):
    def test_damping_opposes_flexion_and_drops_the_assist_command(self):
        core = JointSafetyCore(load_config(), legacy_enabled=False)
        core.reset(0.0)
        core.observe_sensor(1.0, 0.5, 0.0)
        core.observe_command(5.0, 0.0)
        self.assertEqual(core.evaluate(0.0).state, NORMAL)
        core.observe_sensor(1.0, 0.5, 0.051)
        decision = core.evaluate(0.051)
        self.assertEqual(decision.state, DAMPING_ONLY)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 1.0, places=12)
        self.assertNotEqual(decision.safe_extension_torque_nm, 5.0)

    def test_estop_latches_until_reset(self):
        core = JointSafetyCore(load_config(), legacy_enabled=False)
        core.reset(0.0)
        core.observe_sensor(float("nan"), 0.1, 0.0)
        self.assertTrue(core.evaluate(0.0).estop_latched)
        core.observe_sensor(1.0, 0.0, 0.1)
        core.observe_command(1.0, 0.1)
        held = core.evaluate(0.1)
        self.assertEqual(held.state, E_STOP)
        self.assertEqual(held.safe_extension_torque_nm, 0.0)
        core.reset(0.2)
        core.observe_sensor(1.0, 0.0, 0.2)
        core.observe_command(1.0, 0.2)
        self.assertEqual(core.evaluate(0.2).state, NORMAL)

    def test_node_source_uses_the_one_supervisor(self):
        path = pathlib.Path(__file__).resolve().parents[1] / (
            "ros2_ws/src/ai_robotic_joint_ros2/scripts/joint_safety_node.py"
        )
        text = path.read_text(encoding="utf-8")
        self.assertEqual(text.count("class "), 1)
        self.assertIn("class JointSafety", text)
        self.assertIn("JointSafetyCore", text)
        self.assertIn("TOPIC_LEGACY_TARGET", text)
        self.assertIn("TOPIC_ASSIST_COMMAND_SAFE", text)
        self.assertNotIn("class KneeSafety", text)
