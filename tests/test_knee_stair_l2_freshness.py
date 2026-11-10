"""Layer 2 signal freshness. Packet age is not the receipt clock."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.safety import DEGRADED, E_STOP, NORMAL, SAFE_HOLD, JointSafetyCore


def _core():
    core = JointSafetyCore(load_config(), legacy_enabled=False)
    core.reset(0.0)
    return core


class FreshnessTests(unittest.TestCase):
    def test_age_at_the_threshold_stays_normal(self):
        core = _core()
        core.observe_sensor(1.0, 0.1, 0.05, packet_time=0.0)
        core.observe_intent(0.0, 0.05)
        core.observe_command(1.0, 0.05)
        decision = core.evaluate(0.05)
        self.assertEqual(decision.state, NORMAL)

    def test_stale_intent_is_degraded(self):
        core = _core()
        core.observe_sensor(1.0, 0.1, 0.08, packet_time=0.08)
        core.observe_intent(0.0, 0.08)
        core.observe_command(1.0, 0.08)
        decision = core.evaluate(0.08)
        self.assertEqual(decision.state, DEGRADED)
        self.assertIn("stale_intent", decision.reasons)
        self.assertNotIn("stale_sensor", decision.reasons)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 1.0)

    def test_stale_sensor_is_degraded(self):
        core = _core()
        core.observe_sensor(1.0, 0.2, 0.08, packet_time=0.0)
        core.observe_intent(0.08, 0.08)
        core.observe_command(1.0, 0.08)
        decision = core.evaluate(0.08)
        self.assertEqual(decision.state, DEGRADED)
        self.assertIn("stale_sensor", decision.reasons)
        self.assertNotIn("stale_intent", decision.reasons)

    def test_intermittent_staleness_does_not_latch(self):
        core = _core()
        core.observe_sensor(1.0, 0.0, 0.0, packet_time=0.0)
        core.observe_intent(0.0, 0.0)
        core.observe_command(1.0, 0.0)
        self.assertEqual(core.evaluate(0.0).state, NORMAL)
        core.observe_sensor(1.0, 0.0, 0.08, packet_time=0.0)
        core.observe_intent(0.08, 0.08)
        core.observe_command(1.0, 0.08)
        self.assertEqual(core.evaluate(0.08).state, DEGRADED)
        core.observe_sensor(1.0, 0.0, 0.10, packet_time=0.10)
        core.observe_intent(0.10, 0.10)
        core.observe_command(1.0, 0.10)
        self.assertEqual(core.evaluate(0.10).state, NORMAL)
        core.observe_sensor(1.0, 0.0, 0.20, packet_time=0.0)
        core.observe_intent(0.20, 0.20)
        core.observe_command(1.0, 0.20)
        again = core.evaluate(0.20)
        self.assertEqual(again.state, DEGRADED)
        self.assertFalse(again.passive_damping)

    def test_prolonged_staleness_latches_safe_hold_until_reset(self):
        core = _core()
        core.observe_sensor(1.0, 0.4, 0.06, packet_time=0.0)
        core.observe_intent(0.0, 0.06)
        core.observe_command(4.0, 0.06)
        self.assertEqual(core.evaluate(0.06).state, DEGRADED)
        core.observe_sensor(1.0, 0.4, 0.26, packet_time=0.0)
        core.observe_intent(0.0, 0.26)
        core.observe_command(4.0, 0.26)
        held = core.evaluate(0.26)
        self.assertEqual(held.state, SAFE_HOLD)
        self.assertEqual(held.safe_extension_torque_nm, 0.0)
        self.assertTrue(held.passive_damping)
        self.assertIn("safe_hold_latched", held.reasons)
        core.observe_sensor(1.0, 0.0, 0.40, packet_time=0.40)
        core.observe_intent(0.40, 0.40)
        core.observe_command(4.0, 0.40)
        still = core.evaluate(0.40)
        self.assertEqual(still.state, SAFE_HOLD)
        self.assertEqual(still.safe_extension_torque_nm, 0.0)
        core.reset(0.50)
        core.observe_sensor(1.0, 0.0, 0.50, packet_time=0.50)
        core.observe_intent(0.50, 0.50)
        core.observe_command(1.5, 0.50)
        self.assertEqual(core.evaluate(0.50).state, NORMAL)

    def test_estop_still_outranks_freshness_and_holds_the_angle(self):
        core = JointSafetyCore(load_config(), legacy_enabled=True)
        core.reset(0.0)
        core.observe_legacy_target(0.25, 0.0)
        before = core.evaluate(0.0).legacy_safe_target_rad
        core.observe_sensor(float("nan"), 0.0, 0.08, packet_time=0.0)
        core.observe_intent(0.0, 0.08)
        decision = core.evaluate(0.08)
        self.assertEqual(decision.state, E_STOP)
        self.assertEqual(decision.safe_extension_torque_nm, 0.0)
        self.assertAlmostEqual(decision.legacy_safe_target_rad, before)

    def test_degraded_still_clamps_an_over_limit_command(self):
        core = _core()
        core.observe_sensor(1.0, 0.0, 0.08, packet_time=0.0)
        core.observe_intent(0.08, 0.08)
        core.observe_command(80.0, 0.08)
        decision = core.evaluate(0.08)
        self.assertEqual(decision.state, DEGRADED)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 40.0)
        self.assertFalse(decision.passive_damping)

    def test_degraded_recovers_without_reset(self):
        core = _core()
        core.observe_sensor(1.0, 0.0, 0.08, packet_time=0.0)
        core.observe_intent(0.08, 0.08)
        core.observe_command(2.0, 0.08)
        self.assertEqual(core.evaluate(0.08).state, DEGRADED)
        core.observe_sensor(1.0, 0.0, 0.09, packet_time=0.09)
        core.observe_intent(0.09, 0.09)
        core.observe_command(2.0, 0.09)
        recovered = core.evaluate(0.09)
        self.assertEqual(recovered.state, NORMAL)
        self.assertAlmostEqual(recovered.safe_extension_torque_nm, 2.0)
        self.assertFalse(recovered.passive_damping)
