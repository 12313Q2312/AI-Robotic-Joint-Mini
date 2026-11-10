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
        self.assertEqual(core.evaluate(0.10).state, DEGRADED)
        core.observe_sensor(1.0, 0.0, 0.20, packet_time=0.20)
        core.observe_intent(0.20, 0.20)
        core.observe_command(1.0, 0.20)
        self.assertEqual(core.evaluate(0.20).state, NORMAL)
        core.observe_sensor(1.0, 0.0, 0.21, packet_time=0.0)
        core.observe_intent(0.21, 0.21)
        core.observe_command(1.0, 0.21)
        again = core.evaluate(0.21)
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
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 12.0)
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
        held = core.evaluate(0.09)
        self.assertEqual(held.state, DEGRADED)
        self.assertIn("fresh_recover_pending", held.reasons)
        core.observe_sensor(1.0, 0.0, 0.19, packet_time=0.19)
        core.observe_intent(0.19, 0.19)
        core.observe_command(2.0, 0.19)
        recovered = core.evaluate(0.19)
        self.assertEqual(recovered.state, NORMAL)
        self.assertAlmostEqual(recovered.safe_extension_torque_nm, 2.0)
        self.assertFalse(recovered.passive_damping)

    def test_degraded_entry_and_exit_respect_the_slew(self):
        config = load_config()
        core = _core()
        slew = config["safety"]["degraded_slew_nm_per_s"]
        dt = 0.005
        core.observe_sensor(1.0, 0.0, 0.0, packet_time=0.0)
        core.observe_intent(0.0, 0.0)
        core.observe_command(30.0, 0.0)
        previous = core.evaluate(0.0).safe_extension_torque_nm
        self.assertAlmostEqual(previous, 30.0)
        for step in range(1, 30):
            now = step * dt
            core.observe_sensor(1.0, 0.0, now, packet_time=now - 0.08)
            core.observe_intent(now, now)
            core.observe_command(30.0, now)
            decision = core.evaluate(now)
            self.assertEqual(decision.state, DEGRADED)
            self.assertLessEqual(abs(decision.safe_extension_torque_nm - previous), slew * dt + 1e-9)
            previous = decision.safe_extension_torque_nm
        self.assertAlmostEqual(previous, 12.0)
        first_normal = None
        for step in range(30, 55):
            now = step * dt
            core.observe_sensor(1.0, 0.0, now, packet_time=now)
            core.observe_intent(now, now)
            core.observe_command(30.0, now)
            decision = core.evaluate(now)
            self.assertLessEqual(abs(decision.safe_extension_torque_nm - previous), slew * dt + 1e-9)
            if decision.state == NORMAL and first_normal is None:
                first_normal = decision.safe_extension_torque_nm
            previous = decision.safe_extension_torque_nm
        self.assertAlmostEqual(first_normal, 13.0)

    def test_intermittent_staleness_transition_count_is_bounded(self):
        core = _core()
        previous = None
        transitions = 0
        step = 0
        while step * 0.005 <= 1.0 + 1e-12:
            now = round(step * 0.005, 10)
            stale = (int(now / 0.04) % 2) == 1
            packet = 0.0 if stale else now
            core.observe_sensor(1.0, 0.0, now, packet_time=packet)
            core.observe_intent(packet, now)
            core.observe_command(5.0, now)
            state = core.evaluate(now).state
            if previous is not None and state != previous:
                transitions += 1
            previous = state
            step += 1
        self.assertLessEqual(transitions, 4)
        self.assertGreaterEqual(transitions, 1)
