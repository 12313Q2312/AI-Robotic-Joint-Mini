"""RJ-SAFE-01: offline command-publication fault regressions.

These tests exercise the ROS callback's pure-Python body, not DDS, Webots,
motor hardware, or a physical emergency stop.
"""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.safe_publish import on_assist_command, republish_supervised
from knee_stair_l1.safety import DAMPING_ONLY, E_STOP, JointSafetyCore


class SafePublishFaultRegression(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.core = JointSafetyCore(self.config, legacy_enabled=False)
        self.core.reset(0.0)
        self.core.observe_sensor(1.0, 0.0, 0.0)

    def test_nonfinite_after_valid_command_never_republishes_previous_torque(self):
        valid = on_assist_command(self.core, self.config, 0.0, 5.0)
        self.assertGreater(valid.applied_torque_nm, 0.0)
        for now, bad in ((0.005, float("nan")), (0.010, float("inf"))):
            outcome = on_assist_command(self.core, self.config, now, bad)
            self.assertEqual(outcome.state, E_STOP)
            self.assertEqual(outcome.applied_torque_nm, 0.0)
            self.assertTrue(math.isfinite(outcome.applied_torque_nm))
        later = on_assist_command(self.core, self.config, 0.015, 5.0)
        self.assertEqual(later.state, E_STOP)
        self.assertEqual(later.applied_torque_nm, 0.0)
        timer = republish_supervised(self.core, self.config, 0.020)
        self.assertEqual(timer.state, E_STOP)
        self.assertEqual(timer.applied_torque_nm, 0.0)

    def test_sensor_nan_after_valid_command_latches_zero_output(self):
        initial = on_assist_command(self.core, self.config, 0.0, 5.0)
        self.assertGreater(initial.applied_torque_nm, 0.0)
        self.core.observe_sensor(float("nan"), 0.0, 0.005)
        outcome = republish_supervised(self.core, self.config, 0.005)
        self.assertEqual(outcome.state, E_STOP)
        self.assertEqual(outcome.applied_torque_nm, 0.0)
        self.core.observe_sensor(1.0, 0.0, 0.010)
        recovered_packet = on_assist_command(self.core, self.config, 0.010, 5.0)
        self.assertEqual(recovered_packet.state, E_STOP)
        self.assertEqual(recovered_packet.applied_torque_nm, 0.0)

    def test_sensor_receipt_gap_discards_previous_assist(self):
        self.core.observe_sensor(1.0, 0.5, 0.0)
        initial = on_assist_command(self.core, self.config, 0.0, 5.0)
        self.assertGreater(initial.applied_torque_nm, 0.0)
        # Command is refreshed, sensor is not. DAMPING_ONLY is a bounded
        # passive viscous command, not a guarantee of zero motor torque.
        outcome = on_assist_command(self.core, self.config, 0.041, 5.0)
        self.assertEqual(outcome.state, DAMPING_ONLY)
        self.assertLessEqual(abs(outcome.applied_torque_nm), self.config["actuator_torque_limit_nm"])
        self.assertNotEqual(outcome.supervisor_command_nm, 5.0)


if __name__ == "__main__":
    unittest.main()
