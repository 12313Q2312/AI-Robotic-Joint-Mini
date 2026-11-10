"""Fault injection. Each case must reach one supervisor state.

out_of_range_command -> LIMITED_ASSIST
sensor_nan            -> E_STOP (latched, torque 0)
sensor_dropout        -> DAMPING_ONLY
communication_timeout -> DAMPING_ONLY

The closed-loop case checks that an oversized proposal is clamped before the plant.
"""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.loop import run_trial
from knee_stair_l1.safety import DAMPING_ONLY, E_STOP, LIMITED_ASSIST, JointSafetyCore


def _core():
    core = JointSafetyCore(load_config(), legacy_enabled=False)
    core.reset(0.0)
    return core


class FaultInjectionTests(unittest.TestCase):
    def test_out_of_range_command_enters_limited_assist(self):
        core = _core()
        limit = load_config()["safety"]["command_limit_nm"]
        core.observe_sensor(1.0, 0.2, 0.0)
        core.observe_command(500.0, 0.0)
        decision = core.evaluate(0.0)
        self.assertEqual(decision.state, LIMITED_ASSIST)
        self.assertIn("command_out_of_range", decision.reasons)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, limit)
        self.assertLess(abs(decision.safe_extension_torque_nm), 500.0)

    def test_sensor_nan_enters_latched_estop(self):
        core = _core()
        core.observe_sensor(float("nan"), 0.2, 0.0)
        core.observe_command(5.0, 0.0)
        decision = core.evaluate(0.0)
        self.assertEqual(decision.state, E_STOP)
        self.assertIn("sensor_nan", decision.reasons)
        self.assertEqual(decision.safe_extension_torque_nm, 0.0)
        self.assertTrue(decision.estop_latched)
        core.observe_sensor(1.0, 0.0, 0.1)
        core.observe_command(5.0, 0.1)
        recovered = core.evaluate(0.1)
        self.assertEqual(recovered.state, E_STOP)
        self.assertEqual(recovered.safe_extension_torque_nm, 0.0)

    def test_sensor_dropout_enters_damping_only(self):
        core = _core()
        core.observe_sensor(1.0, 0.5, 0.0)
        core.observe_command(5.0, 0.0)
        self.assertNotEqual(core.evaluate(0.0).state, DAMPING_ONLY)
        core.observe_command(5.0, 0.040001)
        decision = core.evaluate(0.040001)
        self.assertEqual(decision.state, DAMPING_ONLY)
        self.assertIn("sensor_dropout", decision.reasons)
        self.assertNotIn("communication_timeout", decision.reasons)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 1.0, places=12)

    def test_communication_timeout_enters_damping_only(self):
        core = _core()
        core.observe_sensor(1.0, 0.5, 0.0)
        core.observe_command(5.0, 0.0)
        core.observe_sensor(1.0, 0.5, 0.0501)
        decision = core.evaluate(0.0501)
        self.assertEqual(decision.state, DAMPING_ONLY)
        self.assertIn("communication_timeout", decision.reasons)
        self.assertNotIn("sensor_dropout", decision.reasons)
        self.assertAlmostEqual(decision.safe_extension_torque_nm, 1.0, places=12)
        self.assertNotEqual(decision.safe_extension_torque_nm, 5.0)

    def test_closed_loop_motor_command_stays_inside_the_supervisor_limit(self):
        config = load_config()
        config["duration_s"] = 0.2
        trial = run_trial(config, seed=0, condition="assist_policy")
        self.assertLessEqual(trial.max_abs_motor_command_nm, config["safety"]["command_limit_nm"] + 1e-9)
        self.assertEqual(trial.source_label, "synthetic")
