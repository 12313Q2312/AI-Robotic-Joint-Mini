"""Admittance, the existing PID recurrence, and the copied AssistPolicy."""

import json
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.assist_policy import AssistPolicy
from knee_stair_l1.config import load_config
from knee_stair_l1.controller import AssistController
from knee_stair_l1.intent import IntentSample
from knee_stair_l1.torque_pid import TorquePid


class TorquePidTests(unittest.TestCase):
    def test_first_step_matches_historical_joint_controller_printout(self):
        pid = TorquePid(0.15, 0.05, 0.04, 0.10)
        step = pid.step(0.523599, 0.0, 0.0, 0.032)
        error = 0.523599
        integral = error * 0.032
        p_torque = 0.15 * error
        i_torque = 0.05 * integral
        self.assertAlmostEqual(step.p_torque_nm, p_torque, places=12)
        self.assertAlmostEqual(step.i_torque_nm, i_torque, places=12)
        self.assertAlmostEqual(step.d_torque_nm, 0.0, places=12)
        self.assertAlmostEqual(step.control_torque_nm, p_torque + i_torque, places=12)
        self.assertFalse(step.anti_windup_active)
        self.assertAlmostEqual(step.control_torque_nm, 0.079378, delta=5e-4)
        self.assertAlmostEqual(step.error_integral_rad_s, 0.016755, delta=5e-4)

    def test_derivative_uses_measured_velocity(self):
        pid = TorquePid(0.0, 0.0, 0.04, 1.0)
        step = pid.step(0.0, 0.0, 0.5, 0.032)
        self.assertAlmostEqual(step.d_torque_nm, -0.02, places=12)
        self.assertAlmostEqual(step.control_torque_nm, -0.02, places=12)

    def test_anti_windup_rejects_integral_that_deepens_saturation(self):
        pid = TorquePid(1.0, 1.0, 0.0, 0.5)
        first = pid.step(1.0, 0.0, 0.0, 0.1)
        second = pid.step(1.0, 0.0, 0.0, 0.1)
        self.assertTrue(first.anti_windup_active)
        self.assertEqual(first.error_integral_rad_s, 0.0)
        self.assertAlmostEqual(first.control_torque_nm, 0.5, places=12)
        self.assertEqual(second.error_integral_rad_s, 0.0)
        self.assertTrue(first.saturated)

    def test_knee_safety_limit_stays_24_nm(self):
        self.assertEqual(load_config()["pid"]["torque_limit_nm"], 24.0)


class AssistPolicyTests(unittest.TestCase):
    def test_saturation_slew_disable_and_nan_match_pr3(self):
        policy = AssistPolicy(0.35, 35.0, 100.0)
        previous = 0.0
        for _ in range(1000):
            value = policy.step(500.0, 0.002, True)
            self.assertLessEqual(value, 35.0 + 1e-12)
            self.assertLessEqual(abs(value - previous), 0.2 + 1e-12)
            previous = value
        self.assertTrue(policy.command_is_saturated(500.0, True))
        for _ in range(1000):
            previous = policy.step(500.0, 0.002, False)
        self.assertLess(abs(previous), 1e-12)
        self.assertEqual(policy.step(float("nan"), 0.002, True), 0.0)

    def test_invalid_parameters_raise(self):
        with self.assertRaises(ValueError):
            AssistPolicy(1.2, 35.0, 100.0)


class AssistControllerTests(unittest.TestCase):
    def test_sources_are_swappable_and_modes_differ(self):
        config = load_config()
        sample = IntentSample("unit-test-double", 0.5, 0.7, -0.2, True, 0.4)
        off = AssistController(config, "off")
        off.reset(1.0)
        off_command = off.command(sample, 1.0, 0.0, 30.0, 0.005)
        self.assertEqual(off_command.torque_extension_nm, 0.0)
        self.assertIsNone(off_command.desired_flexion_rad)

        policy = AssistController(config, "assist_policy")
        policy.reset(1.0)
        policy_command = policy.command(sample, 1.0, 0.0, 30.0, 0.005)
        self.assertAlmostEqual(policy_command.torque_extension_nm, 350.0 * 0.005, places=12)
        self.assertEqual(policy_command.source_label, "unit-test-double")

        admit = AssistController(config, "admittance")
        admit.reset(1.0)
        admit_command = admit.command(sample, 1.05, 0.0, 30.0, 0.005)
        self.assertLess(admit_command.desired_flexion_rad, 1.0)
        self.assertGreater(admit_command.torque_extension_nm, 0.0)
        self.assertEqual(admit_command.source_label, "unit-test-double")


class ActuatorConfigLoadTests(unittest.TestCase):
    def test_shipped_actuator_hard_limit_is_24_nm(self):
        config = load_config()
        self.assertEqual(config["actuator_torque_limit_nm"], 24.0)
        self.assertEqual(config["plant"]["boundary_abs_limit_nm"], 24.0)
        self.assertEqual(config["assist_policy"]["limit_nm"], 24.0)
        self.assertEqual(config["pid"]["torque_limit_nm"], 24.0)
        self.assertEqual(config["safety"]["command_limit_nm"], 40.0)

    def test_load_config_rejects_actuator_limit_above_24_nm(self):
        config = load_config()
        config.pop("_config_path", None)
        config["actuator_torque_limit_nm"] = 30.0
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "over_limit.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaises(ValueError) as caught:
                load_config(path)
        self.assertIn("actuator_torque_limit_nm must satisfy 0 < limit <= 24", str(caught.exception))
