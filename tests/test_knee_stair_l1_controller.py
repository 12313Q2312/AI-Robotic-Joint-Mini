"""Admittance, the existing PID recurrence, and the copied AssistPolicy."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.assist_policy import AssistPolicy
from knee_stair_l1.config import load_config
from knee_stair_l1.controller import AssistController
from knee_stair_l1.intent import IntentSample
from knee_stair_l1.torque_pid import TorquePid


class TorquePidTests(unittest.TestCase):
    def test_first_step_matches_historical_joint_controller_printout(self):
        # experiment_0007 first printed sample. dt is 0.032 s; the controller
        # prints every second step. Gains are the historical rig, not the knee config.
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
        sample = IntentSample("unit-test-double", 0.5, 30.0, True, 0.2, 30.0)
        off = AssistController(config, "off")
        off.reset(1.05)
        off_command = off.command(sample, 1.05, 0.0, 0.005)
        self.assertEqual(off_command.torque_extension_nm, 0.0)
        self.assertIsNone(off_command.desired_flexion_rad)

        policy = AssistController(config, "assist_policy")
        policy.reset(1.05)
        policy_command = policy.command(sample, 1.05, 0.0, 0.005)
        self.assertAlmostEqual(policy_command.torque_extension_nm, 350.0 * 0.005, places=12)
        self.assertEqual(policy_command.source_label, "unit-test-double")

        admit = AssistController(config, "admittance")
        admit.reset(1.05)
        admit_command = admit.command(sample, 1.05, 0.0, 0.005)
        self.assertLess(admit_command.desired_flexion_rad, 1.05)
        self.assertGreater(admit_command.torque_extension_nm, 0.0)
        self.assertEqual(admit_command.source_label, "unit-test-double")
