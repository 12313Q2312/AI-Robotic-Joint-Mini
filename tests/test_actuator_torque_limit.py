"""The actuator clamp is what the plant and the motor topic actually receive."""

import json
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.actuator import clamp_actuator_torque
from knee_stair_l1.bus import TopicBus
from knee_stair_l1.config import load_config, validate_config
from knee_stair_l1.controller import CONDITIONS, ControllerCommand
from knee_stair_l1.loop import run_trial
from knee_stair_l1.plant import KneePlant
from knee_stair_l1.safe_publish import on_assist_command
from knee_stair_l1.safety import E_STOP, LIMITED_ASSIST, JointSafetyCore
from knee_stair_l1.student_t import student_t_crit_95
from knee_stair_l1.topics import TOPIC_ASSIST_COMMAND_SAFE, TOPIC_TELEMETRY
import knee_stair_l1.loop as loop_module


class ActuatorLimitConfigTests(unittest.TestCase):
    def test_load_config_rejects_a_limit_above_24_or_not_positive(self):
        config = load_config()
        self.assertEqual(config["actuator_torque_limit_nm"], 24.0)
        for bad in (0.0, -1.0, 24.1, 80.0):
            broken = dict(config)
            broken["actuator_torque_limit_nm"] = bad
            with self.assertRaises(ValueError):
                validate_config(broken)

    def test_plant_boundary_matches_the_actuator_and_not_the_40_nm_check(self):
        config = load_config()
        self.assertEqual(config["plant"]["boundary_abs_limit_nm"], 24.0)
        self.assertEqual(config["safety"]["command_limit_nm"], 40.0)
        self.assertIn("plausibility", config["safety"]["command_limit_role"].lower())
        self.assertLessEqual(config["assist_policy"]["limit_nm"], 24.0)


class DeliveredTorqueTests(unittest.TestCase):
    def test_each_controller_path_delivers_at_most_24_when_injected_with_80(self):
        config = load_config()
        config["duration_s"] = 0.05
        original = loop_module.AssistController.command

        def injected(self, *args, **kwargs):
            command = original(self, *args, **kwargs)
            return ControllerCommand(
                80.0,
                command.desired_flexion_rad,
                command.desired_flexion_velocity_rad_s,
                command.pid_saturated,
                command.policy_saturated,
                command.intent_time_s,
                command.source_label,
            )

        loop_module.AssistController.command = injected
        try:
            for condition in CONDITIONS:
                trial = run_trial(config, seed=0, condition=condition)
                self.assertEqual(condition, trial.condition)
                self.assertAlmostEqual(trial.max_abs_motor_command_nm, 24.0)
                self.assertLessEqual(trial.max_abs_motor_command_nm, 24.0)
        finally:
            loop_module.AssistController.command = original

    def test_plant_measurement_of_an_80_nm_command_is_24(self):
        plant = KneePlant(load_config())
        plant.advance(0.0, 80.0, 0.005, 0.0)
        self.assertAlmostEqual(plant.last_motor_extension_nm, 24.0)

    def test_published_safe_topic_and_telemetry_use_the_clamped_torque(self):
        config = load_config()
        config["duration_s"] = 0.05
        bus = TopicBus()
        original = loop_module.AssistController.command

        def injected(self, *args, **kwargs):
            command = original(self, *args, **kwargs)
            return ControllerCommand(
                80.0,
                command.desired_flexion_rad,
                command.desired_flexion_velocity_rad_s,
                command.pid_saturated,
                command.policy_saturated,
                command.intent_time_s,
                command.source_label,
            )

        loop_module.AssistController.command = injected
        try:
            run_trial(config, seed=0, condition="assist_policy", bus=bus)
        finally:
            loop_module.AssistController.command = original
        safe = [payload for topic, payload in bus.messages if topic == TOPIC_ASSIST_COMMAND_SAFE]
        telemetry = [json.loads(payload) for topic, payload in bus.messages if topic == TOPIC_TELEMETRY]
        self.assertEqual(len(safe), 10)
        self.assertEqual(len(telemetry), 10)
        for payload in safe:
            self.assertLessEqual(abs(payload["torque_extension_nm"]), 24.0)
            self.assertAlmostEqual(payload["torque_extension_nm"], 24.0)
            self.assertAlmostEqual(payload["supervisor_command_nm"], 40.0)
        for row in telemetry:
            self.assertLessEqual(abs(row["applied_torque_nm"]), 24.0)
            self.assertLessEqual(abs(row["safe_extension_torque_nm"]), 24.0)
            self.assertAlmostEqual(row["supervisor_command_nm"], 40.0)

    def test_ros_callback_publishes_the_clamped_value_without_rclpy(self):
        """on_assist_command is the body of JointSafety.on_assist.

        rclpy is not imported. The DDS publisher is not created. The node
        source is checked to publish outcome.applied_torque_nm.
        """
        config = load_config()
        core = JointSafetyCore(config, legacy_enabled=False)
        core.reset(0.0)
        core.observe_sensor(1.0, 0.0, 0.0)
        published = on_assist_command(core, config, 0.0, 80.0)
        self.assertEqual(published.state, LIMITED_ASSIST)
        self.assertAlmostEqual(published.supervisor_command_nm, 40.0)
        self.assertAlmostEqual(published.applied_torque_nm, 24.0)
        source = (
            pathlib.Path(__file__).resolve().parents[1]
            / "ros2_ws/src/ai_robotic_joint_ros2/scripts/joint_safety_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn("on_assist_command", source)
        self.assertIn("torque.data = outcome.applied_torque_nm", source)
        self.assertNotIn("torque.data = decision.safe_extension_torque_nm", source)

    def test_non_finite_assist_callback_publishes_zero_and_estop(self):
        config = load_config()
        for command in (float("nan"), float("inf"), float("-inf")):
            core = JointSafetyCore(config, legacy_enabled=True)
            core.reset(0.0)
            core.observe_legacy_target(0.4, 0.0)
            core.observe_sensor(1.0, 0.0, 0.0)
            with self.assertRaises(ValueError):
                clamp_actuator_torque(command, config["actuator_torque_limit_nm"])
            published = on_assist_command(core, config, 0.0, command)
            self.assertEqual(published.applied_torque_nm, 0.0)
            self.assertEqual(published.state, E_STOP)
            self.assertTrue(core.evaluate(0.0).estop_latched)
            self.assertAlmostEqual(published.legacy_safe_target_rad, 0.4)
            self.assertTrue(math.isfinite(published.applied_torque_nm))


class StudentTTests(unittest.TestCase):
    def test_known_critical_values(self):
        self.assertAlmostEqual(student_t_crit_95(11), 2.201, places=3)
        self.assertAlmostEqual(student_t_crit_95(29), 2.045, places=3)
        self.assertAlmostEqual(student_t_crit_95(11), 2.200985160082949, places=9)
        self.assertAlmostEqual(student_t_crit_95(29), 2.045229642132703, places=9)
