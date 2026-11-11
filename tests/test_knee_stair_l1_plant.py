"""Human leg, exoskeleton, gravity load and strap constraint."""

import logging
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.plant import BoundaryError, KneePlant


class KneePlantTests(unittest.TestCase):
    def test_gravity_torque_scales_with_weight_share(self):
        plant = KneePlant(load_config())
        load = load_config()["load"]
        expected = (
            load["knee_weight_share"]
            * load["body_mass_kg"]
            * load["gravity_m_s2"]
            * load["moment_arm_m"]
            * math.sin(math.pi / 2.0)
        )
        self.assertAlmostEqual(plant.gravity_flexion_nm(math.pi / 2.0), expected, places=12)
        self.assertAlmostEqual(plant.gravity_flexion_nm(0.0), 0.0, places=12)
        self.assertGreater(plant.gravity_flexion_nm(1.0), 0.0)

    def test_strap_interaction_is_not_the_motor_torque(self):
        config = load_config()
        plant = KneePlant(config)
        plant.q_e = plant.q_h + 0.1
        plant.advance(0.0, 0.0, 0.001, 0.0)
        self.assertAlmostEqual(plant.last_motor_extension_nm, 0.0)
        self.assertAlmostEqual(
            plant.strap_torque_flexion_nm,
            config["strap"]["stiffness_nm_per_rad"] * 0.1,
            places=6,
        )
        self.assertNotEqual(plant.strap_torque_flexion_nm, plant.last_motor_extension_nm)

    def test_motor_command_above_the_actuator_limit_is_clamped(self):
        config = load_config()
        plant = KneePlant(config)
        limit = plant.actuator_limit
        plant.advance(0.0, 80.0, 0.005, 0.0)
        self.assertAlmostEqual(plant.last_motor_extension_nm, limit)
        self.assertLessEqual(abs(plant.last_motor_extension_nm), config["actuator_torque_limit_nm"])
        plant.advance(0.0, -80.0, 0.005, 0.01)
        self.assertAlmostEqual(plant.last_motor_extension_nm, -limit)
        with self.assertRaises(BoundaryError):
            plant.advance(0.0, float("nan"), 0.005, 0.02)

    def test_hard_stop_contact_is_logged_once(self):
        plant = KneePlant(load_config())
        plant.q_h = plant.q_min + 1e-4
        plant.qd_h = -2.0
        with self.assertLogs("knee_stair_l1.plant", level=logging.WARNING) as captured:
            plant.advance(-50.0, 0.0, 0.02, 0.2)
        self.assertEqual(plant.q_h, plant.q_min)
        self.assertEqual(len(plant.anomalies), 1)
        self.assertEqual(plant.anomalies[0].side, "human")
        self.assertEqual(plant.anomalies[0].stop, "extension")
        self.assertTrue(any("anomaly hard_stop_contact" in line for line in captured.output))
        plant.advance(-50.0, 0.0, 0.02, 0.22)
        self.assertEqual(len(plant.anomalies), 1)

    def test_estop_passive_damping_does_not_apply_motor_torque(self):
        config = load_config()
        config["strap"]["stiffness_nm_per_rad"] = 0.0
        config["strap"]["damping_nm_s_per_rad"] = 0.0
        plant = KneePlant(config)
        plant.qd_e = 1.0
        plant.advance(0.0, 12.0, 0.002, 0.0, passive_damping=True)
        self.assertEqual(plant.last_motor_extension_nm, 0.0)
        self.assertLess(plant.qd_e, 1.0)

    def test_world_is_a_trimmed_single_knee(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / "knee_stair_l1" / "webots"))
        from generate_world import generate_world

        text = (root / "knee_stair_l1/webots/worlds/knee_stair_l1.wbt").read_text(encoding="utf-8")
        self.assertEqual(text, generate_world())
        self.assertIn("DEF HUMAN_SHANK", text)
        self.assertIn("DEF EXO_LINK", text)
        self.assertEqual(text.count("RotationalMotor"), 1)
        self.assertNotIn("r_ankle", text)
        self.assertIn("PR #3", text)
