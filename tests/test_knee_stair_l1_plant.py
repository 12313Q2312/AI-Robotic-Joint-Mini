"""Human leg and exoskeleton meet only at the torque boundary."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.plant import BoundaryError, KneePlant


class KneePlantTests(unittest.TestCase):
    def test_human_extension_reduces_flexion_and_is_not_interaction(self):
        plant = KneePlant(load_config())
        start = plant.flexion_rad
        plant.advance(30.0, 0.0, 0.01)
        self.assertLess(plant.flexion_rad, start)
        self.assertEqual(plant.last_interaction_nm, 0.0)

    def test_boundary_rejects_torque_above_the_supervisor_limit(self):
        config = load_config()
        plant = KneePlant(config)
        limit = config["plant"]["boundary_abs_limit_nm"]
        plant.advance(0.0, limit, 0.005)
        self.assertAlmostEqual(plant.last_interaction_nm, limit)
        with self.assertRaises(BoundaryError):
            plant.advance(0.0, limit + 0.01, 0.005)
        with self.assertRaises(BoundaryError):
            plant.advance(0.0, float("nan"), 0.005)

    def test_world_is_a_trimmed_single_knee(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / "knee_stair_l1" / "webots"))
        from generate_world import generate_world

        text = (root / "knee_stair_l1/webots/worlds/knee_stair_l1.wbt").read_text(encoding="utf-8")
        self.assertEqual(text, generate_world())
        self.assertIn("DEF HUMAN_SHANK", text)
        self.assertIn("DEF EXO_LINK", text)
        self.assertEqual(text.count("RotationalMotor"), 1)
        self.assertIn("exo_knee", text)
        self.assertNotIn("r_ankle", text)
        self.assertNotIn("l_knee", text)
        self.assertIn("PR #3", text)
