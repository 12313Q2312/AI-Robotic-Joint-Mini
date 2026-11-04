"""Nominal fixture arithmetic, not native CAD mass certification."""
import math
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from prepare_joint_study import nominal_body, cam_mass_properties
from research.gravity_compensation.geometry import FourierCam


class FixtureTests(unittest.TestCase):
    def test_cam_added_mass_and_inertia_follow_actual_cross_section(self):
        cam = cam_mass_properties(FourierCam(.025))
        mass = 2700 * math.pi * .025 ** 2 * .005
        self.assertAlmostEqual(cam['mass_kg'], mass)
        self.assertAlmostEqual(cam['iyy'], mass * .025 ** 2 / 2)
        self.assertAlmostEqual(cam['ixx'], mass * (.025 ** 2 / 4 + .005 ** 2 / 12))
        self.assertAlmostEqual(cam['com_x_m'], 0)
        self.assertAlmostEqual(cam['com_z_m'], 0)
        self.assertGreater(nominal_body(.25, cam)['mass_kg'], nominal_body(.25)['mass_kg'])

    def test_plate_with_two_holes_mass_and_com_not_solid_box(self):
        body = nominal_body(0.0)
        expected_volume = .180 * .030 * .008 - math.pi * (.006 ** 2 + .003 ** 2) * .008
        self.assertAlmostEqual(body['mass_kg'], 2700 * expected_volume)
        self.assertAlmostEqual(body['com_x_m'], .07620977, places=8)
        self.assertAlmostEqual(body['com_y_m'], 0)
        self.assertAlmostEqual(body['com_z_m'], 0)
        self.assertGreater(body['ixx'], 0)
        self.assertGreater(body['iyy'], body['ixx'])
        self.assertAlmostEqual(body['ixy'], 0)
        self.assertAlmostEqual(body['ixz'], 0)
        self.assertAlmostEqual(body['iyz'], 0)


if __name__ == '__main__': unittest.main()
