import importlib.util
from pathlib import Path
import unittest
import math

path = Path(__file__).resolve().parents[1] / 'scripts/generate_test_link_solidworks.py'
spec = importlib.util.spec_from_file_location('native_link', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class NativeLinkSpecTest(unittest.TestCase):
    def test_nominal(self):
        module.validate_parameters(module.PARAMETERS)
        self.assertAlmostEqual(module.expected_volume(module.PARAMETERS) * 1e9,
                               (180 * 30 - math.pi * (6**2 + 3**2)) * 8)

    def test_invalid_values(self):
        for value in (float('nan'), float('inf'), -1, 0, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                module.validate_parameters(dict(module.PARAMETERS, thickness_mm=value))

    def test_inconsistent_pitch(self):
        with self.assertRaises(ValueError):
            module.validate_parameters(dict(module.PARAMETERS, pitch_mm=149))

    def test_boundary_hole(self):
        with self.assertRaises(ValueError):
            module.validate_parameters(dict(module.PARAMETERS, pivot_diameter_mm=30))


if __name__ == '__main__':
    unittest.main()
