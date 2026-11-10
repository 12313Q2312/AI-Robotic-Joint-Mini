"""Layer 2 synthetic intent shapes. Nominal motion is unchanged before mid-move."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.intent import SyntheticStairIntent


class IntentScenarioTests(unittest.TestCase):
    def _source(self, scenario):
        source = SyntheticStairIntent(load_config(), scenario=scenario)
        source.reset(0)
        return source

    def test_nominal_ends_on_the_target_and_the_others_do_not(self):
        nominal_source = self._source("nominal")
        target = nominal_source.q_target()
        start = nominal_source.q_start
        nominal = nominal_source.step(2.0)
        sudden = self._source("sudden_change").step(2.0)
        stopped = self._source("mid_stop").step(2.0)
        reverse = self._source("reverse").step(2.0)
        self.assertAlmostEqual(nominal.q_ref_rad, target)
        self.assertAlmostEqual(sudden.q_ref_rad, start)
        self.assertAlmostEqual(sudden.qd_ref_rad_s, 0.0)
        self.assertAlmostEqual(stopped.q_ref_rad, start + 0.5 * (target - start))
        self.assertAlmostEqual(stopped.qd_ref_rad_s, 0.0)
        self.assertAlmostEqual(reverse.q_ref_rad, start)
        self.assertAlmostEqual(reverse.qd_ref_rad_s, 0.0)

    def test_scenarios_match_nominal_before_the_midpoint(self):
        nominal = self._source("nominal").step(0.50)
        for scenario in ("sudden_change", "mid_stop", "reverse"):
            sample = self._source(scenario).step(0.50)
            self.assertAlmostEqual(sample.q_ref_rad, nominal.q_ref_rad)
            self.assertAlmostEqual(sample.qd_ref_rad_s, nominal.qd_ref_rad_s)
