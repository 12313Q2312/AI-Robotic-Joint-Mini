"""Intent source: swappable trajectory, v1 labeled synthetic and bounded."""

import logging
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.intent import SyntheticStairIntent, depth_scale_for_seed
from knee_stair_l1.randomization import draw_factors
from knee_stair_l1.topics import TOPIC_INTENT_SYNTHETIC


class SyntheticIntentTests(unittest.TestCase):
    def test_topic_and_samples_say_synthetic(self):
        self.assertIn("synthetic", TOPIC_INTENT_SYNTHETIC)
        config = load_config()
        source = SyntheticStairIntent(config)
        with self.assertLogs("knee_stair_l1.intent", level=logging.INFO) as captured:
            source.reset(3)
        self.assertTrue(any("synthetic" in line for line in captured.output))
        sample = source.step(0.6)
        self.assertEqual(sample.source_label, "synthetic")
        self.assertLess(sample.q_ref_rad, config["intent"]["q_start_rad"])
        self.assertGreater(sample.q_ref_rad, config["plant"]["min_flexion_rad"] + 0.15)

    def test_same_seed_is_paired_and_quiet_before_onset(self):
        config = load_config()
        self.assertEqual(draw_factors(4, config), draw_factors(4, config))
        self.assertNotEqual(draw_factors(1, config).body_mass_kg, draw_factors(2, config).body_mass_kg)
        self.assertEqual(depth_scale_for_seed(4, config), draw_factors(4, config).intent_amplitude_scale)
        factors = draw_factors(7, config)
        self.assertGreaterEqual(factors.angle_noise_std_rad, 0.002)
        self.assertLessEqual(factors.angle_noise_std_rad, 0.01)
        self.assertGreaterEqual(factors.velocity_noise_std_rad_s, 0.02)
        self.assertLessEqual(factors.velocity_noise_std_rad_s, 0.1)
        again = draw_factors(7, config)
        self.assertEqual(factors.angle_noise_std_rad, again.angle_noise_std_rad)
        self.assertEqual(factors.velocity_noise_std_rad_s, again.velocity_noise_std_rad_s)
        source = SyntheticStairIntent(config)
        source.reset(4)
        before = source.step(source.onset_s - config["dt_s"])
        self.assertEqual(before.q_ref_rad, config["intent"]["q_start_rad"])
        self.assertEqual(before.qd_ref_rad_s, 0.0)
        self.assertFalse(before.ascent_active)
        self.assertLess(source.q_target(), config["intent"]["q_start_rad"])
        self.assertGreater(source.q_target(), config["plant"]["min_flexion_rad"])

    def test_v1_rejects_an_unlabeled_config(self):
        config = load_config()
        config["intent"]["source_label"] = "emg"
        with self.assertRaises(ValueError):
            SyntheticStairIntent(config)
