"""Intent source: swappable, and the v1 source is labeled synthetic."""

import logging
import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.config import load_config
from knee_stair_l1.intent import SyntheticStairIntent, amplitude_for_seed
from knee_stair_l1.topics import TOPIC_INTENT_SYNTHETIC


class SyntheticIntentTests(unittest.TestCase):
    def test_topic_and_samples_say_synthetic(self):
        self.assertIn("synthetic", TOPIC_INTENT_SYNTHETIC)
        source = SyntheticStairIntent(load_config())
        with self.assertLogs("knee_stair_l1.intent", level=logging.INFO) as captured:
            source.reset(3)
        self.assertTrue(any("synthetic" in line for line in captured.output))
        sample = source.step(0.55)
        self.assertEqual(sample.source_label, "synthetic")
        self.assertGreater(sample.effort_extension_nm, 0.0)
        self.assertEqual(sample.human_extension_nm, sample.effort_extension_nm)

    def test_same_seed_is_paired_and_quiet_before_onset(self):
        config = load_config()
        self.assertEqual(amplitude_for_seed(4, config), amplitude_for_seed(4, config))
        self.assertNotEqual(amplitude_for_seed(1, config), amplitude_for_seed(2, config))
        source = SyntheticStairIntent(config)
        source.reset(4)
        before = source.step(config["intent"]["onset_s"] - config["dt_s"])
        self.assertEqual(before.effort_extension_nm, 0.0)
        self.assertFalse(before.ascent_active)

    def test_v1_rejects_an_unlabeled_config(self):
        config = load_config()
        config["intent"]["source_label"] = "emg"
        with self.assertRaises(ValueError):
            SyntheticStairIntent(config)
