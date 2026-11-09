"""Paired metrics and the topic contract. Results are experimental simulation."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.bus import TopicBus
from knee_stair_l1.config import load_config
from knee_stair_l1.loop import run_trial
from knee_stair_l1.telemetry import CONDITIONS, run_study
from knee_stair_l1.topics import LAYER1_TOPICS, TOPIC_INTENT_SYNTHETIC, TOPIC_SAFETY_STATE
from scripts.run_knee_stair_l1_metrics import main


class MetricStudyTests(unittest.TestCase):
    def test_paired_properties_and_committed_reproduction(self):
        config = load_config()
        self.assertGreaterEqual(len(config["seeds"]), 10)
        self.assertEqual(
            config["admittance"]["equilibrium_flexion_rad"],
            config["plant"]["initial_flexion_rad"],
        )
        trials = run_study(config)
        self.assertEqual(len(trials), len(config["seeds"]) * len(CONDITIONS))
        by = {(trial.seed, trial.condition): trial for trial in trials}
        for seed in config["seeds"]:
            off = by[(seed, "off")]
            self.assertEqual(off.source_label, "synthetic")
            self.assertFalse(off.assist_onset)
            self.assertEqual(off.peak_interaction_torque_nm, 0.0)
            self.assertEqual(off.saturation_limit_trigger_count, 0)
            self.assertGreater(off.assist_timing_error_s, 1.0)
            for condition in ("admittance", "assist_policy"):
                assisted = by[(seed, condition)]
                self.assertEqual(assisted.source_label, "synthetic")
                self.assertTrue(assisted.assist_onset)
                self.assertGreater(assisted.assist_timing_error_s, 0.0)
                self.assertLess(assisted.assist_timing_error_s, off.assist_timing_error_s)
                self.assertGreater(assisted.peak_interaction_torque_nm, 0.0)
                self.assertAlmostEqual(assisted.end_to_end_latency_s, 0.01, delta=1e-9)
            self.assertGreater(
                by[(seed, "assist_policy")].peak_interaction_torque_nm,
                by[(seed, "admittance")].peak_interaction_torque_nm,
            )
        self.assertTrue(any(by[(seed, "assist_policy")].saturation_limit_trigger_count > 0 for seed in config["seeds"]))
        self.assertTrue(all(by[(seed, "admittance")].saturation_limit_trigger_count > 0 for seed in config["seeds"]))
        self.assertEqual(main(["--check"]), 0)

    def test_bus_uses_the_topic_contract_and_synthetic_label(self):
        config = load_config()
        config["duration_s"] = 0.1
        config["intent"]["onset_s"] = 0.0
        config["intent"]["burst_duration_s"] = 0.1
        bus = TopicBus()
        run_trial(config, 0, "admittance", bus=bus)
        self.assertTrue(bus.topics() <= set(LAYER1_TOPICS))
        self.assertIn(TOPIC_INTENT_SYNTHETIC, bus.topics())
        self.assertIn(TOPIC_SAFETY_STATE, bus.topics())
        intent = [payload for topic, payload in bus.messages if topic == TOPIC_INTENT_SYNTHETIC]
        self.assertTrue(intent)
        self.assertTrue(all(payload["source_label"] == "synthetic" for payload in intent))
        telemetry = [payload for topic, payload in bus.messages if topic.endswith("telemetry")]
        self.assertTrue(all("synthetic" in payload for payload in telemetry))
