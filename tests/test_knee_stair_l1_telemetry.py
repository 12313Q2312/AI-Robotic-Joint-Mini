"""Paired metrics. Definitions stay fixed; the study records pass or fail."""

import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from knee_stair_l1.bus import TopicBus
from knee_stair_l1.config import load_config
from knee_stair_l1.loop import run_trial
from knee_stair_l1.telemetry import (
    BOOLEAN_CRITERIA,
    CONDITIONS,
    percentile,
    require_boolean_criteria,
    run_study,
    summarize,
)
from knee_stair_l1.topics import LAYER1_TOPICS, TOPIC_INTENT_SYNTHETIC, TOPIC_SAFETY_STATE
from scripts.run_knee_stair_l1_metrics import main


class MetricStudyTests(unittest.TestCase):
    def test_percentile_interpolates_sorted_samples(self):
        self.assertEqual(percentile([0.0, 10.0], 50), 5.0)
        self.assertEqual(percentile([1.0, 2.0, 3.0, 4.0], 100), 4.0)
        self.assertAlmostEqual(percentile([0.0, 10.0], 95), 9.5)

    def test_study_records_the_new_metrics_and_matches_committed_raw_json(self):
        config = load_config()
        self.assertEqual(list(config["seeds"]), list(range(30)))
        self.assertEqual(config["pid"]["torque_limit_nm"], config["actuator_torque_limit_nm"])
        trials = run_study(config)
        self.assertEqual(len(trials), 90)
        summary = summarize(trials, config)
        self.assertNotIn("assist_timing_error_s", summary["absolute"]["off"])
        self.assertTrue(summary["primary_std_positive"])
        self.assertIn("simulation loop latency, not real-system latency", summary["latency_definition"])
        self.assertFalse((pathlib.Path(__file__).resolve().parents[1] / "data/verification/knee_stair_l1/metrics_raw.json").exists())
        self.assertIn("assist_timing_error_s", summary["removed_metrics"])
        for trial in trials:
            self.assertEqual(trial.source_label, "synthetic")
            self.assertEqual(trial.hard_stop_contact_count, 0)
            self.assertGreater(len(trial.latency_samples_s), 0)
            self.assertAlmostEqual(trial.latency_samples_s[0], trial.latency_samples_s[-1])
            self.assertLess(trial.peak_interaction_torque_nm, config["metrics"]["comfort_peak_interaction_nm"])
        for condition in ("admittance", "assist_policy"):
            self.assertTrue(summary["criteria"][condition]["human_torque_rms_reduction_ci_excludes_0"])
            self.assertTrue(summary["criteria"][condition]["tracking_error_no_more_than_10_percent_worse"])
            self.assertTrue(summary["criteria"][condition]["hard_stop_contacts_zero"])
            self.assertTrue(summary["criteria"][condition]["peak_interaction_below_comfort"])
        self.assertLess(
            summary["criteria"]["admittance"]["saturation_time_fraction_mean"],
            config["metrics"]["admittance_saturation_fraction_max"],
        )
        for condition in ("admittance", "assist_policy"):
            for key in BOOLEAN_CRITERIA:
                self.assertIsInstance(summary["criteria"][condition][key], bool)
        require_boolean_criteria(summary)

    def test_null_pass_criterion_is_rejected(self):
        row = {key: True for key in BOOLEAN_CRITERIA}
        row["saturation_time_fraction_below_5_percent"] = None
        with self.assertRaises(ValueError):
            require_boolean_criteria({"criteria": {"assist_policy": row}})
        self.assertEqual(main(["--check"]), 0)
        self.assertTrue(CONDITIONS)

    def test_bus_uses_the_topic_contract_and_synthetic_label(self):
        config = load_config()
        config["duration_s"] = 0.1
        bus = TopicBus()
        run_trial(config, 0, "admittance", bus=bus)
        self.assertTrue(bus.topics() <= set(LAYER1_TOPICS))
        self.assertIn(TOPIC_INTENT_SYNTHETIC, bus.topics())
        self.assertIn(TOPIC_SAFETY_STATE, bus.topics())
        intent = [payload for topic, payload in bus.messages if topic == TOPIC_INTENT_SYNTHETIC]
        self.assertTrue(all(payload["source_label"] == "synthetic" for payload in intent))
        telemetry = [payload for topic, payload in bus.messages if topic.endswith("telemetry")]
        self.assertTrue(all("synthetic" in payload and "command_time_s" in payload for payload in telemetry))
