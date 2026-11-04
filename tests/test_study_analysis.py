"""Offline static model checks; these are not Webots feedback measurements."""

import pathlib
import math
import copy
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))

from analyze_study_fixture import analyze_fixture


def fixture():
    body = dict(mass_kg=0.1, com_x_m=0.1, com_y_m=0.0, com_z_m=0.0)
    return dict(
        gravity_m_s2=9.81,
        cam=dict(mean_radius_m=0.025, cos_m=[0, 0, 0, 0], sin_m=[0, 0, 0, 0],
                 roller_radius_m=0.004, stiffness_n_per_m=10000,
                 max_compression_m=0.035),
        cases=[dict(payload_mass_kg=0.0, preload_m=0.01)],
        rigid_bodies=[dict(body, payload_index=0, side="baseline"),
                      dict(body, payload_index=0, side="candidate")],
    )


class StaticStudyTests(unittest.TestCase):
    def test_circular_cam_without_losses_has_same_motor_requirement_as_baseline(self):
        report = analyze_fixture(fixture(), samples=9, stiffness_factors=(1.0,),
                                 preload_offsets_m=(0.0,), loss_ratios=(0.0,))
        self.assertEqual(report["status"], "offline_static_synthetic_scan_not_webots_feedback")
        for row in report["metrics"]:
            self.assertEqual(row["valid"], True)
            self.assertAlmostEqual(row["torque_rms_benefit_fraction"], 0.0)
            self.assertAlmostEqual(row["candidate_motor_rms_nm"], row["baseline_motor_rms_nm"])

    def test_assumed_coulomb_loss_opposes_each_direction_and_dissipates_loop_work(self):
        report = analyze_fixture(fixture(), samples=9, stiffness_factors=(1.0,),
                                 preload_offsets_m=(0.0,), loss_ratios=(0.2,))
        rows = {row["direction"]: row for row in report["metrics"]}
        loss = 0.1 * 9.81 * 0.1 * 0.2
        self.assertGreater(rows["upward"]["candidate_motor_rms_nm"],
                           rows["downward"]["candidate_motor_rms_nm"])
        for row in rows.values():
            self.assertAlmostEqual(row["additional_loss_nm"], loss)
            self.assertAlmostEqual(row["loss_loop_area_j"], loss*math.pi)
        self.assertAlmostEqual(rows["upward"]["candidate_motor_peak_nm"], 0.0981+loss)
        self.assertAlmostEqual(rows["downward"]["candidate_motor_peak_nm"], 0.0981-loss)

    def test_payload_and_added_candidate_mass_use_each_own_center_of_mass(self):
        model = fixture()
        model["cases"].append(dict(payload_mass_kg=0.25, preload_m=0.01))
        link_moment = 0.1*0.1 + 0.25*0.15
        model["rigid_bodies"].extend([
            dict(payload_index=1, side="baseline", mass_kg=0.35,
                 com_x_m=link_moment/0.35, com_z_m=0.0),
            dict(payload_index=1, side="candidate", mass_kg=0.38,
                 com_x_m=(link_moment-0.03*0.002)/0.38, com_z_m=0.0),
        ])
        saved = copy.deepcopy(model)
        report = analyze_fixture(model, samples=9, stiffness_factors=(1.0,),
                                 preload_offsets_m=(0.0,), loss_ratios=(0.0,))
        loaded = report["nominal_curves"][1]
        self.assertAlmostEqual(loaded["baseline_gravity_nm"][4], -9.81*link_moment)
        self.assertAlmostEqual(loaded["candidate_gravity_nm"][4], -9.81*(link_moment-0.00006))
        self.assertEqual(loaded["passive_torque_nm"], [0.0]*9)
        self.assertEqual(model, saved)

    def test_infeasible_uncertainty_is_recorded_not_clipped_or_dropped(self):
        report = analyze_fixture(fixture(), samples=9, stiffness_factors=(1.0,),
                                 preload_offsets_m=(-0.02,), loss_ratios=(0.0, 0.1))
        self.assertEqual(len(report["metrics"]), 4)
        for row in report["metrics"]:
            self.assertFalse(row["valid"])
            self.assertIsNone(row["candidate_motor_rms_nm"])
            self.assertTrue(row["failure"])

    def test_crossovers_preserve_worst_case_failure_instead_of_selecting_best_loss(self):
        model = fixture()
        model["cam"]["cos_m"][0] = -0.002
        model["cases"][0]["preload_m"] = 0.0049
        options = dict(samples=31, stiffness_factors=(1.0,),
                       preload_offsets_m=(0.0,), loss_ratios=(0.0, 0.5, 1.0, 2.0))
        report = analyze_fixture(model, **options)
        crossover = report["crossovers"][0]
        self.assertEqual(crossover["sampled_loss_ratio_bracket"], [0.5, 1.0])
        self.assertLess(crossover["loss_scan"][-1]["worst_direction_benefit_fraction"], 0)
        self.assertEqual(report["config_sha256"], analyze_fixture(model, **options)["config_sha256"])

    def test_invalid_scan_configuration_and_ambiguous_input_are_rejected(self):
        for options in ({"samples": 1}, {"samples": 9.5}, {"loss_ratios": (-0.1,)},
                        {"stiffness_factors": (0.0,)}, {"preload_offsets_m": (float("nan"),)}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                analyze_fixture(fixture(), **options)
        for alteration in ("empty_cases", "duplicate_body", "negative_payload", "invalid_gravity"):
            model = fixture()
            if alteration == "empty_cases": model["cases"] = []
            elif alteration == "duplicate_body": model["rigid_bodies"].append(copy.deepcopy(model["rigid_bodies"][0]))
            elif alteration == "negative_payload": model["cases"][0]["payload_mass_kg"] = -1.0
            else: model["gravity_m_s2"] = float("nan")
            with self.subTest(alteration=alteration), self.assertRaises(ValueError):
                analyze_fixture(model, samples=9, stiffness_factors=(1.0,),
                                preload_offsets_m=(0.0,), loss_ratios=(0.0,))


if __name__ == "__main__":
    unittest.main()
