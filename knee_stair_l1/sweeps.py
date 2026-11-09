"""Latency-jitter and load/comfort sensitivity sweeps.

These reuse the reported seeds. They are evaluations, not a second tuning set.
Controller gains are not changed here.
"""

from __future__ import annotations

import copy

from knee_stair_l1.loop import run_trial
from knee_stair_l1.telemetry import CONDITIONS, PRIMARY, run_study, summarize


def _mean(trials, condition: str, metric: str) -> float:
    rows = [trial for trial in trials if trial.condition == condition]
    return sum(getattr(row, metric) for row in rows) / len(rows)


def _failed_reasons(trials, summary) -> list[str]:
    reasons = []
    for condition in ("admittance", "assist_policy"):
        crit = summary["criteria"][condition]
        if not crit["human_torque_rms_reduction_ci_excludes_0"]:
            reasons.append(f"{condition}: human torque RMS reduction 95% CI does not stay above 0")
        if not crit["tracking_error_no_more_than_10_percent_worse"]:
            reasons.append(f"{condition}: tracking error is more than 10% worse than OFF")
        if not crit["hard_stop_contacts_zero"]:
            reasons.append(f"{condition}: hard-stop contact")
        if not crit["peak_interaction_below_comfort"]:
            reasons.append(f"{condition}: peak interaction torque is at or above the comfort threshold")
        if condition == "admittance" and not crit["saturation_time_fraction_below_5_percent"]:
            reasons.append("admittance: saturation time fraction is not below 5%")
        if any(trial.non_normal_step_fraction > 0.0 for trial in trials if trial.condition == condition):
            reasons.append(f"{condition}: supervisor left NORMAL")
    return reasons


def latency_sweep(config: dict, delays_s: list[float] | None = None) -> dict:
    grid = [0.0, 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035, 0.04, 0.045, 0.05] if delays_s is None else delays_s
    points = []
    first_failure = None
    for delay in grid:
        trials = [
            run_trial(config, int(seed), condition, jitter_max_s=delay)
            for seed in config["seeds"]
            for condition in CONDITIONS
        ]
        summary = summarize(trials, config)
        reasons = _failed_reasons(trials, summary)
        if reasons and first_failure is None:
            first_failure = {"delay_s": delay, "reasons": reasons}
        point = {"delay_s": delay, "delay_ms": round(delay * 1000.0, 3), "failed": bool(reasons), "reasons": reasons}
        for condition in ("admittance", "assist_policy"):
            point[condition] = {
                "human_torque_rms_reduction_nm": summary["reduction_versus_off"][condition]["human_torque_rms_nm"]["mean"],
                "human_positive_work_reduction_j": summary["reduction_versus_off"][condition]["human_positive_work_j"]["mean"],
                "peak_interaction_torque_nm": _mean(trials, condition, "peak_interaction_torque_nm"),
                "human_torque_rms_ci95": summary["reduction_versus_off"][condition]["human_torque_rms_nm"]["ci95_low"],
            }
        points.append(point)
    if first_failure is None:
        conclusion = "No delay from 0 to 50 ms made assist worse than OFF or left the supervisor in a degraded state."
    else:
        conclusion = (
            f"At {first_failure['delay_s'] * 1000:.0f} ms of uniform intent delay, "
            + "; ".join(first_failure["reasons"])
        )
    return {
        "evidence_label": "experimental",
        "delay_model": "Each step draws a command-hold delay uniformly from 0 to the sweep value and applies that earlier supervised torque. The same draw sequence is paired across conditions. This injected delay is separate from the fixed simulation-loop intent queue.",
        "worse_than_off_means": "Torque-RMS reduction CI does not stay above 0, tracking is more than 10% worse than OFF, a hard stop occurs, admittance saturation reaches 5%, interaction reaches the comfort threshold, or the supervisor leaves NORMAL.",
        "points": points,
        "threshold": first_failure,
        "conclusion": conclusion,
    }


def sensitivity_sweep(config: dict) -> dict:
    nominal_trials = run_study(config)
    nominal = summarize(nominal_trials, config)
    cases = [{"name": "nominal", "scale": 1.0, "kind": "baseline", "criteria": nominal["criteria"]}]
    keys = (
        "human_torque_rms_reduction_ci_excludes_0",
        "tracking_error_no_more_than_10_percent_worse",
        "hard_stop_contacts_zero",
        "peak_interaction_below_comfort",
        "saturation_time_fraction_below_5_percent",
    )

    def flipped(criteria) -> list[str]:
        found = []
        for condition in ("admittance", "assist_policy"):
            for key in keys:
                if criteria[condition][key] != nominal["criteria"][condition][key]:
                    found.append(f"{condition}.{key}: {nominal['criteria'][condition][key]} -> {criteria[condition][key]}")
        return found

    for scale in (0.7, 1.3):
        scaled = copy.deepcopy(config)
        scaled["load"]["moment_arm_m"] = float(config["load"]["moment_arm_m"]) * scale
        summary = summarize(run_study(scaled), scaled)
        cases.append({
            "name": f"load_moment_arm_x{scale}",
            "scale": scale,
            "kind": "load",
            "criteria": summary["criteria"],
            "flips": flipped(summary["criteria"]),
        })
        comfort = copy.deepcopy(config)
        comfort["metrics"]["comfort_peak_interaction_nm"] = float(config["metrics"]["comfort_peak_interaction_nm"]) * scale
        rescored = summarize(nominal_trials, comfort)
        cases.append({
            "name": f"comfort_threshold_x{scale}",
            "scale": scale,
            "kind": "comfort",
            "criteria": rescored["criteria"],
            "flips": flipped(rescored["criteria"]),
        })
    any_flip = [case["name"] for case in cases if case.get("flips")]
    if any_flip:
        conclusion = "At least one pass/fail conclusion changes: " + ", ".join(any_flip) + "."
    else:
        conclusion = "No pass/fail conclusion flips when the load moment arm or the comfort threshold is scaled by ±30%."
    return {
        "evidence_label": "experimental",
        "load_parameter": "load.moment_arm_m, which scales the gravity-torque profile. Body-mass and step-height draws stay active.",
        "comfort_parameter": "metrics.comfort_peak_interaction_nm. Rescored on the nominal trials because the threshold does not change the motion.",
        "cases": cases,
        "conclusion": conclusion,
        "primary_metrics_checked": list(PRIMARY),
    }
