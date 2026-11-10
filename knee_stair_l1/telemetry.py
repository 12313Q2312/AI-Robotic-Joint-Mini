"""Paired metrics for the synthetic knee loop.

Primary, each as assisted minus the paired OFF trial is reported as a
reduction (OFF - assisted):
- human knee torque RMS
- human knee positive work

Secondary:
- tracking error RMS
- peak strap interaction torque
- saturation time fraction
- hard-stop contact count

Latency is the measured command timestamp minus the consumed intent timestamp.
Percentiles use linear interpolation: rank = (n - 1) * p / 100.

The 95% CI is the Student t interval with df = n - 1. For the reported 30
seeds, df = 29 and t_0.975 = 2.045229642132703. The reduction criterion passes
only when the lower bound is positive.

The fixed-queue p50/p95/max are simulation loop latency, not real-system latency.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

from knee_stair_l1.config import load_config
from knee_stair_l1.loop import TrialResult, run_trial

PRIMARY = ("human_torque_rms_nm", "human_positive_work_j")
SECONDARY = (
    "tracking_error_rms_rad",
    "peak_interaction_torque_nm",
    "saturation_time_fraction",
    "hard_stop_contact_count",
)
METRICS = PRIMARY + SECONDARY
CONDITIONS = ("off", "admittance", "assist_policy")
BOOLEAN_CRITERIA = (
    "human_torque_rms_reduction_ci_excludes_0",
    "tracking_error_no_more_than_10_percent_worse",
    "hard_stop_contacts_zero",
    "peak_interaction_below_comfort",
    "saturation_time_fraction_below_5_percent",
)
T_CRIT_95 = {11: 2.200985160082949, 29: 2.045229642132703}


def run_study(config: dict | None = None, scenario: str = "nominal") -> list[TrialResult]:
    cfg = config if config is not None else load_config()
    trials = []
    for seed in cfg["seeds"]:
        for condition in CONDITIONS:
            trials.append(run_trial(cfg, int(seed), condition, scenario=scenario))
    return trials


def percentile(values: list[float], percent: float) -> float:
    if not values:
        raise ValueError("percentile of an empty sample")
    if percent < 0 or percent > 100:
        raise ValueError("percent must be between 0 and 100")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (percent / 100.0)
    low = math.floor(rank)
    high = math.ceil(rank)
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def _t_crit(n: int) -> float:
    df = n - 1
    if df not in T_CRIT_95:
        raise ValueError(f"no 95% t critical value tabulated for df={df}")
    return T_CRIT_95[df]


def _stats(values: list[float]) -> dict:
    n = len(values)
    mean = statistics.mean(values)
    std = statistics.stdev(values)
    half = _t_crit(n) * std / math.sqrt(n)
    low = mean - half
    high = mean + half
    return {
        "n": n,
        "mean": _round(mean),
        "std": _round(std),
        "ci95_low": _round(low),
        "ci95_high": _round(high),
        "ci95_excludes_0": low > 0.0 or high < 0.0,
    }


def _round(value: float) -> float:
    return round(float(value), 12)


def _latency_summary(samples: list[float]) -> dict:
    return {
        "n": len(samples),
        "p50_s": _round(percentile(samples, 50)),
        "p95_s": _round(percentile(samples, 95)),
        "max_s": _round(max(samples)),
    }


def summarize(trials: list[TrialResult], config: dict) -> dict:
    by_key = {(trial.seed, trial.condition): trial for trial in trials}
    absolute = {}
    for condition in CONDITIONS:
        rows = [by_key[(int(seed), condition)] for seed in config["seeds"]]
        absolute[condition] = {metric: _stats([getattr(row, metric) for row in rows]) for metric in METRICS}
        pooled = [sample for row in rows for sample in row.latency_samples_s]
        absolute[condition]["latency"] = _latency_summary(pooled)
    reductions = {}
    for condition in ("admittance", "assist_policy"):
        reductions[condition] = {}
        for metric in PRIMARY:
            deltas = []
            for seed in config["seeds"]:
                off = by_key[(int(seed), "off")]
                here = by_key[(int(seed), condition)]
                deltas.append(getattr(off, metric) - getattr(here, metric))
            reductions[condition][metric] = _stats(deltas)
    criteria = {}
    comfort = float(config["metrics"]["comfort_peak_interaction_nm"])
    ratio_max = float(config["metrics"]["tracking_worse_ratio_max"])
    sat_max = float(config["metrics"]["admittance_saturation_fraction_max"])
    for condition in ("admittance", "assist_policy"):
        torque_ci = reductions[condition]["human_torque_rms_nm"]
        tracking_ok = True
        stops_ok = True
        interaction_ok = True
        ratios = []
        for seed in config["seeds"]:
            off = by_key[(int(seed), "off")]
            here = by_key[(int(seed), condition)]
            if off.tracking_error_rms_rad == 0.0:
                tracking_ok = tracking_ok and here.tracking_error_rms_rad == 0.0
                ratios.append(1.0 if here.tracking_error_rms_rad == 0.0 else math.inf)
            else:
                ratio = here.tracking_error_rms_rad / off.tracking_error_rms_rad
                ratios.append(ratio)
                tracking_ok = tracking_ok and ratio <= ratio_max + 1e-12
            stops_ok = stops_ok and here.hard_stop_contact_count == 0 and off.hard_stop_contact_count == 0
            interaction_ok = interaction_ok and here.peak_interaction_torque_nm < comfort
        sat_values = [by_key[(int(seed), condition)].saturation_time_fraction for seed in config["seeds"]]
        sat_mean = statistics.mean(sat_values)
        criteria[condition] = {
            "human_torque_rms_reduction_ci_excludes_0": bool(torque_ci["ci95_low"] > 0.0),
            "human_torque_rms_reduction_ci95": [torque_ci["ci95_low"], torque_ci["ci95_high"]],
            "tracking_error_no_more_than_10_percent_worse": tracking_ok,
            "tracking_error_ratio_max": _round(max(ratios)),
            "hard_stop_contacts_zero": stops_ok,
            "peak_interaction_below_comfort": interaction_ok,
            "comfort_peak_interaction_nm": comfort,
            "saturation_time_fraction_mean": _round(sat_mean),
            "saturation_time_fraction_below_5_percent": bool(sat_mean < sat_max),
        }
    config_bytes = Path(config["_config_path"]).read_bytes() if config.get("_config_path") else json.dumps(
        config, sort_keys=True
    ).encode()
    if not _primary_std_positive(absolute):
        raise AssertionError("primary metrics must have std > 0 across seeds")
    require_boolean_criteria({"criteria": criteria})
    return {
        "evidence_label": "experimental",
        "simulation_only": True,
        "intent_source_label": "synthetic",
        "not_claimed": ["real_humans", "real_emg", "fall_prevention"],
        "removed_metrics": ["assist_timing_error_s"],
        "seed_count": len(config["seeds"]),
        "seeds": list(config["seeds"]),
        "conditions": list(CONDITIONS),
        "std_ddof": 1,
        "ci_method": "Student t, df=n-1. For 30 seeds df=29 and t_0.975=2.045229642132703. mean ± t*std/sqrt(n)",
        "latency_definition": "simulation loop latency, not real-system latency: command_timestamp_s - intent_timestamp_s of the sample consumed by that command",
        "percentile_method": "linear interpolation, rank (n-1)*p/100",
        "interaction_definition": "peak absolute spring-damper strap torque on the human knee",
        "pid_torque_limit_nm": config["pid"]["torque_limit_nm"],
        "comfort_peak_interaction_nm": comfort,
        "comfort_basis": config["metrics"]["comfort_basis"],
        "load_assumption": config["load"]["assumption"],
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "absolute": absolute,
        "reduction_versus_off": reductions,
        "criteria": criteria,
        "primary_std_positive": _primary_std_positive(absolute),
        "positive_work_comparison": _work_comparison(reductions),
    }


def _check_criteria_block(criteria: dict, label: str) -> None:
    if not isinstance(criteria, dict) or not criteria:
        raise ValueError(f"{label} criteria must be a non-empty object")
    for condition, row in criteria.items():
        if not isinstance(row, dict):
            raise ValueError(f"{label}.{condition} criteria must be an object")
        for key in BOOLEAN_CRITERIA:
            value = row.get(key)
            if not isinstance(value, bool):
                raise ValueError(f"{label}.{condition}.{key} must be a boolean, got {value!r}")


def require_boolean_criteria(summary: dict) -> None:
    """Pass-criterion flags are booleans for every condition, including AssistPolicy."""
    _check_criteria_block(summary.get("criteria"), "summary")
    for name, block in summary.get("scenarios", {}).items():
        if isinstance(block, dict) and "criteria" in block:
            _check_criteria_block(block["criteria"], f"scenarios.{name}")


def _primary_std_positive(absolute: dict) -> bool:
    for condition in CONDITIONS:
        for metric in PRIMARY:
            if absolute[condition][metric]["std"] <= 0.0:
                return False
    return True


def _work_comparison(reductions: dict) -> str:
    admit = reductions["admittance"]["human_positive_work_j"]["mean"]
    policy = reductions["assist_policy"]["human_positive_work_j"]["mean"]
    if admit < policy:
        return (
            "Admittance has a smaller positive-work reduction than AssistPolicy. "
            "That ordering is a result of this run, not a tuning target."
        )
    if admit > policy:
        return "Admittance has a larger positive-work reduction than AssistPolicy in this run."
    return "Admittance and AssistPolicy have the same positive-work reduction in this run."


def scenario_advantage(trials: list[TrialResult], config: dict) -> dict:
    """Per-seed admittance reduction minus AssistPolicy reduction. Positive favours admittance."""
    by_key = {(trial.seed, trial.condition): trial for trial in trials}
    compared = {}
    for metric in PRIMARY:
        deltas = []
        for seed in config["seeds"]:
            off = by_key[(int(seed), "off")]
            admit = by_key[(int(seed), "admittance")]
            policy = by_key[(int(seed), "assist_policy")]
            admit_reduction = getattr(off, metric) - getattr(admit, metric)
            policy_reduction = getattr(off, metric) - getattr(policy, metric)
            deltas.append(admit_reduction - policy_reduction)
        compared[metric] = _stats(deltas)
    return compared


def attach_l2_scenarios(summary: dict, config: dict) -> dict:
    """Add the three L2 scenarios and the default-controller decision. Nominal stays the top level."""
    from knee_stair_l1.intent import SCENARIOS

    scenarios = {}
    advantages = {}
    for scenario in SCENARIOS:
        trials = run_study(config, scenario=scenario)
        block = summarize(trials, config)
        scenarios[scenario] = {
            "reduction_versus_off": block["reduction_versus_off"],
            "absolute": {condition: {metric: block["absolute"][condition][metric] for metric in METRICS} for condition in CONDITIONS},
            "criteria": block["criteria"],
        }
        if scenario != "nominal":
            advantages[scenario] = scenario_advantage(trials, config)
    promoted = True
    for scenario, compared in advantages.items():
        for metric in PRIMARY:
            if compared[metric]["ci95_low"] <= 0.0:
                promoted = False
    configured = config.get("default_controller", "assist_policy")
    summary["scenarios"] = scenarios
    summary["admittance_minus_assist_policy"] = advantages
    summary["default_controller"] = {
        "configured": configured,
        "recommended": "admittance" if promoted else "assist_policy",
        "promoted_admittance": promoted,
        "rule": (
            "Promote admittance only if, on every new scenario and both primary metrics, "
            "the 95% CI of admittance reduction minus AssistPolicy reduction is entirely above 0."
        ),
    }
    require_boolean_criteria(summary)
    return summary


def _event_dict(event) -> dict:
    return {
        "time_s": _round(event.time_s),
        "side": event.side,
        "stop": event.stop,
        "angle_rad": _round(event.angle_rad),
    }


def trial_record(trial: TrialResult) -> dict:
    return {
        "seed": trial.seed,
        "condition": trial.condition,
        "source_label": trial.source_label,
        "depth_scale": _round(trial.depth_scale),
        "human_torque_rms_nm": _round(trial.human_torque_rms_nm),
        "human_positive_work_j": _round(trial.human_positive_work_j),
        "tracking_error_rms_rad": _round(trial.tracking_error_rms_rad),
        "peak_interaction_torque_nm": _round(trial.peak_interaction_torque_nm),
        "saturation_time_fraction": _round(trial.saturation_time_fraction),
        "hard_stop_contact_count": trial.hard_stop_contact_count,
        "hard_stop_events": [_event_dict(event) for event in trial.hard_stop_events],
        "latency_p50_s": _round(percentile(trial.latency_samples_s, 50)),
        "latency_p95_s": _round(percentile(trial.latency_samples_s, 95)),
        "latency_max_s": _round(max(trial.latency_samples_s)),
        "latency_samples_s": [_round(sample) for sample in trial.latency_samples_s],
        "max_abs_motor_command_nm": _round(trial.max_abs_motor_command_nm),
        "final_flexion_rad": _round(trial.final_flexion_rad),
    }


def raw_document(trials: list[TrialResult], summary: dict) -> dict:
    return {
        "evidence_label": "experimental",
        "intent_source_label": "synthetic",
        "summary": summary,
        "trials": [trial_record(trial) for trial in trials],
    }


def format_raw_json(document: dict) -> str:
    return json.dumps(document, indent=2, sort_keys=True) + "\n"


def _fmt(stats: dict) -> str:
    return f"{stats['mean']:.6f} ± {stats['std']:.6f}"


def _fmt_ci(stats: dict) -> str:
    return f"[{stats['ci95_low']:.6f}, {stats['ci95_high']:.6f}]"


def format_summary_markdown(summary: dict) -> str:
    lines = [
        "# Layer 1 paired metrics",
        "",
        "Experimental simulation. Synthetic intent. Not a human, EMG, or fall-prevention result.",
        "",
        f"Seeds: {summary['seed_count']} ({summary['seeds'][0]}..{summary['seeds'][-1]}). "
        "Sample std (n-1). 95% CI is the Student t interval.",
        "",
        summary["positive_work_comparison"],
        "",
        "Latency p50/p95/max below are simulation loop latency, not real-system latency.",
        "",
        f"Comfort threshold: {summary['comfort_peak_interaction_nm']} Nm. {summary['comfort_basis']}",
        "",
        f"Load: {summary['load_assumption']}",
        "",
        "The censored timing-error metric is not used.",
        "",
        "## Primary reductions versus OFF (OFF − assisted), mean ± std, 95% CI",
        "",
        "| Metric | admittance | admittance 95% CI | AssistPolicy baseline | AssistPolicy 95% CI |",
        "|---|---:|---:|---:|---:|",
    ]
    for metric in PRIMARY:
        cells = []
        for condition in ("admittance", "assist_policy"):
            stats = summary["reduction_versus_off"][condition][metric]
            cells.extend((_fmt(stats), _fmt_ci(stats)))
        lines.append("| " + " | ".join([metric, *cells]) + " |")
    lines.extend(
        [
            "",
            "## Secondary metrics, mean ± std",
            "",
            "| Metric | OFF | admittance | AssistPolicy baseline |",
            "|---|---:|---:|---:|",
        ]
    )
    for metric in SECONDARY:
        cells = [_fmt(summary["absolute"][condition][metric]) for condition in CONDITIONS]
        lines.append("| " + " | ".join([metric, *cells]) + " |")
    lines.extend(
        [
            "",
            "## Measured latency, pooled over seeds (command time − intent time)",
            "",
            "| Condition | p50 s | p95 s | max s |",
            "|---|---:|---:|---:|",
        ]
    )
    for condition in CONDITIONS:
        latency = summary["absolute"][condition]["latency"]
        lines.append(
            f"| {condition} | {latency['p50_s']:.6f} | {latency['p95_s']:.6f} | {latency['max_s']:.6f} |"
        )
    lines.extend(["", "## Criteria", ""])
    for condition, result in summary["criteria"].items():
        lines.append(f"### {condition}")
        lines.append("")
        for key, value in result.items():
            lines.append(f"- {key}: {value}")
        lines.append("")
    if "scenarios" in summary:
        lines.extend(["## L2 scenarios, primary reduction versus OFF", ""])
        lines.append("| Scenario | Metric | admittance | AssistPolicy |")
        lines.append("|---|---|---:|---:|")
        for scenario, block in summary["scenarios"].items():
            for metric in PRIMARY:
                cells = []
                for condition in ("admittance", "assist_policy"):
                    stats = block["reduction_versus_off"][condition][metric]
                    cells.append(f"{stats['mean']:.6f} ± {stats['std']:.6f}")
                lines.append("| " + " | ".join([scenario, metric, *cells]) + " |")
        decision = summary.get("default_controller", {})
        lines.extend(["", f"Configured default controller: {decision.get('configured')}.", f"Recommended: {decision.get('recommended')}.", ""])
    return "\n".join(lines)


CSV_FIELDS = (
    "seed",
    "condition",
    "source_label",
    "depth_scale",
    "human_torque_rms_nm",
    "human_positive_work_j",
    "tracking_error_rms_rad",
    "peak_interaction_torque_nm",
    "saturation_time_fraction",
    "hard_stop_contact_count",
    "latency_p50_s",
    "latency_p95_s",
    "latency_max_s",
    "final_flexion_rad",
)


def trials_to_rows(trials: list[TrialResult]) -> list[dict]:
    rows = []
    for trial in trials:
        record = trial_record(trial)
        rows.append({key: record[key] for key in CSV_FIELDS})
    return rows
