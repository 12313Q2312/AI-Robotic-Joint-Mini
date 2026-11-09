"""Trial metrics and the paired OFF comparison.

Official metrics, each reported per condition and as a paired difference
from the OFF trial with the same seed:

- assist_timing_error_s: assist-onset time minus synthetic-intent onset.
  Onset of intent is the first sample whose synthetic effort reaches
  intent.onset_effort_nm. Onset of assist is the first applied exo torque
  whose magnitude reaches metrics.assist_onset_nm. If assist never onsets,
  the error is censored to duration_s - intent_onset_s.
- end_to_end_latency_s: mean, over control steps, of
  (apply time - supervisor production time). The transport queue sets this.
- peak_interaction_torque_nm: peak magnitude of the torque that crossed
  InteractionBoundary. Human muscle torque is not included.
- saturation_limit_trigger_count: control steps on which the PID saturated,
  AssistPolicy saturated, or the supervisor state was LIMITED_ASSIST.
  Rising-edge count is stored beside it.

Mean and std use the sample standard deviation (n - 1).
"""

from __future__ import annotations

from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
import statistics

from knee_stair_l1.config import load_config
from knee_stair_l1.loop import TrialResult, run_trial

METRICS = (
    "assist_timing_error_s",
    "end_to_end_latency_s",
    "peak_interaction_torque_nm",
    "saturation_limit_trigger_count",
)
CONDITIONS = ("off", "admittance", "assist_policy")
CONDITION_LABELS = {
    "off": "OFF",
    "admittance": "admittance",
    "assist_policy": "AssistPolicy baseline",
}

CSV_FIELDS = (
    "seed",
    "condition",
    "source_label",
    "amplitude_nm",
    "intent_onset_s",
    "assist_onset_s",
    "assist_onset",
    "assist_timing_error_s",
    "end_to_end_latency_s",
    "peak_interaction_torque_nm",
    "saturation_limit_trigger_count",
    "saturation_limit_rising_edges",
    "final_flexion_rad",
)


def run_study(config: dict | None = None) -> list[TrialResult]:
    cfg = config if config is not None else load_config()
    trials = []
    for seed in cfg["seeds"]:
        for condition in CONDITIONS:
            trials.append(run_trial(cfg, int(seed), condition))
    return trials


def _mean_std(values: list[float]) -> dict:
    return {
        "n": len(values),
        "mean": round(statistics.mean(values), 12),
        "std": round(statistics.stdev(values), 12),
    }


def summarize(trials: list[TrialResult], config: dict) -> dict:
    by_condition: dict[str, list[TrialResult]] = defaultdict(list)
    by_key = {}
    for trial in trials:
        by_condition[trial.condition].append(trial)
        by_key[(trial.seed, trial.condition)] = trial
    absolute = {}
    for condition in CONDITIONS:
        rows = by_condition[condition]
        absolute[condition] = {metric: _mean_std([getattr(row, metric) for row in rows]) for metric in METRICS}
    paired = {}
    for condition in ("admittance", "assist_policy"):
        paired[condition] = {}
        for metric in METRICS:
            deltas = []
            for seed in config["seeds"]:
                here = by_key[(int(seed), condition)]
                off = by_key[(int(seed), "off")]
                deltas.append(getattr(here, metric) - getattr(off, metric))
            paired[condition][metric] = _mean_std(deltas)
    config_bytes = Path(config["_config_path"]).read_bytes() if config.get("_config_path") else json.dumps(config, sort_keys=True).encode()
    return {
        "evidence_label": "experimental",
        "simulation_only": True,
        "intent_source_label": "synthetic",
        "not_claimed": ["real_humans", "real_emg", "fall_prevention"],
        "seed_count": len(config["seeds"]),
        "seeds": list(config["seeds"]),
        "conditions": list(CONDITIONS),
        "std_ddof": 1,
        "timing_censor": "duration_s - intent_onset_s when assist does not reach assist_onset_nm",
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "absolute": absolute,
        "paired_minus_off": paired,
        "definitions": {
            "assist_timing_error_s": "assist onset minus synthetic effort onset; censored to the horizon if assist stays below assist_onset_nm",
            "end_to_end_latency_s": "mean apply-time minus supervisor production-time over control steps",
            "peak_interaction_torque_nm": "peak absolute torque across the human/exoskeleton boundary",
            "saturation_limit_trigger_count": "control steps with PID saturation, AssistPolicy saturation, or LIMITED_ASSIST",
        },
    }


def trials_to_rows(trials: list[TrialResult]) -> list[dict]:
    rows = []
    for trial in trials:
        rows.append(
            {
                "seed": trial.seed,
                "condition": trial.condition,
                "source_label": trial.source_label,
                "amplitude_nm": f"{trial.amplitude_nm:.10f}",
                "intent_onset_s": f"{trial.intent_onset_s:.10f}",
                "assist_onset_s": "" if trial.assist_onset_s is None else f"{trial.assist_onset_s:.10f}",
                "assist_onset": "1" if trial.assist_onset else "0",
                "assist_timing_error_s": f"{trial.assist_timing_error_s:.10f}",
                "end_to_end_latency_s": f"{trial.end_to_end_latency_s:.10f}",
                "peak_interaction_torque_nm": f"{trial.peak_interaction_torque_nm:.10f}",
                "saturation_limit_trigger_count": str(trial.saturation_limit_trigger_count),
                "saturation_limit_rising_edges": str(trial.saturation_limit_rising_edges),
                "final_flexion_rad": f"{trial.final_flexion_rad:.10f}",
            }
        )
    return rows


def write_csv(path: Path, trials: list[TrialResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(trials_to_rows(trials))


def format_summary_json(summary: dict) -> str:
    return json.dumps(summary, indent=2, sort_keys=True) + "\n"


def format_summary_markdown(summary: dict) -> str:
    lines = [
        "# Layer 1 paired metrics",
        "",
        "Experimental simulation results. Synthetic intent. Not a human, EMG, or fall-prevention result.",
        "",
        f"Seeds: {summary['seed_count']} ({summary['seeds'][0]}..{summary['seeds'][-1]}). Sample std (n-1).",
        "",
        "## Absolute, mean ± std",
        "",
        "| Metric | OFF | admittance | AssistPolicy baseline |",
        "|---|---:|---:|---:|",
    ]
    for metric in METRICS:
        cells = []
        for condition in CONDITIONS:
            stats = summary["absolute"][condition][metric]
            cells.append(f"{stats['mean']:.6f} ± {stats['std']:.6f}")
        lines.append("| " + " | ".join([metric, *cells]) + " |")
    lines.extend(
        [
            "",
            "## Paired difference versus OFF, mean ± std",
            "",
            "| Metric | admittance − OFF | AssistPolicy baseline − OFF |",
            "|---|---:|---:|",
        ]
    )
    for metric in METRICS:
        cells = []
        for condition in ("admittance", "assist_policy"):
            stats = summary["paired_minus_off"][condition][metric]
            cells.append(f"{stats['mean']:.6f} ± {stats['std']:.6f}")
        lines.append("| " + " | ".join([metric, *cells]) + " |")
    lines.extend(
        [
            "",
            "Timing for OFF is censored to the horizon because assist torque stays at 0.",
            "Latency is the configured transport delay and is shared by every condition.",
            "",
        ]
    )
    return "\n".join(lines)


def write_summary(directory: Path, summary: dict) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "metrics_summary.json").write_text(format_summary_json(summary), encoding="utf-8")
    (directory / "metrics_summary.md").write_text(format_summary_markdown(summary), encoding="utf-8")
