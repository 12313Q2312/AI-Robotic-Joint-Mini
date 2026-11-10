#!/usr/bin/env python3
"""Reproduce the Layer 1 paired metrics.

Committed outputs are the summary, the markdown table, the per-trial CSV, and
one small sample. The full raw JSON is written to --raw-dir, which is gitignored
for the default path, and CI uploads that file as an artifact.

`--check` compares the committed summary files. It still writes the raw JSON.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from knee_stair_l1.config import load_config
from knee_stair_l1.telemetry import (
    CSV_FIELDS,
    format_raw_json,
    format_summary_markdown,
    attach_l2_scenarios,
    raw_document,
    require_boolean_criteria,
    run_study,
    summarize,
    trials_to_rows,
)

DEFAULT_DIR = ROOT / "data" / "verification" / "knee_stair_l1"
DEFAULT_RAW_DIR = DEFAULT_DIR / "local_raw"


def _csv_text(trials) -> str:
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in trials_to_rows(trials):
        writer.writerow({key: row[key] if not isinstance(row[key], float) else f"{row[key]:.10f}" for key in CSV_FIELDS})
    return buffer.getvalue()


def _sample(trials) -> dict:
    rows = []
    for trial in trials:
        if trial.seed != 0:
            continue
        record = {
            "seed": trial.seed,
            "condition": trial.condition,
            "source_label": trial.source_label,
            "factors": None if trial.factors is None else {
                key: round(value, 10) if isinstance(value, float) else value
                for key, value in trial.factors.as_dict().items()
            },
            "human_torque_rms_nm": round(trial.human_torque_rms_nm, 10),
            "human_positive_work_j": round(trial.human_positive_work_j, 10),
            "tracking_error_rms_rad": round(trial.tracking_error_rms_rad, 10),
            "peak_interaction_torque_nm": round(trial.peak_interaction_torque_nm, 10),
            "saturation_time_fraction": round(trial.saturation_time_fraction, 10),
            "hard_stop_contact_count": trial.hard_stop_contact_count,
            "latency_note": "simulation loop latency, not real-system latency",
            "latency_samples_head_s": [round(value, 10) for value in trial.latency_samples_s[:8]],
        }
        rows.append(record)
    return {
        "evidence_label": "experimental",
        "sample_seed": 0,
        "note": "One paired seed. The full per-step raw file is a CI artifact, not a committed file.",
        "trials": rows,
    }


def build_outputs(config):
    trials = run_study(config)
    summary = attach_l2_scenarios(summarize(trials, config), config)
    require_boolean_criteria(summary)
    committed = {
        "metrics_summary.json": json.dumps(summary, indent=2, sort_keys=True) + "\n",
        "metrics_summary.md": format_summary_markdown(summary),
        "metrics_trials.csv": _csv_text(trials),
        "metrics_sample.json": json.dumps(_sample(trials), indent=2, sort_keys=True) + "\n",
    }
    raw = format_raw_json(raw_document(trials, summary))
    return committed, raw


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Layer 1 synthetic knee-stair metrics")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--raw-dir", type=Path, default=DEFAULT_RAW_DIR)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    committed, raw = build_outputs(config)
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    (args.raw_dir / "metrics_raw.json").write_text(raw, encoding="utf-8")
    if args.check:
        on_disk = json.loads((args.output_dir / "metrics_summary.json").read_text(encoding="utf-8"))
        try:
            require_boolean_criteria(on_disk)
        except ValueError as exc:
            print(f"FAIL: {exc}")
            return 1
        mismatched = [
            name for name, text in committed.items()
            if not (args.output_dir / name).is_file() or (args.output_dir / name).read_text(encoding="utf-8") != text
        ]
        if mismatched:
            print("MISMATCH: " + ", ".join(mismatched))
            return 1
        print(f"PASS: Layer 1 summary matches {args.output_dir}")
        print(f"Wrote raw artifact source {args.raw_dir / 'metrics_raw.json'}")
        return 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, text in committed.items():
        (args.output_dir / name).write_text(text, encoding="utf-8")
    print(committed["metrics_summary.md"])
    print(f"Wrote raw artifact source {args.raw_dir / 'metrics_raw.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
