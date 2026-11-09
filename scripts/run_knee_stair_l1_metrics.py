#!/usr/bin/env python3
"""Reproduce the Layer 1 paired metrics.

Writes data/verification/knee_stair_l1/metrics_raw.json and the summary.
`--check` regenerates the study and compares it with the committed files.
"""

from __future__ import annotations

import argparse
import csv
import io
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
    raw_document,
    run_study,
    summarize,
    trials_to_rows,
)

DEFAULT_DIR = ROOT / "data" / "verification" / "knee_stair_l1"


def _csv_text(trials) -> str:
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    for row in trials_to_rows(trials):
        writer.writerow({key: row[key] if not isinstance(row[key], float) else f"{row[key]:.10f}" for key in CSV_FIELDS})
    return buffer.getvalue()


def build_outputs(config):
    trials = run_study(config)
    summary = summarize(trials, config)
    document = raw_document(trials, summary)
    return {
        "metrics_raw.json": format_raw_json(document),
        "metrics_summary.md": format_summary_markdown(summary),
        "metrics_trials.csv": _csv_text(trials),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Layer 1 synthetic knee-stair metrics")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--check", action="store_true", help="compare a fresh run with the committed files")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    outputs = build_outputs(config)
    if args.check:
        mismatched = []
        for name, text in outputs.items():
            path = args.output_dir / name
            if not path.is_file() or path.read_text(encoding="utf-8") != text:
                mismatched.append(name)
        if mismatched:
            print("MISMATCH: " + ", ".join(mismatched))
            return 1
        print(f"PASS: Layer 1 metrics match {args.output_dir}")
        return 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, text in outputs.items():
        (args.output_dir / name).write_text(text, encoding="utf-8")
    print(outputs["metrics_summary.md"])
    print(f"Wrote {args.output_dir / 'metrics_raw.json'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
