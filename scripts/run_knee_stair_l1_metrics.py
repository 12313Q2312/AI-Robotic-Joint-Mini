#!/usr/bin/env python3
"""Reproduce the Layer 1 paired metric table.

Writes data/verification/knee_stair_l1/metrics_trials.csv and the summary.
`--check` regenerates the study in memory and compares it to those files.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from knee_stair_l1.config import load_config
from knee_stair_l1.telemetry import (
    format_summary_json,
    format_summary_markdown,
    run_study,
    summarize,
    trials_to_rows,
    write_csv,
    write_summary,
)

DEFAULT_DIR = ROOT / "data" / "verification" / "knee_stair_l1"


def _csv_text(trials) -> str:
    import csv
    import io

    from knee_stair_l1.telemetry import CSV_FIELDS

    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(trials_to_rows(trials))
    return buffer.getvalue()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Layer 1 synthetic knee-stair metrics")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--check", action="store_true", help="compare a fresh run with the committed files")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    trials = run_study(config)
    summary = summarize(trials, config)
    csv_text = _csv_text(trials)
    summary_json = format_summary_json(summary)
    summary_md = format_summary_markdown(summary)
    if args.check:
        directory = args.output_dir
        expected = {
            "metrics_trials.csv": (directory / "metrics_trials.csv").read_text(encoding="utf-8"),
            "metrics_summary.json": (directory / "metrics_summary.json").read_text(encoding="utf-8"),
            "metrics_summary.md": (directory / "metrics_summary.md").read_text(encoding="utf-8"),
        }
        actual = {
            "metrics_trials.csv": csv_text,
            "metrics_summary.json": summary_json,
            "metrics_summary.md": summary_md,
        }
        mismatched = [name for name, text in expected.items() if text != actual[name]]
        if mismatched:
            print("MISMATCH: " + ", ".join(mismatched))
            return 1
        print(f"PASS: Layer 1 metrics match {directory}")
        return 0
    write_csv(args.output_dir / "metrics_trials.csv", trials)
    write_summary(args.output_dir, summary)
    print(summary_md)
    print(f"Wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
