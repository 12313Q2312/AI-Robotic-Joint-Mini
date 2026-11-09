#!/usr/bin/env python3
"""Latency jitter sweep and ±30% load/comfort sensitivity.

Writes the sweep JSON, the PNG, and the sensitivity JSON.
`--check` compares the JSON files only. The PNG is regenerated from the same
sweep and is not byte-compared.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from knee_stair_l1.config import load_config
from knee_stair_l1.plot_png import write_delay_plot
from knee_stair_l1.sweeps import latency_sweep, sensitivity_sweep

DEFAULT_DIR = ROOT / "data" / "verification" / "knee_stair_l1"


def _dump(document: dict) -> str:
    return json.dumps(document, indent=2, sort_keys=True) + "\n"


def build(config) -> dict[str, str]:
    latency = latency_sweep(config)
    sensitivity = sensitivity_sweep(config)
    return {
        "latency_sweep.json": _dump(latency),
        "sensitivity.json": _dump(sensitivity),
        "latency_sweep.md": _caption(latency),
    }


def _caption(latency: dict) -> str:
    return "\n".join([
        "# Latency jitter sweep",
        "",
        "The horizontal axis is the upper end of a uniform delay, in milliseconds, applied to the intent and sensor before the safety supervisor.",
        "It is not the fixed simulation-loop latency, and torque is not delayed again after the supervisor.",
        "",
        "Top panel, from the top of the legend order in the generator:",
        "- blue: admittance human-torque RMS reduction, Nm",
        "- green: AssistPolicy human-torque RMS reduction, Nm",
        "- light blue: admittance positive-work reduction, J",
        "- dark green: AssistPolicy positive-work reduction, J",
        "",
        "Bottom panel:",
        "- orange: admittance peak strap torque, Nm",
        "- brown: AssistPolicy peak strap torque, Nm",
        "",
        latency["conclusion"],
        "",
        "Admittance's positive-work reduction stays below AssistPolicy across this sweep. That ordering is reported as it is.",
        "",
    ])


def write_plot(directory: Path, latency: dict) -> None:
    points = latency["points"]
    delays = [point["delay_ms"] for point in points]
    def column(condition: str, key: str) -> list[float]:
        return [point[condition][key] for point in points]

    # Top panel: primary reductions. Bottom: peak strap torque. Units differ and are named in the sweep note.
    write_delay_plot(
        directory / "latency_sweep.png",
        delays,
        [
            ("admittance torque RMS reduction", column("admittance", "human_torque_rms_reduction_nm"), (20, 90, 180)),
            ("AssistPolicy torque RMS reduction", column("assist_policy", "human_torque_rms_reduction_nm"), (20, 140, 60)),
            ("admittance positive work reduction", column("admittance", "human_positive_work_reduction_j"), (80, 140, 210)),
            ("AssistPolicy positive work reduction", column("assist_policy", "human_positive_work_reduction_j"), (10, 90, 50)),
            ("admittance peak interaction", column("admittance", "peak_interaction_torque_nm"), (200, 110, 20)),
            ("AssistPolicy peak interaction", column("assist_policy", "peak_interaction_torque_nm"), (140, 50, 40)),
        ],
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Layer 1 latency and sensitivity sweeps")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_DIR)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)
    config = load_config()
    documents = build(config)
    if args.check:
        mismatched = [
            name for name, text in documents.items()
            if (args.output_dir / name).read_text(encoding="utf-8") != text
        ]
        if mismatched:
            print("MISMATCH: " + ", ".join(mismatched))
            return 1
        print(f"PASS: Layer 1 sweeps match {args.output_dir}")
        return 0
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, text in documents.items():
        (args.output_dir / name).write_text(text, encoding="utf-8")
    write_plot(args.output_dir, json.loads(documents["latency_sweep.json"]))
    print(json.loads(documents["latency_sweep.json"])["conclusion"])
    print(json.loads(documents["sensitivity.json"])["conclusion"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
