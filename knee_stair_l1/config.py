"""Load Layer 1 parameters. Every tuned number lives in the JSON file."""

from __future__ import annotations

import json
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"

_REQUIRED = {
    "dt_s": (int, float),
    "duration_s": (int, float),
    "seeds": list,
    "command_transport_delay_steps": int,
}


def load_config(path: Path | str | None = None) -> dict:
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    data = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("config root must be an object")
    validate_config(data)
    data["_config_path"] = str(config_path)
    return data


def validate_config(data: dict) -> None:
    for key, kinds in _REQUIRED.items():
        if key not in data:
            raise ValueError(f"missing config key: {key}")
        if not isinstance(data[key], kinds):
            raise ValueError(f"config {key} has the wrong type")
    dt = float(data["dt_s"])
    duration = float(data["duration_s"])
    if dt <= 0 or duration <= 0:
        raise ValueError("dt_s and duration_s must be positive")
    steps = duration / dt
    if abs(steps - round(steps)) > 1e-9:
        raise ValueError("duration_s must be an integer multiple of dt_s")
    delay = data["command_transport_delay_steps"]
    if delay < 1:
        raise ValueError("command_transport_delay_steps must be >= 1")
    seeds = data["seeds"]
    if len(seeds) < 10:
        raise ValueError("at least 10 seeds are required")
    if len(set(seeds)) != len(seeds):
        raise ValueError("seeds must be unique")
    if any(not isinstance(seed, int) for seed in seeds):
        raise ValueError("seeds must be integers")
    intent = data["intent"]
    if intent.get("source_label") != "synthetic":
        raise ValueError('intent.source_label must be "synthetic" for v1')
    safety = data["safety"]
    plant = data["plant"]
    if float(plant["boundary_abs_limit_nm"]) != float(safety["command_limit_nm"]):
        raise ValueError("plant boundary limit and supervisor command limit must match")
    for section in ("intent", "assist_policy", "admittance", "pid", "safety", "plant", "metrics"):
        if section not in data:
            raise ValueError(f"missing config section: {section}")
