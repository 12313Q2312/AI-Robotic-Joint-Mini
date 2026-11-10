"""Load Layer 1 parameters. Every tuned number lives in the JSON file."""

from __future__ import annotations

import json
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "default.json"

# Actuator hard limit. The PID clamp stays at this value. It is not a tuning knob.
ACTUATOR_HARD_LIMIT_NM = 24.0


def effective_torque_limit_nm(data: dict) -> float:
    """Return min(PID cap, supervisor cap, AssistPolicy cap), in Nm.

    The loop does not apply this min() to every command. Admittance output is
    clamped by the PID limit. AssistPolicy output skips the PID and is clamped
    by its own limit, then by the supervisor. The plant rejects a motor torque
    above boundary_abs_limit_nm, which must equal the supervisor limit. On the
    shipped file the minimum of the three caps is the PID value. An
    AssistPolicy command can still reach the plant at the supervisor limit.
    """
    pid_limit = float(data["pid"]["torque_limit_nm"])
    supervisor_limit = float(data["safety"]["command_limit_nm"])
    policy_limit = float(data["assist_policy"]["limit_nm"])
    return min(pid_limit, supervisor_limit, policy_limit)


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
    for section in (
        "intent", "human", "load", "strap", "assist_policy", "admittance", "pid", "safety", "plant", "metrics",
        "randomization", "sensor_noise",
    ):
        if section not in data:
            raise ValueError(f"missing config section: {section}")
    if list(data["seeds"]) != list(range(30)):
        raise ValueError("the reported study uses seeds 0..29")
    intent = data["intent"]
    if intent.get("source_label") != "synthetic":
        raise ValueError('intent.source_label must be "synthetic" for v1')
    safety = data["safety"]
    plant = data["plant"]
    if float(plant["boundary_abs_limit_nm"]) != float(safety["command_limit_nm"]):
        raise ValueError("plant boundary limit and supervisor command limit must match")
    if float(plant["initial_flexion_rad"]) != float(intent["q_start_rad"]):
        raise ValueError("plant initial flexion and intent q_start_rad must match")
    for section, key in (
        ("pid", "torque_limit_nm"),
        ("safety", "command_limit_nm"),
        ("assist_policy", "limit_nm"),
    ):
        if key not in data[section]:
            raise ValueError(f"missing config key: {section}.{key}")
    effective = effective_torque_limit_nm(data)
    if effective > ACTUATOR_HARD_LIMIT_NM:
        raise ValueError(
            f"effective torque limit is {effective} Nm and must not exceed {ACTUATOR_HARD_LIMIT_NM:g} Nm"
        )
    if float(data["pid"]["torque_limit_nm"]) != ACTUATOR_HARD_LIMIT_NM:
        raise ValueError("pid torque_limit_nm is a 24 Nm safety parameter and cannot be retuned")
    span = float(intent["q_start_rad"]) - float(intent["q_end_rad"])
    amplitude = 1.0 + float(data["randomization"]["intent_amplitude_fraction"])
    most_extended = float(intent["q_start_rad"]) - span * amplitude
    if most_extended <= float(plant["min_flexion_rad"]) + 0.15:
        raise ValueError("scaled synthetic trajectory reaches the extension stop")
    if float(data["load"]["knee_weight_share"]) <= 0 or float(data["load"]["moment_arm_m"]) <= 0:
        raise ValueError("stair load share and moment arm must be positive")
