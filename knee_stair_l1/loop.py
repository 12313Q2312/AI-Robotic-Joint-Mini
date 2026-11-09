"""Closed loop. Every exo torque is a JointSafetyCore output.

Command ownership
- IntentSource proposes a synthetic (or test-double) effort. It does not move the plant.
- AssistController proposes a raw exo torque.
- JointSafetyCore is the only writer of the torque that enters the plant.
- KneePlant integrates the human torque and that safe exo torque.

Transport delay is an explicit queue of supervisor outputs. End-to-end latency
is apply_time minus the time that command was produced.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from knee_stair_l1.controller import CONDITIONS, AssistController
from knee_stair_l1.intent import IntentSample, IntentSource, SyntheticStairIntent
from knee_stair_l1.plant import KneePlant
from knee_stair_l1.safety import LIMITED_ASSIST, JointSafetyCore
from knee_stair_l1.topics import (
    TOPIC_ASSIST_COMMAND_RAW,
    TOPIC_ASSIST_COMMAND_SAFE,
    TOPIC_DESIRED_MOTION,
    TOPIC_INTENT_SYNTHETIC,
    TOPIC_PLANT_STATE,
    TOPIC_SAFETY_STATE,
    TOPIC_TELEMETRY,
)


@dataclass
class TrialResult:
    seed: int
    condition: str
    source_label: str
    amplitude_nm: float
    intent_onset_s: float
    assist_onset_s: float | None
    assist_onset: bool
    assist_timing_error_s: float
    end_to_end_latency_s: float
    peak_interaction_torque_nm: float
    saturation_limit_trigger_count: int
    saturation_limit_rising_edges: int
    final_flexion_rad: float


def run_trial(
    config: dict,
    seed: int,
    condition: str,
    intent: IntentSource | None = None,
    bus=None,
) -> TrialResult:
    if condition not in CONDITIONS:
        raise ValueError(f"unknown assist condition: {condition}")
    source = intent if intent is not None else SyntheticStairIntent(config)
    source.reset(seed)
    plant = KneePlant(config)
    safety = JointSafetyCore(config, legacy_enabled=False)
    controller = AssistController(config, condition)
    safety.reset(0.0)
    controller.reset(plant.flexion_rad)

    dt = float(config["dt_s"])
    steps = int(round(float(config["duration_s"]) / dt))
    delay = int(config["command_transport_delay_steps"])
    # Padding keeps the delay constant from the first applied sample.
    pending: deque = deque((-((delay - index) * dt), 0.0) for index in range(delay))

    onset_effort = float(config["intent"]["onset_effort_nm"])
    assist_onset_nm = float(config["metrics"]["assist_onset_nm"])
    intent_onset = None
    assist_onset = None
    peak = 0.0
    limit_steps = 0
    rising = 0
    previous_limit = False
    latency_sum = 0.0
    amplitude = getattr(source, "amplitude_nm", float("nan"))

    for index in range(steps):
        time_s = index * dt
        sample = source.step(time_s)
        _require_labeled_sample(sample)
        safety.observe_sensor(plant.flexion_rad, plant.flexion_velocity_rad_s, time_s)
        raw = controller.command(
            sample,
            plant.flexion_rad,
            plant.flexion_velocity_rad_s,
            dt,
        )
        safety.observe_command(raw.torque_extension_nm, time_s)
        decision = safety.evaluate(time_s)
        pending.append((time_s, decision.safe_extension_torque_nm))
        produced_s, applied = pending.popleft()
        plant.advance(sample.human_extension_nm, applied, dt)

        if intent_onset is None and sample.effort_extension_nm >= onset_effort:
            intent_onset = time_s
        if assist_onset is None and abs(applied) >= assist_onset_nm:
            assist_onset = time_s
        peak = max(peak, abs(plant.last_interaction_nm))
        limit_active = raw.pid_saturated or raw.policy_saturated or decision.state == LIMITED_ASSIST
        if limit_active:
            limit_steps += 1
            if not previous_limit:
                rising += 1
        previous_limit = limit_active
        latency_sum += time_s - produced_s
        if bus is not None:
            _publish(bus, sample, raw, decision, applied, plant, time_s)

    if intent_onset is None:
        raise RuntimeError("intent effort never reached onset_effort_nm; timing is undefined")
    horizon = float(config["duration_s"])
    if assist_onset is None:
        timing = horizon - intent_onset
        onset_flag = False
        onset_value = None
    else:
        timing = assist_onset - intent_onset
        onset_flag = True
        onset_value = assist_onset
    return TrialResult(
        seed=int(seed),
        condition=condition,
        source_label=source.source_label,
        amplitude_nm=float(amplitude),
        intent_onset_s=intent_onset,
        assist_onset_s=onset_value,
        assist_onset=onset_flag,
        assist_timing_error_s=timing,
        end_to_end_latency_s=latency_sum / steps,
        peak_interaction_torque_nm=peak,
        saturation_limit_trigger_count=limit_steps,
        saturation_limit_rising_edges=rising,
        final_flexion_rad=plant.flexion_rad,
    )


def _require_labeled_sample(sample: IntentSample) -> None:
    if not sample.source_label:
        raise ValueError("intent sample is missing source_label")


def _publish(bus, sample, raw, decision, applied, plant, time_s) -> None:
    import json

    bus.publish(
        TOPIC_INTENT_SYNTHETIC,
        {
            "source_label": sample.source_label,
            "time_s": sample.time_s,
            "effort_extension_nm": sample.effort_extension_nm,
            "phase": sample.phase,
            "ascent_active": 1.0 if sample.ascent_active else 0.0,
        },
    )
    generated = raw.desired_flexion_rad is not None
    bus.publish(
        TOPIC_DESIRED_MOTION,
        {
            "flexion_rad": 0.0 if raw.desired_flexion_rad is None else raw.desired_flexion_rad,
            "flexion_velocity_rad_s": 0.0
            if raw.desired_flexion_velocity_rad_s is None
            else raw.desired_flexion_velocity_rad_s,
            "generated": 1.0 if generated else 0.0,
        },
    )
    bus.publish(TOPIC_ASSIST_COMMAND_RAW, raw.torque_extension_nm)
    bus.publish(TOPIC_ASSIST_COMMAND_SAFE, applied)
    bus.publish(
        TOPIC_PLANT_STATE,
        {
            "flexion_rad": plant.flexion_rad,
            "flexion_velocity_rad_s": plant.flexion_velocity_rad_s,
            "interaction_extension_nm": plant.last_interaction_nm,
        },
    )
    bus.publish(TOPIC_SAFETY_STATE, decision.state)
    bus.publish(
        TOPIC_TELEMETRY,
        json.dumps(
            {
                "time_s": time_s,
                "source_label": sample.source_label,
                "state": decision.state,
                "safe_extension_torque_nm": decision.safe_extension_torque_nm,
            },
            sort_keys=True,
        ),
    )
