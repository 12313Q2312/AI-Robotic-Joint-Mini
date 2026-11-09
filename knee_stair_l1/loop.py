"""Closed loop. Every exo motor torque is a JointSafetyCore output.

Command ownership
- IntentSource emits a synthetic desired trajectory. It does not move the plant.
- BoundedHumanKnee turns that trajectory into a clipped knee torque.
- AssistController proposes a raw exo torque from a delayed intent sample.
- JointSafetyCore is the only writer of the motor torque that enters the plant.
- KneePlant integrates the human torque, the gravity load, the strap, and that
  motor torque. On E_STOP the motor torque is 0 and the exo damper is passive.

Latency is measured as command_time minus the timestamp on the intent sample
the command consumed. It is not filled in from the delay parameter.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from knee_stair_l1.controller import CONDITIONS, AssistController
from knee_stair_l1.human import BoundedHumanKnee
from knee_stair_l1.intent import IntentSample, IntentSource, SyntheticStairIntent
from knee_stair_l1.plant import HardStopEvent, KneePlant
from knee_stair_l1.randomization import SeedFactors, draw_factors, gaussian, jitter_rng, noise_rng
from knee_stair_l1.safety import LIMITED_ASSIST, NORMAL, JointSafetyCore
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
    depth_scale: float
    human_torque_rms_nm: float
    human_positive_work_j: float
    tracking_error_rms_rad: float
    peak_interaction_torque_nm: float
    saturation_time_fraction: float
    hard_stop_contact_count: int
    hard_stop_events: list[HardStopEvent] = field(default_factory=list)
    latency_samples_s: list[float] = field(default_factory=list)
    max_abs_motor_command_nm: float = 0.0
    final_flexion_rad: float = 0.0
    factors: SeedFactors | None = None
    non_normal_step_fraction: float = 0.0


def run_trial(
    config: dict,
    seed: int,
    condition: str,
    intent: IntentSource | None = None,
    bus=None,
    jitter_max_s: float | None = None,
) -> TrialResult:
    if condition not in CONDITIONS:
        raise ValueError(f"unknown assist condition: {condition}")
    factors = draw_factors(seed, config)
    source = intent if intent is not None else SyntheticStairIntent(config)
    source.reset(seed)
    plant = KneePlant(config, factors)
    human = BoundedHumanKnee(config)
    safety = JointSafetyCore(config, legacy_enabled=False)
    controller = AssistController(config, condition)
    safety.reset(0.0)
    controller.reset(plant.q_e)

    dt = float(config["dt_s"])
    steps = int(round(float(config["duration_s"]) / dt))
    delay = int(config["command_transport_delay_steps"])
    noise = noise_rng(seed)
    angle_std = float(config["sensor_noise"]["angle_std_rad"])
    velocity_std = float(config["sensor_noise"]["velocity_std_rad_s"])
    delays = jitter_rng(seed)
    pending: deque[IntentSample] = deque(source.step(-(delay - index) * dt) for index in range(delay))
    held_commands: list[tuple[float, float]] = []

    torque_sq = 0.0
    positive_work = 0.0
    track_sq = 0.0
    peak_interaction = 0.0
    saturated_steps = 0
    non_normal_steps = 0
    max_command = 0.0
    latencies: list[float] = []

    for index in range(steps):
        time_s = index * dt
        fresh = source.step(time_s)
        _require_labeled_sample(fresh)
        pending.append(fresh)
        used = pending.popleft()
        q_h = plant.q_h
        qd_h = plant.qd_h
        q_meas = q_h + angle_std * gaussian(noise)
        qd_meas = qd_h + velocity_std * gaussian(noise)
        qe_meas = plant.q_e + angle_std * gaussian(noise)
        qde_meas = plant.qd_e + velocity_std * gaussian(noise)
        human_flexion = human.torque_flexion_nm(fresh.q_ref_rad, fresh.qd_ref_rad_s, q_meas, qd_meas)
        human_extension = -human_flexion
        safety.observe_sensor(qe_meas, qde_meas, time_s)
        raw = controller.command(used, qe_meas, qde_meas, human_extension, dt)
        safety.observe_command(raw.torque_extension_nm, time_s)
        decision = safety.evaluate(time_s)
        command_time_s = time_s
        latency_s = command_time_s - used.time_s
        held_commands.append((command_time_s, decision.safe_extension_torque_nm))
        applied_torque = decision.safe_extension_torque_nm
        if decision.passive_damping:
            applied_torque = 0.0
        elif jitter_max_s is not None:
            drawn = 0.0 if jitter_max_s <= 0.0 else delays.uniform(0.0, float(jitter_max_s))
            target_time = time_s - drawn
            applied_torque = 0.0
            for stamp, torque in held_commands:
                if stamp <= target_time + 1e-12:
                    applied_torque = torque
                else:
                    break
        plant.advance(
            human_flexion,
            applied_torque,
            dt,
            time_s,
            passive_damping=decision.passive_damping,
        )
        torque_sq += human_flexion * human_flexion
        power = human_flexion * qd_h
        if power > 0.0:
            positive_work += power * dt
        track_error = fresh.q_ref_rad - q_h
        track_sq += track_error * track_error
        peak_interaction = max(peak_interaction, abs(plant.strap_torque_flexion_nm))
        if raw.pid_saturated or raw.policy_saturated or decision.state == LIMITED_ASSIST:
            saturated_steps += 1
        if decision.state != NORMAL:
            non_normal_steps += 1
        max_command = max(max_command, abs(decision.safe_extension_torque_nm))
        latencies.append(latency_s)
        if bus is not None:
            _publish(bus, fresh, raw, decision, plant, time_s, command_time_s, used.time_s)

    return TrialResult(
        seed=int(seed),
        condition=condition,
        source_label=source.source_label,
        depth_scale=float(getattr(source, "depth_scale", factors.intent_amplitude_scale)),
        factors=factors,
        non_normal_step_fraction=non_normal_steps / steps,
        human_torque_rms_nm=(torque_sq / steps) ** 0.5,
        human_positive_work_j=positive_work,
        tracking_error_rms_rad=(track_sq / steps) ** 0.5,
        peak_interaction_torque_nm=peak_interaction,
        saturation_time_fraction=saturated_steps / steps,
        hard_stop_contact_count=len(plant.anomalies),
        hard_stop_events=list(plant.anomalies),
        latency_samples_s=latencies,
        max_abs_motor_command_nm=max_command,
        final_flexion_rad=plant.q_h,
    )


def _require_labeled_sample(sample: IntentSample) -> None:
    if not sample.source_label:
        raise ValueError("intent sample is missing source_label")


def _publish(bus, sample, raw, decision, plant, time_s, command_time_s, intent_time_s) -> None:
    import json

    bus.publish(
        TOPIC_INTENT_SYNTHETIC,
        {
            "source_label": sample.source_label,
            "time_s": sample.time_s,
            "q_ref_rad": sample.q_ref_rad,
            "qd_ref_rad_s": sample.qd_ref_rad_s,
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
    bus.publish(
        TOPIC_ASSIST_COMMAND_SAFE,
        {"torque_extension_nm": decision.safe_extension_torque_nm, "command_time_s": command_time_s},
    )
    bus.publish(
        TOPIC_PLANT_STATE,
        {
            "human_flexion_rad": plant.q_h,
            "human_velocity_rad_s": plant.qd_h,
            "exo_flexion_rad": plant.q_e,
            "strap_torque_flexion_nm": plant.strap_torque_flexion_nm,
        },
    )
    bus.publish(TOPIC_SAFETY_STATE, decision.state)
    bus.publish(
        TOPIC_TELEMETRY,
        json.dumps(
            {
                "time_s": time_s,
                "intent_time_s": intent_time_s,
                "command_time_s": command_time_s,
                "source_label": sample.source_label,
                "state": decision.state,
                "safe_extension_torque_nm": decision.safe_extension_torque_nm,
                "passive_damping": decision.passive_damping,
            },
            sort_keys=True,
        ),
    )
