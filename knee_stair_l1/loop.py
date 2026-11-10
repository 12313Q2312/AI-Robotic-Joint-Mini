"""Closed loop. Supervisor output is not yet motor torque.

Command ownership
- IntentSource emits a synthetic desired trajectory. It does not move the plant.
- BoundedHumanKnee turns that trajectory into a clipped knee torque.
- AssistController proposes a raw exo torque from a delayed intent sample.
- JointSafetyCore may clamp that proposal to its plausibility limit and may
  replace it with damping or zero. It does not apply the actuator limit.
- torque_to_motor is the only conversion into the torque KneePlant integrates.
  On E_STOP that value is 0 and the exo damper is passive.

Latency is measured as command_time minus the timestamp on the intent sample
the command consumed. It is not filled in from the delay parameter.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from knee_stair_l1.actuator import torque_to_motor
from knee_stair_l1.controller import CONDITIONS, AssistController, ControllerCommand
from knee_stair_l1.human import BoundedHumanKnee
from knee_stair_l1.intent import IntentSample, IntentSource, SyntheticStairIntent
from knee_stair_l1.plant import HardStopEvent, KneePlant
from knee_stair_l1.randomization import SeedFactors, draw_factors, gaussian, noise_rng
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
    steady_non_normal_fraction: float = 0.0
    steady_supervisor_state: str = "NORMAL"
    safety_state_counts: dict = field(default_factory=dict)
    steady_state_counts: dict = field(default_factory=dict)


def run_trial(
    config: dict,
    seed: int,
    condition: str,
    intent: IntentSource | None = None,
    bus=None,
    signal_delay_s: float | None = None,
    scenario: str = "nominal",
) -> TrialResult:
    if condition not in CONDITIONS:
        raise ValueError(f"unknown assist condition: {condition}")
    factors = draw_factors(seed, config)
    source = intent if intent is not None else SyntheticStairIntent(config, scenario=scenario)
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
    angle_std = factors.angle_noise_std_rad
    velocity_std = factors.velocity_noise_std_rad_s
    pending: deque[IntentSample] = deque(source.step(-(delay - index) * dt) for index in range(delay))
    signal_log: list[dict] = []
    steady_counts: dict[str, int] = {}
    all_counts: dict[str, int] = {}

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
        q_h = plant.q_h
        qd_h = plant.qd_h
        q_meas = q_h + angle_std * gaussian(noise)
        qd_meas = qd_h + velocity_std * gaussian(noise)
        qe_meas = plant.q_e + angle_std * gaussian(noise)
        qde_meas = plant.qd_e + velocity_std * gaussian(noise)
        human_flexion = human.torque_flexion_nm(fresh.q_ref_rad, fresh.qd_ref_rad_s, q_meas, qd_meas)
        human_extension = -human_flexion
        signal_log.append({
            "t": time_s,
            "intent": fresh,
            "qe": qe_meas,
            "qde": qde_meas,
            "human_extension": human_extension,
        })
        if signal_delay_s is None:
            pending.append(fresh)
            used = pending.popleft()
            safety.observe_sensor(qe_meas, qde_meas, time_s, packet_time=time_s)
            safety.observe_intent(used.time_s, time_s)
            raw = controller.command(used, qe_meas, qde_meas, human_extension, dt)
            safety.observe_command(raw.torque_extension_nm, time_s)
            intent_time_s = used.time_s
        else:
            delay_steps = int(round(float(signal_delay_s) / dt))
            target_index = index - delay_steps
            released = signal_log[target_index] if target_index >= 0 else None
            if released is None:
                used = fresh
                raw = ControllerCommand(0.0, None, None, False, False, fresh.time_s, fresh.source_label)
                intent_time_s = fresh.time_s
            else:
                safety.observe_sensor(released["qe"], released["qde"], time_s, packet_time=released["t"])
                safety.observe_intent(released["intent"].time_s, time_s)
                used = released["intent"]
                raw = controller.command(
                    used, released["qe"], released["qde"], released["human_extension"], dt,
                )
                safety.observe_command(raw.torque_extension_nm, time_s)
                intent_time_s = used.time_s
        decision = safety.evaluate(time_s)
        command_time_s = time_s
        latency_s = command_time_s - intent_time_s
        applied_torque = torque_to_motor(
            decision.safe_extension_torque_nm,
            config,
            passive_damping=decision.passive_damping,
        )
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
        all_counts[decision.state] = all_counts.get(decision.state, 0) + 1
        if signal_delay_s is None or time_s + 1e-12 >= float(signal_delay_s):
            steady_counts[decision.state] = steady_counts.get(decision.state, 0) + 1
        max_command = max(max_command, abs(plant.last_motor_extension_nm))
        latencies.append(latency_s)
        if bus is not None:
            _publish(
                bus, fresh, raw, decision, plant, time_s, command_time_s, used.time_s, applied_torque,
            )

    return TrialResult(
        seed=int(seed),
        condition=condition,
        source_label=source.source_label,
        depth_scale=float(getattr(source, "depth_scale", factors.intent_amplitude_scale)),
        factors=factors,
        non_normal_step_fraction=non_normal_steps / steps,
        steady_non_normal_fraction=(
            sum(count for state, count in steady_counts.items() if state != "NORMAL")
            / max(sum(steady_counts.values()), 1)
        ),
        steady_supervisor_state=max(steady_counts, key=steady_counts.get) if steady_counts else "NORMAL",
        safety_state_counts=dict(all_counts),
        steady_state_counts=dict(steady_counts),
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


def _publish(bus, sample, raw, decision, plant, time_s, command_time_s, intent_time_s, applied_torque) -> None:
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
        {
            "torque_extension_nm": applied_torque,
            "supervisor_command_nm": decision.safe_extension_torque_nm,
            "command_time_s": command_time_s,
        },
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
                "applied_torque_nm": applied_torque,
                "safe_extension_torque_nm": applied_torque,
                "supervisor_command_nm": decision.safe_extension_torque_nm,
                "passive_damping": decision.passive_damping,
            },
            sort_keys=True,
        ),
    )
