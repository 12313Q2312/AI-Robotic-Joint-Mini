"""Callback body for the knee assist command.

JointSafety.on_assist calls on_assist_command. The timer calls
republish_supervised. Neither function imports rclpy.

clamp_actuator_torque raises ValueError on NaN or inf. It does not return 0.
These functions catch that exception, latch E_STOP through the supervisor's
non-finite command path, and hand the caller 0 Nm to publish.
"""

from __future__ import annotations

from dataclasses import dataclass

from knee_stair_l1.actuator import clamp_actuator_torque, torque_to_motor
from knee_stair_l1.safety import E_STOP


@dataclass(frozen=True)
class AssistPublish:
    applied_torque_nm: float
    supervisor_command_nm: float
    state: str
    legacy_safe_target_rad: float
    reasons: tuple[str, ...]


def on_assist_command(core, config: dict, now: float, command_nm: float) -> AssistPublish:
    """Process one assist command the way JointSafety.on_assist does."""
    try:
        # Finite 80 Nm passes. NaN and inf raise here and are not observed as numbers.
        clamp_actuator_torque(float(command_nm), float(config["actuator_torque_limit_nm"]))
        core.observe_command(float(command_nm), now)
    except ValueError:
        return _estop_after_rejected_clamp(core, now)
    return republish_supervised(core, config, now)


def republish_supervised(core, config: dict, now: float) -> AssistPublish:
    """Torque the safe-command topic may carry for the current supervisor decision."""
    try:
        decision = core.evaluate(now)
        applied = torque_to_motor(
            decision.safe_extension_torque_nm,
            config,
            passive_damping=decision.passive_damping,
        )
    except ValueError:
        return _estop_after_rejected_clamp(core, now)
    return AssistPublish(
        applied_torque_nm=applied,
        supervisor_command_nm=decision.safe_extension_torque_nm,
        state=decision.state,
        legacy_safe_target_rad=decision.legacy_safe_target_rad,
        reasons=decision.reasons,
    )


def _estop_after_rejected_clamp(core, now: float) -> AssistPublish:
    core.observe_command(float("nan"), now)
    decision = core.evaluate(now)
    return AssistPublish(
        applied_torque_nm=0.0,
        supervisor_command_nm=decision.safe_extension_torque_nm,
        state=E_STOP,
        legacy_safe_target_rad=decision.legacy_safe_target_rad,
        reasons=decision.reasons,
    )
