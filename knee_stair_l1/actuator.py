"""Actuator torque clamp for the knee-stair Python path and joint_safety_node.

Controller proposals and the supervisor output are not motor torque.
clamp_actuator_torque is the function those two paths use before torque
reaches KneePlant or /knee_stair/assist_command_safe. Legacy C++ controllers
do not call it. The supervisor's command_limit_nm (40 Nm) is a plausibility
check: a finite command outside it enters LIMITED_ASSIST. It is not this limit.
"""

from __future__ import annotations

import math


def clamp_actuator_torque(torque_nm: float, limit_nm: float) -> float:
    """Return a finite torque limited to ±limit_nm.

    NaN and inf raise ValueError. This function does not return 0 for them.
    joint_safety_node's assist callback catches that exception, latches
    E_STOP, and publishes 0 itself.
    """
    if not math.isfinite(torque_nm) or not math.isfinite(limit_nm) or limit_nm <= 0.0:
        raise ValueError("actuator torque and its limit must be finite, and the limit must be positive")
    if torque_nm > limit_nm:
        return float(limit_nm)
    if torque_nm < -limit_nm:
        return -float(limit_nm)
    return float(torque_nm)


def torque_to_motor(command_nm: float, config: dict, passive_damping: bool = False) -> float:
    """Torque that may reach the plant or the motor topic.

    Passive damping publishes no motor torque. The plant applies its own damper.
    """
    if passive_damping:
        return 0.0
    return clamp_actuator_torque(command_nm, float(config["actuator_torque_limit_nm"]))
