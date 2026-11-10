"""Torque PID used as the admittance inner loop.

The recurrence matches the historical controller in
simulation/webots/controllers/joint_controller/joint_controller.cpp and the
same block in JointControllerPlugin.cpp:

- P on angle error
- I with conditional integration when the candidate torque is saturated
- D on measured angular velocity, not on the error derivative
- output clamped to the configured torque limit

Knee gains and the torque limit come from config. The historical rig values
(kp 0.15, ki 0.05, kd 0.04, limit 0.10 Nm) stay available for the regression
test and are not the stair-assist gains.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PidStep:
    error_rad: float
    p_torque_nm: float
    i_torque_nm: float
    d_torque_nm: float
    raw_torque_nm: float
    control_torque_nm: float
    anti_windup_active: bool
    saturated: bool
    error_integral_rad_s: float


class TorquePid:
    def __init__(self, kp: float, ki: float, kd: float, torque_limit: float):
        values = (kp, ki, kd, torque_limit)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("PID parameters must be finite")
        if kp < 0 or ki < 0 or kd < 0 or torque_limit <= 0:
            raise ValueError("PID gains must be non-negative and the limit positive")
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.torque_limit = float(torque_limit)
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def step(
        self,
        target_rad: float,
        measured_rad: float,
        measured_velocity_rad_s: float,
        dt: float,
    ) -> PidStep:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("PID timestep must be positive and finite")
        if not all(math.isfinite(value) for value in (target_rad, measured_rad, measured_velocity_rad_s)):
            raise ValueError("PID measurement must be finite")
        error = target_rad - measured_rad
        p_torque = self.kp * error
        d_torque = -self.kd * measured_velocity_rad_s
        candidate_integral = self.integral + error * dt
        candidate_i = self.ki * candidate_integral
        candidate_raw = p_torque + candidate_i + d_torque
        saturated_high = candidate_raw > self.torque_limit
        saturated_low = candidate_raw < -self.torque_limit
        integration_reduces_saturation = (saturated_high and error < 0.0) or (
            saturated_low and error > 0.0
        )
        anti_windup = (saturated_high or saturated_low) and not integration_reduces_saturation
        if not anti_windup:
            self.integral = candidate_integral
        i_torque = self.ki * self.integral
        raw = p_torque + i_torque + d_torque
        control = raw
        if control > self.torque_limit:
            control = self.torque_limit
        elif control < -self.torque_limit:
            control = -self.torque_limit
        return PidStep(
            error_rad=error,
            p_torque_nm=p_torque,
            i_torque_nm=i_torque,
            d_torque_nm=d_torque,
            raw_torque_nm=raw,
            control_torque_nm=control,
            anti_windup_active=anti_windup,
            saturated=abs(raw) > self.torque_limit,
            error_integral_rad_s=self.integral,
        )
