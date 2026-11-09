"""Assist command proposals.

Conditions
- off: exo command is 0. The human torque still reaches the plant.
- admittance: admittance generates desired flexion; TorquePid tracks it.
- assist_policy: PR #3 proportional share, saturation and slew. No PID.

The supervisor, not this module, is allowed to command the plant.
Extension-positive newton metres are the command sign. TorquePid itself is
unchanged and still produces a torque whose positive sign increases the
measured angle (flexion). The sign flip below is only the unit conversion
into the extension-positive plant command.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from knee_stair_l1.admittance import Admittance
from knee_stair_l1.assist_policy import AssistPolicy
from knee_stair_l1.intent import IntentSample
from knee_stair_l1.torque_pid import TorquePid

CONDITIONS = ("off", "admittance", "assist_policy")


@dataclass(frozen=True)
class ControllerCommand:
    torque_extension_nm: float
    desired_flexion_rad: float | None
    desired_flexion_velocity_rad_s: float | None
    pid_saturated: bool
    policy_saturated: bool
    intent_time_s: float
    source_label: str


class AssistController:
    def __init__(self, config: dict, condition: str):
        if condition not in CONDITIONS:
            raise ValueError(f"unknown assist condition: {condition}")
        self.condition = condition
        self._config = config
        pid = config["pid"]
        policy = config["assist_policy"]
        admit = config["admittance"]
        plant = config["plant"]
        self.pid = TorquePid(
            pid["kp_nm_per_rad"],
            pid["ki_nm_per_rad_s"],
            pid["kd_nm_s_per_rad"],
            pid["torque_limit_nm"],
        )
        self.policy = AssistPolicy(policy["gain"], policy["limit_nm"], policy["slew_nm_per_s"])
        self.admittance = Admittance(
            admit["mass_kg_m2"],
            admit["damping_nm_s_per_rad"],
            admit["stiffness_nm_per_rad"],
            plant["min_flexion_rad"],
            plant["max_flexion_rad"],
        )

    def reset(self, measured_flexion_rad: float) -> None:
        self.pid.reset()
        self.policy.reset()
        self.admittance.reset(measured_flexion_rad)

    def reference_flexion(self, time_s: float) -> float:
        if not math.isfinite(time_s):
            raise ValueError("reference time must be finite")
        # Equilibrium stays put. Desired motion comes from the effort through
        # the admittance, so a scripted trajectory cannot lead the synthetic intent.
        return float(self._config["admittance"]["equilibrium_flexion_rad"])

    def command(
        self,
        sample: IntentSample,
        measured_flexion_rad: float,
        measured_flexion_velocity_rad_s: float,
        dt: float,
    ) -> ControllerCommand:
        if not all(
            math.isfinite(value)
            for value in (measured_flexion_rad, measured_flexion_velocity_rad_s, dt, sample.effort_extension_nm)
        ):
            raise ValueError("controller measurements must be finite")
        desired_q = None
        desired_qd = None
        pid_saturated = False
        policy_saturated = False
        if self.condition == "off":
            torque = 0.0
        elif self.condition == "assist_policy":
            policy_saturated = self.policy.command_is_saturated(sample.effort_extension_nm, True)
            torque = self.policy.step(sample.effort_extension_nm, dt, True)
        else:
            q_ref = self.reference_flexion(sample.time_s)
            desired_q, desired_qd, _qdd = self.admittance.step(sample.effort_extension_nm, q_ref, dt)
            pid_step = self.pid.step(
                desired_q,
                measured_flexion_rad,
                measured_flexion_velocity_rad_s,
                dt,
            )
            pid_saturated = pid_step.saturated
            # PID positive torque would increase flexion. Extension-positive command is the opposite sign.
            torque = -pid_step.control_torque_nm
        return ControllerCommand(
            torque_extension_nm=torque,
            desired_flexion_rad=desired_q,
            desired_flexion_velocity_rad_s=desired_qd,
            pid_saturated=pid_saturated,
            policy_saturated=policy_saturated,
            intent_time_s=sample.time_s,
            source_label=sample.source_label,
        )
