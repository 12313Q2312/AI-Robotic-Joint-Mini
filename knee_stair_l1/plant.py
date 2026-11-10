"""Two-body knee plant: human shank, exoskeleton link, and a strap.

The strap is a rotational spring-damper. Its torque is the interaction torque.
The exoskeleton motor torque is a different quantity and is not the interaction
metric. A motor command above the supervisor limit is rejected.

Gravity torque is a flexion moment:
    share * body_mass * g * moment_arm * sin(flexion)
It is zero at a straight knee. Assumptions are in config load.assumption.

Hard-stop contact is an anomaly. Extension is the lower flexion limit.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math

logger = logging.getLogger("knee_stair_l1.plant")


class BoundaryError(ValueError):
    """Raised when a motor torque reaches the plant outside the supervisor limit."""


@dataclass(frozen=True)
class HardStopEvent:
    time_s: float
    side: str
    stop: str
    angle_rad: float


class KneePlant:
    def __init__(self, config: dict, factors=None):
        plant = config["plant"]
        load = config["load"]
        strap = config["strap"]
        safety = config["safety"]
        self.human_inertia = float(plant["human_inertia_kg_m2"])
        self.exo_inertia = float(plant["exo_inertia_kg_m2"])
        self.human_viscous = float(plant["human_viscous_nm_s_per_rad"])
        self.exo_viscous = float(plant["exo_viscous_nm_s_per_rad"])
        self.q_min = float(plant["min_flexion_rad"])
        self.q_max = float(plant["max_flexion_rad"])
        self.boundary_limit = float(plant["boundary_abs_limit_nm"])
        self.q_h = float(plant["initial_flexion_rad"])
        self.q_e = float(plant["initial_flexion_rad"])
        self.qd_h = 0.0
        self.qd_e = 0.0
        self.body_mass = float(load["body_mass_kg"])
        self.gravity = float(load["gravity_m_s2"])
        self.weight_share = float(load["knee_weight_share"])
        self.moment_arm = float(load["moment_arm_m"])
        self.strap_k = float(strap["stiffness_nm_per_rad"])
        self.strap_b = float(strap["damping_nm_s_per_rad"])
        if factors is not None:
            reference_height = float(load["reference_step_height_m"])
            self.body_mass = float(factors.body_mass_kg)
            self.moment_arm *= float(factors.step_height_m) / reference_height
            self.strap_k *= float(factors.strap_stiffness_scale)
        self.estop_damping = float(safety["estop_passive_damping_nm_s_per_rad"])
        self.strap_torque_flexion_nm = 0.0
        self.last_motor_extension_nm = 0.0
        self.anomalies: list[HardStopEvent] = []
        self._human_on_stop = False
        self._exo_on_stop = False
        self._check_parameters()

    def _check_parameters(self) -> None:
        if min(self.human_inertia, self.exo_inertia, self.boundary_limit, self.body_mass, self.gravity) <= 0:
            raise ValueError("plant mass, gravity, inertia and motor limit must be positive")
        if self.weight_share <= 0 or self.moment_arm <= 0:
            raise ValueError("knee load share and moment arm must be positive")
        if self.q_min >= self.q_max:
            raise ValueError("plant joint limits are inverted")
        if not (self.q_min < self.q_h < self.q_max):
            raise ValueError("initial flexion must sit strictly inside the stops")
        if min(self.strap_k, self.strap_b, self.estop_damping) < 0:
            raise ValueError("strap and passive damping must be non-negative")

    def gravity_flexion_nm(self, flexion_rad: float) -> float:
        """Flexion moment from the configured share of body weight."""
        return self.weight_share * self.body_mass * self.gravity * self.moment_arm * math.sin(flexion_rad)

    @property
    def flexion_rad(self) -> float:
        return self.q_h

    @property
    def flexion_velocity_rad_s(self) -> float:
        return self.qd_h

    def advance(
        self,
        human_flexion_nm: float,
        exo_extension_nm: float,
        dt: float,
        time_s: float,
        passive_damping: bool = False,
    ) -> None:
        if not all(math.isfinite(value) for value in (human_flexion_nm, exo_extension_nm, dt, time_s)) or dt <= 0:
            raise BoundaryError("plant step requires finite torques, a finite time and a positive dt")
        if abs(exo_extension_nm) > self.boundary_limit + 1e-9:
            raise BoundaryError("exo motor torque above the supervisor limit")
        motor_extension = 0.0 if passive_damping else float(exo_extension_nm)
        self.last_motor_extension_nm = motor_extension
        strap = self.strap_k * (self.q_e - self.q_h) + self.strap_b * (self.qd_e - self.qd_h)
        self.strap_torque_flexion_nm = strap
        exo_damping = self.estop_damping if passive_damping else self.exo_viscous
        human_accel = (
            human_flexion_nm
            + self.gravity_flexion_nm(self.q_h)
            + strap
            - self.human_viscous * self.qd_h
        ) / self.human_inertia
        exo_accel = (
            -motor_extension
            - strap
            - exo_damping * self.qd_e
        ) / self.exo_inertia
        self.qd_h += human_accel * dt
        self.qd_e += exo_accel * dt
        self.q_h += self.qd_h * dt
        self.q_e += self.qd_e * dt
        self._apply_stop("human", time_s)
        self._apply_stop("exo", time_s)

    def _apply_stop(self, side: str, time_s: float) -> None:
        if side == "human":
            q, qd = self.q_h, self.qd_h
        else:
            q, qd = self.q_e, self.qd_e
        stop = None
        if q > self.q_max:
            q = self.q_max
            if qd > 0.0:
                qd = 0.0
            stop = "flexion"
        elif q < self.q_min:
            q = self.q_min
            if qd < 0.0:
                qd = 0.0
            stop = "extension"
        if side == "human":
            self.q_h, self.qd_h = q, qd
            was = self._human_on_stop
            self._human_on_stop = stop is not None
        else:
            self.q_e, self.qd_e = q, qd
            was = self._exo_on_stop
            self._exo_on_stop = stop is not None
        if stop is not None and not was:
            event = HardStopEvent(time_s=time_s, side=side, stop=stop, angle_rad=q)
            self.anomalies.append(event)
            logger.warning(
                "anomaly hard_stop_contact side=%s stop=%s t_s=%.4f angle_rad=%.4f",
                side,
                stop,
                time_s,
                q,
            )
