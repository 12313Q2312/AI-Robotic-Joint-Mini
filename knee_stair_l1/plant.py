"""Simplified single-knee plant with an explicit human/exoskeleton boundary.

The human side contributes only human_extension_nm. The exoskeleton side
contributes only the torque that has already passed the supervisor. Those two
torques meet inside InteractionBoundary. A command above the configured limit
is rejected, so the integrator is not a second, quieter way around the
supervisor.

The constant resist_nm is a theoretical flexion-direction load used as a
stair-support stand-in. It is not a measured ground reaction and not a
full-body stair model. Hip angle is fixed. One degree of freedom.
"""

from __future__ import annotations

import math


class BoundaryError(ValueError):
    """Raised when a torque reaches the plant without passing the boundary check."""


class InteractionBoundary:
    """Ideal torque interface. No compliance and no shared muscle/motor object."""

    def __init__(self, exo_extension_nm: float, abs_limit_nm: float):
        if not math.isfinite(exo_extension_nm):
            raise BoundaryError("non-finite torque at the human/exoskeleton boundary")
        if abs(exo_extension_nm) > abs_limit_nm + 1e-9:
            raise BoundaryError(
                "exo torque above the boundary limit; the supervisor must clamp it first"
            )
        self.torque_on_human_extension_nm = float(exo_extension_nm)
        self.torque_on_exo_extension_nm = -float(exo_extension_nm)


class KneePlant:
    def __init__(self, config: dict):
        plant = config["plant"]
        self.inertia = float(plant["inertia_kg_m2"])
        self.viscous = float(plant["viscous_nm_s_per_rad"])
        self.resist = float(plant["resist_nm"])
        self.q_min = float(plant["min_flexion_rad"])
        self.q_max = float(plant["max_flexion_rad"])
        self.boundary_limit = float(plant["boundary_abs_limit_nm"])
        self.flexion_rad = float(plant["initial_flexion_rad"])
        self.flexion_velocity_rad_s = 0.0
        self.last_interaction_nm = 0.0
        self._check_parameters()

    def _check_parameters(self) -> None:
        if min(self.inertia, self.boundary_limit) <= 0:
            raise ValueError("plant inertia and boundary limit must be positive")
        if self.q_min >= self.q_max:
            raise ValueError("plant joint limits are inverted")
        if not (self.q_min <= self.flexion_rad <= self.q_max):
            raise ValueError("initial flexion is outside the joint limits")

    def advance(self, human_extension_nm: float, exo_extension_nm: float, dt: float) -> None:
        if not math.isfinite(human_extension_nm) or not math.isfinite(dt) or dt <= 0:
            raise BoundaryError("plant step requires a finite human torque and a positive dt")
        boundary = InteractionBoundary(exo_extension_nm, self.boundary_limit)
        # Viscous term opposes flexion velocity and is therefore extension-positive
        # when flexion velocity is positive.
        extension = (
            human_extension_nm
            + boundary.torque_on_human_extension_nm
            - self.resist
            + self.viscous * self.flexion_velocity_rad_s
        )
        flexion_accel = -extension / self.inertia
        self.flexion_velocity_rad_s += flexion_accel * dt
        self.flexion_rad += self.flexion_velocity_rad_s * dt
        if self.flexion_rad > self.q_max:
            self.flexion_rad = self.q_max
            if self.flexion_velocity_rad_s > 0.0:
                self.flexion_velocity_rad_s = 0.0
        elif self.flexion_rad < self.q_min:
            self.flexion_rad = self.q_min
            if self.flexion_velocity_rad_s < 0.0:
                self.flexion_velocity_rad_s = 0.0
        self.last_interaction_nm = boundary.torque_on_human_extension_nm
