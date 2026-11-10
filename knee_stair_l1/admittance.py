"""Admittance map from synthetic extension effort to a desired knee flexion.

M * qdd + B * qd + K * (q - q_ref) = -effort_extension

Positive extension effort reduces desired flexion. The inner PID then tracks
that desired flexion. There is no series compliance here; buffer hardware is
out of scope.
"""

from __future__ import annotations

import math


class Admittance:
    def __init__(self, mass: float, damping: float, stiffness: float, q_min: float, q_max: float):
        if min(mass, damping, stiffness) <= 0:
            raise ValueError("admittance mass, damping and stiffness must be positive")
        if not all(math.isfinite(value) for value in (mass, damping, stiffness, q_min, q_max)):
            raise ValueError("admittance parameters must be finite")
        if q_min >= q_max:
            raise ValueError("admittance joint limits are inverted")
        self.mass = float(mass)
        self.damping = float(damping)
        self.stiffness = float(stiffness)
        self.q_min = float(q_min)
        self.q_max = float(q_max)
        self.q = 0.0
        self.qd = 0.0

    def reset(self, flexion_rad: float) -> None:
        if not math.isfinite(flexion_rad):
            raise ValueError("admittance reset requires a finite angle")
        self.q = min(self.q_max, max(self.q_min, flexion_rad))
        self.qd = 0.0

    def step(self, effort_extension_nm: float, q_ref_rad: float, dt: float) -> tuple[float, float, float]:
        if dt <= 0 or not all(math.isfinite(value) for value in (effort_extension_nm, q_ref_rad, dt)):
            raise ValueError("admittance step requires finite inputs and a positive dt")
        qdd = (
            -effort_extension_nm
            - self.damping * self.qd
            - self.stiffness * (self.q - q_ref_rad)
        ) / self.mass
        self.qd += qdd * dt
        self.q += self.qd * dt
        if self.q > self.q_max:
            self.q = self.q_max
            if self.qd > 0.0:
                self.qd = 0.0
        elif self.q < self.q_min:
            self.q = self.q_min
            if self.qd < 0.0:
                self.qd = 0.0
        return self.q, self.qd, qdd
