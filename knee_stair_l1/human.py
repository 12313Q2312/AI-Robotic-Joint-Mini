"""Bounded synthetic human knee torque.

This is not EMG and not a measured subject. The torque tracks the synthetic
desired trajectory and is clipped to human.torque_limit_nm.
"""

from __future__ import annotations

import math


class BoundedHumanKnee:
    def __init__(self, config: dict):
        human = config["human"]
        self.kp = float(human["kp_nm_per_rad"])
        self.kd = float(human["kd_nm_s_per_rad"])
        self.limit = float(human["torque_limit_nm"])
        if self.kp < 0 or self.kd < 0 or self.limit <= 0:
            raise ValueError("human gains must be non-negative and the torque limit positive")

    def torque_flexion_nm(self, q_ref_rad: float, qd_ref_rad_s: float, q_rad: float, qd_rad_s: float) -> float:
        if not all(math.isfinite(value) for value in (q_ref_rad, qd_ref_rad_s, q_rad, qd_rad_s)):
            raise ValueError("human tracker requires finite inputs")
        raw = self.kp * (q_ref_rad - q_rad) + self.kd * (qd_ref_rad_s - qd_rad_s)
        return min(self.limit, max(-self.limit, raw))
