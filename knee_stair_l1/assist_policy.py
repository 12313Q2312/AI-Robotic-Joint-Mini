"""Proportional-share assist policy.

Copied from PR #3 (branch demo/cooperative-assistance-v0.1,
mobility_demo/include/AssistPolicy.hpp). The gain and slew follow that header.
The knee config passes an internal cap of 24 Nm instead of the header's 80 Nm
so the policy does not aim above the actuator. This cap is not the plant
boundary; clamp_actuator_torque still runs on the delivered command.
This file does not modify PR #3.
"""

from __future__ import annotations

import math


class AssistPolicy:
    def __init__(self, gain: float, limit: float, slew: float):
        if (
            not math.isfinite(gain)
            or not math.isfinite(limit)
            or not math.isfinite(slew)
            or gain < 0
            or gain > 1
            or limit <= 0
            or slew <= 0
        ):
            raise ValueError("invalid assistance parameters")
        self.gain = float(gain)
        self.limit = float(limit)
        self.slew = float(slew)
        self.output = 0.0

    def reset(self) -> None:
        self.output = 0.0

    def step(self, human_torque: float, dt: float, enabled: bool) -> float:
        if not math.isfinite(human_torque) or not math.isfinite(dt) or dt <= 0:
            self.output = 0.0
            return 0.0
        if enabled:
            target = self.gain * human_torque
            if target > self.limit:
                target = self.limit
            elif target < -self.limit:
                target = -self.limit
        else:
            target = 0.0
        delta = target - self.output
        max_step = self.slew * dt
        if delta > max_step:
            delta = max_step
        elif delta < -max_step:
            delta = -max_step
        self.output += delta
        return self.output

    def command_is_saturated(self, human_torque: float, enabled: bool) -> bool:
        if not enabled or not math.isfinite(human_torque):
            return False
        return abs(self.gain * human_torque) > self.limit
