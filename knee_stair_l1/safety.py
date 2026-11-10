"""One local safety supervisor.

States, from least to most restrictive:
NORMAL, LIMITED_ASSIST, DAMPING_ONLY, E_STOP.

The legacy angle port of joint_safety_node and the knee torque port share this
object. There is not a second supervisor beside it.

Fault mapping used by the injection suite:
- finite command outside command_limit_nm -> LIMITED_ASSIST, torque clamped
- sensor dropout (last finite sample older than sensor_dropout_s) -> DAMPING_ONLY
- communication timeout (last finite command older than command_timeout_s) -> DAMPING_ONLY
- non-finite sensor or command -> E_STOP, assist torque 0, latched until reset()

E_STOP outranks DAMPING_ONLY, which outranks LIMITED_ASSIST.
On E_STOP the assist command is 0 and passive_damping is true: the exo does
not actively drive. The legacy safe target angle is left on its existing
contract and is not forced to 0. DAMPING_ONLY, used for dropout and timeout,
still publishes a viscous command. This is a software state machine, not a
hardware emergency stop. /joint/safety_state is published for readers and is
not a command topic.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

NORMAL = "NORMAL"
LIMITED_ASSIST = "LIMITED_ASSIST"
DAMPING_ONLY = "DAMPING_ONLY"
E_STOP = "E_STOP"

_RANK = {NORMAL: 0, LIMITED_ASSIST: 1, DAMPING_ONLY: 2, E_STOP: 3}


@dataclass(frozen=True)
class SafetyDecision:
    state: str
    reasons: tuple[str, ...]
    safe_extension_torque_nm: float
    legacy_safe_target_rad: float
    clamped: bool
    estop_latched: bool
    passive_damping: bool


class JointSafetyCore:
    def __init__(self, config: dict, legacy_enabled: bool = True):
        safety = config["safety"]
        self.legacy_enabled = bool(legacy_enabled)
        self.legacy_min = float(safety["legacy_min_angle_rad"])
        self.legacy_max = float(safety["legacy_max_angle_rad"])
        self.legacy_timeout_s = float(safety["legacy_timeout_s"])
        self.command_limit_nm = float(safety["command_limit_nm"])
        self.command_timeout_s = float(safety["command_timeout_s"])
        self.sensor_dropout_s = float(safety["sensor_dropout_s"])
        self.damping = float(safety["damping_nm_s_per_rad"])
        self.damping_limit_nm = float(safety["damping_limit_nm"])
        self.reset(0.0)

    def reset(self, now: float) -> None:
        if not math.isfinite(now):
            raise ValueError("safety reset time must be finite")
        self.legacy_target = 0.0
        self.legacy_out_of_range = False
        self.last_legacy_time = float(now)
        self.knee_port_enabled = False
        self.sensor_seen = False
        self.command_seen = False
        self.last_sensor_time = float(now)
        self.last_command_time = float(now)
        self.last_command = 0.0
        self.last_qd = 0.0
        self._sensor_nonfinite = False
        self._command_nonfinite = False
        self._estop_latched = False

    def observe_legacy_target(self, angle_rad: float, now: float) -> bool:
        """Accept a finite angle. Non-finite samples are ignored, matching the historical node."""
        if not math.isfinite(angle_rad) or not math.isfinite(now):
            return False
        self.legacy_out_of_range = angle_rad < self.legacy_min or angle_rad > self.legacy_max
        self.legacy_target = min(self.legacy_max, max(self.legacy_min, angle_rad))
        self.last_legacy_time = float(now)
        return True

    def observe_sensor(self, flexion_rad: float, flexion_velocity_rad_s: float, now: float) -> None:
        self.knee_port_enabled = True
        if not all(math.isfinite(value) for value in (flexion_rad, flexion_velocity_rad_s, now)):
            self._sensor_nonfinite = True
            self._estop_latched = True
            return
        self.last_qd = float(flexion_velocity_rad_s)
        self.last_sensor_time = float(now)
        self.sensor_seen = True

    def observe_command(self, extension_torque_nm: float, now: float) -> None:
        self.knee_port_enabled = True
        if not math.isfinite(extension_torque_nm) or not math.isfinite(now):
            self._command_nonfinite = True
            self._estop_latched = True
            return
        self.last_command = float(extension_torque_nm)
        self.last_command_time = float(now)
        self.command_seen = True

    def evaluate(self, now: float) -> SafetyDecision:
        if not math.isfinite(now):
            self._estop_latched = True
            return self._decision(E_STOP, ("non_finite_time", "e_stop_latched"), 0.0, self.legacy_target, False)

        legacy_state, legacy_angle, legacy_reasons = self._legacy(now)
        if self.knee_port_enabled:
            knee_state, knee_reasons, clamped = self._knee_state(now)
        else:
            knee_state, knee_reasons, clamped = NORMAL, (), False
        state = knee_state if _RANK[knee_state] >= _RANK[legacy_state] else legacy_state
        if self._estop_latched:
            state = E_STOP
        reasons = tuple(dict.fromkeys((*knee_reasons, *legacy_reasons, *(("e_stop_latched",) if self._estop_latched else ()))))
        torque = self._torque_for(state, clamped)
        return self._decision(state, reasons, torque, legacy_angle, clamped and state == LIMITED_ASSIST)

    def _legacy(self, now: float) -> tuple[str, float, tuple[str, ...]]:
        age = now - self.last_legacy_time
        # Historical joint_safety_node: age <= timeout keeps the clamped target, else 0.
        if self.legacy_enabled and age > self.legacy_timeout_s:
            return DAMPING_ONLY, 0.0, ("legacy_command_timeout",)
        angle = self.legacy_target
        if self.legacy_enabled and self.legacy_out_of_range:
            return LIMITED_ASSIST, angle, ("legacy_command_out_of_range",)
        return NORMAL, angle, ()

    def _legacy_angle(self, now: float) -> float:
        return self._legacy(now)[1]

    def _knee_state(self, now: float) -> tuple[str, tuple[str, ...], bool]:
        reasons = []
        if self._sensor_nonfinite:
            reasons.append("sensor_nan")
        if self._command_nonfinite:
            reasons.append("command_nan")
        if self._estop_latched or self._sensor_nonfinite or self._command_nonfinite:
            return E_STOP, tuple(reasons), False
        sensor_age = math.inf if not self.sensor_seen else now - self.last_sensor_time
        command_age = math.inf if not self.command_seen else now - self.last_command_time
        if sensor_age > self.sensor_dropout_s:
            reasons.append("sensor_dropout")
        if command_age > self.command_timeout_s:
            reasons.append("communication_timeout")
        if reasons:
            return DAMPING_ONLY, tuple(reasons), False
        if abs(self.last_command) > self.command_limit_nm:
            return LIMITED_ASSIST, ("command_out_of_range",), True
        return NORMAL, (), False

    def _torque_for(self, state: str, clamped: bool) -> float:
        if not self.knee_port_enabled:
            return 0.0
        if state == E_STOP:
            return 0.0
        if state == DAMPING_ONLY:
            # Oppose flexion: positive flexion velocity produces extension torque.
            viscous = self.damping * self.last_qd
            return min(self.damping_limit_nm, max(-self.damping_limit_nm, viscous))
        if state == LIMITED_ASSIST or clamped:
            return min(self.command_limit_nm, max(-self.command_limit_nm, self.last_command))
        return self.last_command

    def _decision(self, state, reasons, torque, angle, clamped) -> SafetyDecision:
        return SafetyDecision(
            state=state,
            reasons=reasons,
            safe_extension_torque_nm=torque,
            legacy_safe_target_rad=angle,
            clamped=clamped,
            estop_latched=self._estop_latched,
            passive_damping=state == E_STOP,
        )
