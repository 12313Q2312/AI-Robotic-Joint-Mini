"""One local safety supervisor.

States, from least to most restrictive:
NORMAL, LIMITED_ASSIST, DEGRADED, DAMPING_ONLY, SAFE_HOLD, E_STOP.

The legacy angle port of joint_safety_node and the knee torque port share this
object. There is not a second supervisor beside it.

Fault mapping used by the injection suite:
- finite command outside command_limit_nm -> LIMITED_ASSIST.
  command_limit_nm is a plausibility check, not the actuator torque limit.
  The supervisor may still emit up to that check. clamp_actuator_torque is
  what limits the torque that reaches the plant or the motor topic.
- sensor dropout (last finite sample older than sensor_dropout_s) -> DAMPING_ONLY
- communication timeout (last finite command older than command_timeout_s) -> DAMPING_ONLY
- non-finite sensor, intent, or command -> E_STOP, assist torque 0, latched until reset()
- intent or sensor packet age above freshness_threshold_s -> DEGRADED
- that staleness lasting stale_safe_duration_s -> SAFE_HOLD, latched until reset()

E_STOP outranks SAFE_HOLD, which outranks DAMPING_ONLY, which outranks
DEGRADED, which outranks LIMITED_ASSIST.
On E_STOP and SAFE_HOLD the assist command is 0 and passive_damping is true.
DEGRADED caps torque at degraded_torque_fraction times the 24 Nm PID limit
(0.5 * 24 = 12 Nm) and slews into and out of that cap at degraded_slew_nm_per_s.
It returns to NORMAL only after both packets have stayed fresh for
fresh_recover_duration_s. The actuator clamp still runs after this output.
SAFE_HOLD does not clear when fresh packets return; reset() clears it.
DAMPING_ONLY, used for a receipt gap or command timeout, still publishes a
viscous command and does not latch. The legacy safe target angle is left on
its existing contract and is not forced to 0. This is a software state
machine, not a hardware emergency stop. /joint/safety_state is published for
readers and is not a command topic.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

NORMAL = "NORMAL"
LIMITED_ASSIST = "LIMITED_ASSIST"
DEGRADED = "DEGRADED"
DAMPING_ONLY = "DAMPING_ONLY"
SAFE_HOLD = "SAFE_HOLD"
E_STOP = "E_STOP"

_RANK = {NORMAL: 0, LIMITED_ASSIST: 1, DEGRADED: 2, DAMPING_ONLY: 3, SAFE_HOLD: 4, E_STOP: 5}


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
        self.freshness_threshold_s = float(safety.get("freshness_threshold_s", 0.05))
        self.stale_safe_duration_s = float(safety.get("stale_safe_duration_s", 0.2))
        fraction = float(safety.get("degraded_torque_fraction", 0.5))
        pid_limit = float(config["pid"]["torque_limit_nm"])
        self.degraded_cap_nm = fraction * pid_limit
        actuator_limit = float(config.get("actuator_torque_limit_nm", pid_limit))
        if self.degraded_cap_nm > actuator_limit:
            self.degraded_cap_nm = actuator_limit
        self.degraded_slew_nm_per_s = float(safety.get("degraded_slew_nm_per_s", 200.0))
        self.fresh_recover_s = float(safety.get("fresh_recover_duration_s", 0.10))
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
        self._safe_hold_latched = False
        self.intent_seen = False
        self.last_intent_packet_time = float(now)
        self.last_sensor_packet_time = float(now)
        self._stale_since = None
        self._fresh_since = None
        self._needs_fresh_recover = False
        self._slew_output = 0.0
        self._slew_time = float(now)
        self._slew_initialized = False
        self._recover_slew = False

    def observe_legacy_target(self, angle_rad: float, now: float) -> bool:
        """Accept a finite angle. Non-finite samples are ignored, matching the historical node."""
        if not math.isfinite(angle_rad) or not math.isfinite(now):
            return False
        self.legacy_out_of_range = angle_rad < self.legacy_min or angle_rad > self.legacy_max
        self.legacy_target = min(self.legacy_max, max(self.legacy_min, angle_rad))
        self.last_legacy_time = float(now)
        return True

    def observe_sensor(
        self,
        flexion_rad: float,
        flexion_velocity_rad_s: float,
        now: float,
        packet_time: float | None = None,
    ) -> None:
        """Record a sensor sample.

        now is the receipt time and still drives the dropout clock.
        packet_time is when the sample was produced. It defaults to now so
        callers that do not carry a timestamp keep a fresh packet.
        """
        self.knee_port_enabled = True
        stamped = now if packet_time is None else packet_time
        if not all(math.isfinite(value) for value in (flexion_rad, flexion_velocity_rad_s, now, stamped)):
            self._sensor_nonfinite = True
            self._estop_latched = True
            return
        self.last_qd = float(flexion_velocity_rad_s)
        self.last_sensor_time = float(now)
        self.last_sensor_packet_time = float(stamped)
        self.sensor_seen = True

    def observe_intent(self, packet_time: float, receipt_time: float) -> None:
        """Record an intent packet. Age is receipt-independent: it uses packet_time."""
        self.knee_port_enabled = True
        if not math.isfinite(packet_time) or not math.isfinite(receipt_time):
            self._command_nonfinite = True
            self._estop_latched = True
            return
        self.last_intent_packet_time = float(packet_time)
        self.intent_seen = True

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
        torque = self._torque_for(state, clamped, now)
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
        stale_reasons = self._update_staleness(now)
        if self._safe_hold_latched:
            return SAFE_HOLD, tuple(dict.fromkeys((*stale_reasons, "safe_hold_latched"))), False
        sensor_age = math.inf if not self.sensor_seen else now - self.last_sensor_time
        command_age = math.inf if not self.command_seen else now - self.last_command_time
        if sensor_age > self.sensor_dropout_s:
            reasons.append("sensor_dropout")
        if command_age > self.command_timeout_s:
            reasons.append("communication_timeout")
        if reasons:
            return DAMPING_ONLY, tuple(reasons), False
        if stale_reasons:
            self._needs_fresh_recover = True
            return DEGRADED, tuple(stale_reasons), False
        if self._needs_fresh_recover:
            fresh_for = 0.0 if self._fresh_since is None else now - self._fresh_since
            if fresh_for + 1e-12 < self.fresh_recover_s:
                return DEGRADED, ("fresh_recover_pending",), False
            self._needs_fresh_recover = False
        if abs(self.last_command) > self.command_limit_nm:
            return LIMITED_ASSIST, ("command_out_of_range",), True
        return NORMAL, (), False

    def _packet_is_stale(self, now: float, packet_time: float) -> bool:
        # One nanosecond absorbs a 5 ms step that lands on the threshold in float64.
        return now - packet_time > self.freshness_threshold_s + 1e-9

    def _update_staleness(self, now: float) -> tuple[str, ...]:
        reasons = []
        if self.sensor_seen and self._packet_is_stale(now, self.last_sensor_packet_time):
            reasons.append("stale_sensor")
        if self.intent_seen and self._packet_is_stale(now, self.last_intent_packet_time):
            reasons.append("stale_intent")
        if not reasons:
            self._stale_since = None
            if self._fresh_since is None:
                self._fresh_since = now
            return ()
        self._fresh_since = None
        if self._stale_since is None:
            self._stale_since = now
        if now - self._stale_since >= self.stale_safe_duration_s:
            self._safe_hold_latched = True
        return tuple(reasons)

    def _torque_for(self, state: str, clamped: bool, now: float) -> float:
        target = self._target_torque(state, clamped)
        return self._slew_if_degraded(state, target, now)

    def _target_torque(self, state: str, clamped: bool) -> float:
        if not self.knee_port_enabled:
            return 0.0
        if state in (E_STOP, SAFE_HOLD):
            return 0.0
        if state == DAMPING_ONLY:
            # Oppose flexion: positive flexion velocity produces extension torque.
            viscous = self.damping * self.last_qd
            return min(self.damping_limit_nm, max(-self.damping_limit_nm, viscous))
        if state == DEGRADED:
            return min(self.degraded_cap_nm, max(-self.degraded_cap_nm, self.last_command))
        if state == LIMITED_ASSIST or clamped:
            # Plausibility clamp only. The actuator clamp is clamp_actuator_torque.
            return min(self.command_limit_nm, max(-self.command_limit_nm, self.last_command))
        return self.last_command

    def _slew_if_degraded(self, state: str, target: float, now: float) -> float:
        """Slew only while entering or leaving DEGRADED.

        E_STOP and SAFE_HOLD still go to 0 immediately. NORMAL tracking is
        unchanged until a DEGRADED interval has to be left.
        """
        if state in (E_STOP, SAFE_HOLD, DAMPING_ONLY) or not self.knee_port_enabled:
            self._recover_slew = False
            self._slew_output = target
            self._slew_time = now
            self._slew_initialized = True
            return target
        if state == DEGRADED:
            self._recover_slew = True
        rate_limited = state == DEGRADED or self._recover_slew
        dt = now - self._slew_time
        if rate_limited and self._slew_initialized and dt > 0.0:
            max_delta = self.degraded_slew_nm_per_s * dt
            delta = target - self._slew_output
            if delta > max_delta:
                delta = max_delta
            elif delta < -max_delta:
                delta = -max_delta
            self._slew_output += delta
        else:
            self._slew_output = target
        self._slew_time = now
        self._slew_initialized = True
        if state != DEGRADED and abs(self._slew_output - target) <= 1e-9:
            self._recover_slew = False
            self._slew_output = target
        return self._slew_output

    def _decision(self, state, reasons, torque, angle, clamped) -> SafetyDecision:
        return SafetyDecision(
            state=state,
            reasons=reasons,
            safe_extension_torque_nm=torque,
            legacy_safe_target_rad=angle,
            clamped=clamped,
            estop_latched=self._estop_latched,
            passive_damping=state in (E_STOP, SAFE_HOLD),
        )
