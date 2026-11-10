"""One local safety supervisor.

States, from least to most restrictive:
NORMAL, LIMITED_ASSIST, DEGRADED, DAMPING_ONLY, SAFE_HOLD, E_STOP.

The legacy angle port of joint_safety_node and the knee torque port share this
object. There is not a second supervisor beside it.

Fault mapping used by the injection suite:
- finite command outside command_limit_nm -> LIMITED_ASSIST, torque clamped
- sensor dropout (last finite sample older than sensor_dropout_s) -> DAMPING_ONLY
- communication timeout (last finite command older than command_timeout_s) -> DAMPING_ONLY
- non-finite sensor, intent, or command -> E_STOP, assist torque 0, latched until reset()
- intent or sensor packet age above freshness_threshold_s -> DEGRADED
- that staleness lasting stale_safe_duration_s -> SAFE_HOLD, latched until reset()

E_STOP outranks SAFE_HOLD, which outranks DAMPING_ONLY, which outranks
DEGRADED, which outranks LIMITED_ASSIST.
On E_STOP and SAFE_HOLD the assist command is 0 and passive_damping is true.
DEGRADED still passes the clamped assist command and returns to NORMAL, or
LIMITED_ASSIST, on the first evaluation where both packet ages are fresh.
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
            over_limit = abs(self.last_command) > self.command_limit_nm
            reasons = list(stale_reasons)
            if over_limit:
                reasons.append("command_out_of_range")
            return DEGRADED, tuple(reasons), over_limit
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
            return ()
        if self._stale_since is None:
            self._stale_since = now
        if now - self._stale_since >= self.stale_safe_duration_s:
            self._safe_hold_latched = True
        return tuple(reasons)

    def _torque_for(self, state: str, clamped: bool) -> float:
        if not self.knee_port_enabled:
            return 0.0
        if state in (E_STOP, SAFE_HOLD):
            return 0.0
        if state == DAMPING_ONLY:
            # Oppose flexion: positive flexion velocity produces extension torque.
            viscous = self.damping * self.last_qd
            return min(self.damping_limit_nm, max(-self.damping_limit_nm, viscous))
        if state in (DEGRADED, LIMITED_ASSIST) or clamped:
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
            passive_damping=state in (E_STOP, SAFE_HOLD),
        )
