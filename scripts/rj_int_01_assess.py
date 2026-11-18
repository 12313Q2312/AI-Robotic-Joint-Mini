"""Independent, ROS-free acceptance logic for RJ-INT-01.

Observations are captured by the live ROS 2 smoke client, never manufactured
by this module. Unit tests exercise the assertion logic, not a simulator.
"""
import json
import math
from pathlib import Path

# End of the move phase must be this close to the commanded angle.
# 0.02 rad is about 1.1 degrees, the same band already used when checking that
# the supervisor forwarded the target. The recorded Webots run finished the
# move phase at the 0.6 rad command, so this rejects a joint that only twitched
# and stopped far from the target. It is not a comfort or hardware tolerance.
SETTLE_TOLERANCE_RAD = 0.02

# safety.legacy_timeout_s is 1.0 s (age strictly greater than 1.0 s). The
# supervisor publishes on a 0.05 s timer. 1.5 s covers that timeout plus
# delivery, and is inside the smoke client's 2.2 s timeout window.
DEGRADE_WITHIN_S = 1.5


def _committed_actuator_limit_nm():
    # The checker is installed beside the smoke client. The same JSON is the
    # source-tree file, the copy under the installed package, or the share/
    # config file. A missing file is an error; there is no substitute number.
    here = Path(__file__).resolve()
    candidates = (
        here.parents[1] / "knee_stair_l1" / "config" / "default.json",
        here.parent / "knee_stair_l1" / "config" / "default.json",
        here.parents[2] / "share" / "ai_robotic_joint_ros2" / "config" / "default.json",
    )
    for path in candidates:
        if path.is_file():
            return float(json.loads(path.read_text(encoding="utf-8"))["actuator_torque_limit_nm"])
    raise FileNotFoundError("committed actuator config default.json was not found")


def assess(observations, target_rad=0.6, minimum_motion_rad=0.03,
           actuator_limit_nm=None, settle_tolerance_rad=SETTLE_TOLERANCE_RAD,
           degrade_within_s=DEGRADE_WITHIN_S):
    if actuator_limit_nm is None:
        actuator_limit_nm = _committed_actuator_limit_nm()
    samples = observations.get("joint_states", [])
    forwarded = observations.get("safe_targets", [])
    states = observations.get("safety_states", [])
    failures = []

    if not samples:
        failures.append("No /joint/state samples received from the Webots driver")
    if not forwarded or not any(
        item["phase"] == "move" and abs(item["value"] - target_rad) <= 0.02
        for item in forwarded
    ):
        failures.append("Supervisor did not forward the requested target angle")
    if not any(
        item["phase"] == "timeout" and abs(item["value"]) <= 1e-5
        for item in forwarded
    ):
        failures.append("No zero safe target observed after command timeout")
    if not any(
        item["phase"] == "timeout" and item["value"] == "DAMPING_ONLY"
        for item in states
    ):
        failures.append("Supervisor did not report DAMPING_ONLY after timeout")

    angles = [item["angle_rad"] for item in samples]
    torques = [item["effort_nm"] for item in samples]
    def finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)

    if any(not finite(x) for x in angles + torques):
        failures.append("Non-finite joint state value")
    if any(finite(torque) and abs(torque) > actuator_limit_nm for torque in torques):
        failures.append("Commanded torque left the actuator limit")
    move_samples = [item for item in samples if item.get("phase") == "move"]
    if not move_samples:
        failures.append("No move-phase joint sample to judge settling")
    elif finite(move_samples[-1]["angle_rad"]) and abs(move_samples[-1]["angle_rad"] - target_rad) > settle_tolerance_rad:
        failures.append("Joint did not settle near the target at the end of the move")
    elif not finite(move_samples[-1]["angle_rad"]):
        failures.append("Joint did not settle near the target at the end of the move")
    timeout_times = []
    for group in ("joint_states", "safe_targets", "safety_states"):
        for item in observations.get(group, []):
            if item.get("phase") == "timeout" and isinstance(item.get("elapsed_wall_s"), (int, float)):
                timeout_times.append(float(item["elapsed_wall_s"]))
    if timeout_times:
        timeout_start = min(timeout_times)
        deadline = timeout_start + degrade_within_s
        zero_targets = [
            item for item in forwarded
            if item.get("phase") == "timeout" and finite(item.get("value")) and abs(item["value"]) <= 1e-5
            and isinstance(item.get("elapsed_wall_s"), (int, float))
        ]
        damping_states = [
            item for item in states
            if item.get("phase") == "timeout" and item.get("value") == "DAMPING_ONLY"
            and isinstance(item.get("elapsed_wall_s"), (int, float))
        ]
        if not zero_targets or min(item["elapsed_wall_s"] for item in zero_targets) > deadline:
            failures.append("Safe target did not reach 0 within the timeout bound")
        if not damping_states or min(item["elapsed_wall_s"] for item in damping_states) > deadline:
            failures.append("DAMPING_ONLY did not appear within the timeout bound")
    else:
        failures.append("Timeout degradation time was not recorded")
    direction = 1.0 if target_rad >= 0.0 else -1.0
    motion_rad = (
        max(direction * (x - angles[0]) for x in angles)
        if angles and all(finite(x) for x in angles) else 0.0
    )
    if motion_rad < minimum_motion_rad:
        failures.append("Observed joint motion below threshold (no proven physical simulation)")
    return {
        "passed": not failures,
        "failures": failures,
        "samples": len(samples),
        "maximum_target_direction_motion_rad": motion_rad,
        "minimum_required_motion_rad": minimum_motion_rad,
        "settle_tolerance_rad": settle_tolerance_rad,
        "degrade_within_s": degrade_within_s,
        "actuator_limit_nm": actuator_limit_nm,
        "safe_target_samples": len(forwarded),
        "safety_state_samples": len(states),
        "evidence_scope": "Webots physics and ROS 2 topic observations; not real hardware or knee assistance",
    }
