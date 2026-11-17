"""Independent, ROS-free acceptance logic for RJ-INT-01.

Observations are captured by the live ROS 2 smoke client, never manufactured
by this module. Unit tests exercise the assertion logic, not a simulator.
"""
import math


def assess(observations, target_rad=0.6, minimum_motion_rad=0.03):
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
    if any(not math.isfinite(x) for x in angles + torques):
        failures.append("Non-finite joint state value")
    direction = 1.0 if target_rad >= 0.0 else -1.0
    motion_rad = (
        max(direction * (x - angles[0]) for x in angles)
        if angles and all(math.isfinite(x) for x in angles) else 0.0
    )
    if motion_rad < minimum_motion_rad:
        failures.append("Observed joint motion below threshold (no proven physical simulation)")
    return {
        "passed": not failures,
        "failures": failures,
        "samples": len(samples),
        "maximum_target_direction_motion_rad": motion_rad,
        "minimum_required_motion_rad": minimum_motion_rad,
        "safe_target_samples": len(forwarded),
        "safety_state_samples": len(states),
        "evidence_scope": "Webots physics and ROS 2 topic observations; not real hardware or knee assistance",
    }
