#!/usr/bin/env python3
"""Live RJ-INT-01 acceptance: ROS 2 -> supervisor -> Webots -> ROS 2.

Run after joint_native_launch.py in another terminal. This script does not
create joint-state observations or claim any human/hardware validation.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from rj_int_01_assess import assess


class JointIntegrationSmoke(Node):
    def __init__(self):
        super().__init__("rj_int_01_smoke")
        self.phase = "startup"
        self.observations = {
            "joint_states": [],
            "safe_targets": [],
            "safety_states": [],
        }
        self.publisher = self.create_publisher(Float64, "/joint/target_angle", 10)
        self.create_subscription(
            JointState, "/joint/state", self.on_joint, qos_profile_sensor_data
        )
        self.create_subscription(
            Float64, "/joint/safe_target_angle", self.on_safe_target, 10
        )
        self.create_subscription(
            String, "/joint/safety_state", self.on_safety_state, 10
        )

    def on_joint(self, msg):
        if not msg.position:
            return
        def safe_number(value):
            value = float(value)
            return value if math.isfinite(value) else None

        self.observations["joint_states"].append({
            "phase": self.phase,
            "elapsed_wall_s": round(time.monotonic() - self.started_wall, 6),
            "angle_rad": safe_number(msg.position[0]),
            "velocity_rad_s": safe_number(msg.velocity[0]) if msg.velocity else None,
            "effort_nm": safe_number(msg.effort[0]) if msg.effort else None,
            "ros_stamp_sec": int(msg.header.stamp.sec),
            "ros_stamp_nanosec": int(msg.header.stamp.nanosec),
        })

    def on_safe_target(self, msg):
        self.observations["safe_targets"].append({
            "phase": self.phase, "value": float(msg.data),
        })

    def on_safety_state(self, msg):
        self.observations["safety_states"].append({
            "phase": self.phase, "value": str(msg.data),
        })

    def send(self, target_rad):
        msg = Float64()
        msg.data = target_rad
        self.publisher.publish(msg)


def spin_for(node, duration_s, target_rad=None):
    start = time.monotonic()
    next_publish = start
    while rclpy.ok() and time.monotonic() - start < duration_s:
        now = time.monotonic()
        if target_rad is not None and now >= next_publish:
            node.send(target_rad)
            next_publish = now + 0.10
        rclpy.spin_once(node, timeout_sec=0.03)


def execute(args):
    if not math.isfinite(args.target_rad) or abs(args.target_rad) > 1.4:
        raise ValueError("target_rad must be finite and within +/-1.4 rad")
    if args.move_seconds <= 0 or args.timeout_seconds < 1.5 or args.startup_seconds <= 0:
        raise ValueError("Invalid durations; timeout_seconds must be >=1.5")
    rclpy.init()
    node = JointIntegrationSmoke()
    node.started_wall = time.monotonic()
    try:
        startup_deadline = time.monotonic() + args.startup_seconds
        while rclpy.ok() and not node.observations["joint_states"]:
            if time.monotonic() >= startup_deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.observations["joint_states"]:
            node.phase = "move"
            spin_for(node, args.move_seconds, target_rad=args.target_rad)
            # No more commands. The safety supervisor should time out the
            # legacy target and publish a zero reference + DAMPING_ONLY.
            node.phase = "timeout"
            spin_for(node, args.timeout_seconds)

        result = assess(node.observations, target_rad=args.target_rad)
        report = {
            "experiment": "RJ-INT-01 / existing single-joint angle interface",
            "status": "PASS" if result["passed"] else "FAIL",
            "target_rad": args.target_rad,
            "move_seconds": args.move_seconds,
            "timeout_seconds": args.timeout_seconds,
            "startup_seconds": args.startup_seconds,
            "time_basis": "monotonic wall clock; ROS message timestamps separately recorded",
            "result": result,
            "observations": node.observations,
            "limitations": [
                "This is the legacy /joint/target_angle angle control path.",
                "It does not validate the knee_stair_l1 torque loop, exoskeleton assistance, or hardware safety.",
                "JointState.effort is the applied command from the plugin, not a torque sensor measurement.",
            ],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(f"RJ-INT-01 {report['status']} | {args.output} | {result}")
        return 0 if result["passed"] else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-rad", type=float, default=0.6)
    parser.add_argument("--startup-seconds", type=float, default=30.0)
    parser.add_argument("--move-seconds", type=float, default=8.0)
    parser.add_argument("--timeout-seconds", type=float, default=2.2)
    parser.add_argument(
        "--output", type=Path, default=Path("local/rj_int_01_smoke.json")
    )
    return execute(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
