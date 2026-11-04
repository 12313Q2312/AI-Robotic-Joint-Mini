#!/usr/bin/env python3
"""Repeatable target-and-state capture for the AI Robotic Joint Mini."""
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class Experiment(Node):
    def __init__(self, target: float, duration: float, output: Path):
        super().__init__("joint_experiment")
        self.target = target
        self.end_time = time.monotonic() + duration
        self.output = output
        self.latest = None
        self.publisher = self.create_publisher(Float64, "/joint/target_angle", 10)
        self.subscription = self.create_subscription(
            JointState, "/joint/state", self.on_state, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.1, self.tick)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            ["stamp_sec", "stamp_nanosec", "joint_name", "target_angle_rad",
             "actual_angle_rad", "velocity_rad_s", "effort_nm"]
        )

    def tick(self):
        message = Float64()
        message.data = self.target
        self.publisher.publish(message)
        if time.monotonic() >= self.end_time:
            self.get_logger().info(f"Captured experiment data: {self.output}")
            rclpy.shutdown()

    def on_state(self, message: JointState):
        if not message.position:
            return
        stamp = message.header.stamp
        self.writer.writerow([
            stamp.sec,
            stamp.nanosec,
            message.name[0] if message.name else "",
            self.target,
            message.position[0],
            message.velocity[0] if message.velocity else math.nan,
            message.effort[0] if message.effort else math.nan,
        ])
        self.file.flush()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-deg", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Experiment(math.radians(args.target_deg), args.duration, args.output)
    try:
        rclpy.spin(node)
    finally:
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
