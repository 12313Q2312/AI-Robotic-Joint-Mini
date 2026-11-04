#!/usr/bin/env python3
"""Safety supervisor between high-level commands and the torque controller."""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class JointSafety(Node):
    def __init__(self):
        super().__init__("joint_safety")
        self.min_angle = math.radians(-90.0)
        self.max_angle = math.radians(90.0)
        self.timeout_s = 1.0
        self.target = 0.0
        self.last_command_time = self.get_clock().now()
        self.publisher = self.create_publisher(Float64, "/joint/safe_target_angle", 10)
        self.subscription = self.create_subscription(
            Float64, "/joint/target_angle", self.on_target, 10
        )
        self.timer = self.create_timer(0.05, self.publish_safe_target)

    def on_target(self, message: Float64):
        if not math.isfinite(message.data):
            self.get_logger().warn("Rejected non-finite target angle")
            return
        self.target = max(self.min_angle, min(self.max_angle, message.data))
        self.last_command_time = self.get_clock().now()

    def publish_safe_target(self):
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        output = Float64()
        output.data = self.target if age <= self.timeout_s else 0.0
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = JointSafety()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
