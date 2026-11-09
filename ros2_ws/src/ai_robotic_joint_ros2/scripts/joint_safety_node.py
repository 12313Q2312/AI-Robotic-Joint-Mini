#!/usr/bin/env python3
"""Single safety supervisor for the legacy angle port and the knee assist port.

The numeric legacy map is unchanged: non-finite targets are ignored, finite
targets are clamped to the configured angle window, and a command older than
the configured timeout publishes 0 rad. Knee torque is published only after
JointSafetyCore has evaluated it. On E_STOP the assist torque is 0 and the
state topic reports the latch; the safe target angle is not rewritten.
/joint/safety_state is published for readers and is not a command input.
"""
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String


def _ensure_layer1_path() -> None:
    here = Path(__file__).resolve()
    for candidate in (here.parent, *here.parents):
        if (candidate / "knee_stair_l1" / "safety.py").is_file():
            entry = str(candidate)
            if entry not in sys.path:
                sys.path.insert(0, entry)
            return
    raise ImportError("knee_stair_l1 is required by the merged joint safety node")


_ensure_layer1_path()

from knee_stair_l1.config import load_config
from knee_stair_l1.safety import JointSafetyCore
from knee_stair_l1.topics import (
    TOPIC_ASSIST_COMMAND_RAW,
    TOPIC_ASSIST_COMMAND_SAFE,
    TOPIC_LEGACY_SAFE_TARGET,
    TOPIC_LEGACY_TARGET,
    TOPIC_PLANT_STATE,
    TOPIC_SAFETY_STATE,
)


class JointSafety(Node):
    def __init__(self):
        super().__init__("joint_safety")
        self.core = JointSafetyCore(load_config(), legacy_enabled=True)
        self.core.reset(self._now())
        self._previous_state = None
        self.publisher = self.create_publisher(Float64, TOPIC_LEGACY_SAFE_TARGET, 10)
        self.state_publisher = self.create_publisher(String, TOPIC_SAFETY_STATE, 10)
        self.safe_torque_publisher = self.create_publisher(Float64, TOPIC_ASSIST_COMMAND_SAFE, 10)
        self.subscription = self.create_subscription(Float64, TOPIC_LEGACY_TARGET, self.on_target, 10)
        self.command_subscription = self.create_subscription(
            Float64, TOPIC_ASSIST_COMMAND_RAW, self.on_assist, 10
        )
        self.plant_subscription = self.create_subscription(
            Float64MultiArray, TOPIC_PLANT_STATE, self.on_plant, 10
        )
        self.timer = self.create_timer(0.05, self.publish_supervised)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_target(self, message: Float64) -> None:
        if not self.core.observe_legacy_target(message.data, self._now()):
            self.get_logger().warn("Rejected non-finite target angle")

    def on_assist(self, message: Float64) -> None:
        self.core.observe_command(message.data, self._now())

    def on_plant(self, message: Float64MultiArray) -> None:
        values = list(message.data)
        if len(values) < 2:
            self.get_logger().warn("Rejected plant state with fewer than 2 fields")
            return
        self.core.observe_sensor(values[0], values[1], self._now())

    def publish_supervised(self) -> None:
        decision = self.core.evaluate(self._now())
        angle = Float64()
        angle.data = decision.legacy_safe_target_rad
        self.publisher.publish(angle)
        state = String()
        state.data = decision.state
        self.state_publisher.publish(state)
        torque = Float64()
        torque.data = decision.safe_extension_torque_nm
        self.safe_torque_publisher.publish(torque)
        if decision.state != self._previous_state:
            self.get_logger().info("safety state %s reasons=%s", decision.state, ",".join(decision.reasons))
            self._previous_state = decision.state


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
