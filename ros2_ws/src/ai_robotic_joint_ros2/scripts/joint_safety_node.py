#!/usr/bin/env python3
"""Single safety supervisor for the legacy angle port and the knee assist port.

The numeric legacy map is unchanged: non-finite targets are ignored, finite
targets are clamped to the configured angle window, and a command older than
the configured timeout publishes 0 rad. /knee_stair/assist_command_safe carries
the torque after clamp_actuator_torque, the same value the headless plant
applies. That clamp raises ValueError on NaN or inf and does not return 0.
on_assist_command catches the exception, latches E_STOP, and this node
publishes 0. The safe target angle is not rewritten. /joint/safety_state is
published for readers and is not a command input. Legacy C++ controllers do
not use this clamp.
"""
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


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
from knee_stair_l1.safe_publish import on_assist_command, republish_supervised
from knee_stair_l1.safety import JointSafetyCore
from knee_stair_l1.stamp import stamped_packet_time
from knee_stair_l1.topics import (
    TOPIC_ASSIST_COMMAND_RAW,
    TOPIC_ASSIST_COMMAND_SAFE,
    TOPIC_INTENT_SYNTHETIC,
    TOPIC_LEGACY_SAFE_TARGET,
    TOPIC_LEGACY_TARGET,
    TOPIC_PLANT_STATE,
    TOPIC_SAFETY_STATE,
)


class JointSafety(Node):
    def __init__(self):
        super().__init__("joint_safety")
        self._config = load_config()
        self.core = JointSafetyCore(self._config, legacy_enabled=True)
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
            JointState, TOPIC_PLANT_STATE, self.on_plant, 10
        )
        self.intent_subscription = self.create_subscription(
            JointState, TOPIC_INTENT_SYNTHETIC, self.on_intent, 10
        )
        self.timer = self.create_timer(0.05, self.publish_supervised)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_target(self, message: Float64) -> None:
        if not self.core.observe_legacy_target(message.data, self._now()):
            self.get_logger().warn("Rejected non-finite target angle")

    def on_assist(self, message: Float64) -> None:
        # on_assist_command catches ValueError from clamp_actuator_torque.
        self._emit(on_assist_command(self.core, self._config, self._now(), message.data))

    def on_plant(self, message: JointState) -> None:
        if len(message.position) < 1 or len(message.velocity) < 1:
            self.get_logger().warn("Rejected plant state without position and velocity")
            return
        now = self._now()
        packet_time, reason = stamped_packet_time(
            self.core, message, now, float(self._config["safety"]["stamp_future_tolerance_s"]),
        )
        if reason is not None:
            self.get_logger().warn("Plant stamp is stale (%s); using source time, not receipt time", reason)
        self.core.observe_sensor(message.position[0], message.velocity[0], now, packet_time=packet_time)

    def on_intent(self, message: JointState) -> None:
        now = self._now()
        packet_time, reason = stamped_packet_time(
            self.core, message, now, float(self._config["safety"]["stamp_future_tolerance_s"]),
        )
        if reason is not None:
            self.get_logger().warn("Intent stamp is stale (%s); using source time, not receipt time", reason)
        self.core.observe_intent(packet_time, now)

    def publish_supervised(self) -> None:
        self._emit(republish_supervised(self.core, self._config, self._now()))

    def _emit(self, outcome) -> None:
        angle = Float64()
        angle.data = outcome.legacy_safe_target_rad
        self.publisher.publish(angle)
        state = String()
        state.data = outcome.state
        self.state_publisher.publish(state)
        torque = Float64()
        torque.data = outcome.applied_torque_nm
        self.safe_torque_publisher.publish(torque)
        if outcome.state != self._previous_state:
            self.get_logger().info(
                "safety state %s reasons=%s", outcome.state, ",".join(outcome.reasons),
            )
            self._previous_state = outcome.state


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
