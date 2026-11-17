"""Native-Linux single-joint ROS 2/Webots integration entry point.

Unlike joint_launch.py (legacy Windows-host Webots), this launch starts
Webots on the same Linux host. It verifies the existing /joint/* angle loop,
NOT the synthetic knee_stair_l1 torque/biomechanics loop.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory("ai_robotic_joint_ros2")
    config_path = os.path.join(package_dir, "config", "default.json")
    urdf_path = os.path.join(package_dir, "resource", "joint_controller.urdf")
    world_path = os.path.join(package_dir, "worlds", "joint_minimal_ros2.wbt")

    # The C++ actuator gate must read the installed config, independent of
    # the shell's current working directory. An absent file fails closed.
    config = SetEnvironmentVariable(
        name="AI_ROBOTIC_JOINT_CONFIG", value=config_path
    )
    webots = WebotsLauncher(world=world_path)
    driver = WebotsController(
        robot_name="joint_base",
        parameters=[{"robot_description": urdf_path}],
        respawn=False,
    )
    supervisor = Node(
        package="ai_robotic_joint_ros2",
        executable="joint_safety_node.py",
        name="joint_safety",
        output="screen",
    )

    return LaunchDescription([
        config,
        webots,
        supervisor,
        driver,
        RegisterEventHandler(
            OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
