import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import webots_ros2_driver.webots_controller as webots_controller


def _windows_host_gateway():
    route = subprocess.check_output(
        ["ip", "route", "show", "default"], text=True
    ).split()
    try:
        return route[route.index("via") + 1]
    except (ValueError, IndexError) as error:
        raise RuntimeError("Unable to find the Windows host gateway.") from error


def generate_launch_description():
    package_dir = get_package_share_directory("ai_robotic_joint_ros2")
    robot_description = os.path.join(
        package_dir, "resource", "joint_controller.urdf"
    )

    # webots_ros2 2025.0.0 reads /etc/resolv.conf on WSL, which can point to
    # a VPN/DNS server instead of the Windows host. Use the WSL default route
    # for this launch without modifying the installed ROS package or network.
    host_gateway = _windows_host_gateway()
    webots_controller.controller_ip_address = lambda: host_gateway

    joint_driver = webots_controller.WebotsController(
        robot_name="joint_base",
        parameters=[{"robot_description": robot_description}],
        respawn=True,
        port="1234",
    )

    safety_node = Node(
        package="ai_robotic_joint_ros2",
        executable="joint_safety_node.py",
        name="joint_safety",
        output="screen",
    )

    return LaunchDescription([safety_node, joint_driver])
