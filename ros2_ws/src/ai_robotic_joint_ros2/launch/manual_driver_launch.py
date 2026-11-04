"""Connect only the dedicated demo controller; do not start another panel."""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import webots_ros2_driver.webots_controller as webots_controller


def generate_launch_description():
    package_dir = get_package_share_directory('ai_robotic_joint_ros2')
    route = subprocess.check_output(['ip', 'route', 'show', 'default'], text=True).split()
    gateway = route[route.index('via') + 1]
    webots_controller.controller_ip_address = lambda: gateway
    driver = webots_controller.WebotsController(
        robot_name='joint_manual_demo', port='1235', respawn=False,
        parameters=[{'robot_description': os.path.join(package_dir, 'resource', 'manual_controller.urdf')}])
    return LaunchDescription([driver])
