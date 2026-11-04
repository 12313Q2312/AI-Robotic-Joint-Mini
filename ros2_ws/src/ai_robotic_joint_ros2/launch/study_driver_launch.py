"""Connect the isolated paired fixture. No second reference or panel process."""
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
        robot_name='joint_study', port='1236', respawn=False,
        parameters=[{'robot_description': os.path.join(package_dir, 'resource', 'study_controller.urdf'),
                     'study_package_share': package_dir}])
    return LaunchDescription([driver])
