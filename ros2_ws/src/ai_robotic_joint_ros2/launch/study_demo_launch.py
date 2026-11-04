"""Isolated paired controller and local study panel. No old Demo takeover."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('ai_robotic_joint_ros2')

    def panel(context):
        args = ['--port', '8766']
        record = LaunchConfiguration('record_file').perform(context)
        if record: args += ['--record', record]
        if LaunchConfiguration('read_only').perform(context).lower() == 'true':
            args += ['--read-only']
        return [Node(package='ai_robotic_joint_ros2', executable='study_panel_server.py',
                     name='joint_study_panel', output='screen', arguments=args)]

    return LaunchDescription([
        DeclareLaunchArgument('record_file', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'study_driver_launch.py'))),
        OpaqueFunction(function=panel),
    ])
