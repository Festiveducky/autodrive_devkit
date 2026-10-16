"""Launch the controller with its editable YAML tuning file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Load package defaults or a user-supplied params_file."""
    config = os.path.join(
        get_package_share_directory('autodrive_roboracer'), 'config', 'wall_follow.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(
            package='autodrive_roboracer', executable='wall_follow_node',
            name='wall_follow_node', output='screen', emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
