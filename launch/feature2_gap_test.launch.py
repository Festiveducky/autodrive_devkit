"""Run the bridge watchdog with the existing gap follower at a bounded throttle."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Use one bridge and one controller; a controller exit shuts down this test."""
    config = os.path.join(
        get_package_share_directory('autodrive_roboracer'), 'config', 'wall_follow.yaml')
    bridge = Node(
        package='autodrive_roboracer', executable='autodrive_bridge',
        name='autodrive_bridge', output='screen', parameters=[{
            'command_timeout': ParameterValue(
                LaunchConfiguration('command_timeout'), value_type=float),
            'command_owner': '/wall_follow_node',
        }])
    controller = Node(
        package='autodrive_roboracer', executable='wall_follow_node',
        name='wall_follow_node', output='screen', parameters=[config, {
            'max_throttle': ParameterValue(
                LaunchConfiguration('max_throttle'), value_type=float),
        }])
    return LaunchDescription([
        DeclareLaunchArgument('max_throttle', default_value='0.12'),
        DeclareLaunchArgument('command_timeout', default_value='0.25'),
        bridge, controller,
        RegisterEventHandler(OnProcessExit(
            target_action=controller, on_exit=[Shutdown(reason='test controller exited')])),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge, on_exit=[Shutdown(reason='test bridge exited')])),
    ])
