"""Launch passive diagnostics; use the mounted source tree for persistent logs."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose recording and health settings without starting a driving controller."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value='/home/autodrive_devkit/src/autodrive_devkit/diagnostics'),
        DeclareLaunchArgument('include_ground_truth', default_value='false'),
        DeclareLaunchArgument('stale_timeout', default_value='0.5'),
        DeclareLaunchArgument('report_period', default_value='1.0'),
        DeclareLaunchArgument('topic_prefix', default_value='/autodrive/roboracer_1'),
        Node(
            package='autodrive_roboracer', executable='sensor_diagnostics_node',
            name='sensor_diagnostics_node', output='screen', emulate_tty=True,
            parameters=[{
                'output_dir': ParameterValue(LaunchConfiguration('output_dir'), value_type=str),
                'topic_prefix': ParameterValue(
                    LaunchConfiguration('topic_prefix'), value_type=str),
                'include_ground_truth': ParameterValue(
                    LaunchConfiguration('include_ground_truth'), value_type=bool),
                'stale_timeout': ParameterValue(
                    LaunchConfiguration('stale_timeout'), value_type=float),
                'report_period': ParameterValue(
                    LaunchConfiguration('report_period'), value_type=float),
            }],
        ),
    ])
