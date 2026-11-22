"""Publish the v2 description, with optional standalone joint states."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('big_bertha_v2_description'))
    description = (package_share / 'urdf' / 'big_bertha_v2.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use the Isaac /clock topic'),
        DeclareLaunchArgument(
            'publish_jsp', default_value='true',
            description='Publish standalone joint states; disable when Isaac supplies them'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'robot_description': ParameterValue(description, value_type=str),
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_jsp')),
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'robot_description': ParameterValue(description, value_type=str),
            }],
        ),
    ])
