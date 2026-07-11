import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('leg_odometry')
    default_params = os.path.join(pkg, 'config', 'legged_odometry.yaml')
    imu_topic = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        Node(
            package='leg_odometry',
            executable='legged_odometry_node',
            name='legged_odometry',
            output='screen',
            parameters=[default_params, {'imu_topic': imu_topic}],
        ),
    ])
