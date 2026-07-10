#!/usr/bin/env python3
"""
Launch the AHRS 3D visualizer for Big Bertha.

Starts the Open3D-based visualizer that subscribes to IMU data
and displays robot orientation in real time.

If ``use_hardware:=true``, also launches the hardware bridge node
(requires the physical robot or the Python TCP relay).

Usage::

    # With hardware
    ros2 launch ahrs visualizer.launch.py use_hardware:=true

    # Without hardware (for development / testing)
    ros2 launch ahrs visualizer.launch.py

    # Custom config path
    ros2 launch ahrs visualizer.launch.py config:=/path/to/config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ahrs_pkg = get_package_share_directory("ahrs")
    default_config = os.path.join(ahrs_pkg, "config", "config.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to YAML configuration file",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Launch hardware bridge alongside the visualizer",
        ),

        LogInfo(
            msg=[
                "Starting AHRS Visualizer (config: ",
                LaunchConfiguration("config"),
                ")",
            ]
        ),

        Node(
            package="ahrs",
            executable="ahrs_visualizer",
            name="ahrs_visualizer",
            output="screen",
            arguments=[
                "--config",
                LaunchConfiguration("config"),
            ],
        ),
    ])
