#!/usr/bin/env python3
"""
Launch the AHRS 3D visualizer for Big Bertha.

Starts the Open3D-based visualizer that subscribes to IMU data
and displays robot orientation in real time.

If ``use_hardware:=true``, also launches the hardware bridge node and the
imu_filter_madgwick node so /filtered/imu is available standalone.

Usage::

    # With hardware (launches bridge + madgwick + RSP + visualizer)
    ros2 launch ahrs visualizer.launch.py use_hardware:=true

    # Without hardware (for development / testing with sim running)
    ros2 launch ahrs visualizer.launch.py

    # Custom config path
    ros2 launch ahrs visualizer.launch.py config:=/path/to/config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ahrs_pkg = get_package_share_directory("ahrs")
    bringup_pkg = get_package_share_directory("big_bertha_bringup")
    desc_pkg = get_package_share_directory("big_bertha_description")
    default_config = os.path.join(ahrs_pkg, "config", "config.yaml")

    use_hardware = LaunchConfiguration("use_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_jsp = LaunchConfiguration("publish_jsp")

    # ── Hardware support nodes (only when use_hardware:=true) ──────────
    # Launches the hardware bridge, the madgwick IMU filter, and the
    # robot state publisher so the visualizer has /filtered/imu,
    # /robot_description, and /joint_states.
    bridge_params = os.path.join(
        bringup_pkg, "config", "hardware_bridge.yaml")
    imu_filter_params = os.path.join(
        bringup_pkg, "config", "imu_filter_madgwick.yaml")

    hardware_nodes = GroupAction([
        Node(
            package="big_bertha_bringup",
            executable="hardware_bridge_node",
            name="hardware_bridge",
            output="screen",
            parameters=[bridge_params, {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_hardware),
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[imu_filter_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("imu/data_raw", "/imu"),
                ("imu/data", "/filtered/imu"),
            ],
            condition=IfCondition(use_hardware),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, "launch", "rsp.launch.py")),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_gz": "false",
                "publish_jsp": publish_jsp,
            }.items(),
            condition=IfCondition(use_hardware),
        ),
    ], scoped=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Path to YAML configuration file",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Launch hardware bridge + madgwick filter + RSP",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for the AHRS nodes",
        ),
        DeclareLaunchArgument(
            "publish_jsp",
            default_value="false",
            description="Publish joint states from joint_state_publisher "
                        "(set true when leg_odometry is not running)",
        ),

        LogInfo(
            msg=[
                "Starting AHRS Visualizer (config: ",
                LaunchConfiguration("config"),
                ")",
            ]
        ),

        hardware_nodes,

        Node(
            package="ahrs",
            executable="ahrs_visualizer",
            name="ahrs_visualizer",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "--config",
                LaunchConfiguration("config"),
            ],
        ),
    ])
