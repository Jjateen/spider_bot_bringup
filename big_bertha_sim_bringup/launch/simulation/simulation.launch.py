#!/usr/bin/env python3
# Copyright 2026 Jjateen Gundesha
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Simulation bringup for Big Bertha in the obstacle world.

Starts the Gazebo Harmonic server (optionally headless), publishes the robot
description (robot_state_publisher from big_bertha_description), spawns the
robot into ``obstacle_world.sdf``, bridges the gz sensor/clock/odom topics to
ROS 2 via ``ros_gz_bridge``, and loads the ros2_control controllers
(joint_state_broadcaster + position_controller).

Launch arguments
----------------
gui            Run the Gazebo GUI client (default: ``false`` -> headless).
world          World SDF basename in this package's ``worlds/`` (default:
               ``obstacle_world.sdf``).
use_sim_time   Use ``/clock`` (default: ``true``).
x, y, z, yaw   Spawn pose; default is demo point A (-3.5, -3.5, 0.12).
spawn_controllers  Load the ros2_control spawners (default: ``true``).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the simulation launch description."""
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    desc_pkg = get_package_share_directory('big_bertha_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_controllers = LaunchConfiguration('spawn_controllers')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    world_path = PathJoinSubstitution([sim_pkg, 'worlds', world])
    bridge_config = os.path.join(sim_pkg, 'config', 'ros_gz_bridge.yaml')

    # Make the world's referenced meshes/models resolvable by gz.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(desc_pkg, '..'),
            os.path.join(sim_pkg, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]),
    )

    # robot_state_publisher (full URDF with gz plugins + odometry publisher).
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gz': 'true',
            'publish_odom': 'true',
        }.items(),
    )

    # Gazebo server (+ optional GUI). `-r` runs immediately, `-s` server-only.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v 2 ', world_path],
            'gz_version': '8',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-g -v 2',
            'gz_version': '8',
        }.items(),
        condition=IfCondition(gui),
    )

    # Spawn the robot from the /robot_description topic.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_big_bertha',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'big_bertha',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
    )

    # gz <-> ROS bridge (clock, scan, imu, odom, tf).
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    # ros2_control spawners (loaded by gz_ros2_control inside the sim).
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(spawn_controllers),
    )

    position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='position_controller_spawner',
        arguments=['position_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(spawn_controllers),
    )

    # Order the controllers after the robot is spawned (controller_manager
    # only exists once gz_ros2_control loads with the model).
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )
    load_position_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[position_spawner])
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Run the Gazebo GUI client'),
        DeclareLaunchArgument('world', default_value='obstacle_world.sdf',
                              description='World SDF basename in worlds/'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock time'),
        DeclareLaunchArgument('spawn_controllers', default_value='true',
                              description='Load ros2_control spawners'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-3.5'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='0.785'),

        set_resource_path,
        rsp,
        gz_server,
        gz_gui,
        bridge,
        spawn_robot,
        load_jsb_after_spawn,
        load_position_after_jsb,
    ])
