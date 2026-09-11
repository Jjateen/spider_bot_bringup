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
Isaac Sim bringup for Big Bertha -- the Isaac analog of
simulation/simulation.launch.py, same role: description + simulation, ready
for the locomotion module (big_bertha_policy_controller) to attach on top.

Isaac Sim itself is not a ROS 2 node -- it's a standalone Kit/Python process
(scripts/isaac/run_isaac_sim.sh) that publishes/subscribes ROS 2 topics from
inside via isaacsim.ros2.bridge. This launch file starts that process and,
separately, robot_state_publisher (the same big_bertha_description package
Gazebo uses) to publish the URDF TF tree from /joint_states -- something the
Isaac process itself does not do (it only publishes sensor/joint topics, not
the robot_description-driven kinematic tree).

Launch arguments
----------------
headless       Run Isaac Sim without the Kit GUI window. Default: ``true``.
use_sim_time   Use the ``/clock`` topic. Default: ``true``.
odom_tf        Isaac publishes odom->base_link tf (false: the EKF owns it
               instead, for the full SLAM bringup). Default: ``true``.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_isaac_sim(context, *args, **kwargs):
    """Build the Isaac Sim ExecuteProcess with args resolved from context.

    A plain ExecuteProcess can't branch its own cmd list on a
    LaunchConfiguration without either substitution gymnastics or an empty
    argv token argparse would reject -- OpaqueFunction resolves headless/
    odom_tf to real strings here instead, so the cmd list is just plain
    Python.
    """
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    isaac_script = os.path.join(sim_pkg, 'scripts', 'isaac', 'run_isaac_sim.sh')

    cmd = [isaac_script]
    if LaunchConfiguration('headless').perform(context) == 'true':
        cmd.append('--headless')
    if LaunchConfiguration('odom_tf').perform(context) != 'true':
        cmd.append('--no-odom-tf')

    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    """Build the Isaac Sim bringup launch description."""
    desc_pkg = get_package_share_directory('big_bertha_description')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # robot_state_publisher (plain URDF, no gz plugins -- same expansion the
    # Isaac URDF import itself uses via use_gz:=false). This is what's
    # missing on the Isaac path: it only publishes /joint_states, /scan,
    # /imu, /odom, not the URDF-driven TF tree (base_link -> lidar_link /
    # imu_link, both all-fixed-joint chains) that slam_toolbox and RViz need.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gz': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Isaac Sim without the Kit GUI window'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock time'),
        DeclareLaunchArgument(
            'odom_tf', default_value='true',
            description='Isaac publishes odom->base_link tf '
                        '(false: EKF owns it)'),

        rsp,
        OpaqueFunction(function=_launch_isaac_sim),
    ])
