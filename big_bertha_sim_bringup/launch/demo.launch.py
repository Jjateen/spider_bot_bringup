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
One-command A->B demo with RViz (known-map mode by default).

Brings up the full stack and auto-sends a NavigateToPose goal from the pre-fed
point A (spawn) to point B, with the combined RViz view (map, lidar /scan,
global + local costmaps, planned path, robot).

Localization mode
-----------------
By default the demo runs in KNOWN-MAP mode (``slam:=false``): map_server serves
the saved ``maps/obstacle_world.yaml`` (which covers the whole arena) and AMCL
localizes against it. The global costmap is therefore fully populated from the
static map, so a goal at world B is plannable and the A->B run is repeatable.

Pass ``slam:=true`` to run live SLAM (slam_toolbox) instead. In SLAM mode the
map only covers the area explored so far, so a goal far outside it may be
unplannable until the robot drives there; the NavfnPlanner is configured with
``allow_unknown: true`` so it at least degrades gracefully.

Coordinates
-----------
In known-map mode the saved map is world-axis-aligned, so the ``map`` frame
matches the Gazebo world. The robot spawns at world A = (-3.5, -3.5) facing
+x (East) and the goal is world B = (3.5, -3.5) -- the default below: a
straight East traverse of the obstacle-free bottom corridor (all boxes/pillars
sit at y >= -1.8). The diagonal toward (3.5, 3.5) is walled off by
box_1/pillar_1/box_2 and would need hard turns the gait cannot make.

    ros2 launch big_bertha_sim_bringup demo.launch.py
    ros2 launch big_bertha_sim_bringup demo.launch.py slam:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Full-stack known-map (or SLAM) + RViz + automatic A->B navigation."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')

    slam = LaunchConfiguration('slam')
    localization = LaunchConfiguration('localization')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_delay = LaunchConfiguration('goal_delay')
    patrol = LaunchConfiguration('patrol')
    goal2_x = LaunchConfiguration('goal2_x')
    goal2_y = LaunchConfiguration('goal2_y')
    goal3_x = LaunchConfiguration('goal3_x')
    goal3_y = LaunchConfiguration('goal3_y')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'slam': slam,              # default false: known-map mode
            # ground_truth default: AMCL is ambiguous in the symmetric arena
            # (wrong pose -> false goal success).
            'localization': localization,
            'rviz': LaunchConfiguration('rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_sim_time': 'true',
        }.items(),
    )

    # Single-goal mode (patrol:=false, default): one NavigateToPose to B.
    goal_yaml = [
        '{pose: {header: {frame_id: map}, pose: {position: {x: ',
        goal_x, ', y: ', goal_y,
        ', z: 0.0}, orientation: {w: 1.0}}}}',
    ]
    send_goal = TimerAction(
        period=goal_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose', goal_yaml,
                ],
                output='screen',
            ),
        ],
        condition=UnlessCondition(patrol),
    )

    # Patrol mode: 3-goal tour A -> NE -> NW -> A, each goal blocking until
    # reached. The A->NE leg crosses the obstacle diagonal on purpose -- the
    # patrol exists to exercise planning AROUND obstacles, not corridor
    # walking (the single-goal demo covers that). Deliberately does NOT reuse
    # the single-goal B: that would either break the tour or turn it into an
    # obstacle-free perimeter walk.
    patrol_script = os.path.join(pkg, 'scripts', 'send_patrol.sh')
    send_patrol = TimerAction(
        period=goal_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', patrol_script,
                    [goal2_x, ',', goal2_y],    # NE, through the obstacle field
                    [goal3_x, ',', goal3_y],    # NW
                    [start_x, ',', start_y],    # A (back to the start)
                ],
                output='screen',
            ),
        ],
        condition=IfCondition(patrol),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: live SLAM; false: known-map for the repeatable '
                        'A->B demo (default)'),
        DeclareLaunchArgument(
            'localization', default_value='ground_truth',
            description='known-map map->odom provider: ground_truth (static '
                        'identity, honest pose, the A->B default) or amcl '
                        '(scan-match, ambiguous in the symmetric arena)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz. Set false for headless GIF capture '
                        '(a clean-env RViz is recorded separately).'),
        DeclareLaunchArgument(
            'rviz_config', default_value='integration',
            description='RViz config basename in config/rviz/. integration '
                        '(whole-arena top-down) or patrol (close follow cam '
                        'to see the gait while navigating).'),
        DeclareLaunchArgument(
            'goal_x', default_value='3.5',
            description='Goal B x (map frame, world-aligned in known-map). '
                        'A=(-3.5,-3.5) facing +x; B=(3.5,-3.5) is a straight '
                        'East traverse of the clear bottom corridor.'),
        DeclareLaunchArgument(
            'goal_y', default_value='-3.5',
            description='Goal B y in the map frame. Default matches the '
                        'documented B=(3.5,-3.5); the old 3.5 default sent '
                        'the single-goal demo into the walled-off diagonal.'),
        DeclareLaunchArgument(
            'goal_delay', default_value='20.0',
            description='Seconds to wait for Nav2 before sending the goal '
                        '(45 let the idle gait slide into the SW corner).'),
        DeclareLaunchArgument(
            'patrol', default_value='false',
            description='true: 3-goal patrol A->NE->NW->A whose first leg '
                        'crosses the obstacle field (plans around boxes/'
                        'pillars); false: the single A->B goal (default).'),
        DeclareLaunchArgument(
            'goal2_x', default_value='3.5',
            description='Patrol 1st goal x (map frame; default NE corner, '
                        'reached through the obstacle diagonal).'),
        DeclareLaunchArgument(
            'goal2_y', default_value='3.5',
            description='Patrol 1st goal y (map frame; default NE corner).'),
        DeclareLaunchArgument(
            'goal3_x', default_value='-3.5',
            description='Patrol 2nd goal x (map frame; default NW corner).'),
        DeclareLaunchArgument(
            'goal3_y', default_value='3.5',
            description='Patrol 2nd goal y (map frame; default NW corner).'),
        DeclareLaunchArgument(
            'start_x', default_value='-3.5',
            description='Patrol return point x (the spawn/start; world A).'),
        DeclareLaunchArgument(
            'start_y', default_value='-3.5',
            description='Patrol return point y (the spawn/start; world A).'),
        bringup,
        send_goal,
        send_patrol,
    ])
