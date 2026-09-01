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
One-shot Big Bertha sim bringup on Isaac Sim: chains every functional module.

The Isaac analog of bringup.launch.py -- same data-flow order and same
module launch files (state_estimation, mapping, localization, planning,
visualization are all reused unmodified), with ``simulation`` swapped for
``simulation/isaac_simulation.launch.py`` and the Gazebo-only args (``gui``,
``world``, ``sim_drive``) dropped, since Isaac has no SDF-world equivalent
yet -- the robot spawns on a bare ground plane at the world origin, not
inside ``obstacle_world.sdf``. Mapping in this configuration builds a map of
that empty plane; that's enough to verify the SLAM *plumbing* (TF tree,
topic wiring, lifecycle bringup) even though there's nothing to map yet.

Data-flow order (see PLAN.md sec 3, same as bringup.launch.py):

    description -> simulation -> locomotion -> state_estimation
                -> {mapping | localization} -> planning [-> rviz]

The Isaac simulation launch already includes ``description``
(robot_state_publisher), so it is the entry point. The EKF owns the
``odom -> base_link`` transform, so Isaac is started with its own odom tf
publisher disabled. The ``map -> odom`` transform comes from exactly one of
two sources, selected by the ``slam`` argument (default ``true`` here,
unlike bringup.launch.py's ``false`` -- there is no saved map for Isaac's
world yet, so known-map localization has nothing meaningful to localize
against).

Launch arguments
----------------
slam           ``true`` SLAM mode (mapping), ``false`` known-map (localization).
               Default: ``true``.
rviz           Also launch RViz with the integration view. Default: ``false``.
use_sim_time   Use the ``/clock`` topic. Default: ``true``.
headless       Run Isaac Sim without the Kit GUI window. Default: ``true``.
map            Saved map YAML for known-map mode. Default: the bundled
               ``maps/obstacle_world.yaml`` (Gazebo-specific -- only
               meaningful if you supply your own Isaac-world map).
x, y, yaw      Spawn pose, for slam_toolbox's map_start_pose / AMCL's seed
               only (Isaac itself does not yet support a configurable spawn
               pose -- the robot always lands at the world origin).
               Defaults: ``0 0 0`` (Isaac's actual spawn point, unlike
               bringup.launch.py's obstacle_world-specific ``-3.5 -3.5``).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the full Isaac Sim bringup launch description."""
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    launch_dir = os.path.join(sim_pkg, 'launch')

    slam = LaunchConfiguration('slam')
    rviz_enabled = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    map_yaml = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    default_map = PathJoinSubstitution(
        [FindPackageShare('big_bertha_sim_bringup'),
         'maps', 'obstacle_world.yaml'])

    def include(rel_path, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, rel_path)),
            launch_arguments=args.items(),
            condition=condition,
        )

    # description + simulation (rsp is included by the Isaac sim launch).
    # The EKF owns odom->base_link, so Isaac's own odom tf is disabled.
    simulation = include(
        os.path.join('simulation', 'isaac_simulation.launch.py'),
        {
            'use_sim_time': use_sim_time,
            'headless': headless,
            'odom_tf': 'false',
        },
    )

    # locomotion: the C++ ONNX gait controller (/cmd_vel -> joint targets).
    # Scoped so its ``params_file:=policy.yaml`` default stays local (see the
    # leak note in include()).
    locomotion = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    policy_pkg, 'launch', 'policy_controller.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ], scoped=True)

    # state_estimation: robot_localization EKF (fuses /odom + /imu).
    # ekf_isaac.yaml, not the Gazebo-facing default ekf.yaml: it additionally
    # fuses /odom's absolute yaw, safe here (and only here) because Isaac's
    # /odom orientation is PhysX ground truth, not integrated leg odometry --
    # see that file's own header comment for the full rationale.
    state_estimation = include(
        os.path.join('state_estimation', 'ekf.launch.py'),
        {'use_sim_time': use_sim_time,
         'ekf_config': os.path.join(sim_pkg, 'config', 'ekf_isaac.yaml')},
    )

    # map->odom: SLAM (mapping) OR known-map (localization), never both.
    # slam_toolbox_isaac.yaml, not the Gazebo-facing default: occupancy_threshold
    # raised (see that file's own header) because the residual, direction-
    # biased ghost rate that survives scan_ground_filter bakes a phantom wall
    # into /map over a sustained walk at the Gazebo-tuned 0.4 -- measured with
    # monitor_costmap_ghosts.py, not assumed.
    mapping = include(
        os.path.join('mapping', 'slam.launch.py'),
        {'use_sim_time': use_sim_time, 'x': x, 'y': y, 'yaw': yaw,
         'slam_config': os.path.join(sim_pkg, 'config', 'slam_toolbox_isaac.yaml')},
        condition=IfCondition(slam),
    )
    localization = include(
        os.path.join('localization', 'localization.launch.py'),
        {'use_sim_time': use_sim_time, 'map': map_yaml,
         'localization': LaunchConfiguration('localization'),
         'x': x, 'y': y, 'yaw': yaw},
        condition=UnlessCondition(slam),
    )

    # perception: IMU-gated ghost-wall filter. floor_margin raised from the
    # Gazebo-tuned 0.04 default: measured (measure_scan.py + a geometric
    # reprojection of surviving points against the known obstacle layout)
    # ~11% of surviving points are unexplained by real geometry, concentrated
    # at 1-3 m range in a ~45deg front-left sector -- not self-hits (zero at
    # <1m) or random noise, but this gait's asymmetric leg-phase offsets
    # producing a persistent lean bias while walking straight, which the
    # single-IMU-sample-per-scan design averages out but doesn't fully
    # correct for on the more-tilted side. Not touching the C++ default
    # (scan_ground_filter.cpp:61) since it's shared with Gazebo, which is
    # already at ~95% near-band survival and shouldn't be pushed to over-cull
    # real geometry to fix an Isaac-specific lean magnitude. 0.07 cut the
    # unexplained fraction from 10.8% to 4.6%; pushed further to 0.10 next
    # and it did NOT confirm further improvement (6.9%, worse) in that trial
    # -- but the two runs walked visibly different paths through the room
    # (this sim has real run-to-run path variance), so treat that as
    # inconclusive rather than a proven regression. Settled on 0.07 as the
    # one value with a clean, repeatable win; re-verify with
    # scripts/measure_scan.py plus a geometric reprojection against the
    # known obstacle layout (not just measure_scan.py's raw band %, which
    # mixes real and ghost hits and isn't reliable across different walked
    # paths) before pushing this further.
    scan_filter = Node(
        package='big_bertha_sim_bringup',
        executable='scan_ground_filter',
        name='scan_ground_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'floor_margin': 0.07}],
    )

    # obstacle_markers: RViz has no other way to render the scene's static
    # obstacles (FixedCuboid/FixedCylinder are pure PhysX collision geometry,
    # nothing publishes their shape to ROS) -- without this, a correctly-
    # detected real wall/box/pillar hit and an actual floor ghost look
    # identical in the raw LaserScan display, since there's nothing to
    # visually compare a scan point against. See
    # scripts/isaac/publish_obstacle_markers.py's own header for the full
    # rationale; add its MarkerArray display to whichever RViz config you
    # use if it isn't already there.
    obstacle_markers = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([
                FindPackageShare('big_bertha_sim_bringup'),
                'scripts', 'isaac', 'publish_obstacle_markers.py']),
            '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen',
    )

    # planning: Nav2 servers (planner/controller/costmaps/BT).
    planning = include(
        os.path.join('planning', 'nav2.launch.py'),
        {'use_sim_time': use_sim_time,
         'nav_speed': LaunchConfiguration('nav_speed')},
    )

    # Optional RViz view.
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'visualization', 'rviz.launch.py')),
        launch_arguments={
            'config': rviz_config,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true: SLAM (mapping); false: known-map (localization)'),
        DeclareLaunchArgument(
            'localization', default_value='ground_truth',
            description="known-map map->odom provider: 'ground_truth' "
                        "(static identity) or 'amcl' (scan-match)"),
        DeclareLaunchArgument(
            'nav_speed', default_value='0.29',
            description='Nav2 FollowPath desired_linear_vel (m/s)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also launch RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value='integration',
            description='RViz config in config/rviz/ (simulation|mapping|'
                        'planning|integration)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock time'),
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Isaac Sim without the Kit GUI window'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Saved map YAML for known-map mode (Gazebo-specific '
                        'default -- supply your own for an Isaac world)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Only used to seed slam_toolbox/AMCL -- Isaac '
                        'itself always spawns at the world origin'),

        simulation,
        locomotion,
        state_estimation,
        scan_filter,
        obstacle_markers,
        mapping,
        localization,
        planning,
        rviz,
    ])
