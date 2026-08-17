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
Frontier exploration: drive the map boundary until the arena is covered.

Replaces the old fixed 4-corner tour, which was a patrol with mapping
switched on -- it visited preset coordinates whether or not they were
informative, and a single aborted goal stalled the whole run.

Each round: find free cells that touch unknown space (the frontier), cluster
them, drive to the nearest cluster, repeat. Nav2 owns obstacle avoidance.
When no cluster larger than --min-frontier remains the map is complete, and
the robot returns to where it started.

A goal that aborts is blacklisted and the next cluster is tried, so a single
planner failure costs one goal instead of the run.

Usage: frontier_explore.py [--budget S] [--min-frontier CELLS] [--goal-timeout S]
"""

import argparse
import math
import sys
import time

# frontier_lib sits next to this script (sys.path[0] when run directly); the
# frontier maths lives there ROS-free so it can be unit-tested without ROS.
from frontier_lib import find_frontiers
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros


class FrontierExplorer(Node):
    """Drives Nav2 to successive frontiers until the map stops growing."""

    def __init__(self, args):
        super().__init__('frontier_explore')
        self.args = args
        self.map = None
        self.blacklist = []
        # slam_toolbox latches /map; without transient_local we would wait
        # for the next periodic publish instead of getting the current map.
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _on_map(self, msg):
        self.map = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def cell_to_world(self, index):
        info = self.map.info
        x = info.origin.position.x + (index % info.width + 0.5) * info.resolution
        y = info.origin.position.y + (index // info.width + 0.5) * info.resolution
        return x, y

    def pick_goal(self, robot):
        """Nearest non-blacklisted frontier cluster, or None when done."""
        clear_cells = int(round(self.args.goal_clearance / self.map.info.resolution))
        clusters = find_frontiers(
            self.map.data, self.map.info.width, self.map.info.height,
            self.args.min_frontier, clear_cells)
        candidates = []
        for rep, size in clusters:
            x, y = self.cell_to_world(rep)
            if any(math.hypot(x - bx, y - by) < self.args.blacklist_radius
                   for bx, by in self.blacklist):
                continue
            candidates.append((math.hypot(x - robot[0], y - robot[1]), x, y, size))
        if not candidates:
            return None
        return min(candidates)

    def goto(self, x, y, from_xy, label):
        """Send one NavigateToPose. Returns True only on SUCCEEDED."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.atan2(y - from_xy[1], x - from_xy[0])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            print(f'=== {label} ({x:.2f}, {y:.2f}) REJECTED ===', flush=True)
            return False
        result = handle.get_result_async()
        deadline = time.time() + self.args.goal_timeout
        # Stuck watchdog: the controller stops dead when it predicts a
        # collision, then loops abort -> clear costmap -> replan -> stop. Left
        # alone that burns the whole goal timeout in one spot, which is what
        # makes the demo look frozen. Give up as soon as the robot stops
        # closing on the goal instead of waiting out the clock.
        best = None
        best_at = time.time()
        while rclpy.ok() and not result.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            here = self.robot_xy()
            if here:
                dist = math.hypot(x - here[0], y - here[1])
                if best is None or dist < best - self.args.progress_dist:
                    best, best_at = dist, time.time()
                elif time.time() - best_at > self.args.stuck_timeout:
                    handle.cancel_goal_async()
                    print(f'=== {label} ({x:.2f}, {y:.2f}) STUCK '
                          f'({self.args.stuck_timeout:.0f}s without progress) ===',
                          flush=True)
                    return False
            if time.time() > deadline:
                handle.cancel_goal_async()
                print(f'=== {label} ({x:.2f}, {y:.2f}) TIMEOUT ===', flush=True)
                return False
        ok = result.result() is not None and result.result().status == 4  # SUCCEEDED
        print(f'=== {label} ({x:.2f}, {y:.2f}) '
              f'{"SUCCEEDED" if ok else "ABORTED"} ===', flush=True)
        return ok

    def run(self):
        print('=== waiting for map + nav2 ===', flush=True)
        if not self.nav.wait_for_server(timeout_sec=60.0):
            print('=== FAILED: navigate_to_pose action server never came up ===',
                  flush=True)
            return 1
        while rclpy.ok() and self.map is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        start = None
        while rclpy.ok() and start is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            start = self.robot_xy()
        print(f'=== explore start at ({start[0]:.2f}, {start[1]:.2f}) ===',
              flush=True)

        deadline = time.time() + self.args.budget
        visited = 0
        complete = False
        while rclpy.ok() and time.time() < deadline:
            self.spin_for(2.0)  # let /map catch up with the last leg
            robot = self.robot_xy() or start
            pick = self.pick_goal(robot)
            if pick is None:
                print('=== map complete: no frontiers left ===', flush=True)
                complete = True
                break
            _, x, y, size = pick
            if self.goto(x, y, robot, f'frontier ({size} cells) ->'):
                visited += 1
            else:
                self.blacklist.append((x, y))
        else:
            print('=== budget spent; returning with the map so far ===', flush=True)

        robot = self.robot_xy() or start
        print('=== returning to start ===', flush=True)
        returned = self.goto(start[0], start[1], robot, 'return to start ->')
        # Say WHICH ending this was. The old line printed the same "explore
        # complete" whether the map was finished or the budget simply ran out,
        # so a truncated map read as a successful run.
        print(f'=== explore {"complete" if complete else "TRUNCATED (budget spent)"}: '
              f'{visited} frontiers visited, '
              f'returned={"yes" if returned else "no"} ===', flush=True)
        return 0 if complete else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    # 600 s was sized when the plant was faster. Big Bertha crawls the arena at
    # ~0.075 m/s, so 600 s ran out with 13% of the interior still unknown and
    # the run still reported success, which is how a truncated map reached the
    # demo gif. 1800 s covers the 8x8 m arena with margin; exploration exits as
    # soon as no frontier is left, so a finished map still stops early.
    parser.add_argument('--budget', type=float, default=1800.0,
                        help='wall-clock seconds before giving up (default 1800)')
    parser.add_argument('--min-frontier', type=int, default=12,
                        help='ignore clusters smaller than this many cells')
    parser.add_argument('--goal-timeout', type=float, default=90.0,
                        help='hard cap on one frontier goal (s)')
    parser.add_argument('--stuck-timeout', type=float, default=25.0,
                        help='abandon a goal after this long without progress')
    parser.add_argument('--progress-dist', type=float, default=0.15,
                        help='metres of closing that counts as progress')
    parser.add_argument('--goal-clearance', type=float, default=0.25,
                        help='required free radius (m) around a frontier goal; '
                             'robot_radius is 0.18 and the costmap inflates to '
                             '0.38, so a goal tight against a wall cannot be '
                             'driven to')
    parser.add_argument('--blacklist-radius', type=float, default=0.6,
                        help='skip frontiers within this radius of a failed one')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FrontierExplorer(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
