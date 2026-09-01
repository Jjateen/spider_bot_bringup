#!/usr/bin/env python3
"""Publish static RViz markers for launch_isaac.py's obstacle scene.

The obstacles launch_isaac.py places (FixedCuboid/FixedCylinder, mirroring
worlds/obstacle_world.sdf shifted +3.5/+3.5 -- see that file's obstacle
loop) are pure PhysX collision geometry: Isaac's own viewport renders them,
but nothing publishes their shape to ROS. RViz then has no way to draw
them, so a correctly-detected real wall/box/pillar hit and an actual floor
ghost look visually identical in the raw LaserScan display -- both are just
a colored dot floating over an empty grid, with nothing to compare against.
This closes that gap: publish the same obstacle table as a MarkerArray so
RViz can render the room, and a human can actually tell real hits (points
that land on a drawn box/wall/pillar) from ghosts (points that don't) by
eye, instead of only being able to measure the distinction offline with
diagnose_ghost_geometry.py.

Frame is 'odom', not 'map': it exists from t=0 (the EKF publishes it
immediately; slam_toolbox's map->odom takes a few seconds), and Isaac's
/odom is PhysX ground truth anchored at the world-origin spawn point with
no drift, so 'odom' already is the world frame these obstacle coordinates
were placed in -- matching the same assumption diagnose_ghost_geometry.py
makes for its own reprojection.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'odom'

# Must match big_bertha_sim_bringup/scripts/isaac/launch_isaac.py's obstacle
# loop exactly -- both mirror worlds/obstacle_world.sdf shifted +3.5/+3.5.
BOXES = [
    # name, cx, cy, size_x, size_y, size_z
    ("wall_north", 3.5, 8.5, 10.2, 0.1, 0.5),
    ("wall_south", 3.5, -1.5, 10.2, 0.1, 0.5),
    ("wall_east", 8.5, 3.5, 0.1, 10.2, 0.5),
    ("wall_west", -1.5, 3.5, 0.1, 10.2, 0.5),
    ("box_1", 2.3, 2.3, 0.8, 0.8, 0.4),
    ("box_2", 4.5, 4.5, 0.9, 0.9, 0.4),
    ("box_3", 3.7, 1.7, 0.7, 0.7, 0.4),
]
CYLINDERS = [
    # name, cx, cy, radius, height
    ("pillar_1", 3.5, 3.5, 0.3, 0.5),
    ("pillar_2", 5.5, 3.0, 0.25, 0.5),
]


def build_markers(stamp):
    arr = MarkerArray()
    mid = 0
    for name, cx, cy, sx, sy, sz in BOXES:
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = stamp
        m.ns = 'obstacles'
        m.id = mid
        mid += 1
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = sz / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color.r, m.color.g, m.color.b, m.color.a = 0.6, 0.6, 0.65, 0.6
        arr.markers.append(m)
    for name, cx, cy, r, h in CYLINDERS:
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = stamp
        m.ns = 'obstacles'
        m.id = mid
        mid += 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = h / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = r * 2.0
        m.scale.y = r * 2.0
        m.scale.z = h
        m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.45, 0.2, 0.6
        arr.markers.append(m)
    return arr


class ObstacleMarkers(Node):
    def __init__(self):
        super().__init__('obstacle_markers')
        # use_sim_time: rclpy's Node base already declares this itself, no
        # need to (and doing so throws ParameterAlreadyDeclaredException).
        # transient_local so RViz picks these up even if it connects after
        # this node starts; still re-published on a timer since a fresh
        # scene each relaunch means stale retained markers from a prior run
        # aren't a concern here, and periodic republish is simpler than
        # tracking late-joining subscribers by hand.
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(MarkerArray, 'obstacle_markers', qos)
        self.create_timer(1.0, self.publish)

    def publish(self):
        self.pub.publish(build_markers(self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    node = ObstacleMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
