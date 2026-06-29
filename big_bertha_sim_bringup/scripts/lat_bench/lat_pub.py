#!/usr/bin/env python3
"""Latency benchmark publisher: sensor_msgs/JointState sized like the real
/joint_states (12 joints, name+position+velocity), header.stamp set from
wall-clock (RCL_STEADY_TIME, independent of any /clock or sim time) so the
subscriber can compute one-way transport latency directly.

Usage: lat_pub.py [rate_hz=500]
Run lat_sub.py in a second process/terminal with a matching RMW/CYCLONEDDS_URI
env to measure one-way transport latency for that config.
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

JOINTS = [f"Revolute_{n}" for n in (110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121)]
N = 12


class LatPub(Node):
    def __init__(self, rate_hz: float):
        super().__init__('lat_pub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub = self.create_publisher(JointState, '/lat_test', qos)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

    def tick(self):
        msg = JointState()
        now = self.clock.now()
        msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        msg.name = JOINTS
        msg.position = [0.0] * N
        msg.velocity = [0.0] * N
        self.pub.publish(msg)


def main():
    rate_hz = float(sys.argv[1]) if len(sys.argv) > 1 else 500.0
    rclpy.init()
    node = LatPub(rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
