#!/usr/bin/env python3
"""
Throughput/FPS benchmark publisher.

Publishes sensor_msgs/Image at a configurable resolution, as fast as the
transport allows (no rate limiter), to find each transport's sustained
delivery ceiling -- the camera-FPS analogue. header.stamp set from
wall-clock for matching latency-under-load stats.

Usage: fps_pub.py [width=640] [height=480]
Run fps_sub.py in a second process with a matching RMW/CYCLONEDDS_URI env.
"""
import sys

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image


class FpsPub(Node):

    def __init__(self, width: int, height: int):
        super().__init__('fps_pub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.pub = self.create_publisher(Image, '/fps_test', qos)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.width = width
        self.height = height
        self.payload = bytes(width * height * 3)
        # Timer at the shortest period rclpy allows -- the transport itself
        # is the rate limiter we're measuring, not this loop.
        self.timer = self.create_timer(0.0001, self.tick)

    def tick(self):
        msg = Image()
        now = self.clock.now()
        msg.header.stamp.sec = now.seconds_nanoseconds()[0]
        msg.header.stamp.nanosec = now.seconds_nanoseconds()[1]
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.step = self.width * 3
        msg.data = self.payload
        self.pub.publish(msg)


def main():
    width = int(sys.argv[1]) if len(sys.argv) > 1 else 640
    height = int(sys.argv[2]) if len(sys.argv) > 2 else 480
    rclpy.init()
    node = FpsPub(width, height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
