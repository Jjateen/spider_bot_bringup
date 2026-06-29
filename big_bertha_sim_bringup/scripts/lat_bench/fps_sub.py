#!/usr/bin/env python3
"""
Throughput/FPS benchmark subscriber.

Counts Image messages received over a fixed wall-clock duration, reports
achieved rate (the camera-FPS analogue), measured throughput (MB/s), and
per-message latency under this load.

Usage: fps_sub.py [duration_s=5.0] [label]
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image


class FpsSub(Node):

    def __init__(self, duration_s: float, label: str):
        super().__init__('fps_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.sub = self.create_subscription(Image, '/fps_test', self.on_msg, qos)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.duration_s = duration_s
        self.label = label
        self.count = 0
        self.bytes_total = 0
        self.latencies_us = []
        self.start_perf = None
        self.timer = self.create_timer(0.05, self.check_done)

    def on_msg(self, msg: Image):
        if self.start_perf is None:
            self.start_perf = time.perf_counter()
        now_ns = self.clock.now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        lat_us = (now_ns - stamp_ns) / 1000.0
        if 0 <= lat_us < 1_000_000:
            self.latencies_us.append(lat_us)
        self.count += 1
        self.bytes_total += len(msg.data)

    def check_done(self):
        if self.start_perf is None:
            return
        if (time.perf_counter() - self.start_perf) >= self.duration_s:
            self.report()
            raise SystemExit(0)

    def report(self):
        elapsed = time.perf_counter() - self.start_perf
        fps = self.count / elapsed
        mb_s = (self.bytes_total / 1_000_000) / elapsed
        arr = np.array(self.latencies_us) if self.latencies_us else np.array([0.0])
        print(f'=== {self.label} (n={self.count}, elapsed={elapsed:.3f}s) ===')
        print(f'achieved_fps={fps:.1f} throughput={mb_s:.1f}MB/s '
              f'latency_mean={arr.mean():.1f}us latency_p99={np.percentile(arr, 99):.1f}us '
              f'latency_max={arr.max():.1f}us')


def main():
    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    label = sys.argv[2] if len(sys.argv) > 2 else 'run'
    rclpy.init()
    node = FpsSub(duration_s, label)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        if node.count:
            node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
