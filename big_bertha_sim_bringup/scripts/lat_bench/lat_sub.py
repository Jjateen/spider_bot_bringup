#!/usr/bin/env python3
"""Latency benchmark subscriber: computes one-way wall-clock latency
(now - header.stamp, both RCL_STEADY_TIME) for each received message, prints
min/mean/p50/p99/max/stddev in microseconds after N_SAMPLES, then exits.

Usage: lat_sub.py [n_samples=5000] [label]

Example A/B run (3 configs, see HANDOFF_BRINGUP.md for measured numbers):
  # Fast DDS (default RMW, no env needed)
  python3 lat_pub.py 500 &
  python3 lat_sub.py 5000 'Fast DDS'

  # Cyclone DDS, no SHM
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 lat_pub.py 500 &
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 lat_sub.py 5000 'Cyclone no-SHM'

  # Cyclone DDS + iceoryx SHM (RouDi must be running first: ../roudi.sh)
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
    CYCLONEDDS_URI="file://$(pwd)/../../config/dds/cyclonedds_shm.xml" \\
    python3 lat_pub.py 500 &
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \\
    CYCLONEDDS_URI="file://$(pwd)/../../config/dds/cyclonedds_shm.xml" \\
    python3 lat_sub.py 5000 'Cyclone + SHM'
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class LatSub(Node):
    def __init__(self, n_samples: int, label: str):
        super().__init__('lat_sub')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(JointState, '/lat_test', self.on_msg, qos)
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.n_samples = n_samples
        self.label = label
        self.latencies_us = []

    def on_msg(self, msg: JointState):
        now_ns = self.clock.now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        lat_us = (now_ns - stamp_ns) / 1000.0
        # Discard obviously bad samples (clock skew across process start, first message).
        if 0 <= lat_us < 1_000_000:
            self.latencies_us.append(lat_us)
        if len(self.latencies_us) >= self.n_samples:
            self.report()
            raise SystemExit(0)

    def report(self):
        arr = np.array(self.latencies_us)
        print(f"=== {self.label} (n={len(arr)}) ===")
        print(f"min={arr.min():.1f}us mean={arr.mean():.1f}us p50={np.percentile(arr, 50):.1f}us "
              f"p99={np.percentile(arr, 99):.1f}us max={arr.max():.1f}us std={arr.std():.1f}us")


def main():
    n_samples = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    label = sys.argv[2] if len(sys.argv) > 2 else 'run'
    rclpy.init()
    node = LatSub(n_samples, label)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        if node.latencies_us:
            node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
