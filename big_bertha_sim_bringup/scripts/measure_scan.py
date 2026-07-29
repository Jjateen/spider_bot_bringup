#!/usr/bin/env python3
"""
Quantify lidar ghost walls: compare /scan against /scan_filtered.

scan_ground_filter republishes the *same* LaserScan with culled rays set to
inf, so header.stamp is preserved and raw/filtered pairs match exactly. That
lets us report not just "how many returns" but *which* rays the filter removed.

Reading the output: with the lidar only ~0.15 m above ground, a floor strike
lands at r = h/|dir_z|, i.e. roughly 1-4 m for a few degrees of gait tilt --
NOT sub-metre. So "is it culling floor hits?" cannot be judged from a single
near-range threshold. Judge it from the culled-range distribution (floor
strikes are near/mid, real distant walls are far) and from per-band survival:
near real geometry must survive, mid/far downhill rays are the ghost source.

Usage: measure_scan.py [duration_s]
"""

import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
BANDS = [(0.0, 1.5), (1.5, 2.5), (2.5, 4.0), (4.0, 8.1)]


def is_hit(x, rmax):
    """Return True if this range is a real finite return (not inf/NaN/oob)."""
    return not math.isnan(x) and x != float('inf') and x < rmax


class MeasureScan(Node):
    """Match raw/filtered scans by stamp and tally what the filter removed."""

    def __init__(self):
        """Subscribe to the raw and filtered scans."""
        super().__init__('measure_scan')
        self.raw = {}
        self.raw_finite = []
        self.filt_finite = []
        self.all_raw = []        # ranges of every raw hit in a matched pair
        self.culled = []         # ranges the filter removed
        self.create_subscription(
            LaserScan, '/scan', self.on_raw, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_filt, 10)

    def on_raw(self, m):
        """Cache the raw scan by stamp so the filtered copy can match it."""
        key = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        self.raw[key] = (list(m.ranges), m.range_max)
        if len(self.raw) > 200:
            for k in sorted(self.raw)[:100]:
                del self.raw[k]
        self.raw_finite.append(
            sum(1 for x in m.ranges if is_hit(x, m.range_max)))

    def on_filt(self, m):
        """Tally finite returns and diff against the matching raw scan."""
        rmax = m.range_max
        self.filt_finite.append(
            sum(1 for x in m.ranges if is_hit(x, rmax)))
        key = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        pair = self.raw.get(key)
        if pair is None:
            return
        praw, praw_max = pair
        for a, b in zip(praw, m.ranges):
            if not is_hit(a, praw_max):
                continue
            self.all_raw.append(a)
            if not is_hit(b, rmax):
                self.culled.append(a)


def mean(xs):
    """Mean, or NaN for an empty sequence."""
    return statistics.mean(xs) if xs else float('nan')


def main():
    """Collect for DUR seconds and print the ghost-wall report."""
    rclpy.init()
    node = MeasureScan()
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('=== GHOST-WALL SCAN REPORT ===')
    print(f'scans: raw={len(node.raw_finite)} filtered={len(node.filt_finite)}')
    print(f'finite returns/scan: raw={mean(node.raw_finite):6.1f}  '
          f'filtered={mean(node.filt_finite):6.1f}')
    print()
    print(f'rays culled (matched pairs): {len(node.culled)}')
    if node.culled:
        print(f'  culled range: mean={mean(node.culled):.2f} m  '
              f'min={min(node.culled):.2f}  max={max(node.culled):.2f}')
    print()
    print('survival by range band:')
    for lo, hi in BANDS:
        nraw = sum(1 for x in node.all_raw if lo <= x < hi)
        ncul = sum(1 for x in node.culled if lo <= x < hi)
        surv = (nraw - ncul) / nraw if nraw else float('nan')
        print(f'  {lo:4.1f}-{hi:4.1f} m : raw={nraw:6d} culled={ncul:6d} '
              f'survival={surv:6.1%}')
    print()
    print('Expect near-band (real walls) survival high; mid/far culled more '
          '(floor strikes).')
    print('Over-culling shows up as the near band dropping.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
