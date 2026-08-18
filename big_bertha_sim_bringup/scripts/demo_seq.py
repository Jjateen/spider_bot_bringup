#!/usr/bin/env python3
"""Replay the Isaac demo command sequence on /cmd_vel.

The training repo records its verification gif by driving the policy through a
fixed script (``DEMO_SEQ`` in ``scripts/rsl_rl/play_fixed_vel.py``): forward,
turn right 90, forward, reverse, turn left 180, stop. Reproducing the SAME
script in Gazebo is what makes the sim gif comparable to the Isaac one -- a
constant-forward clip and a scripted clip cannot be judged side by side.

Format matches the training script exactly: ``vx,vy,wz:steps`` segments joined
by ``;``, where steps are 50 Hz control periods. Keep the default in sync with
DEMO_SEQ; the turn durations there are derived from the measured yaw rate, so
editing one without the other silently desynchronises the two gifs.

Usage: demo_seq.py [--seq "..."] [--rate 50]
"""
import argparse

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# The Isaac script, verbatim from play_fixed_vel.py. Kept for reference: its
# turn durations are derived from Isaac's measured 0.4326 rad/s.
ISAAC_SEQ = (
    "0.30,0,0:150;"    # forward        3.00 s
    "0,0,-0.5:182;"    # turn right 90  3.64 s (negative wz = clockwise)
    "0.30,0,0:150;"    # forward        3.00 s
    "-0.15,0,0:125;"   # reverse        2.50 s
    "0,0,0.5:363;"     # turn left 180  7.26 s
    "0,0,0:135"        # stop           2.70 s
)

# Same manoeuvres, durations re-derived for the Gazebo plant, which yaws
# slower than Isaac (measured 0.29-0.34 rad/s against Isaac's 0.4326).
#
# Sized so each turn COMPLETES inside its own segment. With Isaac's counts the
# turn reached only part of the angle and heading_hold worked the residual off
# during the following forward segment (measured: -59 deg in the turn, another
# -28 deg while walking forward), so the robot kept rotating when the clip
# should show it walking straight. Re-measure with yaw_probe if the plant
# changes; the achieved angle, not the step count, is what has to match.
#
# Midpoint of the bracketing pair actually measured on recorded runs: 248/496
# undershot (-64, +138 deg) and 300/554 overshot (-113, +209). Per-segment
# angle varies by tens of degrees run to run because the heading wanders during
# the forward segments, so do not chase it. The stable quantity is the NET
# heading over the whole script, +75 to +95 deg against +90 commanded.
#
# Measure with the yaw log taken DURING the recorded run. An earlier log whose
# clock was not aligned to the segment transitions reported an undershoot that
# did not exist, and sizing against it overshot badly.
# Forward and reverse legs are far longer than Isaac's. Gazebo crawls at
# ~0.075 m/s against Isaac's ~0.146, so Isaac's 3 s legs move this plant barely
# 0.2 m; with the turns cancelling direction the script netted 0.05 m and the
# robot looked stuck on the spot. 10 s legs only reached 0.42 m, still too
# little to read as travel, hence the 20 s opening leg (~1.5 m).
#
# The second forward leg is capped at 8 s on purpose: after the right turn the
# robot heads south from the spawn at y=-3.5, and the arena wall is near -4.5.
DEFAULT_SEQ = (
    "0.30,0,0:1000;"   # forward       20.00 s  ~1.5 m
    "0,0,-0.5:275;"    # turn right 90  5.50 s
    "0.30,0,0:400;"    # forward        8.00 s  ~0.6 m
    "-0.15,0,0:400;"   # reverse        8.00 s  ~0.6 m back
    "0,0,0.5:525;"     # turn left 180 10.50 s
    "0,0,0:100"        # stop           2.00 s
)


def parse(seq):
    out = []
    for part in seq.split(";"):
        cmd, steps = part.split(":")
        vx, vy, wz = (float(v) for v in cmd.split(","))
        out.append((vx, vy, wz, int(steps)))
    return out


class SeqDriver(Node):
    def __init__(self, segments, rate):
        super().__init__("demo_seq")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.segments = segments
        self.i = 0
        self.done = False
        self.left = segments[0][3]
        total = sum(s[3] for s in segments)
        self.get_logger().info(
            "demo sequence: %d segments, %d steps, %.2f s at %g Hz"
            % (len(segments), total, total / rate, rate))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if self.i >= len(self.segments):
            self.pub.publish(Twist())      # leave the robot commanded to stop
            self.get_logger().info("demo sequence complete")
            self.done = True
            self.timer.cancel()
            return
        vx, vy, wz, _ = self.segments[self.i]
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
        self.pub.publish(m)
        self.left -= 1
        if self.left <= 0:
            self.i += 1
            if self.i < len(self.segments):
                self.left = self.segments[self.i][3]
                nvx, nvy, nwz, n = self.segments[self.i]
                self.get_logger().info(
                    "segment %d: vx=%.2f vy=%.2f wz=%.2f for %d steps"
                    % (self.i, nvx, nvy, nwz, n))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seq", default=DEFAULT_SEQ)
    p.add_argument("--rate", type=float, default=50.0)
    args, _ = p.parse_known_args()
    rclpy.init()
    n = SeqDriver(parse(args.seq), args.rate)
    # Spin until the script finishes rather than raising out of a timer
    # callback: SystemExit inside a callback escapes through the executor and
    # the node exits non-zero, which reads as a failed launch.
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
