#!/usr/bin/env python3
"""Servo diagnostic report — exercises the hardware bridge via ROS 2 services.

The C++ hardware_bridge_node talks directly to the arduino-router socket and
exposes firmware diagnostics as ROS 2 services:

    /hardware_bridge/status      last hw_status (I2C/PCA9685/BNO055 IMU health)
    /hardware_bridge/scan_i2c    trigger an I2C bus scan
    /hardware_bridge/servo_diag  run the on-MCU servo write/readback test
    /hardware_bridge/imu_diag    last IMU identity/magnetometer diagnostic

Usage:
    ros2 run big_bertha_bringup hardware_bridge_node   # in another shell
    python3 scripts/servo_diag.py [--node hardware_bridge]
"""

import argparse
import subprocess
import sys

SERVICE_PREFIX = "/hardware_bridge"


def call_service(service: str) -> str:
    """Call a std_srvs/Trigger service and return its response message."""
    full = f"{SERVICE_PREFIX}{service}"
    try:
        result = subprocess.run(
            ["ros2", "service", "call", full, "std_srvs/srv/Trigger"],
            capture_output=True, text=True, timeout=30,
        )
    except FileNotFoundError:
        print("ERROR: ros2 CLI not found — is ROS sourced?")
        sys.exit(1)
    except subprocess.TimeoutExpired:
        return f"TIMEOUT waiting for {full}"
    out = result.stdout + result.stderr
    return out.strip() or f"no response from {full}"


def print_header(title):
    print()
    print(f"  {'=' * 50}")
    print(f"  {title}")
    print(f"  {'=' * 50}")


def main():
    parser = argparse.ArgumentParser(description="Servo diagnostic report via ROS 2 services")
    parser.add_argument("--node", default="hardware_bridge",
                        help="hardware bridge node name (default hardware_bridge)")
    args = parser.parse_args()

    global SERVICE_PREFIX
    SERVICE_PREFIX = f"/{args.node}"

    print_header("Servo Diagnostic Report")

    print_header("Continuous Health")
    print("  " + call_service("/status").replace("\n", "\n  "))

    print_header("Running Active Servo Test...")
    print("  " + call_service("/servo_diag").replace("\n", "\n  "))

    print_header("I2C Scan")
    print("  " + call_service("/scan_i2c").replace("\n", "\n  "))

    print_header("IMU Diagnostics")
    print("  " + call_service("/imu_diag").replace("\n", "\n  "))


if __name__ == "__main__":
    main()
