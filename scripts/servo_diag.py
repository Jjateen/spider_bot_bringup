#!/usr/bin/env python3
"""Servo diagnostic report — exercises the full PCA9685 path and prints a
per-channel pass/fail report.

Connects to the IMU TCP port (default 50008) on the UNO Q board, queries
the continuous health status, then triggers an active servo diagnostic that
writes test PWMs to all 12 channels, reads each back, and compares.

Usage:
    python3 scripts/servo_diag.py [--host HOST] [--port PORT]
"""

import argparse
import json
import socket
import sys
import time

PCA9685_CHANNELS = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
LEG_LABELS = ["FL0", "FL1", "FL2", "FR0", "FR1", "FR2",
              "HL0", "HL1", "HL2", "HR0", "HR1", "HR2"]


def send_recv(sock, cmd, timeout=5.0):
    sock.sendall((json.dumps(cmd) + "\n").encode())
    sock.settimeout(timeout)
    buf = b""
    while b"\n" not in buf:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            break
    line = buf.split(b"\n", 1)[0] if b"\n" in buf else buf
    if not line:
        return None
    return json.loads(line.decode())


def print_header(title):
    print()
    print(f"  {'=' * 50}")
    print(f"  {title}")
    print(f"  {'=' * 50}")


def print_status(status):
    print("  Continuous Health:")
    print(f"    PCA9685 present:     {'YES' if status.get('pca9685_ok') else 'NO'}")
    print(f"    MPU6050 present:     {'YES' if status.get('mpu6050_ok') else 'NO'}")
    print(f"    AI bit (MODE1):     {'OK' if status.get('ai_ok') else 'FAIL'}")
    print(f"    Servo calls:         {status.get('servo_calls', 0)}")
    print(f"    Write attempts:      {status.get('pwm_write_attempts', 0)}")
    print(f"    Write failures:      {status.get('pwm_write_fails', 0)}")
    last_ch = status.get('pwm_last_fail_ch', -1)
    last_code = status.get('pwm_last_fail_code', 0)
    code_str = {0: "OK", 1: "OVERFLOW", 2: "NACK_ADDR", 3: "NACK_DATA", 4: "OTHER"}
    if last_ch >= 0:
        print(f"    Last fail channel:   {last_ch}")
        print(f"    Last fail code:      {last_code} ({code_str.get(last_code, 'UNKNOWN')})")
    else:
        print(f"    Last fail:           none")
    print(f"    Bridge RPC last len: {status.get('set_servo_last_len', 0)} chars")
    print(f"    Bridge RPC last idx: {status.get('set_servo_last_idx', 0)} (12 = clean)")
    readback = status.get('pwm_readback_ch0', -1)
    if readback >= 0:
        print(f"    Ch0 readback:        {readback}")
    else:
        print(f"    Ch0 readback:        FAIL (I2C read error)")


def print_report(report):
    print()
    test_pwms = report.get("test_pwms", [])
    readback = report.get("readback", [])
    passed = report.get("pass", [])
    mode1 = report.get("mode1", -1)
    pca_present = report.get("pca_present", False)
    ai_ok = report.get("ai_ok", False)

    print("  Active Servo Test:")
    print(f"    PCA9685 on bus:        {'YES' if pca_present else 'NO'}")
    print(f"    MODE1 register:        0x{mode1:02x}" if mode1 >= 0 else "    MODE1 register:        READ FAILED")
    print(f"    Auto-Increment:        {'OK' if ai_ok else 'MISSING'}")
    print()
    print(f"    {'Ch':>3}  {'Label':>5}  {'Written':>7}  {'Readback':>8}  {'Delta':>5}  {'Status':>6}")
    print(f"    {'-' * 48}")

    total_pass = 0
    for i in range(12):
        w = test_pwms[i] if i < len(test_pwms) else -1
        r = readback[i] if i < len(readback) else -2
        p = passed[i] if i < len(passed) else 0
        ch = PCA9685_CHANNELS[i] if i < len(PCA9685_CHANNELS) else i
        label = LEG_LABELS[i] if i < len(LEG_LABELS) else f"S{i}"
        delta = r - w if r >= 0 and w >= 0 else -99
        status = "PASS" if p else ("FAIL" if r >= 0 else "NO_RD")
        if p:
            total_pass += 1
        print(f"    {ch:>3}  {label:>5}  {w:>7}  {r:>8}  {delta:>5}  {status:>6}")

    print(f"    {'-' * 48}")
    print(f"    Result: {total_pass}/12 PASS")
    return total_pass == 12


def main():
    parser = argparse.ArgumentParser(description="Servo diagnostic report")
    parser.add_argument("--host", default="127.0.0.1", help="UNO Q host (default 127.0.0.1)")
    parser.add_argument("--port", type=int, default=50008, help="IMU TCP port (default 50008)")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    try:
        sock.connect((args.host, args.port))
    except socket.error as e:
        print(f"ERROR: cannot connect to {args.host}:{args.port} — {e}")
        sys.exit(1)

    print(f"Connected to {args.host}:{args.port}")

    # 1. Query continuous health status
    print_header("Servo Diagnostic Report")
    status = send_recv(sock, {"cmd": "status"})
    if status and "error" not in status:
        print_status(status)
    else:
        print(f"  Status: {status}")

    # 2. Run active servo diagnostic
    print_header("Running Active Servo Test...")
    report = send_recv(sock, {"cmd": "servo_diag"}, timeout=10.0)
    if report and "error" not in report:
        all_pass = print_report(report)
        print()
        if all_pass:
            print("  VERDICT: ALL CHANNELS PASS — I2C writes and readbacks match.")
            print("  If servos still don't move, check power, OE pull-up, and servo wiring.")
        else:
            print("  VERDICT: SOME CHANNELS FAILED — see per-channel results above.")
            print("  Possible causes: I2C bus issue, wrong PCA9685 register map,")
            print("  or the PCA9685 is not properly initialized.")
    else:
        print(f"  Servo diag failed: {report}")
        sys.exit(1)

    sock.close()


if __name__ == "__main__":
    main()
