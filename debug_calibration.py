#!/usr/bin/env python3
"""
Calibration debug: find what servo angle/PWM gives the correct standing pose.

Run this on the UNO Q board (ssh jijteen@192.168.43.101).
Sends test PWMs to one ankle at a time via TCP port 50007.

Usage:
    # Test ankle index 8 at various PWMs
    python3 debug_calibration.py 8

    # Test all ankles at their computed 1.82 rad mapping
    python3 debug_calibration.py --compute
"""

import argparse
import json
import socket
import sys
import math

HOST = "127.0.0.1"
PORT = 50007

SERVO_LOWER = [45, 30, 180, 140, 135, 140, 50, 50, 40, 180, 150, 0]
SERVO_UPPER = [180, 150, 50, 0, 0, 0, 180, 180, 180, 40, 0, 150]
SERVO_OFFSET = [0, 0, 0, 0, 0, 0, 10, 10, 0, 8, 2, 5]
SERVO_DIR = [1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
PWM_MIN = 102
PWM_MAX = 512

def rad_to_pwm(rad, idx):
    deg = rad * 180.0 / math.pi
    deg = deg * SERVO_DIR[idx]
    deg = deg + SERVO_OFFSET[idx]
    deg = deg + 90.0
    lo = min(SERVO_LOWER[idx], SERVO_UPPER[idx])
    hi = max(SERVO_LOWER[idx], SERVO_UPPER[idx])
    clamped = max(lo, min(deg, hi))
    t = clamped / 180.0
    pwm = round(t * (PWM_MAX - PWM_MIN) + PWM_MIN)
    return deg, clamped, int(max(0, min(pwm, 4095)))

def send_pwms(pwms, label=""):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2)
    try:
        s.connect((HOST, PORT))
        payload = {"cmd": "servo", "pwms": pwms}
        msg = json.dumps(payload) + "\n"
        s.sendall(msg.encode())
        if label:
            print(f"  {label}: sent PWMS={pwms}")
    except Exception as e:
        print(f"  ERROR: {e}")
    finally:
        s.close()

NEUTRAL = (PWM_MIN + PWM_MAX) // 2  # 307

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--compute", action="store_true", help="show computed PWM for 1.82 rad on all ankles")
    parser.add_argument("index", nargs="?", type=int, help="ankle joint index (8-11) to test")
    parser.add_argument("--pwm", nargs="+", type=int, help="PWM values to test on the given index")
    args = parser.parse_args()

    if args.compute:
        print("Computed mapping: 1.82 rad (training default) -> servo angle -> PWM")
        print(f"{'idx':>3}  {'joint':>12}  {'dir':>3}  {'off':>3}  {'lo':>4}  {'hi':>4}  {'deg_raw':>7}  {'clamped':>7}  {'pwm':>4}")
        print("-" * 70)
        for i in range(8, 12):
            deg_raw, clamped, pwm = rad_to_pwm(1.82, i)
            print(f"{i:>3}  {['hip','knee','ankle'][min(i//4,2)]:>12}  {SERVO_DIR[i]:>3}  {SERVO_OFFSET[i]:>3}  {SERVO_LOWER[i]:>4}  {SERVO_UPPER[i]:>4}  {deg_raw:>7.1f}  {clamped:>7.1f}  {pwm:>4}")
            if abs(deg_raw - clamped) > 0.1:
                print(f"  *** CLAMPED! needle={deg_raw:.1f}° but range=[{SERVO_LOWER[i]},{SERVO_UPPER[i]}]")

    if args.index is not None:
        idx = args.index
        if idx < 0 or idx > 11:
            print("ERROR: index must be 0-11")
            sys.exit(1)

        if args.pwm:
            for p in args.pwm:
                pwms = [NEUTRAL] * 12
                pwms[idx] = p
                send_pwms(pwms, f"idx={idx} pwm={p}")
        else:
            print(f"Suggested: sweep ankle {idx} from PWM {PWM_MIN} to {PWM_MAX} step 50")
            for pwm_val in range(PWM_MIN, PWM_MAX + 1, 50):
                pwms = [NEUTRAL] * 12
                pwms[idx] = pwm_val
                send_pwms(pwms, f"idx={idx} pwm={pwm_val}")
