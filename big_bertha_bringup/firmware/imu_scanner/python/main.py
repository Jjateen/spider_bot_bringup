#!/usr/bin/env python3
# IMU Scanner — standalone diagnostic tool for the Big Bertha UNO Q.
#
# Probes the MPU6050 and PCA9685 on the M33's I2C bus via Bridge RPC.
# Run as an arduino-app-cli App:
#
#   arduino-app-cli app start ~/ArduinoApps/imu_scanner
#
# Press Ctrl+C to exit.

from arduino.app_utils import App, Bridge
import time

# ── State ─────────────────────────────────────────────────────────────────

last_imu = None
last_scan = None
last_pca = None
last_servo = None

# ── Bridge RPC handlers (called when the M33 pushes data) ────────────────

def on_imu_result(found, ax, ay, az, gx, gy, gz):
    global last_imu
    last_imu = {
        "found": bool(found),
        "ax": ax, "ay": ay, "az": az,
        "gx": gx, "gy": gy, "gz": gz,
    }


def on_bus_scan_result(addrs):
    global last_scan
    last_scan = list(addrs)


def on_pca9685_result(present, mode1, pre_scale, ai_ok):
    global last_pca
    last_pca = {
        "present": bool(present),
        "mode1": mode1,
        "pre_scale": pre_scale,
        "ai_ok": bool(ai_ok),
    }


def on_servo_test_result(ok, ch, pwm):
    global last_servo
    last_servo = {
        "ok": bool(ok),
        "ch": ch,
        "pwm": pwm,
    }


# ── Display helpers ──────────────────────────────────────────────────────

def clear():
    print("\033[2J\033[H", end="")


SAMPLE = 0


def print_imu():
    global SAMPLE
    SAMPLE += 1
    clear()
    print("IMU Scanner  —  Ctrl+C to exit")
    print("=" * 56)

    if last_imu is None:
        print("  Waiting for M33 response...")
        return

    if not last_imu["found"]:
        print("  MPU6050 at 0x68:  \033[91mNOT FOUND\033[0m")
        print()
        print("  Possible causes:")
        print("    • 3.3 V power missing or insufficient")
        print("    • GND not shared between UNO Q and IMU breakout")
        print("    • SDA / SCL ribbon cable disconnected or swapped")
        print("    • AD0 pin not pulled LOW (address should be 0x68)")
        print("    • I2C pull-up resistors missing (4.7 kΩ to 3.3 V)")
        print()
        print("  Run [b] from the menu to list all I2C devices.")
        local_time = time.strftime("%H:%M:%S")
        print(f"  Last checked:  {local_time}")
        return

    r = last_imu
    print(f"  MPU6050 at 0x68:  \033[92mFOUND\033[0m    sample #{SAMPLE}")
    print()
    print(f"    Accel  X:  {r['ax']:>8.3f}   Y:  {r['ay']:>8.3f}   Z:  {r['az']:>8.3f}   m/s²")
    print(f"    Gyro   X:  {r['gx']:>8.3f}   Y:  {r['gy']:>8.3f}   Z:  {r['gz']:>8.3f}   rad/s")
    print()
    g = (r["gx"] ** 2 + r["gy"] ** 2 + r["gz"] ** 2) ** 0.5
    a = (r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2) ** 0.5
    print(f"    Gyro magnitude:     {g:.4f}  rad/s  (should be near 0 when still)")
    print(f"    Accel magnitude:    {a:.4f}  m/s²  (should be near 9.81 when level)")


def print_bus_scan():
    if last_scan is None:
        print("  No scan results yet.")
        return
    if not last_scan:
        print("  \033[93mNo I2C devices found at all.\033[0m  Bus may be locked or pull-ups missing.")
        return
    print(f"  I2C devices found  ({len(last_scan)}):")
    names = {0x40: "PCA9685", 0x68: "MPU6050"}
    for addr in sorted(last_scan):
        label = names.get(addr, "")
        if label:
            print(f"    0x{addr:02X}  ←  {label}")
        else:
            print(f"    0x{addr:02X}")


def print_pca9685():
    if last_pca is None:
        print("  No PCA9685 data yet. Select [p] from the menu.")
        return

    if not last_pca["present"]:
        print("  PCA9685 at 0x40:  \033[91mNOT FOUND\033[0m")
        print()
        print("  Possible causes:")
        print("    • External 5V supply not connected or insufficient")
        print("    • J3-J6 ribbon cables disconnected")
        print("    • PCA9685 address jumpers changed (should be 0x40)")
        print("    • I2C bus fault (check with bus scan [b])")
        return

    r = last_pca
    print("  PCA9685 at 0x40:  \033[92mFOUND\033[0m")
    print(f"    MODE1 register:  0x{r['mode1']:02X}")
    print(f"    PRE_SCALE:       {r['pre_scale']}  (expect 121 for 50 Hz)")
    print(f"    Auto-increment:  {'\033[92mOK\033[0m' if r['ai_ok'] else '\033[91mFAIL\033[0m'}")
    print()
    if r["ai_ok"] and r["mode1"] & 0x10:
        print("    Note: SLEEP bit still set — chip not fully awake")
    if not r["ai_ok"]:
        print("    \033[93mAuto-increment disabled — multi-byte PWM writes will corrupt\033[0m")
    print()
    print("  Use [s] to test a servo channel (set PWM pulse).")


def print_servo_result():
    if last_servo is None:
        return
    r = last_servo
    if r["ok"]:
        print(f"  Servo CH{r['ch']}:  PWM={r['pwm']}  \033[92mOK\033[0m")
    else:
        print(f"  Servo CH{r['ch']}:  PWM={r['pwm']}  \033[91mFAILED\033[0m  (I2C error)")


# ── Main loop ────────────────────────────────────────────────────────────

MENU = """
  \033[1mIMU\033[0m
    [i]  Poll IMU (one shot)
    [r]  Continuous IMU refresh (1 Hz)

  \033[1mBus\033[0m
    [b]  Full I2C bus scan
    [p]  PCA9685 status check

  \033[1mServo test\033[0m
    [s]  Set PWM on a channel

  [q]  Quit
"""

MODE_CONT = False


def user_loop():
    global MODE_CONT

    if MODE_CONT:
        print_imu()
        Bridge.notify("read_imu")
        time.sleep(1.0)
        return

    clear()
    print("IMU Scanner  —  Ctrl+C to exit")
    print("=" * 56)
    print(MENU)

    try:
        cmd = input("  > ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print()
        raise

    if cmd == "i":
        Bridge.notify("read_imu")
        time.sleep(0.3)
        print_imu()
    elif cmd == "r":
        MODE_CONT = True
    elif cmd == "b":
        Bridge.notify("scan_bus")
        time.sleep(0.5)
        clear()
        print_bus_scan()
        input("\n  Press Enter to return to menu...")
    elif cmd == "p":
        Bridge.notify("check_pca9685")
        time.sleep(0.3)
        clear()
        print_pca9685()
        input("\n  Press Enter to return to menu...")
    elif cmd == "s":
        try:
            ch = int(input("  Channel (0-14): ").strip())
            pwm = int(input("  PWM value (0-4095): ").strip())
        except ValueError:
            print("  Invalid input")
            input("  Press Enter...")
            return
        ch = max(0, min(14, ch))
        pwm = max(0, min(4095, pwm))
        Bridge.notify("servo_test", ch, pwm)
        time.sleep(0.3)
        clear()
        print_servo_result()
        input("\n  Press Enter to return to menu...")
    elif cmd == "q":
        raise SystemExit(0)


def main():
    Bridge.provide("imu_result", on_imu_result)
    Bridge.provide("bus_scan_result", on_bus_scan_result)
    Bridge.provide("pca9685_result", on_pca9685_result)
    Bridge.provide("servo_test_result", on_servo_test_result)
    App.run(user_loop=user_loop)


if __name__ == "__main__":
    main()
