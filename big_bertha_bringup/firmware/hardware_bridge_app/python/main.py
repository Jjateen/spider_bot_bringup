#!/usr/bin/env python3
# Big Bertha Hardware Bridge — MPU-side relay
#
# Relays commands between the ROS 2 C++ node (TCP socket) and the STM32U585
# sketch (Bridge RPC). Runs as the Python component of an arduino-app-cli App.
#
# Architecture: notification-based (no Bridge.call from Python).
#   servos:   Bridge.notify("set_servo_pwms", pwms)         — one-way to MCU
#   imu:      MCU sends Bridge.notify("imu", ...) @ 30 Hz    — cached here
#   status:   MCU sends Bridge.notify("hw_status", ...) @ 1 Hz — cached here
#   i2c scan: MCU sends Bridge.notify("i2c_scan", ...) on request (notify)

from arduino.app_utils import App, Bridge
import json
import socket
import threading
import time

TCP_HOST = "0.0.0.0"

# Store the latest data from the robot's sensors
cache = {
    "imu": None,         # most recent IMU reading
    "hw_status": None,   # most recent hardware health check
    "i2c_scan": None,    # None = never scanned, [] = scanned but empty
    "servo_pwms": None,  # latest servo PWM targets, flushed from main thread
    "servo_diag": None,  # latest servo diagnostic report
}
cache_lock = threading.Lock()   # only one thread reads or writes the cache at a time
scan_count = 0                   # incremented each time on_i2c_scan is called


SERVO_PORT = 50007
IMU_PORT = 50008


def handle_servo_client(conn):
    # Handles ONLY servo commands and ping — runs on port 50007.
    # IMU and status requests go to port 50008.
    buf = b""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    req = json.loads(line.decode())
                except json.JSONDecodeError as e:
                    conn.sendall(json.dumps({"error": str(e)}).encode() + b"\n")
                    continue

                cmd = req.get("cmd")

                if cmd == "ping":
                    conn.sendall(b'{"ok":true}\n')

                elif cmd == "servo":
                    pwms = req.get("pwms")
                    if not isinstance(pwms, list) or len(pwms) != 12:
                        conn.sendall(json.dumps({"error": "pwms must be list of 12 ints"}).encode() + b"\n")
                        continue
                    if not all(isinstance(p, int) and 0 <= p <= 4095 for p in pwms):
                        conn.sendall(json.dumps({"error": "each pwm must be int 0-4095"}).encode() + b"\n")
                        continue
                    with cache_lock:
                        cache["servo_pwms"] = pwms
                    conn.sendall(b'{"ok":true}\n')

                else:
                    conn.sendall(
                        json.dumps({"error": "unknown cmd"}).encode() + b"\n"
                    )
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()


def handle_imu_client(conn):
    # Handles ONLY IMU, status, and scan commands — runs on port 50008.
    buf = b""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    req = json.loads(line.decode())
                except json.JSONDecodeError as e:
                    conn.sendall(json.dumps({"error": str(e)}).encode() + b"\n")
                    continue

                cmd = req.get("cmd")

                if cmd == "ping":
                    conn.sendall(b'{"ok":true}\n')

                elif cmd == "imu":
                    with cache_lock:
                        imu = cache["imu"]
                    if imu is not None:
                        conn.sendall(json.dumps(imu).encode() + b"\n")
                    else:
                        conn.sendall(json.dumps({"error": "no imu data yet"}).encode() + b"\n")

                elif cmd == "status":
                    with cache_lock:
                        hw = cache["hw_status"]
                    if hw is not None:
                        conn.sendall(json.dumps(hw).encode() + b"\n")
                    else:
                        conn.sendall(json.dumps({"error": "no status yet"}).encode() + b"\n")

                elif cmd == "scan_i2c":
                    Bridge.notify("scan_i2c")
                    time.sleep(0.5)
                    with cache_lock:
                        val = cache["i2c_scan"]
                    if val is None:
                        conn.sendall(
                            json.dumps({"error": "no scan data yet"}).encode() + b"\n"
                        )
                    else:
                        scan = {"addrs": sorted(val)}
                        conn.sendall(json.dumps(scan).encode() + b"\n")

                elif cmd == "servo_diag":
                    cache["servo_diag"] = None
                    Bridge.notify("servo_diag")
                    for _ in range(50):
                        time.sleep(0.1)
                        with cache_lock:
                            diag = cache["servo_diag"]
                        if diag is not None:
                            conn.sendall(json.dumps(diag).encode() + b"\n")
                            break
                    else:
                        conn.sendall(
                            json.dumps({"error": "no servo diag result after 5s"}).encode() + b"\n"
                        )

                else:
                    conn.sendall(
                        json.dumps({"error": "unknown cmd"}).encode() + b"\n"
                    )
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()


def tcp_servo_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((TCP_HOST, SERVO_PORT))
    sock.listen()
    sock.settimeout(1.0)

    while True:
        try:
            conn, addr = sock.accept()
            threading.Thread(
                target=handle_servo_client, args=(conn,), daemon=True
            ).start()
        except socket.timeout:
            continue


def tcp_imu_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((TCP_HOST, IMU_PORT))
    sock.listen()
    sock.settimeout(1.0)

    while True:
        try:
            conn, addr = sock.accept()
            threading.Thread(
                target=handle_imu_client, args=(conn,), daemon=True
            ).start()
        except socket.timeout:
            continue


# ── Bridge.notify handlers (called when STM32 pushes data) ──────────────────

def on_imu(ax, ay, az, gx, gy, gz, sample_id=None, timestamp=None):
    if all(v == 0.0 for v in (ax, ay, az, gx, gy, gz)):
        print("[bridge] WARNING: IMU reading all zeros — sensor may be missing")
    with cache_lock:
        cache["imu"] = {
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "lin_acc_x": ax, "lin_acc_y": ay, "lin_acc_z": az,
        }


def on_hw_status(scan, ai_ok, servo_calls=0, ping_count=0,
                 pwm_attempts=0, pwm_fails=0, pwm_last_fail_ch=-1,
                 pwm_last_fail_code=0, set_servo_last_len=0, set_servo_last_idx=0,
                 pwm_readback_ch0=-1):
    with cache_lock:
        cache["hw_status"] = {
            "i2c_scan": scan,
            "ai_ok": bool(ai_ok),
            "pca9685_ok": (scan & 1) == 0,
            "mpu9250_ok": (scan & 2) == 0,
            "servo_calls": servo_calls,
            "ping_count": ping_count,
            "pwm_write_attempts": pwm_attempts,
            "pwm_write_fails": pwm_fails,
            "pwm_last_fail_ch": pwm_last_fail_ch,
            "pwm_last_fail_code": pwm_last_fail_code,
            "set_servo_last_len": set_servo_last_len,
            "set_servo_last_idx": set_servo_last_idx,
            "pwm_readback_ch0": pwm_readback_ch0,
        }


def on_i2c_scan(addrs):
    # STM32 finished scanning the I2C bus — save the list of devices found
    global scan_count
    scan_count += 1
    with cache_lock:
        cache["i2c_scan"] = list(addrs)


def on_servo_diag_result(report_str):
    # STM32 returned a servo diagnostic report (JSON string)
    try:
        report = json.loads(report_str)
    except json.JSONDecodeError:
        report = {"error": "invalid JSON from MCU", "raw": report_str}
    with cache_lock:
        cache["servo_diag"] = report


last_pwms = None   # track last sent PWMs to skip duplicates
notify_errs = 0     # count consecutive Bridge.notify failures

def loop():
    global last_pwms, notify_errs
    last_log = 0
    while True:
        # Flush servo PWMs to the STM32 from the main thread
        with cache_lock:
            pwms = cache["servo_pwms"]
            cache["servo_pwms"] = None

        if pwms is not None:
            # Only send if values changed significantly (avoid flooding IPC)
            changed = last_pwms is None or any(
                abs(a - b) > 2 for a, b in zip(pwms, last_pwms)
            )
            if changed:
                last_pwms = list(pwms)
                print(f"[bridge] PWM: ch0={pwms[0]} ch11={pwms[11]} full={pwms}")
                try:
                    Bridge.notify("set_servo_pwms", ",".join(str(p) for p in pwms))  # single string, avoids 12-arg limit
                    notify_errs = 0
                except Exception as e:
                    notify_errs += 1
                    if notify_errs <= 3:
                        print(f"[bridge] Bridge.notify failed: {e}")

        with cache_lock:
            has_imu = cache["imu"] is not None
            has_status = cache["hw_status"] is not None
            has_scan = cache["i2c_scan"] is not None
        now = time.time()
        if now - last_log >= 5:
            print(f"[bridge] loop: imu={'yes' if has_imu else 'no'} status={'yes' if has_status else 'no'} i2c_scan={'yes' if has_scan else 'no'} notify_errs={notify_errs}")
            last_log = now
        time.sleep(0.01)


def main():
    # Register the handlers before the STM32 starts sending data
    print("[bridge] registering notification handlers...")
    Bridge.provide("imu", on_imu)
    print("[bridge] registered imu handler")
    Bridge.provide("hw_status", on_hw_status)
    print("[bridge] registered hw_status handler")
    Bridge.provide("i2c_scan", on_i2c_scan)
    print("[bridge] registered i2c_scan handler")
    Bridge.provide("servo_diag_result", on_servo_diag_result)
    print("[bridge] registered servo_diag_result handler")

    t_servo = threading.Thread(target=tcp_servo_server, daemon=True)
    t_servo.start()
    t_imu = threading.Thread(target=tcp_imu_server, daemon=True)
    t_imu.start()
    print(f"[bridge] servo TCP server on {TCP_HOST}:{SERVO_PORT}, IMU TCP server on {TCP_HOST}:{IMU_PORT}")
    App.run(user_loop=loop)                    # this blocks — keeps the program alive until stopped


if __name__ == "__main__":
    main()
