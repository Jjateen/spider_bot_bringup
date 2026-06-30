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
TCP_PORT = 50007

# Store the latest data from the robot's sensors
cache = {
    "imu": None,         # most recent IMU reading
    "hw_status": None,   # most recent hardware health check
    "i2c_scan": [],      # list of I2C devices found
}
cache_lock = threading.Lock()   # only one thread reads or writes the cache at a time


def handle_client(conn):
    buf = b""
    try:
        while True:
            # Read whatever the ROS node sent us
            data = conn.recv(4096)
            if not data:
                break                     # connection closed
            buf += data
            # Commands are separated by newlines — handle each one
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    req = json.loads(line.decode())   # turn the text into a dictionary
                except json.JSONDecodeError as e:
                    conn.sendall(json.dumps({"error": str(e)}).encode() + b"\n")
                    continue

                cmd = req.get("cmd")

                # ── Move the servos ──
                if cmd == "servo":
                    pwms = req["pwms"]
                    Bridge.notify("set_servo_pwms", pwms)   # tell the STM32
                    conn.sendall(b'{"ok":true}\n')

                # ── Return the latest IMU reading ──
                elif cmd == "imu":
                    with cache_lock:
                        imu = cache["imu"]                 # grab the latest IMU
                    if imu is not None:
                        conn.sendall(json.dumps(imu).encode() + b"\n")
                    else:
                        conn.sendall(json.dumps({"error": "no imu data yet"}).encode() + b"\n")

                # ── Return hardware health status ──
                elif cmd == "status":
                    with cache_lock:
                        hw = cache["hw_status"]
                    if hw is not None:
                        conn.sendall(json.dumps(hw).encode() + b"\n")
                    else:
                        conn.sendall(json.dumps({"error": "no status yet"}).encode() + b"\n")

                # ── Scan the I2C bus for connected devices ──
                elif cmd == "scan_i2c":
                    Bridge.notify("scan_i2c")
                    time.sleep(0.5)                          # wait for the scan to finish
                    with cache_lock:
                        scan = {"addrs": sorted(cache["i2c_scan"])}
                    conn.sendall(json.dumps(scan).encode() + b"\n")

                else:
                    conn.sendall(
                        json.dumps({"error": "unknown cmd"}).encode() + b"\n"
                    )
    except (ConnectionResetError, BrokenPipeError):
        pass               # client disconnected — nothing to do
    finally:
        conn.close()


def tcp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # allow restart without waiting
    sock.bind((TCP_HOST, TCP_PORT))
    sock.listen()
    sock.settimeout(1.0)                                         # wake up every second to check for shutdown

    while True:
        try:
            conn, addr = sock.accept()                           # a new ROS node connected
            threading.Thread(
                target=handle_client, args=(conn,), daemon=True
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
        }


def on_hw_status(scan, ai_ok, servo_calls=0, ping_count=0):
    with cache_lock:
        cache["hw_status"] = {
            "i2c_scan": scan,
            "ai_ok": bool(ai_ok),
            "pca9685_ok": (scan & 1) == 0,
            "mpu6050_ok": (scan & 2) == 0,
            "servo_calls": servo_calls,
            "ping_count": ping_count,
        }


def on_i2c_scan(addrs):
    # STM32 finished scanning the I2C bus — save the list of devices found
    with cache_lock:
        cache["i2c_scan"] = list(addrs)


def loop():
    pass


def main():
    # Register the handlers before the STM32 starts sending data
    Bridge.provide("imu", on_imu)
    Bridge.provide("hw_status", on_hw_status)
    Bridge.provide("i2c_scan", on_i2c_scan)

    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()
    App.run(user_loop=loop)                    # this blocks — keeps the program alive until stopped


if __name__ == "__main__":
    main()
