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

# Cache for data pushed by the MCU via Bridge.notify
cache = {
    "imu": None,
    "hw_status": None,
    "i2c_scan": [],
}
cache_lock = threading.Lock()
servo_req_count = 0
servo_req_lock = threading.Lock()


def handle_client(conn):
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

                if cmd == "servo":
                    pwms = req["pwms"]
                    global servo_req_count
                    with servo_req_lock:
                        servo_req_count += 1
                    Bridge.notify("set_servo_pwms", pwms)
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
                        with servo_req_lock:
                            hw["servo_reqs"] = servo_req_count
                        conn.sendall(json.dumps(hw).encode() + b"\n")
                    else:
                        conn.sendall(json.dumps({"error": "no status yet"}).encode() + b"\n")

                elif cmd == "ping":
                    Bridge.notify("ping")
                    conn.sendall(b'{"ok":true}\n')

                elif cmd == "scan_i2c":
                    Bridge.notify("scan_i2c")
                    # Give the MCU a moment to scan and push the result
                    time.sleep(0.5)
                    with cache_lock:
                        scan = {"addrs": sorted(cache["i2c_scan"])}
                    conn.sendall(json.dumps(scan).encode() + b"\n")

                else:
                    conn.sendall(
                        json.dumps({"error": "unknown cmd"}).encode() + b"\n"
                    )
    except (ConnectionResetError, BrokenPipeError):
        pass
    finally:
        conn.close()


def tcp_server():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((TCP_HOST, TCP_PORT))
    sock.listen()
    sock.settimeout(1.0)

    while True:
        try:
            conn, addr = sock.accept()
            threading.Thread(
                target=handle_client, args=(conn,), daemon=True
            ).start()
        except socket.timeout:
            continue


# ── Bridge.notify handlers (called when MCU pushes data) ──────────────────

def on_imu(ax, ay, az, gx, gy, gz):
    with cache_lock:
        cache["imu"] = {
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
        }


def on_hw_status(scan, ai_ok, servo_calls=0, ping_count=0):
    with cache_lock:
        cache["hw_status"] = {"servo_calls": servo_calls, "ping_count": ping_count,
            "i2c_scan": scan,
            "ai_ok": bool(ai_ok),
            "pca9685_ok": (scan & 1) == 0,
            "mpu6050_ok": (scan & 2) == 0,
        }


def on_i2c_scan(addrs):
    with cache_lock:
        cache["i2c_scan"] = list(addrs)


def loop():
    pass


def main():
    # Register notification handlers BEFORE the MCU starts sending data.
    Bridge.provide("imu", on_imu)
    Bridge.provide("hw_status", on_hw_status)
    Bridge.provide("i2c_scan", on_i2c_scan)

    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()
    App.run(user_loop=loop)


if __name__ == "__main__":
    main()
