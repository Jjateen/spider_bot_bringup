#!/usr/bin/env python3
# Big Bertha Hardware Bridge — MPU-side relay
#
# Relays commands between the ROS 2 C++ node (TCP socket) and the STM32U585
# sketch (Bridge RPC). Runs as the Python component of an arduino-app-cli App.

from arduino.app_utils import App, Bridge
import json
import socket
import threading

TCP_HOST = "0.0.0.0"
TCP_PORT = 50007


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

                if req.get("cmd") == "servo":
                    pwms = req["pwms"]
                    Bridge.notify("set_servo_pwms", pwms)
                    conn.sendall(b'{"ok":true}\n')

                elif req.get("cmd") == "imu":
                    result = Bridge.call("get_imu_data")
                    resp = json.dumps({
                        "ax": result[0], "ay": result[1], "az": result[2],
                        "gx": result[3], "gy": result[4], "gz": result[5],
                    })
                    conn.sendall(resp.encode() + b"\n")

                else:
                    conn.sendall(
                        json.dumps({"error": "unknown cmd"}).encode() + b"\n"
                    )
    except ConnectionResetError:
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


def loop():
    pass


def main():
    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()
    App.run(user_loop=loop)


if __name__ == "__main__":
    main()
