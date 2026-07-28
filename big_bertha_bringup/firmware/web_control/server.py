#!/usr/bin/env python3
import http.server
import json
import os
import sys
import threading
from http import HTTPStatus

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

SERVO_LOWER_LIMIT = [45.0, 30.0, 180.0, 140.0, 135.0, 140.0, 50.0, 50.0, 40.0, 180.0, 150.0, 0.0]
SERVO_UPPER_LIMIT = [180.0, 150.0, 50.0, 0.0, 0.0, 0.0, 180.0, 180.0, 180.0, 40.0, 0.0, 150.0]
SERVO_OFFSET = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 8.0, 2.0, 5.0]
SERVO_DIRECTION = [1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1]
POLICY_CENTER = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 1.57, 1.57, 1.57]
SERVO_CENTER = [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 100.0, 100.0, 90.0, 98.0, 92.0, 95.0]

JOINT_NAMES = [
    "arm_a_4_1 (hip)", "arm_a_1_1 (hip)", "arm_a_2_1 (hip)", "arm_a_3_1 (hip)",
    "arm_b_4_1 (knee)", "arm_b_1_1 (knee)", "arm_b_2_1 (knee)", "arm_b_3_1 (knee)",
    "arm_c_4_1 (ankle)", "arm_c_1_1 (ankle)", "arm_c_2_1 (ankle)", "arm_c_3_1 (ankle)",
]

LEG_GROUPS = [
    {"name": "Leg 4 back-left (arm_*_4_1)", "indices": [0, 4, 8], "joints": ["arm_a_4_1 (hip)", "arm_b_4_1 (knee)", "arm_c_4_1 (ankle)"]},
    {"name": "Leg 1 front-left (arm_*_1_1)", "indices": [1, 5, 9], "joints": ["arm_a_1_1 (hip)", "arm_b_1_1 (knee)", "arm_c_1_1 (ankle)"]},
    {"name": "Leg 2 front-right (arm_*_2_1)", "indices": [2, 6, 10], "joints": ["arm_a_2_1 (hip)", "arm_b_2_1 (knee)", "arm_c_2_1 (ankle)"]},
    {"name": "Leg 3 back-right (arm_*_3_1)", "indices": [3, 7, 11], "joints": ["arm_a_3_1 (hip)", "arm_b_3_1 (knee)", "arm_c_3_1 (ankle)"]},
]

JOINT_LIMIT = 3.141592653589793
_RAD = 3.141592653589793 / 180.0
RAD_RANGES = []
for i in range(12):
    lo = min(SERVO_LOWER_LIMIT[i], SERVO_UPPER_LIMIT[i])
    hi = max(SERVO_LOWER_LIMIT[i], SERVO_UPPER_LIMIT[i])
    rad_lo = ((lo - SERVO_OFFSET[i] - 90.0) / SERVO_DIRECTION[i]) * _RAD + POLICY_CENTER[i]
    rad_hi = ((hi - SERVO_OFFSET[i] - 90.0) / SERVO_DIRECTION[i]) * _RAD + POLICY_CENTER[i]
    rad_lo, rad_hi = sorted((rad_lo, rad_hi))
    rad_lo = max(rad_lo, -JOINT_LIMIT)
    rad_hi = min(rad_hi, JOINT_LIMIT)
    RAD_RANGES.append((round(rad_lo, 3), round(rad_hi, 3)))


class WebControlNode(Node):
    def __init__(self):
        super().__init__("web_control_server")
        self.pub = self.create_publisher(Float64MultiArray, "/position_controller/commands", 1)
        self.get_logger().info("Web control server node started")

    def send_servo_angles(self, angles_rad):
        msg = Float64MultiArray()
        msg.data = angles_rad
        self.pub.publish(msg)
        rad_str = ", ".join(f"{r:.3f}" for r in angles_rad)
        self.get_logger().info(f"Sent radian angles: [{rad_str}]")

    def send_home(self):
        msg = Float64MultiArray()
        msg.data = POLICY_CENTER
        self.pub.publish(msg)
        self.get_logger().info("Sent home position")


class ServoHTTPHandler(http.server.BaseHTTPRequestHandler):
    node = None
    static_dir = None

    def do_GET(self):
        if self.path == "/":
            self._serve_file("index.html", "text/html")
        elif self.path == "/api/config":
            self._json_response({
                "joint_names": JOINT_NAMES,
                "leg_groups": LEG_GROUPS,
                "rad_ranges": RAD_RANGES,
                "initial_angles": POLICY_CENTER,
                "servo_offset": SERVO_OFFSET,
                "servo_direction": SERVO_DIRECTION,
                "policy_center": POLICY_CENTER,
            })
        elif self.path.startswith("/static/"):
            self._serve_file(self.path[len("/static/"):], None)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _serve_file(self, rel_path, content_type):
        base = self.__class__.static_dir
        filepath = os.path.normpath(os.path.join(base, rel_path))
        if not filepath.startswith(base):
            self.send_error(HTTPStatus.FORBIDDEN)
            return
        if not os.path.isfile(filepath):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if content_type is None:
            ext = os.path.splitext(filepath)[1]
            content_type = {
                ".html": "text/html",
                ".css": "text/css",
                ".js": "application/javascript",
            }.get(ext, "application/octet-stream")
        with open(filepath, "rb") as f:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(f.read())

    def do_POST(self):
        if self.path == "/api/servos":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body)
                angles = data.get("angles", [])
                if len(angles) != 12:
                    raise ValueError("Need exactly 12 angles")
                for a in angles:
                    if not isinstance(a, (int, float)):
                        raise TypeError("Angles must be numbers")
                self.__class__.node.send_servo_angles(angles)
                self._json_response({"ok": True})
            except (TypeError, ValueError) as e:
                self._json_response({"ok": False, "error": str(e)}, HTTPStatus.BAD_REQUEST)
        elif self.path == "/api/home":
            self.__class__.node.send_home()
            self._json_response({"ok": True})
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def _json_response(self, data, status=HTTPStatus.OK):
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args):
        if self.__class__.node:
            msg = format % args
            self.__class__.node.get_logger().info(f"HTTP: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    ServoHTTPHandler.node = node

    script_dir = os.path.dirname(os.path.abspath(__file__))
    local_static = os.path.join(script_dir, "static")
    if os.path.isdir(local_static):
        ServoHTTPHandler.static_dir = local_static
    else:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("big_bertha_bringup")
            ServoHTTPHandler.static_dir = os.path.join(pkg_share, "web_control", "static")
        except RuntimeError:
            node.get_logger().error("Cannot find static files directory")
            sys.exit(1)

    server = http.server.ThreadingHTTPServer(("0.0.0.0", 8080), ServoHTTPHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    node.get_logger().info("Web control server listening on http://0.0.0.0:8080")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
