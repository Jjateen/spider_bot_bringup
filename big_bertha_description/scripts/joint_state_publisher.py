#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish)
        self.joints = [
            'Revolute_110', 'Revolute_111', 'Revolute_112',
            'Revolute_113', 'Revolute_114', 'Revolute_115',
            'Revolute_116', 'Revolute_117', 'Revolute_118',
            'Revolute_119', 'Revolute_120', 'Revolute_121',
        ]
        self.defaults = [
            0.0, -0.32, 2.00,
            0.0, -0.32, 2.00,
            0.0, -0.32, 2.00,
            0.0, -0.32, 2.00,
        ]

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joints
        msg.position = self.defaults
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(JointStatePublisher())
    rclpy.shutdown()
