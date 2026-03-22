#!/usr/bin/env python3
import struct
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import fcntl
import os

JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

class JS0Teleop(Node):
    def __init__(self):
        super().__init__('js0_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.js = open('/dev/input/js0', 'rb')
        # ★ ノンブロッキングにする
        flags = fcntl.fcntl(self.js, fcntl.F_GETFL)
        fcntl.fcntl(self.js, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        self.event_format = 'IhBB'
        self.event_size = struct.calcsize(self.event_format)

        self.axes = [0.0] * 8

        self.last_event_time = time.time()
        self.timeout = 10

        self.create_timer(0.001, self.read_joystick)
        self.create_timer(0.01, self.publish_cmd_vel)

        self.get_logger().info("JS0 Teleop (axis1 forward/back, axis3 yaw) started.")

    def read_joystick(self):
        try:
            data = self.js.read(self.event_size)

            # ★ データ空でもタイムアウトは更新しておく
            #    → これで cmd_vel が止まらなくなる
            self.last_event_time = time.time()

            if not data:
                return

            t, raw_value, etype, num = struct.unpack(self.event_format, data)

            if etype & JS_EVENT_INIT:
                etype &= ~JS_EVENT_INIT

            if etype == JS_EVENT_AXIS and num < len(self.axes):
                self.axes[num] = raw_value / 32767.0

        except Exception as e:
            self.get_logger().error(f"Joystick read error: {e}")


    def publish_cmd_vel(self):
        twist = Twist()

        if time.time() - self.last_event_time > self.timeout:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            lin = -self.axes[1]
            ang = -self.axes[3]

            max_linear_speed = 0.15
            max_angular_speed = 0.6

            twist.linear.x  = lin * max_linear_speed
            twist.angular.z = ang * max_angular_speed

        # 🎯 ← ここでログ出す
        self.get_logger().info(
            f"cmd_vel => linear.x: {twist.linear.x:.3f}, angular.z: {twist.angular.z:.3f}"
        )

        self.pub.publish(twist)

def main():
    rclpy.init()
    node = JS0Teleop()
    rclpy.spin(node)
