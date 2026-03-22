#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import time


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 速度
        self.lin = 0.2
        self.ang = 0.6

        # 現在の指令
        self.cmd = Twist()

        # 入力監視
        self.last_key_time = time.time()
        self.key_timeout = 0.2   # ★ 無入力で停止するまでの時間 [s]

        # publish 周期（20Hz）
        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.get_logger().info('Keyboard teleop started')
        self.get_logger().info('w/s/a/d move, x stop, q quit')

    def get_key_nonblock(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def publish_cmd(self):
        # ★ 一定時間入力がなければ停止
        if time.time() - self.last_key_time > self.key_timeout:
            self.cmd = Twist()

        self.pub.publish(self.cmd)

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)

        try:
            while rclpy.ok():
                key = self.get_key_nonblock()

                if key is not None:
                    self.last_key_time = time.time()

                    if key == 'w':
                        self.cmd.linear.x = self.lin
                        self.cmd.angular.z = 0.0
                    elif key == 's':
                        self.cmd.linear.x = -self.lin
                        self.cmd.angular.z = 0.0
                    elif key == 'a':
                        self.cmd.angular.z = self.ang
                        self.cmd.linear.x = 0.0
                    elif key == 'd':
                        self.cmd.angular.z = -self.ang
                        self.cmd.linear.x = 0.0
                    elif key == 'x':
                        self.cmd = Twist()
                    elif key == 'q':
                        break

                rclpy.spin_once(self, timeout_sec=0.01)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.cmd = Twist()
            self.pub.publish(self.cmd)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
