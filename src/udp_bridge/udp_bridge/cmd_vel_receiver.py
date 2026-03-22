#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading


class CmdVelReceiver(Node):

    def __init__(self):
        super().__init__('cmd_vel_tcp_receiver')

        # -------------------------
        # 🔹 パラメータ宣言
        # -------------------------
        self.declare_parameter('tcp_port', 5065)
        self.declare_parameter('topic_name', '/cmd_vel')
        self.declare_parameter('buffer_size', 1024)

        # -------------------------
        # 🔹 パラメータ取得
        # -------------------------
        self.tcp_port = self.get_parameter('tcp_port').value
        self.topic_name = self.get_parameter('topic_name').value
        self.buf_size = self.get_parameter('buffer_size').value

        # -------------------------
        # 🔹 publisher
        # -------------------------
        self.pub = self.create_publisher(
            Twist,
            self.topic_name,
            10
        )

        # -------------------------
        # 🔹 TCP server
        # -------------------------
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('', self.tcp_port))
        self.server.listen(1)

        self.get_logger().info(
            f"Listening TCP on port {self.tcp_port}"
        )

        threading.Thread(
            target=self.accept_loop,
            daemon=True
        ).start()

    def accept_loop(self):
        while rclpy.ok():
            conn, addr = self.server.accept()
            self.get_logger().info(f"Connected from {addr}")

            threading.Thread(
                target=self.handle_client,
                args=(conn, addr),
                daemon=True
            ).start()

    def handle_client(self, conn, addr):

        with conn:
            while rclpy.ok():

                data = conn.recv(self.buf_size)
                if not data:
                    self.get_logger().info(f"Disconnected {addr}")
                    break

                text = data.decode('utf-8').strip()

                try:
                    vx, wz = map(float, text.split(','))

                    msg = Twist()
                    msg.linear.x = vx
                    msg.angular.z = wz

                    self.pub.publish(msg)

                    self.get_logger().info(
                        f"TCP '{text}' → publish {self.topic_name}"
                    )

                except Exception as e:
                    self.get_logger().warn(
                        f"Invalid data '{text}': {e}"
                    )


def main():
    rclpy.init()
    node = CmdVelReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()