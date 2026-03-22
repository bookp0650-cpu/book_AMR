#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading


class ErrorXReceiver(Node):
    def __init__(self):
        # ⚠️ YAMLと名前合わせる
        super().__init__('error_x_receiver')

        # -------------------------
        # 🔹 パラメータ宣言
        # -------------------------
        self.declare_parameter('tcp_port', 5012)
        self.declare_parameter('topic_name', '/error_x')
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
            String,
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

                msg = String()
                msg.data = text
                self.pub.publish(msg)

                self.get_logger().info(
                    f"TCP '{text}' → publish {self.topic_name}"
                )


def main():
    rclpy.init()
    node = ErrorXReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()