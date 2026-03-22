#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading


class AmrTcpReceiver(Node):
    def __init__(self):
        # ⚠️ YAMLと一致させる
        super().__init__('shelf_id_receiver')

        # -------------------------
        # 🔹 パラメータ宣言
        # -------------------------
        self.declare_parameter('tcp_port', 5000)
        self.declare_parameter('topic_name', '/shelf_id')
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
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind(('', self.tcp_port))
        self.server_sock.listen(1)

        self.get_logger().info(
            f"TCP receiver listening on port {self.tcp_port}"
        )

        threading.Thread(
            target=self.accept_loop,
            daemon=True
        ).start()

    def accept_loop(self):
        while rclpy.ok():
            conn, addr = self.server_sock.accept()
            self.get_logger().info(f"TCP connection from {addr}")

            try:
                self.recv_loop(conn, addr)
            except Exception as e:
                self.get_logger().warn(f"TCP error {addr}: {e}")
            finally:
                conn.close()
                self.get_logger().info(f"TCP disconnected {addr}")

    def recv_loop(self, conn, addr):
        buffer = ""
        while rclpy.ok():
            data = conn.recv(self.buf_size)
            if not data:
                break

            buffer += data.decode('utf-8')

            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                shelf_id = line.strip()
                if shelf_id == "":
                    continue

                msg = String()
                msg.data = shelf_id
                self.pub.publish(msg)

                self.get_logger().info(
                    f"Received '{shelf_id}' → publish {self.topic_name}"
                )


def main():
    rclpy.init()
    node = AmrTcpReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()