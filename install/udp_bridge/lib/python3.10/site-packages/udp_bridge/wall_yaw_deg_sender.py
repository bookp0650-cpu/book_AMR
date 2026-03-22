#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import time


class WallYawDegSender(Node):
    def __init__(self):
        # ⚠️ YAMLと一致
        super().__init__('wall_yaw_deg_sender')

        # -------------------------
        # 🔹 パラメータ宣言
        # -------------------------
        self.declare_parameter('target_ip', '172.20.10.3')
        self.declare_parameter('tcp_port', 5030)
        self.declare_parameter('retry_interval', 1.0)
        self.declare_parameter('topic_name', '/wall_yaw_deg')
        self.declare_parameter('decimal_places', 4)

        # -------------------------
        # 🔹 パラメータ取得
        # -------------------------
        self.target_ip = self.get_parameter('target_ip').value
        self.tcp_port = self.get_parameter('tcp_port').value
        self.retry_interval = self.get_parameter('retry_interval').value
        self.topic_name = self.get_parameter('topic_name').value
        self.decimal_places = self.get_parameter('decimal_places').value

        # -------------------------
        # 🔹 TCP接続
        # -------------------------
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.get_logger().info(
            f"Waiting for receiver {self.target_ip}:{self.tcp_port}"
        )

        while rclpy.ok():
            try:
                self.sock.connect((self.target_ip, self.tcp_port))
                break
            except (ConnectionRefusedError, OSError):
                self.get_logger().warn("Receiver not ready, retrying...")
                time.sleep(self.retry_interval)

        self.get_logger().info("Connected to receiver")

        # -------------------------
        # 🔹 subscriber
        # -------------------------
        self.sub = self.create_subscription(
            Float32,
            self.topic_name,
            self.cb,
            10
        )

    def cb(self, msg: Float32):
        text = f"{msg.data:.{self.decimal_places}f}"

        try:
            self.sock.sendall((text + "\n").encode('utf-8'))
        except BrokenPipeError:
            self.get_logger().error("TCP connection lost")
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")


def main():
    rclpy.init()
    node = WallYawDegSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()