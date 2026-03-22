#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import socket
import threading

UDP_IP = "127.0.0.1"
PORT_ROS1_TO_ROS2 = 5005
PORT_ROS2_TO_ROS1 = 5006

sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind((UDP_IP, PORT_ROS1_TO_ROS2))

sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class BridgeNode(Node):
    def __init__(self):
        super().__init__("ros2_to_ros1_bridge")

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cb_ros2_to_ros1,
            10
        )

        thread = threading.Thread(target=self.loop_ros1_to_ros2)
        thread.daemon = True
        thread.start()

    def cb_ros2_to_ros1(self, msg):
        d = {"linear_x": msg.linear.x, "angular_z": msg.angular.z}
        sock_send.sendto(json.dumps(d).encode(), (UDP_IP, PORT_ROS2_TO_ROS1))

    def loop_ros1_to_ros2(self):
        while rclpy.ok():
            data, _ = sock_recv.recvfrom(1024)
            d = json.loads(data.decode())
            msg = Twist()
            msg.linear.x = d["linear_x"]
            msg.angular.z = d["angular_z"]
            self.pub_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
