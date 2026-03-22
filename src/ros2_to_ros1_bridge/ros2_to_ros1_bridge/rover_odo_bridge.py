import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json

class R1OdoToRos2(Node):
    def __init__(self):
        super().__init__('ros1_odo_bridge')

        # UDP受信用 (ROS1 → UDP)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5010))
        self.sock.setblocking(False)

        # ROS2 Publisher
        self.pub = self.create_publisher(Twist, "/rover_odo", 10)

        print("📡 UDP → /rover_odo bridge started (port 5010)")

        # timerで定期チェック
        self.timer = self.create_timer(0.01, self.check_udp)

    def check_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
        except BlockingIOError:
            return  # データなし

        try:
            d = json.loads(data.decode())
            msg = Twist()
            msg.linear.x = d["x"]
            msg.angular.z = d["z"]

            self.pub.publish(msg)
            print(f"Recv UDP → publish: {d}")

        except Exception as e:
            print("JSON decode error:", e)


def main():
    rclpy.init()
    node = R1OdoToRos2()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
