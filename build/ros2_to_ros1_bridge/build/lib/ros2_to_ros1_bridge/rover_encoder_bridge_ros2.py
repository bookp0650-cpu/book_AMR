import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import socket
import json

class EncoderBridge(Node):
    def __init__(self):
        super().__init__('encoder_bridge_ros2')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5011))
        self.sock.setblocking(False)

        self.pub = self.create_publisher(Int32MultiArray, "/rover_encoder_raw", 10)

        self.timer = self.create_timer(0.01, self.check_udp)

        print("📡 UDP → /rover_encoder_raw bridge started (port 5011)")

    def check_udp(self):
        try:
            data, _ = self.sock.recvfrom(1024)
        except BlockingIOError:
            return

        try:
            d = json.loads(data.decode())

            msg = Int32MultiArray()
            msg.data = [-d["encL"], d["encR"]]

            self.pub.publish(msg)
            print("recv encoder:", msg.data)

        except Exception as e:
            print("JSON decode error:", e)

def main():
    rclpy.init()
    rclpy.spin(EncoderBridge())

if __name__ == "__main__":
    main()
