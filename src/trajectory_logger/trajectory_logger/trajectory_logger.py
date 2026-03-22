import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')

        # Path publisher
        self.path_pub = self.create_publisher(
            Path,
            '/trajectory_path',
            10
        )

        # AMCL pose subscriber
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.path = Path()
        self.path.header.frame_id = 'map'

        self.get_logger().info('Trajectory Logger started')

    def pose_callback(self, msg):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp

        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = TrajectoryLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
