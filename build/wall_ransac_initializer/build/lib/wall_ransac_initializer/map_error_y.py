#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import yaml
import os
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped


class FinalGoalInitialPoseSetter(Node):
    def __init__(self):
        super().__init__('finalgoal_initialpose_setter')

        # =====================
        # Parameters
        # =====================
        self.half_corridor_width = 0.4

        # =====================
        # Cached state
        # =====================
        self.corridor_status = None
        self.amcl_pose = None
        self.wall_distance = None
        self.last_cmd_vel_x = 0.0
        self.pending_final_goal = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.offset = 0.00

        # =====================
        # Subscribers
        # =====================
        self.create_subscription(
            PoseStamped,
            '/final_goal_pose',
            self.cb_trigger,
            10
        )

        self.create_subscription(
            Int32,
            '/corridor_status',
            lambda msg: setattr(self, 'corridor_status', msg.data),
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cb_cmd_vel,
            10
        )

        self.create_subscription(
            Float32,
            '/wall_distance',
            self.cb_wall_distance,
            10
        )


        # =====================
        # Publisher
        # =====================
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # =====================
        # Load YAML
        # =====================
        pkg_path = get_package_share_directory('shelf_navigator')
        yaml_path = os.path.join(pkg_path, 'config', 'area_home2.yaml')

        with open(yaml_path, 'r') as f:
            self.area_home = yaml.safe_load(f)['areas']

        self.get_logger().info('FinalGoalInitialPoseSetter ready')

    def publish_initialpose(self):
        # =====================
        # corridor → home 定義確認
        # =====================
        key = self.corridor_status
        if key not in self.area_home:
            self.get_logger().warn(f'No home defined for corridor {key}')
            return
        home = self.area_home[key]['home']

        # =====================
        # 壁距離補正
        # =====================
        error_y = self.half_corridor_width - self.wall_distance

        # =====================
        # TF: map → base_footprint 取得
        # =====================
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        base_x = tf.transform.translation.x
        base_y = tf.transform.translation.y 

        q = tf.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # =====================
        # cmd_vel による微小補正
        # =====================
        dx = 0.0
        if self.last_cmd_vel_x > 0.05:
            dx = self.offset
        elif self.last_cmd_vel_x < -0.05:
            dx = -self.offset

        # =====================
        # 目標 initial pose
        # =====================
        target_x = base_x + dx
        target_y = home['y'] + error_y

        q_new = quaternion_from_euler(0.0, 0.0, yaw)

        # =====================
        # initialpose publish
        # =====================
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = target_x
        msg.pose.pose.position.y = target_y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = q_new[0]
        msg.pose.pose.orientation.y = q_new[1]
        msg.pose.pose.orientation.z = q_new[2]
        msg.pose.pose.orientation.w = q_new[3]

        # covariance は AMCL 用に「弱め固定」がおすすめ
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.003, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]

        self.pub_initialpose.publish(msg)
        self.get_logger().info(
            f'Published initialpose: x={target_x:.3f}, y={target_y:.3f}, yaw={math.degrees(yaw):.1f}deg'
        )

    def cb_wall_distance(self, msg):
        self.wall_distance = msg.data

    def cb_trigger(self, msg): #これは幅とただログに出してる関数
        self.get_logger().info(
            f'[TRIGGER] final_goal_pose received '
            f'(x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f})'
        )

        self.pending_final_goal = True

        # 条件チェック
        if self.corridor_status is None:
            self.get_logger().warn('[SKIP] corridor_status not received yet')
            return

        if self.wall_distance is None:
            self.get_logger().warn('[SKIP] wall_distance not received yet')
            return

        # OKなら publish
        self.get_logger().info(
            f'[READY] corridor={self.corridor_status}, wall_distance={self.wall_distance:.3f}'
        )
        self.publish_initialpose()
        self.get_logger().info(
            '[INITIALPOSE] start publishing initialpose'
        )



    def cb_cmd_vel(self, msg):
        self.last_cmd_vel_x = msg.linear.x



def main():
    rclpy.init()
    node = FinalGoalInitialPoseSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
