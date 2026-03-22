#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import yaml
import math
from std_msgs.msg import Float32, Int32
from std_msgs.msg import Bool


class ShelfXCorrection(Node):
    def __init__(self):
        super().__init__('shelf_x_correction')

        # -----------------------------
        # parameters (例)
        # -----------------------------
        self.x_offset = -1.0 #2.0 2号館  #-0.272 野田図書館   MAP上で最初の書架のx座標 
        self.shelf_length = 1.793 #0.9
        self.column_length = 0.3506
        self.half_column_length = self.column_length / 2
        self.latest_e_start_x = None
        self.start_x_from_id = None   # shelf_id 由来（id_error = 0）
        self.final_goal_timer = None
        # wall-based localization inputs
        self.wall_distance = None      # [m]
        self.wall_yaw_deg = None       # [deg]
        self.corridor_status = None   # Int32
        self.current_home = None
        # --- retry control ---
        self.last_shelf_id_str = None
        self.retry_count = 0
        self.max_retry = 2
        self.retry_timer = None
        self.nav_x_offset_capture = 0.4 
        self.nav_x_offset = 0.85

        # -----------------------------
        # load area_home.yaml
        # -----------------------------
        yaml_path = '/home/rover/ros2_ws/src/shelf_navigator/config/TOHAN.yaml'
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)

        self.areas = yaml_data['areas']
        self.current_width = None

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.create_subscription(
            String,
            '/error_x',
            self.error_x_cb,
            10
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.create_subscription(
            String,
            '/shelf_id',
            self.shelf_id_cb,
            10
        )
        self.final_goal_pub = self.create_publisher(
            PoseStamped,
            '/final_goal_pose',
            10
        )

        self.final_nav_goal_pub = self.create_publisher(
            Bool,
            '/navigation_goal_final',
            10
        )

        # 壁距離
        self.create_subscription(
            Float32,
            '/wall_distance',
            self.wall_distance_cb,
            10
        )

        # 壁yaw
        self.create_subscription(
            Float32,
            '/wall_yaw_deg',
            self.wall_yaw_cb,
            10
        )

        # 通路状態
        self.create_subscription(
            Int32,
            '/corridor_status',
            self.corridor_status_cb,
            10
        )

        self.shelf_id_pub = self.create_publisher(
            String,
            '/shelf_id',
            10
        )
        self.get_logger().info('ShelfXCorrection ready')

    # =================================================
    # callback
    # =================================================
    def wall_distance_cb(self, msg: Float32):
        self.wall_distance = msg.data
        self.get_logger().debug(f'[WALL_DISTANCE] {self.wall_distance:.3f} m')

    def wall_yaw_cb(self, msg: Float32):
        self.wall_yaw_deg = msg.data
        self.get_logger().debug(f'[WALL_YAW] {self.wall_yaw_deg:.2f} deg')


    def corridor_status_cb(self, msg: Int32):
        self.corridor_status = msg.data
        #self.get_logger().info(f'[CORRIDOR] status={self.corridor_status}')

    def send_nav_final(self):
        msg = Bool()
        msg.data = True
        self.final_nav_goal_pub.publish(msg)

    def error_x_cb(self, msg: String):
        # --- parse ---
        shelf_id_str, id_error_str = msg.data.split('/')
        A, B, C, D = map(int, shelf_id_str.split('-'))
        id_error = float(id_error_str)

        # --- calc theoretical x ---
        e_start_x = self.calc_start_x(A, B, C, D, id_error)
        e_start_x += (self.nav_x_offset_capture + 0.15)
        self.latest_e_start_x = e_start_x

        if self.start_x_from_id is None:
            self.get_logger().warn('start_x_from_id not ready')
            return

        diff_x = abs(e_start_x - self.start_x_from_id)

        self.get_logger().info(
            f'[COMPARE] current={e_start_x:.3f}, '
            f'target={self.start_x_from_id:.3f}, '
            f'diff_x={diff_x:.3f}'
        )

        if diff_x <= 0.05:
            self.send_nav_final()
            self.get_logger().info(f"Comlete Navigation")
        else:
            if self.retry_count < self.max_retry:
                self.retry_count += 1
                self.get_logger().warn(f"[RETRY] diff too large, retry={self.retry_count}")
                self.apply_initialpose(e_start_x)
                self.schedule_resend()
            else:
                self.get_logger().error("Max retry reached. Giving up.")

    def calc_start_x(self, A, B, C, D, id_error):

        column_index = (B - 1) // 2
        is_left = (B % 2 == 0)

        base_x = self.x_offset + column_index * self.shelf_length

        if is_left:
            # 偶数 → left
            start_x = base_x + (self.column_length * D + 0.041) - self.half_column_length 
        else:
            # 奇数 → right
            start_x = (base_x + self.shelf_length) - self.column_length * D + self.half_column_length 

        # カメラ補正（誤差補正）
        start_x -= id_error 

        return start_x


    def shelf_id_cb(self, msg: String):
        self.last_shelf_id_str = msg.data
        self.retry_count = 0
        """
        msg.data example:
        "1-2-3-2"
        A = area_id
        """
        try:
            A, B, C, D = map(int, msg.data.split('-'))
        except Exception:
            self.get_logger().error(f'Invalid shelf_id format: {msg.data}')
            return

        # -------------------------
        # area → corridor width
        # -------------------------
        if A not in self.areas:
            self.get_logger().warn(f'Area {A} not found in yaml')
            return

        self.current_width = self.areas[A]['corridor']['width']
        self.current_home = self.areas[A]['home']

        self.get_logger().info(
            f'[AREA] A={A}, corridor_width={self.current_width:.3f} m'
        )

        # -------------------------
        # existing logic
        # -------------------------
        start_x = self.calc_start_x(
            A=A,
            B=B,
            C=C,
            D=D,
            id_error=0.0
        )

        self.start_x_from_id = start_x + self.nav_x_offset

        self.get_logger().info(
            f'[SHELF_ID] A={A}, B={B}, C={C}, D={D} '
            f'-> target={self.start_x_from_id:.3f}'
        )

    def calc_y_now_from_wall(self):
        """
        y_now = home_y + (width/2 - wall_distance)

        Returns:
            float or None
        """
        if self.current_home is None:
            self.get_logger().warn('current_home not set')
            return None

        if self.current_width is None:
            self.get_logger().warn('current_width not set')
            return None

        if self.wall_distance is None:
            self.get_logger().warn('wall_distance not available')
            return None

        home_y = self.current_home['y']
        width = self.current_width
        d_wall = self.wall_distance

        y_now = home_y + (width / 2.0 - d_wall)

        self.get_logger().info(
            f'[Y_NOW] home_y={home_y:.3f}, '
            f'width={width:.3f}, wall_d={d_wall:.3f} '
            f'-> y_now={y_now:.3f}'
        )

        return y_now

    def calc_q_now_from_wall(self):
        """
        q_now = quaternion from (home_yaw + wall_yaw)

        wall_yaw_deg : deg (wall-based)
        home['yaw']  : rad (map-based)

        Returns:
            geometry_msgs.msg.Quaternion or None
        """
        if self.current_home is None:
            self.get_logger().warn('current_home not set')
            return None

        if self.wall_yaw_deg is None:
            self.get_logger().warn('wall_yaw_deg not available')
            return None

        # home yaw (map基準)
        yaw_home = 0 # [rad] 前提
        yaw_wall = math.radians(self.wall_yaw_deg)  # deg → rad

        yaw_now = yaw_home + yaw_wall

        q = PoseStamped().pose.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_now / 2.0)
        q.w = math.cos(yaw_now / 2.0)

        self.get_logger().info(
            f'[Q_NOW] yaw_home={math.degrees(yaw_home):.2f} deg, '
            f'wall_yaw={self.wall_yaw_deg:.2f} deg '
            f'-> yaw_now={math.degrees(yaw_now):.2f} deg'
        )

        return q


    def resend_shelf_id(self):
        if self.last_shelf_id_str is None:
            self.get_logger().warn("No shelf_id to resend")
            return

        msg = String()
        msg.data = self.last_shelf_id_str

        self.get_logger().warn(f"[RESEND] shelf_id={msg.data}")
        self.shelf_id_pub.publish(msg)

    def schedule_resend(self):
        if self.retry_timer is not None:
            self.retry_timer.cancel()

        self.retry_timer = self.create_timer(0.5, self._resend_once)

    def _resend_once(self):
        self.retry_timer.cancel()
        self.retry_timer = None
        self.resend_shelf_id()

    # =================================================
    # core logic
    # =================================================
    def apply_initialpose(self, e_start_x):
        y_now = self.calc_y_now_from_wall()
        q_now = self.calc_q_now_from_wall()
        if y_now is None or q_now is None:
            return
        # --- initialpose ---
        ip = PoseWithCovarianceStamped()
        ip.header.stamp = self.get_clock().now().to_msg()
        ip.header.frame_id = 'map'

        ip.pose.pose.position.x = e_start_x 
        ip.pose.pose.position.y = y_now
        ip.pose.pose.position.z = 0.0

        ip.pose.pose.orientation = q_now

        ip.pose.covariance = [
            0.003, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.003, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]

        self.initialpose_pub.publish(ip)

        # yaw を人間が読める形に変換
        yaw_deg = math.degrees(
            math.atan2(
                2.0 * (q_now.w * q_now.z + q_now.x * q_now.y),
                1.0 - 2.0 * (q_now.y * q_now.y + q_now.z * q_now.z)
            )
        )
        self.get_logger().info(
            f'[APPLY] current_x={ip.pose.pose.position.x:.3f}, target_x={self.start_x_from_id:.3f}'
        )

        self.get_logger().info(
            f'[POSE_APPLIED] x={ip.pose.pose.position.x:.3f}, '
            f'y={ip.pose.pose.position.y:.3f}, '
            f'yaw={yaw_deg:.2f} deg'
        )


def main():
    rclpy.init()
    node = ShelfXCorrection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
