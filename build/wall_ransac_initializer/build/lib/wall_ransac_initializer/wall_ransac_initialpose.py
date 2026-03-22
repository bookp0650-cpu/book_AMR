#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
import random
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32



class WallRansacInitialPose(Node):
    def __init__(self):
        super().__init__('wall_ransac_initialpose')

        # ----------------------------
        # QoS
        # ----------------------------
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # LaserScan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            scan_qos
        )

        # /initialpose
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.final_goal_sub = self.create_subscription(
            PoseStamped,
            '/final_goal_pose',
            self.final_goal_cb,
            10
        )

        self.wall_dist_pub = self.create_publisher(
            Float32,
            '/wall_distance',
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cb_cmd_vel,
            10
        )

        # 壁角度（deg）
        self.wall_yaw_deg_pub = self.create_publisher(
            Float32,
            '/wall_yaw_deg',
            10
        )
        # 状態
        self.waiting_for_samples = False
        self.yaw_buffer = []
        self.yaw_buffer_size = 10
        self.triggered = False   # ★ 多重発火防止
        self.last_cmd_vel_x = 0.0

        self.get_logger().info(
            'WallRansacInitialPose started '
            '(triggered by Nav2 SUCCEEDED)'
        )

        # 状態
        self.waiting_for_samples = False
        self.yaw_buffer = []
        self.yaw_buffer_size = 10
        self.triggered = False   # ★ 多重発火防止
        # LPF
        self.wall_dist_lpf = None
        self.wall_lpf_alpha = 0.2
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.offset = 0.0

    # =================================================
    # Nav2 SUCCEEDED トリガ
    # =================================================
    def final_goal_cb(self, msg: PoseStamped):
        if self.triggered:
            return

        self.get_logger().info(
            'Final goal pose received → start yaw averaging'
        )

        # ★ ここで初めて RANSAC を有効化
        self.triggered = True
        self.waiting_for_samples = True
        self.yaw_buffer.clear()


    # =================================================
    # LaserScan
    # =================================================
    def scan_cb(self, scan: LaserScan):

        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        deg = np.degrees(angles)

        # ---- 側壁（60–120 deg）----
        mask = (deg >= 0.0) & (deg <= 180.0)
        ranges = ranges[mask]
        angles = angles[mask]

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        if len(ranges) < 3:
            return

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.stack([xs, ys], axis=1)

        model, inliers = self.ransac_line(points)
        if model is None:
            return

        a, b, c = model
        norm = math.hypot(a, b)

        # ===== 壁までの距離（ロボット原点 → 壁）=====
        raw_wall_dist = abs(c) / norm

        # ===== LPF =====
        if self.wall_dist_lpf is None:
            self.wall_dist_lpf = raw_wall_dist
        else:
            self.wall_dist_lpf = (
                (1.0 - self.wall_lpf_alpha) * self.wall_dist_lpf
                + self.wall_lpf_alpha * raw_wall_dist
            )

        wall_dist = self.wall_dist_lpf       # [m]
        wall_dist_signed = c / norm        # 符号付き
        msg = Float32()
        msg.data = wall_dist
        self.wall_dist_pub.publish(msg)

        # --- 壁平行方向ベクトル（正規化）---
        tx = b / norm
        ty = -a / norm

        # 各インライア点
        xs = inliers[:, 0]
        ys = inliers[:, 1]

        # --- 壁平行方向距離 ---
        dist_parallel = xs * tx + ys * ty

        # ★ 角度方向（壁法線方向）の距離
        dists = (a * inliers[:,0] + b * inliers[:,1] + c) / norm

        
        #self.get_logger().info(
            #f'[RANSAC] n={len(inliers)}, '
            #f'normal_mean={np.mean(dists):.3f}, '
            #f'normal_std={np.std(dists):.3f}, '
            #f'parallel_min={np.min(dist_parallel):.3f}, '
            #f'parallel_max={np.max(dist_parallel):.3f}'
            #f'wall_distance = {wall_dist:.3f} m '
            #f'(signed={wall_dist_signed:.3f})'
        #)



        wall_angle = math.atan2(-a, b)
        yaw = -wall_angle
        yaw = self.normalize_yaw_pm90(yaw)

        # ---- 壁角度（deg） publish ----
        yaw_deg_msg = Float32()
        yaw_deg_msg.data = math.degrees(yaw)
        self.wall_yaw_deg_pub.publish(yaw_deg_msg)
        
        if not self.waiting_for_samples:
            return  

        self.yaw_buffer.append(yaw)
   

        if len(self.yaw_buffer) < self.yaw_buffer_size:
            return

        yaw_avg = sum(self.yaw_buffer) / len(self.yaw_buffer)

        self.get_logger().info(
            f'yaw average = {math.degrees(yaw_avg):.2f} deg → publish /initialpose'
        )

        self.publish_initialpose(yaw_avg)
        

        # リセット
        self.waiting_for_samples = False
        self.yaw_buffer.clear()

    
    def normalize_yaw_pm90(self, yaw):
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        if yaw > math.pi / 2:
            yaw -= math.pi
        elif yaw < -math.pi / 2:
            yaw += math.pi
        return yaw

    def publish_initialpose(self, yaw):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',   # or base_footprint
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        base_x = tf.transform.translation.x
        base_y = tf.transform.translation.y

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        dx = 0.0
        if self.last_cmd_vel_x > 0.0:
            dx = self.offset
        elif self.last_cmd_vel_x < 0.0:
            dx = -self.offset

        msg.pose.pose.position.x = base_x + dx
        msg.pose.pose.position.y = base_y
        msg.pose.pose.position.z = 0.0

        # yaw を RANSAC 結果で上書き
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        cov = [0.0] * 36
        cov[0]  = 0.1
        cov[7]  = 0.1
        cov[35] = 0.02
        msg.pose.covariance = cov

        #self.init_pub.publish(msg)

        self.get_logger().info(
            f'Initialpose sent (xy from TF, yaw reset): '
            f'x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}, '
            f'yaw={math.degrees(yaw):.2f} deg'
        )

        self.triggered = False


    # =================================================
    def ransac_line(self, points, iterations=100, threshold=0.03):
        best_inliers = []
        best_model = None

        for _ in range(iterations):
            p1, p2 = random.sample(list(points), 2)

            a = p2[1] - p1[1]
            b = p1[0] - p2[0]
            c = p2[0]*p1[1] - p1[0]*p2[1]

            norm = math.hypot(a, b)
            if norm == 0:
                continue

            dists = np.abs(
                a * points[:, 0] + b * points[:, 1] + c
            ) / norm

            inliers = points[dists < threshold]

            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_model = (a, b, c)

        if best_model is None or len(best_inliers) < 20:
            return None, None

        return best_model, best_inliers

    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_vel_x = msg.linear.x



def main():
    rclpy.init()
    node = WallRansacInitialPose()
    rclpy.spin(node)
    rclpy.shutdown()
