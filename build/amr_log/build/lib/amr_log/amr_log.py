#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import time
from datetime import datetime
from datetime import datetime, timezone, timedelta
from std_msgs.msg import Float32
from std_msgs.msg import String



JST = timezone(timedelta(hours=9))

def to_timestr(sec: float) -> str:
    return datetime.fromtimestamp(sec, JST).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


class CmdVelSafetyLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_logger')

        # =========================
        # 起動時刻（ファイル名用）
        # =========================
        self.start_datetime = datetime.now()
        timestamp_str = self.start_datetime.strftime('%m-%d_%H-%M-%S')

        # =========================
        # Log file
        # =========================
        log_dir = os.path.expanduser('~/ros2_ws/src/amr_log/log_file')
        os.makedirs(log_dir, exist_ok=True)

        self.log_path = os.path.join(
            log_dir,
            f'{timestamp_str}.log'
        )
        self.log_file = open(self.log_path, 'a')

        self.get_logger().info(f'Safety log file: {self.log_path}')

        self.log_file.flush()

        # =========================
        # TF
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # =========================
        # Subscriber
        # =========================
        qos_cmd_vel = QoSProfile(depth=10)

        self.create_subscription(
            String,
            '/shelf_id',
            self.cb_shelf_id,
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cb_cmd_vel,
            qos_cmd_vel
        )
        # -------------------------
        # wall info cache
        # -------------------------
        self.wall_yaw_deg = None
        self.wall_distance = None

        # wall yaw (deg)
        self.create_subscription(
            Float32,
            '/wall_yaw_deg',
            self.cb_wall_yaw_deg,
            10
        )

        # wall distance (m)
        self.create_subscription(
            Float32,
            '/wall_distance',
            self.cb_wall_distance,
            10
        )

        self.save_requested = False
        self.shelf_id = "N/A"


        # --- logging state ---
        self.logging_active = False
        self.last_cmd_vel_time = None
        self.cmd_vel_timeout = 5.0   # ← 0.5秒来なかったら終了

        # 定期チェック用タイマー        
        self.create_timer(0.1, self.check_cmd_vel_timeout)


    def cb_shelf_id(self, msg: String):
        new_id = msg.data

        # ★ 同じIDが連続したら無視
        if new_id == self.shelf_id:
            return

        # ★ 進行中セッションがあれば終了
        if self.logging_active:
            self.finish_logging()

        # ★ 新セッション開始
        self.shelf_id = new_id
        self.logging_active = True
        self.last_cmd_vel_time = None

        self.log_file.write("\n[SESSION START]\n")
        self.log_file.write(f"shelf_id : {self.shelf_id}\n")
        self.log_file.write(
            f"start_time : {to_timestr(time.time())}\n"
        )
        self.log_file.write("=" * 50 + "\n")
        self.log_file.flush()

        self.get_logger().info(
            f"Logging started for shelf_id={self.shelf_id}"
        )

    def cb_wall_yaw_deg(self, msg: Float32):
        self.wall_yaw_deg = msg.data

    def cb_wall_distance(self, msg: Float32):
        self.wall_distance = msg.data

    def cb_cmd_vel(self, msg: Twist):
        if not self.logging_active:
            return

        stamp_cmd_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp_now_sec = time.time()

        time_cmd_str = to_timestr(stamp_cmd_sec)
        time_now_str = to_timestr(stamp_now_sec)

        # 最終受信時刻を更新
        self.last_cmd_vel_time = time.time()

        # ===== ここから下は今のログ処理そのまま =====
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.log_file.write("[CMD_VEL]\n")
        self.log_file.write(f"  linear.x  : {linear_x:.3f}\n")
        self.log_file.write(f"  angular.z : {angular_z:.3f}\n")
        self.log_file.flush()
        # -------------------------
        # TF 取得（map → odom フォールバック）
        # -------------------------
        tf = None
        frame_used = None
        tf_error = ""

        for frame in ['map', 'odom']:
            if self.tf_buffer.can_transform(
                frame,
                'base_link',
                Time(),
                timeout=Duration(seconds=0.05)
            ):
                try:
                    tf = self.tf_buffer.lookup_transform(
                        frame,
                        'base_link',
                        Time()
                    )
                    frame_used = frame
                    break
                except Exception as e:
                    tf_error = str(e)

        # -------------------------
        # ログ出力（必ず書く）
        # -------------------------
        # -------------------------
        # wall info
        # -------------------------
        self.log_file.write(f"shelf_id : {self.shelf_id}\n")

        self.log_file.write("wall_info:\n")

        if self.wall_yaw_deg is not None:
            self.log_file.write(
                f"  wall_yaw_deg : {self.wall_yaw_deg:.2f} deg\n"
            )
        else:
            self.log_file.write(
                "  wall_yaw_deg : N/A\n"
            )

        if self.wall_distance is not None:
            self.log_file.write(
                f"  wall_distance: {self.wall_distance:.3f} m\n"
            )
        else:
            self.log_file.write(
                "  wall_distance: N/A\n"
            )

        self.log_file.write("\n")
        
        self.log_file.write("\n" + "=" * 50 + "\n")
        self.log_file.write("[CMD_VEL EVENT]\n")
        self.log_file.write(f"time_cmd : {time_cmd_str}\n")
        self.log_file.write(f"time_now : {time_now_str}\n\n")

        self.log_file.write("cmd_vel:\n")
        self.log_file.write(f"  linear.x  : {linear_x:.3f} m/s\n")
        self.log_file.write(f"  angular.z : {angular_z:.3f} rad/s\n\n")

        if tf:
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )[2]
            yaw=180*yaw/3.1415

            self.log_file.write(
                f"pose ({frame_used} -> base_link):\n"
            )
            self.log_file.write(f"  x   : {x:.3f} m\n")
            self.log_file.write(f"  y   : {y:.3f} m\n")
            self.log_file.write(f"  yaw : {yaw:.3f} degree\n")
        else:
            self.log_file.write(
                "pose : UNAVAILABLE\n"
            )
            self.log_file.write(
                f"  reason : TF not available (map/odom)\n"
            )
            if tf_error:
                self.log_file.write(
                    f"  detail : {tf_error}\n"
                )

        self.log_file.write("=" * 50 + "\n")
        self.log_file.flush()
        

    def check_cmd_vel_timeout(self):
        if not self.logging_active:
            return

        if self.last_cmd_vel_time is None:
            return

        elapsed = time.time() - self.last_cmd_vel_time

        if elapsed > self.cmd_vel_timeout:
            self.finish_logging()

    def finish_logging(self):
        self.logging_active = False
        self.last_cmd_vel_time = None   # 次セッション用にリセット

        self.log_file.write("[SESSION END]\n")
        self.log_file.write(
            f"end_time : {to_timestr(time.time())}\n"
        )
        self.log_file.write("=" * 50 + "\n\n")
        self.log_file.flush()

        self.get_logger().info(
            f"Logging finished for shelf_id={self.shelf_id}"
        )



def main():
    rclpy.init()
    node = CmdVelSafetyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
