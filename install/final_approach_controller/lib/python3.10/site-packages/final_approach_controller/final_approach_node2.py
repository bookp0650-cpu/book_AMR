import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
import math
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Float32


class FinalApproachController(Node):
    def __init__(self):
        super().__init__('final_approach_controller')
        self.wall_ref = 0.4       # 目標壁距離 [m]ここは変えるところ！野田図書館:0.47,２号館:0.40
        self.phase = 'POSITION'   # POSITION / ANGLE / DONE
        self.goal_yaw = 0.0       # ← 最終的に向きたい角度（棚正対）
        self.EPS_POS = 0.001       # 2cm 不感帯
        self.max_v_back = 0.02    # 後退は遅く
        self.angle_ok_count = 0
        self.ANGLE_OK_N = 20        # ← 3回連続でOKなら終了
        self.ANGLE_TOL_DEG = 3.0  # ±3 deg
        self.kd_ang = 0.002     # ← Dゲイン（まずは小さく）
        self.prev_yaw_err = None
        self.prev_time = None

        self.kp_lin = 0.1
        self.kp_ang1 = -0.01 #角度のみの制御ゲイン【位置合わせ】
        self.kp_ang3 = -0.01 #角度のみの制御ゲイン【角度合わせ】
        self.kp_ang2 = 0.1 #壁との距離ゲイン【位置合わせ】
        self.max_v = 0.05
        self.max_w = 0.07
        self.goal = None

        self.kp_wall_dist = 0.5    # ← 小さめ
        self.max_w_dist = 0.03      # ← yawよりかなり小さく


        # Subscriber: 最終ゴール
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/final_goal_pose',
            self.goal_cb,
            10
        )

        # Publisher: cmd_vel
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.nav_result_pub = self.create_publisher(
            Bool,
            '/navigation_goal',
            10
        )

        self.wall_yaw_deg = None

        self.create_subscription(
            Float32,
            '/wall_yaw_deg',
            self.wall_yaw_cb,
            10
        )

        self.wall_distance = None

        self.create_subscription(
            Float32,
            '/wall_distance',
            self.wall_distance_cb,
            10
        )

        self.result_sent = False     # ★ true/false を二重送信しないため
        self.start_time = None       # ★ ゴール受信時刻
        self.TIMEOUT = 500.0          # ★ 30秒で失敗扱い（必要なら調整）

        # TF
        self.tf_fail_cnt = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )
        # 制御ループ 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('FinalApproachController started')

    def wall_distance_cb(self, msg: Float32):
        self.wall_distance = msg.data   # [m]


    def wall_yaw_cb(self, msg: Float32):
        self.wall_yaw_deg = msg.data   # deg

    def goal_cb(self, msg: PoseStamped):
        self.tf_fail_cnt = 0
        self.goal = msg
        self.phase = 'POSITION'
        self.result_sent = False
        self.start_time = self.get_clock().now()

        q = msg.pose.orientation
        _, _, self.goal_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        self.get_logger().info(
            f'Final goal received: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, '
            f'yaw={math.degrees(self.goal_yaw):.1f} deg'
        )



    def control_loop(self):

        if self.phase == 'DONE':
            return

        if self.goal is None:
            return

        # ===== timeout 判定 =====
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed > self.TIMEOUT:
                self.get_logger().warn('[FA] Final approach timeout → FAILED')
                self.stop_robot()
                self.goal = None
                self.phase = 'DONE'

                if not self.result_sent:
                    self.publish_result(False)
                return    
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            self.tf_fail_cnt = 0   # 成功時にリセット

        except Exception:
            self.tf_fail_cnt += 1

            if self.tf_fail_cnt > 40:   # 約1秒（20Hz）
                self.get_logger().warn('[FA] TF lookup failed → FAILED')
                self.stop_robot()
                self.goal = None
                self.phase = 'DONE'

                if not self.result_sent:
                    self.publish_result(False)
            return


        cur_x = tf.transform.translation.x
        cur_y = tf.transform.translation.y

        q = tf.transform.rotation
        _, _, cur_yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        dx = gx - cur_x
        dy = gy - cur_y
        # 狭路前進専用：距離は前後方向(dx)のみ評価
        dist = dx #math.hypot(dx, dy)
        pos_err = dist   
        cmd = Twist()

        # =========================
        # Phase 1：位置合わせ
        # =========================
        if self.phase == 'POSITION':
            if abs(dist) < 0.03:
                self.phase = 'ANGLE'
                self.angle_ok_count = 0

                # ★ PD用リセット（これが無いと暴れる）
                self.prev_yaw_err = None
                self.prev_time = None

                return


            # =====================
            # 並進速度（前後＋不感帯）
            # =====================
            if dist > self.EPS_POS:
                # まだ足りない → 前進
                v = self.kp_lin * dist
                v = min(v, self.max_v)

            elif dist < -self.EPS_POS:
                # 行きすぎ → 後退
                v = self.kp_lin * dist
                v = max(v, -self.max_v_back)

            else:
                # 十分近い → 止まる
                v = 0.0

            # =====================
            # yaw（壁向き）＋距離（平行ズレ）による角度補正
            # =====================
            w = 0.0

            # --- yaw 主制御 ---
            if self.wall_yaw_deg is not None:
                yaw_err_rad = math.radians(self.wall_yaw_deg)
                w_yaw = self.kp_ang1 * yaw_err_rad
            else:
                w_yaw = 0.0

            # --- distance 副制御 ---
            if self.wall_distance is not None:
                err_dist = self.wall_distance - self.wall_ref
                w_dist = self.kp_wall_dist * err_dist
                w_dist = max(-self.max_w_dist, min(self.max_w_dist, w_dist))
            else:
                w_dist = 0.0

            # --- 合成 ---
            w = w_yaw + w_dist
            w = max(-self.max_w, min(self.max_w, w))

            cmd.linear.x = v
            cmd.angular.z = w

            # デバッグ
            self.debug_cnt = getattr(self, 'debug_cnt', 0) + 1
            if self.debug_cnt % 20 == 0:
                self.get_logger().info(
                    f'[FA] POSITION '
                    f'dist={dist:.3f} m '
                    f'v={v:.3f} '
                    f'yaw={self.wall_yaw_deg:.2f} deg '
                    f'wall_dist={self.wall_distance:.3f} m '
                    f'w={w:.3f}'
                )

            self.cmd_pub.publish(cmd)
            return

        # =========================
        # Phase 2：角度合わせ（壁基準・並進なし）
        # =========================
        if self.phase == 'ANGLE':

            # 壁が見えていなければ回らない（安全）
            if self.wall_yaw_deg is None:
                self.stop_robot()
                return

            # 壁に対する角度誤差（rad）
            yaw_err = math.radians(self.wall_yaw_deg)

            # ---- ANGLE 安定判定（連続OK）----
            yaw_err_deg = abs(self.wall_yaw_deg)

            if yaw_err_deg < self.ANGLE_TOL_DEG:
                self.angle_ok_count += 1
            else:
                self.angle_ok_count = 0   # 外れたらリセット

            # ---- 完了判定 ----
            if self.angle_ok_count >= self.ANGLE_OK_N:
                self.stop_robot()
                self.phase = 'DONE'
                self.goal = None

                if not self.result_sent:
                    self.publish_result(True)

                self.get_logger().info(
                    f'[FA] ANGLE DONE (stable {self.ANGLE_OK_N} cycles)'
                )
                return
       
            # ===== PD制御 =====
            now = self.get_clock().now().nanoseconds * 1e-9

            if self.prev_yaw_err is not None and self.prev_time is not None:
                dt = now - self.prev_time
                if dt > 0.0:
                    yaw_err_dot = (yaw_err - self.prev_yaw_err) / dt
                else:
                    yaw_err_dot = 0.0
            else:
                yaw_err_dot = 0.0

            w = self.kp_ang3 * yaw_err + self.kd_ang * yaw_err_dot

            # 状態更新
            self.prev_yaw_err = yaw_err
            self.prev_time = now



            # ===== 最小角速度制限（publish直前）=====
            MIN_W = 0.01
            if w != 0.0 and abs(w) < MIN_W:
                w = math.copysign(MIN_W, w) 
 
            # 上限制限
            w = max(-self.max_w, min(self.max_w, w))

            # ===== デバッグログ =====
            self.debug_cnt = getattr(self, 'debug_cnt', 0) + 1
            if self.debug_cnt % 20 == 0:   # 約1秒に1回
                self.get_logger().info(
                    f'[FA] ANGLE '
                    f'wall_yaw={self.wall_yaw_deg:.2f} deg '
                    f'yaw_err={math.degrees(yaw_err):.2f} deg '
                    f'w={w:.3f}'
                )

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            return

    #@staticmethod

    def stop_robot(self):
        stop = Twist()
        for _ in range(5):        #  5回くらい打つ
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.02)


    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def publish_result(self, success: bool):
        msg = Bool()
        msg.data = success
        self.nav_result_pub.publish(msg)
        self.result_sent = True

        self.get_logger().info(
            f'[FA] navigation_goal = {success}'
        )


def main():
    rclpy.init()
    node = FinalApproachController()
    rclpy.spin(node)
    rclpy.shutdown()
