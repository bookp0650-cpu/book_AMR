import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
import math
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool



class FinalApproachController(Node):
    def __init__(self):
        super().__init__('final_approach_controller')

        self.phase = 'POSITION'   # POSITION / ANGLE / DONE
        self.goal_yaw = 0.0       # ← 最終的に向きたい角度（棚正対）


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

        self.result_sent = False     # ★ true/false を二重送信しないため
        self.start_time = None       # ★ ゴール受信時刻
        self.TIMEOUT = 30.0          # ★ 30秒で失敗扱い（必要なら調整）

        # TF
        self.tf_fail_cnt = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )

        # パラメータ
        self.kp_lin = 0.2
        self.kp_ang1 = 0.005
        self.kp_ang2 = 0.2
        self.max_v = 0.04
        self.max_w = 0.05
        self.goal = None

        # 制御ループ 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('FinalApproachController started')

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
                'capture_link',
                rclpy.time.Time()
            )
            self.tf_fail_cnt = 0   # 成功時にリセット

        except Exception:
            self.tf_fail_cnt += 1

            if self.tf_fail_cnt > 20:   # 約1秒（20Hz）
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

            # ===== 狭路用 POSITION（旋回なし）=====

            # ゴール十分近い → ANGLE へ
            if dist < 0.03:
                self.phase = 'ANGLE'
                self.cmd_pub.publish(Twist())
                return

            # 距離比例の速度
            v = self.kp_lin * dist

            # ロボット前方ベクトル
            fx = math.cos(cur_yaw)
            fy = math.sin(cur_yaw)

            # ゴール方向ベクトル
            gx_vec = dx
            gy_vec = dy

            # 内積（前後判定）
            dot = fx * gx_vec + fy * gy_vec

            # ゴールが後ろなら後退
            if dot < 0:
                v = -v

            # 速度制限
            v = max(-self.max_v, min(self.max_v, v))

            # ★ 旋回は完全に禁止
            cmd.linear.x = v
            cmd.angular.z = 0.0
           
            # ===== 状態サマリログ（1Hz）=====
            self.debug_cnt = getattr(self, 'debug_cnt', 0) + 1
            if self.debug_cnt % 20 == 0:

                self.get_logger().info(
                    f'[FA] phase=POSITION '
                    f'pos_err={pos_err:.3f} m '
                    f'dot={dot:.3f} '
                    f'v={v:.3f}'
                )
            # ================================

            self.cmd_pub.publish(cmd)
            return

        # =========================
        # Phase 2：角度合わせ
        # =========================
        if self.phase == 'ANGLE':
            yaw_err = math.atan2(
                math.sin(self.goal_yaw - cur_yaw),
                math.cos(self.goal_yaw - cur_yaw)
            )       


            if abs(yaw_err) < 5.0 * math.pi / 180.0:
                self.stop_robot()
                self.phase = 'DONE'
                self.goal = None

                if not self.result_sent:
                    self.publish_result(True)

                self.get_logger().info(
                    f'Final approach completed (yaw={math.degrees(yaw_err):.2f} deg)'
                )
                return

            w = self.kp_ang2 * yaw_err

            MIN_W = 0.05
            if abs(w) < MIN_W:
                w = math.copysign(MIN_W, w)

            w = max(-self.max_w, min(self.max_w, w))

            cmd.linear.x = 0.0
            cmd.angular.z = w

            self.debug_cnt = getattr(self, 'debug_cnt', 0) + 1
            if self.debug_cnt % 20 == 0:
                self.get_logger().info(
                    f'[FA] phase=ANGLE '
                    f'yaw_err={math.degrees(yaw_err):.1f} deg '
                    f'w={w:.3f}'
                )

            self.cmd_pub.publish(cmd)

    #@staticmethod

    def stop_robot(self):
        stop = Twist()
        for _ in range(5):        # ★ 5回くらい打つ
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
