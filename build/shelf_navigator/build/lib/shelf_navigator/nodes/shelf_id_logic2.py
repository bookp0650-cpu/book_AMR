import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import Int32
from rcl_interfaces.msg import SetParametersResult



class ShelfIDLogic(Node):
    def __init__(self):
        super().__init__('shelf_id_logic')

        # --- 幾何パラメータ ---
        self.target_area = None
        self.CORRIDOR_TRANSITION = -1
        self.in_transition = False
        self.nav_client = ActionClient(
        self,
        NavigateToPose,
        'navigate_to_pose'
        )

        self.waypoint_client = ActionClient(
        self,
        FollowWaypoints,
        'follow_waypoints'
        )

        self.final_goal_pub = self.create_publisher(
            PoseStamped,
            '/final_goal_pose',
            10
        )

        self.corridor_pub = self.create_publisher(Int32, '/corridor_status', 10)
        self.corridor_timer = self.create_timer(0.1, self.publish_corridor_status)

        # --- callback登録 ---
        self.add_on_set_parameters_callback(self.param_cb)

        # --- param（先に宣言）---
        self.declare_parameter('force_initial_area', -1, ignore_override=True)
        self.declare_parameter('config_file', '')

        # --- config_file取得 ---
        yaml_path = self.get_parameter('config_file').value

        if yaml_path == "":
            raise RuntimeError("config_file is not set")

        self.get_logger().info(f"Using config: {yaml_path}")

        # --- YAML 読み込み ---
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # --- shelf ---
        self.shelf_length = config['shelf']['shelf_length']
        self.column_length = config['shelf']['column_length']
        self.half_column_length = self.column_length / 2
        self.pillar_width = config['shelf']['pillar_width']

        # --- nav ---
        self.x_offset = config['nav']['x_offset']
        self.nav_x_offset = config['nav']['nav_x_offset']

        # --- areas ---
        self.area_home = config['areas']

        # --- system（まずYAML）---
        self.current_area = config['system']['initial_area']

        # --- paramで上書き ---
        forced_area = self.get_parameter('force_initial_area').value

        if forced_area >= 1:
            self.current_area = forced_area
            self.get_logger().warn(f'Force override → {self.current_area}')
        else:
            self.get_logger().info(f'Initial area from YAML → {self.current_area}')

        self.get_logger().info('Loaded TOHAN.yaml')

        # --- Shelf ID Subscriber ---
        self.sub = self.create_subscription(
            String,
            '/shelf_id',
            self.cb_shelf_id,
            10
        )


    def param_cb(self, params):
        for param in params:
            if param.name == 'force_initial_area':
                new_area = param.value

                # 👇 修正ポイント
                if new_area == -1:
                    return SetParametersResult(successful=True)

                if new_area >= 1:
                    self.current_area = new_area
                    self.target_area = None
                    self.in_transition = False

                    self.get_logger().warn(
                        f'[PARAM] Force corridor updated → {self.current_area}'
                    )
                else:
                    return SetParametersResult(
                        successful=False,
                        reason='force_initial_area must be >= 1 or -1'
                    )

        return SetParametersResult(successful=True)

    def cb_shelf_id(self, msg: String):
        try:
            # --- ID パース ---
            A, B, C, D = map(int, msg.data.split('-'))
            self.get_logger().info(f'Received shelf ID: A={A}, B={B}, C={C}, D={D}')
            home = self.area_home[A]['home']

            # --- 区画判定 ---
            if A == self.current_area:
                self.get_logger().info('Same area → direct shelf approach')

                # 何列目か（2冊で1列想定なら）
                column_index = (B - 1) // 2
                # 左右判定
                is_left = (B % 2 == 0)
                base_x = self.x_offset + column_index * self.shelf_length

                if is_left:
                    # 偶数 → left
                    start_x = base_x + (self.column_length * D + self.pillar_width) - self.half_column_length
                else:
                    # 奇数 → right
                    start_x = (base_x + self.shelf_length) - self.column_length * D + self.half_column_length

                start_y = home['y']

                nav_x = start_x + self.nav_x_offset
                self.send_nav_goal(nav_x, start_y)


            else:
                self.get_logger().info(
                    f'Area change {self.current_area} → {A}'
                )

                # 今の区画ホーム
                cur_home = self.area_home[self.current_area]['home']
                self.get_logger().info(
                    f'Return to current home: {cur_home}'
                )

                # 移動先区画ホーム
                new_home = self.area_home[A]['home']
                self.get_logger().info(
                    f'Go to new area home: {new_home}'
                )


                # 区画更新
                #self.current_area = A
                self.target_area = A
                # --- 横通路用 yaw 決定 ---
                if A > self.current_area:
                    corridor_yaw = math.pi / 2
                else:
                    corridor_yaw = -math.pi / 2

                # 何列目か（2冊で1列想定なら）
                column_index = (B - 1) // 2
                # 左右判定
                is_left = (B % 2 == 0)
                base_x = self.x_offset + column_index * self.shelf_length
                
                if is_left:
                    # 偶数 → left
                    start_x = base_x + (self.column_length * D + self.pillar_width) - self.half_column_length
                else:
                    # 奇数 → right
                    start_x = (base_x + self.shelf_length) - self.column_length * D + self.half_column_length

                start_y = home['y']

                # --- Waypoints 作成 ---
                poses = [
                    #self.make_pose(cur_home['x'], cur_home['y'],0),
                    self.make_pose(cur_home['x'], cur_home['y'],corridor_yaw),
                    self.make_pose(new_home['x'], new_home['y'],0),
                    #self.make_pose(new_home['x'], new_home['y'], corridor_yaw),
                ]

                # --- FollowWaypoints に送信 ---
                self.send_waypoints(
                    poses,
                    start_x,
                    start_y
                )


            self.get_logger().info(
                f'Computed shelf goal → x={start_x:.3f}, y={start_y:.3f}'
            )

        except Exception as e:
            self.get_logger().error(f'Invalid shelf ID "{msg.data}": {e}')

    def send_nav_goal(self, x, y, yaw=0.0):
        self.last_goal = (x, y, yaw)   # 保存しておく
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x 
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_cb)

        self.get_logger().info(
            f'Sent Nav2 goal → x={x:.3f}, y={y:.3f}'
        )

    def send_waypoints(self, poses, final_x, final_y):
        self.pending_final_goal = (final_x, final_y)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.waypoint_client.wait_for_server()
        future = self.waypoint_client.send_goal_async(
            goal_msg,
            feedback_callback=self.waypoint_feedback_cb
        )

        future.add_done_callback(self.waypoint_response_cb)

        self.get_logger().info(
            f'Sent FollowWaypoints with {len(poses)} waypoints'
        )

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status

        if status == 4:  # SUCCEEDED
            self.get_logger().info('Nav2 goal succeeded')

            x, y, yaw = self.last_goal
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            self.final_goal_pub.publish(msg)

    def waypoint_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoints rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.waypoint_result_cb)

    def waypoint_result_cb(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Waypoints completed')

            # ★ ここで初めて区画を更新
            if self.target_area is not None:
                self.current_area = self.target_area
                self.get_logger().info(
                    f'Corridor updated → {self.current_area}'
                )
                self.target_area = None
                self.in_transition = False

            # 最終棚位置へ
            x, y = self.pending_final_goal
            nav_x = x + self.nav_x_offset
            final_yaw = 0.0   # 棚に正対（例）
            self.send_nav_goal(nav_x, y, final_yaw)

    def publish_corridor_status(self):
        msg = Int32()
        msg.data = self.current_area
        self.corridor_pub.publish(msg)

    
    def waypoint_feedback_cb(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint

        # waypoint[0] に「到達した後」だけ TRANSITION に入る
        if (
            self.target_area is not None
            and not self.in_transition
            and idx >= 1
        ):
            self.get_logger().info(
                'Arrived at current area home → TRANSITION'
            )
            self.current_area = self.CORRIDOR_TRANSITION
            self.in_transition = True




def main():
    rclpy.init()
    node = ShelfIDLogic()
    rclpy.spin(node)
    rclpy.shutdown()
