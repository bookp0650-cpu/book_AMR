from launch import LaunchDescription
from launch_ros.actions import Node


PARAM_FILE = '/home/rover/ros2_ws/src/megarover3_ros2/megarover3_navigation/config/f120a_nav2_paramms.yaml'


def generate_launch_description():

    navigation_goal_sender = Node(
        package='udp_bridge',
        executable='navigation_goal_sender',
        name='navigation_goal_sender',
        parameters=[PARAM_FILE],
        output='screen'
    )

    shelf_id_receiver = Node(
        package='udp_bridge',
        executable='shelf_id_receiver',
        name='shelf_id_receiver',
        parameters=[PARAM_FILE],
        output='screen'
    )

    navigation_goal_final_sender = Node(
        package='udp_bridge',
        executable='navigation_goal_final_sender',
        name='navigation_goal_final_sender',
        parameters=[PARAM_FILE],
        output='screen'
    )

    error_x_receiver = Node(
        package='udp_bridge',
        executable='error_x_receiver',
        name='error_x_receiver',
        parameters=[PARAM_FILE],
        output='screen'
    )

    wall_distance_sender = Node(
        package='udp_bridge',
        executable='wall_distance_sender',
        name='wall_distance_sender',
        parameters=[PARAM_FILE],
        output='screen'
    )

    wall_yaw_deg_sender = Node(
        package='udp_bridge',
        executable='wall_yaw_deg_sender',
        name='wall_yaw_deg_sender',
        parameters=[PARAM_FILE],
        output='screen'
    )

    cmd_vel_receiver = Node(
        package='udp_bridge',
        executable='cmd_vel_receiver',
        name='cmd_vel_receiver',
        parameters=[PARAM_FILE],
        output='screen'
    )

    return LaunchDescription([
        navigation_goal_sender,
        navigation_goal_final_sender,
        shelf_id_receiver,
        error_x_receiver,
        wall_distance_sender,
        cmd_vel_receiver,
        wall_yaw_deg_sender
    ])