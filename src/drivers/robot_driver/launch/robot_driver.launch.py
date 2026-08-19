# 驱动层启动：3 action（chassis_move/arm_move/gripper_move）+ /sensors/*_raw + /health
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_driver', executable='robot_driver', name='robot_driver',
             output='screen',
             parameters=[{
                 'server_ip': '172.22.224.1',
                 'server_port': 8109,
                 'default_arm_vel': 0.2,
                 'servo_on_start': True,
                 'poll_rate': 20.0,
             }]),
    ])