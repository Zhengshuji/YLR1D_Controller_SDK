# 传感器层启动：SDK 原始话题 -> 框架 /joint_states（供 ylr1d_perception）
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_sensors', executable='sensor_bridge',
             name='sensor_bridge', output='screen'),
    ])