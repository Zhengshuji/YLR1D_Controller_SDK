# 真机栈基线：robot_driver + robot_sensors + ylr1d_perception + robot_state_publisher
# 全墙钟（无 /clock）；感知/驱动/传感器层自维护，不经仿真时钟。
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    desc = get_package_share_directory("ylr1d_description")
    sys.path.insert(0, os.path.join(desc, "launch", "python_utils"))
    from xacro_utils import process_xacro_to_urdf  # noqa: E402

    config_dir = os.path.join(desc, "config")
    robot_desc, _ = process_xacro_to_urdf(
        os.path.join(desc, "urdf", "ylr1d.xacro"),
        config_dir,
        os.path.join(config_dir, "controllers.yaml"),
    )

    # WSL 单机：loopback DDS，绕开 multicast 慢发现（CLAUDE 陷阱 22）
    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        # ── 驱动层（3 action + /sensors/*_raw + /health）──
        Node(package="robot_driver", executable="robot_driver",
             name="robot_driver", output="screen"),
        # ── 传感器层（SDK 原始 -> 框架 30 关节 /joint_states）──
        Node(package="robot_sensors", executable="sensor_bridge",
             name="sensor_bridge", output="screen"),
        # ── 感知层（ylr1d_perception 原样复用）──
        Node(package="ylr1d_perception", executable="joint_state_receiver",
             name="joint_state_receiver", output="screen"),
        Node(package="ylr1d_perception", executable="joint_state_estimator",
             name="joint_state_estimator", output="screen"),
        Node(package="ylr1d_perception", executable="control_feedback_guard",
             name="control_feedback_guard", output="screen"),
        Node(package="ylr1d_perception", executable="wheel_odometry",
             name="wheel_odometry", output="screen"),
        Node(package="ylr1d_perception", executable="sensor_dispatch",
             name="sensor_dispatch", output="screen"),
        # ── TF（真机 URDF 与框架模型一致；订阅我们的 /joint_states）──
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_desc}], output="screen"),
    ])