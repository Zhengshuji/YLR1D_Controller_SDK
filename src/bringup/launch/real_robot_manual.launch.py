# 真机 + 手动控制 HMI 一键启动（全墙钟，无 /clock，无需 moveit）
# 等价：real_robot.launch.py + hmi_translate（转译面板：/chassis_move /arm_move /gripper_move
# 直发 robot_driver——手动控制底座/臂/夹爪）
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg: str, name: str):
    path = get_package_share_path(pkg) / "launch" / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description():
    return LaunchDescription([
        _include("bringup", "real_robot.launch.py"),
        _include("ylr1d_hmi", "hmi_translate.launch.py"),
    ])