# 真机 + 规划层 moveit + HMI 一键启动（全墙钟，无 /clock）
# 等价：real_robot.launch.py + moveit.launch.py(rviz:=true, use_sim_time:=false) + hmi_moveit
# 注意：move_group 在 WSL 下初始化 ~90s，HMI 出现后等 move_group "You can start planning now!"
# 再发 MoveItMove 目标。
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg: str, name: str, args=None):
    path = get_package_share_path(pkg) / "launch" / name
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        _include("bringup", "real_robot.launch.py"),
        _include("ylr1d_plan_moveit", "moveit.launch.py",
                 {"rviz": "true", "use_sim_time": "false"}),
        _include("ylr1d_hmi", "hmi_moveit.launch.py"),
    ])