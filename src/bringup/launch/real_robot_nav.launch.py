# 真机导航一键启动（现成 hmi_plan 控制）：
# nav_core(基线+里程计+地图+nav2+bridge) + hmi_plan(NavigateToPose 面板)
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(pkg: str, name: str):
    path = get_package_share_path(pkg) / "launch" / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description():
    return LaunchDescription([
        _include("bringup", "real_robot_nav_core.launch.py"),
        # 现成规划 HMI：NavigateToPose + 导航状态
        _include("ylr1d_hmi", "hmi_plan.launch.py"),
    ])