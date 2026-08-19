# 真机导航一键启动（现成 hmi_plan 控制）：
# 基线(驱动+传感器+感知+rsp) + odom_pub(速度积分->/odom+TF) + map_server(简单地图)
# + 静态 map->odom(恒等，无 amcl/激光) + nav2 核心(planner/controller/bt/behavior + lifecycle)
# + cmd_vel_bridge(/cmd_vel -> /chassis_move) + hmi_plan(NavigateToPose 面板)
# 全墙钟（use_sim_time:=false，nav2_real.yaml）
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _include(pkg: str, name: str):
    path = get_package_share_path(pkg) / "launch" / name
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def generate_launch_description():
    nav2_params = str(get_package_share_path("bringup") / "config" / "nav2_real.yaml")
    map_yaml = str(get_package_share_path("ylr1d_description") / "maps" / "nav_test.yaml")

    return LaunchDescription([
        _include("bringup", "real_robot.launch.py"),
        # 里程计适配：SDK 速度积分 -> /odom + odom->Link_Base TF
        Node(package="robot_sensors", executable="odom_pub", name="odom_pub", output="screen"),
        # 静态地图（无障碍场景：固定 map==odom，无 amcl/激光）
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen", parameters=[{"yaml_filename": map_yaml}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="static_map_odom", output="screen",
             arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        # nav2 核心（真机参数：use_sim_time=false，odom=/odom，base=Link_Base）
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "use_sim_time": False,
                 "autostart": True,
                 "node_names": ["map_server", "planner_server", "controller_server",
                                "bt_navigator", "behavior_server"],
             }]),
        # nav2 cmd_vel -> 转译层 /chassis_move -> 驱动（速度放大已改 1.0）
        Node(package="ylr1d_plan_nav", executable="cmd_vel_bridge",
             name="cmd_vel_bridge", output="screen"),
        # 现成规划 HMI：NavigateToPose + 导航状态
        _include("ylr1d_hmi", "hmi_plan.launch.py"),
    ])