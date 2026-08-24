# 真机 + 决策层一键启动（等价框架 bringup_decision：物理+感知+定位+转译+nav2+moveit+决策层）
# 真机平替：驱动+传感器（robot_driver + robot_sensors + rsp）替换 物理层+感知层；
# 导航组件（odom_pub+map_server+nav2+bridge）替换 定位+转译；moveit / 决策层 / 决策 HMI 原样。
# 组成：
#   bringup/real_robot_nav_core.launch.py  驱动+传感器+感知+rsp+导航核心（0s）
#   ylr1d_plan_moveit/moveit.launch.py     move_group + moveit_bridge + moveit_goal_server（0s，
#                                           WSL 下 move_group 初始化 ~90-150s，期间任务由决策层 BT Retry 兜底；
#                                           rviz 恒关——"只要一个 rviz"，与框架一致）
#   ylr1d_decision/decision.launch.py      决策层 composition（mission_server + decision + plan_client，5s）
#   rviz2_decision（顶层）                  单一决策 rviz（RobotModel + 决策/plan 状态面板，rviz:=true 时起）
#   ylr1d_hmi/hmi_decision.launch.py       决策 HMI（任务下拉框 + 参数表单，10s；其内部 rviz 恒关）
# 用法：ros2 launch bringup real_robot_decision.launch.py [rviz:=true] [hmi:=true]
import os

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _include(pkg: str, name: str, args=None):
    path = get_package_share_path(pkg) / "launch" / name
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=(args or {}).items(),
    )


def _decision_rviz_node():
    """单一决策 rviz：RobotModel + 决策/plan 状态面板（配置归属 description 层，用户可调布局）。"""
    desc = get_package_share_directory("ylr1d_description")
    rviz_cfg = os.path.join(desc, "rviz", "decision_display.rviz")
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_decision",
        output="screen",
        parameters=[{"use_sim_time": False}],
        arguments=["-d", rviz_cfg],
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
        condition=IfCondition(LaunchConfiguration("rviz")),
    )


def generate_launch_description():
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="起单一决策 rviz（RobotModel + 决策/plan 状态面板）")
    declare_hmi = DeclareLaunchArgument(
        "hmi", default_value="true",
        description="起决策 HMI（任务下拉框 + 参数表单，Qt 窗口需图形界面）")
    # A5：顶层统一注入 loopback（与既有 bringup 一致）
    set_loopback = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1")
    return LaunchDescription([
        set_loopback,
        declare_rviz,
        declare_hmi,
        # 单一决策 rviz 必须先于 moveit/nav 等 include——它们都声明同名 rviz 参数（默认 false），
        # include 共享 context 会覆盖顶层值（陷阱 16）；rviz 节点先起即不受后续影响（同框架 bringup_decision）
        _decision_rviz_node(),
        # 0s：驱动 + 传感器 + 感知 + rsp + 导航核心（nav2 慢初始化并行进行）
        _include("bringup", "real_robot_nav_core.launch.py"),
        # 0s：move_group + bridge + goal_server（rviz 恒关——单一决策 rviz 提供观测）
        _include("ylr1d_plan_moveit", "moveit.launch.py",
                 {"rviz": "false", "use_sim_time": "false"}),
        # 5s：决策层 composition
        TimerAction(period=5.0, actions=[
            _include("ylr1d_decision", "decision.launch.py",
                     {"use_sim_time": "false"}),
        ]),
        # 10s：决策 HMI（hmi:=true 时起；其内部 rviz 恒关，观测走顶层 rviz2_decision）
        TimerAction(period=10.0, actions=[
            _include("ylr1d_hmi", "hmi_decision.launch.py"),
        ], condition=IfCondition(LaunchConfiguration("hmi"))),
    ])