# 真机驱动 + 感知设计 v2（以 YLR1D_Controller 为框架，复用主体）

> **v2 架构决策（用户确认）**：YLR1D_Controller 大部分框架保留复用——
> ① 感知层 = 新写**传感器层**（主动接收 SDK 信息，映射成框架关节名发给 `ylr1d_perception`）；
> ② 驱动层 = 新写 **robot_driver**，平替 `ylr1d_translate` 及以下，对外暴露与转译层同语义的 3 个 action，直调 SDK；
> ③ 规划层（ylr1d_plan_nav / ylr1d_plan_moveit）+ HMI（ylr1d_hmi）原样复用。
> 范围：底座运动 / 机械臂关节空间运动 / 夹爪 + 轮式里程计 / 关节两个传感器。三阶段全部完成 ✅

---

## 1. 总体架构（v2，三阶段完成）

```
[规划层 ylr1d_plan_nav / ylr1d_plan_moveit]（✅ 原样复用）
        |  nav：/cmd_vel -> cmd_vel_bridge -> /chassis_move
        |  moveit：/plan/moveit/moveit_move(action) -> moveit_goal_server -> moveit_bridge -> /arm_move + /gripper_move
        v
[驱动层 robot_driver]（✅ 平替 translate + control + algorithm + plant）
        |  直调 SDK：setMotionControl / moveABSJoint / setClawState / setServoState
        |  轮询 SDK 状态 -> /sensors/arm_raw、/sensors/vehicle_raw
        v
RobotConSys 服务端（真机 / 模拟器 @172.22.224.1:8109）
        ^
        |  状态轮询
[传感器层 robot_sensors]（✅ 主动接收 SDK 信息）
        |  SDK 数据 -> 框架 30 关节名 -> /joint_states
        v
[感知层 ylr1d_perception]（✅ 原样复用）
        |  /perception/sensors/joint_states、/perception/joint_state、/perception/odometry
        v
[HMI ylr1d_hmi]（✅ 原样复用：hmi_moveit 规划面板 / hmi_translate 手动面板）
```

## 2. 启动方式（✅ 已验证）

### 一键启动：规划层 moveit + HMI（推荐）

```bash
ros2 launch bringup real_robot_moveit.launch.py
```

等价：`real_robot.launch.py`（驱动+传感器+感知+rsp）+ `moveit.launch.py`（rviz:=true, use_sim_time:=false）+ `hmi_moveit.launch.py`。
实测节点/action（模拟器在线时）：

```
node list:  robot_driver, sensor_bridge, joint_state_receiver, joint_state_estimator,
            control_feedback_guard, sensor_dispatch, move_group, moveit_bridge,
            moveit_goal_server, ylr1d_hmi_moveit, robot_state_publisher ...
action list: /arm_move  /chassis_move  /gripper_move     （驱动层）
             /plan/moveit/moveit_move                    （规划层）
```

**注意**：move_group 在 WSL 下初始化 ~90s，HMI 窗口出现后等日志出现
"You can start planning now!" 再发目标。

### 手动控制（无需 moveit）

```bash
ros2 launch bringup real_robot_manual.launch.py
```

等价：`real_robot.launch.py` + `hmi_translate.launch.py`（转译面板直发
/chassis_move /arm_move /gripper_move 到驱动层，手动控底座/臂/夹爪）。

### 分层启动（调试用）

```bash
ros2 launch bringup real_robot.launch.py                      # 基线：驱动+传感器+感知+rsp
ros2 launch ylr1d_plan_moveit moveit.launch.py use_sim_time:=false   # 规划层 moveit（rviz:=false 可关）
ros2 launch ylr1d_hmi hmi_moveit.launch.py                    # HMI 规划面板
```

## 3. HMI 控制流（已实测的链路）

| HMI 面板 | 操作 | 链路 |
|---|---|---|
| hmi_moveit（MoveitPanel） | 输 part + 位姿/关节目标 + 夹爪模式，发送 | /plan/moveit/moveit_move → moveit_goal_server → moveit_bridge → /arm_move + /gripper_move → robot_driver → SDK |
| hmi_translate（TranslatePanel） | 底盘模式/臂关节/夹爪开合 | /chassis_move /arm_move /gripper_move → robot_driver → SDK |
| 状态显示 | 当前位姿 / 规划结果 / 关节 | /sensors/*_raw → sensor_bridge → /joint_states → ylr1d_perception → /perception/* |

实测（模拟器）：MoveItMove(part=1, [0.6,…], gripper close) → OMPL 规划 8 点 →
2 waypoint → /arm_move ×2 succeeded → /gripper_move succeeded → action SUCCEEDED "完成"。

## 4. 文件清单

```
src/drivers/robot_sensors/        # 传感器层：sensor_bridge
src/drivers/robot_driver/         # 驱动层：robot_driver + robot_sdk.hpp
src/drivers/robot_package/        # SDK 库 + demo（armDemo/vehicleDemo 发布 /sensors/*_raw）
src/bringup/launch/real_robot.launch.py        # 基线栈（driver+sensors+perception+rsp）
src/bringup/launch/real_robot_moveit.launch.py # 一键：基线+moveit+HMI
src/bringup/launch/real_robot_manual.launch.py # 一键：基线+手动面板
src/ThirdParty/YLR1D_Controller/  # 构建复用（零代码修改）：description/perception/translate/algorithm/plan_nav/plan_moveit/hmi
docs/real_robot_driver_design.md  # 本文档（v2）
```

## 5. 已知标定项（真机投入前）

1. **kTranslateScale=5**：cmd_vel_bridge 规划速度放大（仿真标定），真机改 1.0；
2. **轮式里程计标定**：odom twist 与命令速度差 ~8%（chassis_kinematics 参数按真机轮径/轮距修订）；
3. **torso（arm_move part=0）**：真机无独立躯干则拒绝；若有需映射 SDK 关节；
4. **导航**：未跑完整 nav2（无地图/amcl，场景无障碍按用户要求简单应付）；hmi_plan 面板需 nav2 就绪后才能用 NavigateToPose，当前用 hmi_translate/hmi_moveit 控制。