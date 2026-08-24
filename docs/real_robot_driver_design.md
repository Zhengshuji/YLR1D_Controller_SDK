# 真机驱动 + 感知设计（最终版 v3）

> 以 YLR1D_Controller（`src/ThirdParty/YLR1D_Controller`，同步自 `ros2_ylr1d_controller_ws`）为框架，对真实机器人（双机械臂 + 全向底盘 + 夹爪 + 躯干，RobotConSys 控制器 @ `172.22.224.1:8109`）实现控制。
> 已全部实现并实测通过：驱动层 / 传感器层 / moveit / 导航 / **决策层**。
> 真机 vs Gazebo 仿真对比见 `real_vs_gazebo.md`；README 为使用手册。

---

## 一、架构（最终）

```
[决策层 ylr1d_decision]（复用）
   /decision/mission -> BT(bt_xml/*.xml) -> plan_client
   -> /navigate_to_pose + /plan/moveit/moveit_move
        v
[规划层 ylr1d_plan_nav / ylr1d_plan_moveit + HMI ylr1d_hmi]（复用）
   moveit: /plan/moveit/moveit_move -> goal_server -> moveit_bridge -> /arm_move + /gripper_move
   nav:    NavigateToPose -> nav2 -> /cmd_vel -> cmd_vel_bridge -> /chassis_move
        v
[驱动层 robot_driver]（平替 translate+control+algorithm+plant）
   3 action（ylr1d_translate 类型）+ 内联主线程 SDK 调用 + 度/弧度换算 + 三伺服使能
        v
RobotConSys 服务端（真机 / SDK 仿真）
        ^
[传感器层 robot_sensors] -> 30 框架关节 /joint_states + odom_pub(/odom + TF)
        v
[感知层 ylr1d_perception]（复用）-> /perception/*
```

## 二、驱动层 robot_driver

- **3 action**（复用 ylr1d_translate 类型）：`/chassis_move`（mode/direction/speed/duration）、`/arm_move`（part+positions）、`/gripper_move`（part+open）；
- **SDK 调用约束（实测根因）**：
  1. **线程亲和**：运动指令（moveABSJoint + waitMotionCMDFinish）必须在**发起连接的线程**内联执行（worker 线程发会被忽略/卡死）；
  2. **单位制**：SDK 臂关节=**度**、末端=mm/度 → `robot_sdk.hpp` 边界换算（下发 rad→deg、上报 deg→rad、mm→m）；
  3. **伺服使能**：三组伺服（ARM_1/2/3）必须 setServoState(ON)，否则 moveABSJoint 返回 -1；
- **结构映射**（SDK 探测）：ARM_1 左臂(7) / ARM_2 右臂(7) / **ARM_3 躯干(4)**；`part 0→ARM_3 / 1→ARM_1 / 2→ARM_2`；
- **底盘**：平移用 `MOVE_XY`（全向 vx/vy，NORMAL 只认 vx/wz）、旋转 `ROTATE`、停车 NORMAL(0,0,0)；`cmd_vel_bridge` 速度放大 1.0、wz 直通；
- 健康 `/health`、启动回零（home_on_start，默认关）、零位移幂等。

## 三、传感器层 robot_sensors

- `sensor_bridge`：/sensors/arm_raw（arm0/1/2）+ /sensors/vehicle_raw → 30 框架关节 `/joint_states`（转向=servo_pos、轮=wheel_vel、臂=ARM_1/2、**躯干=ARM_3**、夹指=开合映射）；
- `odom_pub`：**命令速度死推**（SDK getVehicleState 不回报实际速度）→ `/odom` + odom→Link_Base TF。

## 四、里程计（实测根因）

- SDK `getVehicleState` 恒 0（无底盘速度反馈）→ 用**最后命令速度**（驱动记录）积分；
- 仿真执行忠实于命令，死推与实动一致；真机需按实际标定比例。

## 五、moveit / 导航 / 决策层

- **moveit**：模型与框架一致，`moveit.launch.py use_sim_time:=false`；move_group WSL 启动慢（1-2 分钟）→ bridge 服务等待重试 180s；躯干大轨迹（~178° 瞄准）→ goal_server 段超时 240s；
- **导航**：`real_robot_nav_core.launch.py` = 基线 + odom_pub + map_server(nav_test) + 静态 map=odom + nav2(planner/controller/bt/behavior) + cmd_vel_bridge（无 HMI，供 nav / decision 两 bringup 共用）；无激光 → 无 amcl；`real_robot_nav.launch.py` = nav_core + hmi_plan；
- **决策层**：`decision.launch.py use_sim_time:=false`（composition 三节点 mission_server/decision/plan_client）；Mission 任务 → BT → plan_client → 规划层两个 action；
  一键启动 **`real_robot_decision.launch.py`** = nav_core + moveit + decision + 决策 HMI + 单一决策 rviz（等价框架 `bringup_decision`，去掉 Gazebo/转译层；rviz 节点须在顶层、先于声明同名 rviz 参数的 include 启动——陷阱 16 参数污染）。

## 六、验证结果（SDK 仿真实测）

| 链路 | 结果 |
|---|---|
| 驱动层 | ✅ 底盘(平移/旋转/横移)/臂/躯干/夹爪全 SUCCEEDED；/joint_states 30 关节 20-33Hz |
| moveit | ✅ 臂关节/姿态(IK)/躯干瞄准/夹爪全链路 |
| nav | ✅ NavigateToPose SUCCESS |
| 决策层 | ✅ 三任务全 outcome=0：arm_move（左臂 +x 5cm）/ torso_aim（瞄 (0.6,0,1.2)）/ base_move（0.6m，odom→(0.42,0.02)） |

## 七、框架改动清单（最小集）

| 文件 | 改动 |
|---|---|
| `ylr1d_plan_moveit/moveit_bridge.cpp` | 服务等待 180s 重试（已并入仓库） |
| `ylr1d_plan_moveit/moveit_goal_server.cpp` | 段超时 240s（已并入仓库） |
| `ylr1d_plan_nav/cmd_vel_bridge.cpp` | 速度放大 1.0 + wz 直通（已并入仓库） |
| `ylr1d_hmi`（moveit_panel） | 曾加关节输入，已**回退** |

## 八、启动

```bash
# 1. 决策层全栈一键启动（推荐）：nav_core + moveit + decision + 决策 HMI + 单一决策 rviz
ros2 launch bringup real_robot_decision.launch.py      # 等 1-2 分钟 move_group + nav2 就绪

# 2. 导航（hmi_plan）
ros2 launch bringup real_robot_nav.launch.py

# 3. moveit + HMI（单臂规划，决策层单独叠）
ros2 launch bringup real_robot_moveit.launch.py
ros2 launch ylr1d_decision decision.launch.py use_sim_time:=false

# 4. 手动控制（3 action 直发）
ros2 launch bringup real_robot_manual.launch.py
```

**决策任务使用注意**：
- `base_move [dx,dy,dθ]`：增量建议 **≥0.5m**——nav2 xy_goal_tolerance 0.25m，0.2m 目标会被判"立即到达"（假成功不动）；
- `torso_aim [x,y,z]`：瞄点（map 系）须在**可达范围**——(1.0,0,0.6) 在倾角盲区（ray_error=0.065>3cm 判不可达），自检已知可达：(0.5,±0.2,0.8)/(0.8,0,1.0)/(0.3,0.3,0.6)/(0.6,0,1.2)；aim_distance 默认 0.5m，可经 `/plan/moveit/control` set_aim_distance [0.1,3.0] 调整；