# 真机控制系统 vs YLR1D_Controller Gazebo 仿真：区别与联系

> 本文对比两套系统的控制实现：**Gazebo 仿真**（YLR1D_Controller 原版，src/ThirdParty）与**真机控制系统**（本工作区，驱动层 + 传感器层 + 复用框架）。

---

## 一、联系（共同点）

两套系统共享同一套**接口契约与上层框架**：

| 共用资产 | 说明 |
|---|---|
| 3 个转译 action | `/chassis_move` `/arm_move` `/gripper_move`（ylr1d_translate 类型）——上层接口完全一致 |
| 规划层 moveit | `/plan/moveit/moveit_move`（MoveItMove action）+ moveit_bridge/goal_server 原样复用 |
| 规划层 nav | `NavigateToPose` + nav2（planner/controller/bt/behavior）+ cmd_vel_bridge 原样复用 |
| HMI | MoveitPanel / PlanPanel（ylr1d_hmi）原样复用 |
| 感知层 | ylr1d_perception 原样复用（清洗/估计/里程计），输入都是 30 关节 `/joint_states` |
| **模型** | 真机模型文件 = 框架 URDF（用户确认），moveit_config / 关节名 / 分组全部一致 |
| 状态出口 | `/joint_states`（30 框架关节名）、`/perception/*` ——两边共同语言 |

**一句话**：真机系统 = 框架的「规划层 + 感知层 + HMI」原样，把「转译层下游的仿真执行链」替换为 SDK 直驱，并在中间加了一层「传感器桥」（SDK 状态 → 框架 30 关节）。

## 二、区别（逐层对照）

| 维度 | Gazebo 仿真（原版） | 真机（本系统） |
|---|---|---|
| **执行链** | 转译层 → **控制层**（采样保持）→ **算法层**（PID 算加速度）→ **plant**（SimPlant 积分）→ Gazebo | **驱动层直接调 SDK**（moveABSJoint / setMotionControl / setClawState）；转译/控制/算法/plant **整段不存在** |
| **闭环位置** | ROS 内：算法层 PID 以期望-反馈算加速度，plant 积分成运动 | SDK/真机控制器内：轨迹规划 + 伺服闭环；ROS 侧只发目标、不做控制律 |
| **时钟** | 仿真时钟 `use_sim_time=true`（/clock） | 墙钟 `use_sim_time=false`（无 /clock） |
| **单位制** | 弧度 + 米（URDF/Gazebo 原生） | SDK 臂关节=**度**、末端=**mm/度**；`robot_sdk.hpp` 边界换算（下发 rad→deg、上报 deg→rad、mm→m） |
| **SDK 调用约束** | 无 | ①运动指令必须在**发起连接的线程**发出（worker 线程被忽略）；②需 `setServoState` 使能（含 ARM_3 躯干，否则 moveABSJoint 返回 -1）；③权限 OPERATOR |
| **里程计** | wheel_odometry（转向/轮 LS）→ **EKF**(robot_localization) → /odom+TF | SDK **不回报底盘速度**（getVehicleState 恒 0）→ **命令速度死推**积分 → /odom + odom→Link_Base TF（无 EKF） |
| **定位** | amcl + 激光 /scan + 地图 + EKF | 无激光 → **静态 map=odom**（恒等 TF），无 amcl；地图用 nav_test |
| **moveit 可用性** | 直接可用 | WSL 下 move_group 初始化慢（1-2 分钟）→ bridge 服务等待重试 180s；躯干大轨迹（~178° 瞄准）>90s → 段超时放宽 240s |
| **传感器** | 27 路 Gazebo 传感器（相机/雷达/IMU/超声） | 仅关节（三组）+ 底盘状态（SDK 上报）；相机/雷达暂无数据（`/perception/health` 相应 unavailable 属预期） |
| **结构映射** | 30 关节全在 URDF | SDK 分组：ARM_1 左臂(7)、ARM_2 右臂(7)、**ARM_3 躯干(4)**；ARM_4 探测会卡死服务端（勿调用） |
| **速度放大** | cmd_vel_bridge 平移×5/旋转×2.5（DWB 输出偏小补偿） | 已改 **1.0**（真机命令即真实速度） |
| **wz 方向** | bridge 取反（转译层 ROTATE 正 speed=顺时针的仿真标定） | 已改**直通**（SDK +wz=逆时针，与 nav2 ROS 约定一致） |

## 三、共性问题与注意事项

1. **"动作成功"≠"真动"**：真机链路早期踩坑——SDK 只回显目标状态（getRobotMotion 返回期望而非实际），导致 action 报成功但仿真界面不动；最终判定以**仿真界面/真机实动**为准，勿只信日志/状态回显。
2. **/joint_states 是两边共同语言**：真机侧由传感器层把 SDK 状态映射为框架 30 关节名（转向=servo_pos、轮=wheel_vel、臂=ARM_1/2、躯干=ARM_3、夹指=夹爪开合），感知/规划/HMI 零感知差异。
3. **模型一致但语义有差异**：URDF 与 SDK 模型一致（关节数/分组），但单位制与部分约定（伺服使能、权限、线程）不同——真机侧统一在驱动层适配，上层不动。
4. **nav2/moveit 参数大部分复用**：仅时钟（use_sim_time=false）、里程计源（/odom 死推）、定位（静态 map=odom）差异，其余（costmap/DWB/BT/规划组）沿用框架配置。

## 四、一句话总结

- **联系**：同一套决策友好接口（3 action + MoveItMove + NavigateToPose）、同一模型、同一感知/HMI/规划框架。
- **区别**：真机把「ROS 内闭环仿真链」换成「SDK 直驱（闭环在控制器内）」，并适配了 SDK 的单位制、线程亲和、伺服使能、无速度反馈（死推里程计）、无激光（静态定位）这些真机特性。