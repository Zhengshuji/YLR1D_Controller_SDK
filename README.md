# YLR1D 真机控制系统（RobotConSys SDK）

以 YLR1D_Controller（`src/ThirdParty/YLR1D_Controller`）为框架，对真实机器人（双机械臂 + 全向底盘 + 夹爪 + 躯干，RobotConSys 控制器 @ `172.22.224.1:8109`，SDK 自带仿真可驱动）实现控制。框架主体复用（感知/规划/moveit/HMI/nav2 原样），**驱动层平替「转译+控制+算法+plant」**。

对比文档：真机控制系统 vs Gazebo 仿真，见 `docs/real_vs_gazebo.md`。

---

## 一、架构

```
[规划层 ylr1d_plan_nav / ylr1d_plan_moveit + HMI ylr1d_hmi]（复用）
   moveit: /plan/moveit/moveit_move(action) -> goal_server -> moveit_bridge -> /arm_move + /gripper_move
   nav:    NavigateToPose -> nav2 -> /cmd_vel -> cmd_vel_bridge -> /chassis_move
        v
[驱动层 robot_driver]（平替 translate+control+algorithm+plant）
   3 action（ylr1d_translate 类型）+ 内联主线程 SDK 调用 + 度/弧度换算 + 三伺服使能
   直调 SDK：moveABSJoint / setMotionControl / setClawState / setServoState
        v
RobotConSys 服务端（真机 / SDK 仿真）
        ^
[传感器层 robot_sensors] -> 30 框架关节 /joint_states + odom_pub(/odom + TF 死推)
        v
[感知层 ylr1d_perception]（复用）-> /perception/*
```

## 二、包清单

| 包 | 位置 | 职责 |
|---|---|---|
| `robot_driver` | `src/drivers/robot_driver/` | 驱动层：3 action + SDK 直调 + `/sensors/*_raw` + `/health` |
| `robot_sensors` | `src/drivers/robot_sensors/` | 传感器层：`sensor_bridge`（→/joint_states）+ `odom_pub`（速度积分→/odom+TF） |
| `robot_package` | `src/drivers/robot_package/` | SDK 库 + demo（armDemo/vehicleDemo 已修限位内目标） |
| `bringup` | `src/bringup/` | 一键启动 + `config/nav2_real.yaml`（真机 nav2 参数） |
| `ylr1d_*` | `src/ThirdParty/YLR1D_Controller/src/` | 复用（perception/translate(类型)/plan_nav/plan_moveit/hmi/description） |

## 三、启动

> 前置：仿真/真机服务端在线；WSL 桌面（WSLg）；`source install/setup.bash`。

```bash
# 1. 规划层 moveit + HMI（臂/躯干/夹爪，姿态或关节目标）
ros2 launch bringup real_robot_moveit.launch.py     # 等 1-2 分钟 move_group 就绪

# 2. 导航（hmi_plan，NavigateToPose）
ros2 launch bringup real_robot_nav.launch.py

# 3. 手动控制（3 action 直发）
ros2 launch bringup real_robot_manual.launch.py
```

## 四、控制

### HMI
- **MoveitPanel**（moveit）：part 0=躯干 1=左臂 2=右臂 + 位姿（Link_Base 系，IK）或关节目标 + 夹爪模式；
- **PlanPanel**（nav）：NavigateToPose 目标 + 导航状态。

### CLI（等价）
```bash
ros2 action send_goal /chassis_move ylr1d_translate/action/ChassisMove "{mode: 0, direction: 0.0, speed: 0.2, duration: 3.0}" --feedback
ros2 action send_goal /arm_move ylr1d_translate/action/ArmMove "{part: 1, positions: [0.5, -0.3, 0.2, 0, 0, 0, 0]}" --feedback
ros2 action send_goal /arm_move ylr1d_translate/action/ArmMove "{part: 0, positions: [0.1, 0.3, -0.2, 0]}" --feedback   # 躯干
ros2 action send_goal /gripper_move ylr1d_translate/action/GripperMove "{part: 0, open: true}" --feedback
ros2 action send_goal /plan/moveit/moveit_move ylr1d_plan_moveit/action/MoveItMove "{part: 1, joint_positions: [0.8, -0.4, 0.3, 0, 0, 0, 0], gripper_mode: 1}" --feedback
```

## 五、已验证（SDK 仿真实测）

| 链路 | 结果 |
|---|---|
| 连接 + 结构探测 | ✅ ARM_1 左臂7 / ARM_2 右臂7 / ARM_3 躯干4；三组伺服使能 |
| 传感器层 | ✅ /joint_states 30 关节 20-33Hz，等长 |
| 底盘 | ✅ 平移(MOVE_XY)/旋转/横移全通；/odom 死推积分 |
| 臂/躯干/夹爪 | ✅ 动作全 SUCCEEDED，状态回显与命令弧度一致 |
| moveit | ✅ 臂关节目标/臂姿态(IK)/躯干瞄准/夹爪全链路 SUCCEEDED |
| nav | ✅ NavigateToPose SUCCESS，底盘实际移动 |

## 六、排障记录（三个根因，均已实锤修复）

1. **SDK 指令线程亲和**：`moveABSJoint` 必须在发起连接的线程（主线程）发出；worker 线程发会被忽略/卡死 → 驱动内联主线程执行。
2. **单位制**：SDK 臂关节=度、末端=mm/度；框架/moveit=弧度+米 → `robot_sdk.hpp` 边界统一换算（下发 rad→deg、上报 deg→rad、mm→m）。这也是早期 OMPL "invalid bounds" 的真相。
3. **底盘无速度反馈**：SDK `getVehicleState` 不回报实际速度（恒 0）→ 里程计按**最后命令速度死推**（与速度控制对齐）。

另：`cmd_vel_bridge` 速度放大 5.0→1.0、wz 取反移除（仿真标定，真机不需要）；`moveit_goal_server` 段超时 90→240s（躯干大轨迹）；`moveit_bridge` 服务等待 180s 重试。

## 七、框架改动清单（最小集）

| 文件 | 改动 |
|---|---|
| `cmd_vel_bridge.cpp` | 速度放大 1.0、wz 直通 |
| `moveit_bridge.cpp` | 服务等待 180s 重试 |
| `moveit_goal_server.cpp` | 段超时 240s |
| `ylr1d_hmi`（moveit_panel） | 已回退（恢复原样） |

## 八、真机投入前标定项

1. 里程计死推比例（命令速度 vs 实际位移，当前 nav 到位 ~0.95m/1.5m 目标）；
2. SDK +wz 旋转方向（当前按逆时针直通，若实测相反改回取反）；
3. 位姿 IK 可达性（躯干转过后臂可达空间变化，选当前可达位姿）。

## 九、测试脚本（`tmp/`）

```bash
bash tmp/verify_all.sh   # 驱动层全链路验证（11+ 项 PASS）
bash tmp/test_moveit_final.sh / tmp/moveit_pose_goal.py   # moveit
bash tmp/test_nav2.sh / tmp/nav_goal_send.py              # nav
```