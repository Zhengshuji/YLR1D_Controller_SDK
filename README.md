# YLR1D 真机控制系统（RobotConSys SDK）

以 YLR1D_Controller（`src/ThirdParty/YLR1D_Controller`，同步自 `ros2_ylr1d_controller_ws`）为框架，对真实机器人（双机械臂 + 全向底盘 + 夹爪 + 躯干，RobotConSys 控制器 @ `172.22.224.1:8109`，SDK 自带仿真可驱动）实现控制。框架主体复用（感知/规划/moveit/HMI/nav2/**决策层**原样），**驱动层平替「转译+控制+算法+plant」**。

文档导航：
- `docs/real_vs_gazebo.md` —— 真机控制系统 vs Gazebo 仿真（区别与联系）
- `docs/real_robot_driver_design.md` —— 最终设计（驱动/传感器/里程计/moveit/nav/决策层）
- `docs/real_robot_remediation.md` —— 整改过程记录（三个根因）
- `src/ThirdParty/YLR1D_Controller/CLAUDE.md` —— 框架（Gazebo 仿真项目）开发备忘

---

## 一、架构

```
[决策层 ylr1d_decision]（复用，2026-08 同步）
   Mission(/decision/mission) -> BT 引擎(bt_xml/*.xml) -> plan_client
   -> /navigate_to_pose + /plan/moveit/moveit_move
        v
[规划层 ylr1d_plan_nav / ylr1d_plan_moveit + HMI ylr1d_hmi]（复用）
   moveit: /plan/moveit/moveit_move(action) -> goal_server -> moveit_bridge -> /arm_move + /gripper_move
   nav:    NavigateToPose -> nav2 -> /cmd_vel -> cmd_vel_bridge -> /chassis_move
        v
[驱动层 robot_driver]（平替 translate+control+algorithm+plant）
   3 action（ylr1d_translate 类型）+ 内联主线程 SDK 调用 + 度/弧度换算 + 三伺服使能
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
| `robot_sensors` | `src/drivers/robot_sensors/` | 传感器层：`sensor_bridge` + `odom_pub` |
| `robot_package` | `src/drivers/robot_package/` | SDK 库 + demo |
| `bringup` | `src/bringup/` | 一键启动 + nav2 真机参数 |
| `ylr1d_*` | `src/ThirdParty/YLR1D_Controller/src/` | 复用（含 **ylr1d_decision** 决策层） |

## 三、启动

> 前置：仿真/真机服务端在线；WSL 桌面（WSLg）；`source install/setup.bash`。

```bash
# 1. 决策层全栈一键启动（等价框架 bringup_decision：驱动/感知/导航核心 + moveit + 决策层
#    + 决策 HMI + 单一决策 rviz；三个任务 arm_move / torso_aim / base_move 均可用）
ros2 launch bringup real_robot_decision.launch.py          # 等 1-2 分钟 move_group + nav2
#     torso_aim 瞄点须在可达范围（如 0.6,0,1.2）；base_move 增量建议 ≥0.5m（nav2 容差 0.25m）

# 2. 导航（hmi_plan）
ros2 launch bringup real_robot_nav.launch.py

# 3. moveit + HMI（单臂规划）
ros2 launch bringup real_robot_moveit.launch.py            # 等 1-2 分钟 move_group
ros2 launch ylr1d_decision decision.launch.py use_sim_time:=false

# 4. 手动控制（3 action 直发）
ros2 launch bringup real_robot_manual.launch.py
```

## 四、控制

### 决策层（Mission action，任务编排）
```bash
# 臂末端相对移动：part=1(左臂) +x 5cm
ros2 action send_goal /decision/mission ylr1d_decision/action/Mission \
  "{task: 'arm_move', args: [1, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]}" --feedback
# 底座相对移动：[dx, dy, dθ]（需导航栈）
ros2 action send_goal /decision/mission ylr1d_decision/action/Mission \
  "{task: 'base_move', args: [1.0, 0.0, 0.0]}" --feedback
# 躯干瞄准：[aim_x, aim_y, aim_z]（map 系）
ros2 action send_goal /decision/mission ylr1d_decision/action/Mission \
  "{task: 'torso_aim', args: [0.6, 0.0, 0.9]}" --feedback
```

### HMI / CLI（下层直发）
- MoveitPanel（moveit）：part + 位姿/关节目标 + 夹爪；
- PlanPanel（nav）：NavigateToPose；
- CLI：`/chassis_move` `/arm_move` `/gripper_move`（ylr1d_translate 类型）。

## 五、已验证（SDK 仿真实测）

| 链路 | 结果 |
|---|---|
| 驱动层 | ✅ 底盘(平移/旋转/横移)/臂/躯干/夹爪全 SUCCEEDED；/joint_states 30 关节 |
| moveit | ✅ 臂关节/姿态(IK)/躯干瞄准/夹爪全链路 |
| nav | ✅ NavigateToPose SUCCESS，底盘实动 |
| **决策层** | ✅ **Mission(arm_move) → BT → plan_client → moveit → 驱动 → SDK，outcome=0 完成，左臂末端实动 5cm** |

## 六、排障记录（根因已实锤修复）

1. **SDK 指令线程亲和**：运动指令必须在发起连接的主线程发出 → 驱动内联主线程执行；
2. **单位制**：SDK 臂关节=度、末端=mm/度；框架=弧度+米 → `robot_sdk.hpp` 边界统一换算；
3. **底盘无速度反馈**：SDK getVehicleState 恒 0 → 里程计按命令速度死推。

（以上修复已并入 `ros2_ylr1d_controller_ws` 仓库提交，cp 同步后无需重打。）

## 七、框架同步说明

- `src/ThirdParty/YLR1D_Controller` 是 `github.com:Zhengshuji/YLR1D_Controller` 的子模块，与 `ros2_ylr1d_controller_ws` 同仓库；
- 同步方式：从 `ros2_ylr1d_controller_ws` **cp** 新增/修改文件（决策层 + hmi 面板 + bringup/rviz/test），对源工作区零改动；
- **不构建** `ylr1d_bringup`/`ylr1d_test`（新版依赖仿真层 ylr1d_plant/ylr1d_control，真机栈用自带 bringup + 决策层独立 launch）。

## 八、真机投入前标定项

1. 里程计死推比例；2. SDK +wz 旋转方向；3. 位姿 IK 可达性。