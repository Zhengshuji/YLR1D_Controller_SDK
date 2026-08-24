# 真机驱动整改方案（v5 · 已完成主要项）

> 用户确认的三件事实 + 两个**已实锤并修复**的根因：
> 1. 仿真界面一直在开，SDK 控制/查询的就是它；`armDemo`/`vehicleDemo` 能驱动（Demo 是权威参照）；
> 2. 真机模型文件与框架 ylr1d_description URDF 一致（moveit 配置正确）；
> 3. 驱动下发后 SDK 起初完全没动 → 两个根因（见下）。

---

## 一、已实锤的两个根因与修复 ✅

### 根因 1：SDK 运动指令的线程亲和
- **现象**：`moveABSJoint` 由 worker 线程发出时被服务端忽略/卡死（waitMotionCMDFinish 永不返回或秒回但不动）；armDemo 能动是因为它在**发起连接的线程**（main）里发指令。
- **修复**：`robot_driver` 把 `moveABSJoint + waitMotionCMDFinish` **内联在 executor（主线程）**执行（与 armDemo 逐行一致）。
- **验证**：`/arm_move` 在仿真界面真实转动 ✅。

### 根因 2：单位制（弧度 vs 度）
- **现象**：SDK 臂关节是**度**、末端位置 **mm**、姿态 **度**；框架/moveit 用**弧度+米**。弧度当度发 → 肉眼不可见；SDK 的度当弧度喂 moveit → OMPL 报 invalid bounds（早期"越限"假象）。
- **修复**：`robot_sdk.hpp`（SDK 边界）统一换算——下发 rad→deg，上报 deg→rad（末端 mm→m、姿态 deg→rad）。
- **验证**：`/arm_move [0.5,-0.3,0.2]`（弧度）→ SDK 收到 [28.6°,-17.2°,11.5°] → 仿真界面明显转动 → `/joint_states` 回显 [0.500,-0.300,0.200] ✅。

## 二、结构探测（SDK 模型 = 框架模型，已确认）

| SDK 索引 | 角色 | DOF |
|---|---|---|
| ARM_1 (index 0) | 左臂 | 7 |
| ARM_2 (index 1) | 右臂 | 7 |
| **ARM_3 (index 2)** | **躯干**（升/偏航/双俯仰） | 4 |
| ARM_4 (index 3) | （探测会卡死服务端，勿调用） | — |

- `robot_driver`：`/arm_move part 0→ARM_3 / 1→ARM_1 / 2→ARM_2`；启动使能三组伺服（含 ARM_3，否则躯干 moveABSJoint 返回 -1）。
- `robot_sensors`：`/sensors/arm_raw name="arm2"` → 框架躯干 4 关节名（`kLeftArmJoints` 之外的 `kTorsoJoints`）。

## 三、moveit 完成状态 ✅

| 目标类型 | 结果 |
|---|---|
| 臂关节目标（MoveItMove joint_positions） | ✅ SUCCEEDED（OMPL 规划 → waypoint → 驱动 → 仿真真实转动） |
| 臂姿态目标（IK，HMI 默认模式） | ✅ SUCCEEDED（起点合法弧度后 IK 可用） |
| 躯干瞄准（part=0，torso_aim） | ✅ SUCCEEDED（~178° 回转 + 俯仰，240s 超时内完成） |
| 夹爪段 | ✅ SUCCEEDED |
| HMI（姿态输入） | ✅ 可用（位姿需当前可达；躯干转过后臂可达空间变化属正常运动学） |

- **改动**：`moveit_goal_server` 段超时 `kSegmentTimeout 90s → 240s`（躯干大轨迹 >90s 会被误判失败）。

## 四、启动

```bash
ros2 launch bringup real_robot_moveit.launch.py   # 驱动+传感器+感知+moveit+HMI（等 1-2 分钟 move_group 就绪）
ros2 launch bringup real_robot_manual.launch.py   # 手动 3 action（无 moveit）
```

## 五、待办

- [ ] nav：现成 `hmi_plan` + 完整 nav2（NavigeToPose），不用键盘；（当前无独立 nav 启动）
- [ ] rviz2 与实际仿真界面对拍（模型一致前提下，验证初始位姿/零位/显示一致）
- [ ] 文档最终核对（README 已更新）

## 六、框架改动清单（最小集）

| 文件 | 改动 | 是否必要 |
|---|---|---|
| `robot_driver`（新代码） | 内联主线程运动 / 度弧度换算 / torso 映射 / 三伺服使能 | ✅ 真机必需 |
| `robot_sensors`（新代码） | arm2→躯干关节名 | ✅ 真机必需 |
| `ylr1d_plan_moveit/moveit_bridge.cpp` | 服务等待 180s 重试 | ✅ 时序健壮性 |
| `ylr1d_plan_moveit/moveit_goal_server.cpp` | 段超时 240s | ✅ 大轨迹必需 |
| `ylr1d_hmi/moveit_panel.*` | 已**回退**（恢复姿态输入原样） | 回退完成 |
| `bringup/real_robot_nav.launch.py` | 已**删除**（键盘） | 回退完成 |
---

## 七、最终状态（2026-08-22）

本整改文档为过程记录。最终状态：

- **全部实现并实测通过**：驱动层（底盘/臂/躯干/夹爪）、传感器层（30 关节 + 死推里程计）、moveit（关节/姿态/躯干瞄准/夹爪）、导航（NavigateToPose）、**决策层**（Mission → BT → plan_client → 规划层 → 驱动 → SDK，outcome=0）；
- **三个根因已修复并入仓库**（线程亲和 / 单位制 / 无速度反馈死推），cp 同步无需重打；
- **决策层已接入**：`ylr1d_decision` 从 ros2_ylr1d_controller_ws cp 进子模块，`decision.launch.py use_sim_time:=false` 叠在真机栈上；
- 不构建 `ylr1d_bringup`/`ylr1d_test`（新版依赖仿真层 plant/control，真机用自带 bringup + 决策层独立 launch）；
- 当前使用手册：`README.md`；最终设计：`real_robot_driver_design.md`；对比：`real_vs_gazebo.md`。

### 决策层可视化与一键 bringup（2026-08-22 补充）

- **一键启动**：`real_robot_decision.launch.py` = `real_robot_nav_core`（基线+odom_pub+map_server+nav2+bridge）+ moveit + `decision.launch.py` + `hmi_decision.launch.py` + 单一决策 rviz——等价框架 `bringup_decision`（去掉 Gazebo/转译层）；导航核心抽成 `real_robot_nav_core.launch.py` 供 `real_robot_nav` / `real_robot_decision` 共用（原 nav 里 6 个 nav Node 直接内联，未抽包）。
- **rviz 启动修复（include 参数污染，陷阱 16 实战）**：`moveit.launch.py` / `hmi_decision.launch.py` 都声明同名 `rviz` 参数（默认 false），被 include 时在共享 context **覆盖**顶层 `rviz:=true` → 决策 rviz 条件失效（log 里完全没有 rviz2 输出）。照框架 `bringup_decision`：**决策 rviz 节点放顶层、先于所有声明 rviz 的 include**；hmi_decision 内部 rviz 恒关；`moveit_rviz` 多余参数已删（"只要一个 rviz"）。
- **顺带修复**：`real_robot.launch.py` 末尾残留 `_include("ylr1d_hmi", "hmi_plan.launch.py")`（`_include` 未定义，hmi_plan 本属 nav launch）——install 旧副本掩盖，rebuild 后必炸，已删，基线恢复纯净。
- **三任务实测（真机栈 + SDK 仿真）**：
  | 任务 | 结果 | 注意 |
  |---|---|---|
  | arm_move (1, +x5cm) | ✅ outcome=0 | 左臂末端 TF 实测 x:-0.393→-0.345 |
  | torso_aim (0.6,0,1.2) | ✅ outcome=0 | 瞄点须可达：**(1.0,0,0.6) 在倾角盲区**（ray_error=0.065>3cm 判不可达）；自检可达点见 `real_robot_driver_design.md` |
  | base_move (0.6m) | ✅ outcome=0 | 真移，odom→(0.42,0.02)；**<0.25m 目标因 nav2 容差假成功不动**，增量建议 ≥0.5m |

