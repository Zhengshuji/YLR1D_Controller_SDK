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