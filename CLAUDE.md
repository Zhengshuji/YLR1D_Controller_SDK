# CLAUDE.md — YLR1D 项目备忘录

> **路径约定**：本文中 `<WS_ROOT>` 指工作空间根目录。\
工作空间重命名后，将 `<WS_ROOT>` 替换为新路径即可，代码与文档均不依赖具体路径。

---

## 一、项目概述

双机械臂 + 全向四轮底盘 + 升降躯干的复合移动机器人仿真，基于 **ROS2 Humble + Gazebo Classic**，全模型 30 关节。控制层按"算法 / 控制"解耦，共九个包（八个功能包 + `ylr1d_test` 功能测试包）：

| 包 | 职责 |
|----|------|
| `ylr1d_description` | 模型资产单一来源（xacro / mesh / config / rviz / world）+ **C++ 配置头**（`include/ylr1d_description/config/joint_config.hpp`：关节分组/工作限位单一来源，三阶段起从算法层迁入） |
| `ylr1d_plant` | 物理层（中控）：Gazebo + ros2_control + **仿真组件**（plant_sim 把控制层加速度积分成位置/速度，composition 容器共享进程，初始化源感知层 `/perception/sensors/joint_states`） |
| `ylr1d_algorithm` | **算法层**（ROS2 **接口包**）：纯 C++ 算法核心（统一 Controller 接口 + `ControllerT<LawT>` 单元模板，PID / P=1 前馈→逐关节 PID，D 项用反馈速度，Eigen 向量化）+ 5 个**纯控制器**节点（组合控制器+节点一体，只算加速度 u，不持有被控对象，**独立可执行（src/nodes/ 含 main）与 composition 组件（src/components/ 注册）双轨**）+ 自建接口 msg/srv；pid/输出限幅在头文件 `Params`，分组/限位在 description `joint_config.hpp` |
| `ylr1d_control` | **控制层**（采样保持器 + 通信节点）：接收期望值与算法层输出，采样保持后转发（`/ctrl/<组>/*` 六接口、`/plant/<组>/cmd`），反馈经 **A-lite 模型预测器**（独立纯算法类 `ModelPredictor`，`predictor/`，100Hz 虚拟对象积分 + 感知层 `/perception/joint_state` 测量（p,v）独立互补校正 αp=0.2/αv=0.1，见控制层 README 6.7）封装回发 `/ctrl/<组>/feedback`（模型状态，消除延迟震荡），提供聚合 command service 路由到算法层，不下发物理层命令，不做控制算法计算 |
| `ylr1d_translate` | 转译层：上层 action → `/desired_joint_states` |
| `ylr1d_hmi` | 人机界面：Qt5 七面板（控制 / 转译 / 传感器 / 算法层 / 规划导航 / 规划机械臂 / 监视） |
| `ylr1d_perception` | **感知层**：接收层 5 类接收节点（`image_receiver` / `camera_info_receiver` / `point_cloud_receiver` / `laser_scan_receiver` / `imu_receiver`，统一模板 `SensorReceiverNode` 恒等转发 27 路传感器到内部 `/perception_self/sensors/*`，批次 3 拆 `sensor_proxy`）+ `health_aggregator`（聚合各节点 `/perception_self/health/<节点名>` 片段发布 `/perception/health`）；`joint_state_receiver` 清洗关节状态（等长校验 + NaN→0，内部 `/perception_self/sensors/joint_states`，批次 2 由物理层 joint_state_filter 改造归位）；`joint_state_estimator` 用**跟踪微分器（TD）**估机械臂组速度/加速度，发布内部 `/perception_self/joint_state`；`wheel_odometry`（批次 5 C1 轮式里程计，单路订阅解超定最小二乘出内部 `/perception_self/odometry`）；**分发层**（批次 4，对外 `/perception/*` 仅本层发布；2026-08-13 重构为 `DispatchNodeBase` 骨架 + `SyncUnit` 同步器单元，每单元独立线程）：`control_feedback_guard` D3 核查（维度/数值/范围/新鲜度）出 `/perception/joint_state`（**控制层反馈源**）、`sensor_dispatch`（原 `joint_states_exit` + `hmi_dispatch` 合并）出全量 `/perception/sensors/joint_states` + 27 路传感器恒等转发 `/perception/sensors/*` + 按 5 组分话题 `/perception/hmi/joints/<组>` 出纯原始关节 + 里程计对外出口 `/perception/odometry`（B4，内部 `/perception_self/odometry` 直通） |
| `ylr1d_bringup` | 一键启动：聚合各包 launch |
| `ylr1d_test` | 功能测试包：统一入口跑各层冒烟测试（Tier0 env / Tier1 冒烟 / Tier2 集成），结果落盘 `test_results/` | 独立 launch、独立运行，不并入 bringup（见[功能测试约定](#功能测试约定)） |

> **语言约定**：项目主体基于 C++（各包节点均为 C++）。Python 仅限 launch 与 test 层
> （`launch/*.py`、`launch/python_utils/*.py`、`test/*.py`），核心逻辑禁止用 Python。

> **详细说明见 [README.md](README.md)**：项目简介、架构总览、关键特性、快速开始、演示概览；架构详解与接口速查见 `docs/` 开发手册（[architecture.md](docs/architecture.md) / [api-reference.md](docs/api-reference.md)）。
> 各包功能细节见对应包 README（根 README「七、文档导航」有索引；扩展开发先看 [docs/development.md](docs/development.md)）。本文件仅保留备忘录性质的注意项与工作流程。

---

## 二、注意事项

### 环境与执行

#### WSL 执行命令
本项目在 WSL Ubuntu-22.04 中运行。从 Windows CLI 调用 WSL 时必须添加 `MSYS2_ARG_CONV_EXCL="*"` 防止 Git Bash/MSYS2 路径转换：

```bash
MSYS2_ARG_CONV_EXCL="*" wsl.exe -d Ubuntu-22.04 bash -c 'source <WS_ROOT>/install/setup.bash; ros2 ...'
```

#### 环境初始化
```bash
cd <WS_ROOT>
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src
```

环境初始化见根 README「快速开始-构建」；`ROS_LOCALHOST_ONLY=1` 已由 plant launch 自动注入（见陷阱 22）。

#### 注意
- WSL 下 GPU 渲染受限，Gazebo 需 `LIBGL_ALWAYS_SOFTWARE=1`
- Gazebo Classic 在 WSL 下启动很慢（~30-60s 控制器才完全加载），务必耐心等待 `ros2 control list_controllers` 全部显示 `active`

### 功能测试约定
- 功能测试统一收敛到 `ylr1d_test` 包（`src/ylr1d_test`），入口脚本 `src/ylr1d_test/scripts/run_tests.sh`，结果落盘 `test_results/`（已加入 `.gitignore`）。详见 [ylr1d_test README](src/ylr1d_test/README.md)。
- 新增/改动功能包时，同步补充/更新对应测试（Tier0 `env` 环境检查、Tier1 各层冒烟、Tier2 集成/全流程）。
- **B1 故障注入（2026-08-14）**：`fault_inject_test.py`（Tier1 无 Gazebo，`--tests fault` 显式跑）验证鲁棒性机制——C1/C2 断流停转、模型冻结、E_STOP 三态+急停、健康话题死亡、B3 respawn；改这些机制时同步更新对应断言（kill 目标/停发话题/断言窗口）。
- 需 ROS2 的测试必须**独立 launch**（`ylr1d_test` 自带 `plant_stack.launch.py` / `test_stack.launch.py`），**不得并入 bringup**（bringup 一行不改）。
- 测试互不干扰：独立进程组 + 测试前后清理 + 结束核查无残留；失败诊断只给「问题信息 + 解决方案建议」。

### 仿真验证（常用命令）

#### 一键启动（完整闭环，推荐）
```bash
ros2 launch ylr1d_bringup bringup_control.launch.py
```
等价于依次启动 `gazebo_position.launch.py`（position 方案）+ `control.launch.py`（控制层，**已内嵌算法层** `sim_controller.launch.py` composition）+ `hmi_control.launch.py` + `hmi_sensor.launch.py`。

#### 力控测试（effort 方案，单独启动）
```bash
ros2 launch ylr1d_plant gazebo_effort.launch.py
```

#### 转译层链路（上层 action 驱动）
```bash
ros2 launch ylr1d_bringup bringup_translate.launch.py
```

#### 验证控制器激活（等待输出全部为 active）
```bash
ros2 control list_controllers
```

#### 获取广义坐标
```bash
ros2 topic echo /joint_states --once
```
所有 30 个关节的 position / velocity / effort 均在 `/joint_states` 中。prismatic 关节的 NaN 由感知层 `joint_state_receiver` 清洗为 0.0（等长校验 + NaN→0，批次 2 由物理层 joint_state_filter 改造归位），清洗后数据在 `/perception/sensors/joint_states`。

### 已知弯路 / 常见陷阱

#### 1. WSL 路径转换（MSYS2_ARG_CONV_EXCL）
**问题**: Git Bash 自动将 `/home/...` 转换为 `C:/Program Files/Git/home/...`
**解决**: 所有 `wsl.exe` 调用前加 `MSYS2_ARG_CONV_EXCL="*"`

#### 2. gzserver 生命周期
**问题**: `ros2 launch` 退出后 `gzserver` 仍在后台运行，再次启动会冲突（`Entity already exists`）
**解决**: 2026-08-14（A4）起 plant / description display launch 已加 `OnShutdown` 退出清理（正常 Ctrl-C 不再残留）；异常退出（如 `kill -9`）后用根目录 `stop_all.sh` 一键清理，或手动：
```bash
pkill -x gzserver; pkill -x gzclient
```
**相关（B5，2026-08-14）**：四个 bringup 有**单实例互斥锁** `/tmp/ylr1d_bringup.lock`（`launch/python_utils/bringup_lock.py`，O_EXCL 原子创建）——已有一个存活 bringup 时第二个直接报错拒绝启动（防并行实例抢 Gazebo 11345 端口）；正常退出 / Ctrl-C 自动释放，`kill -9` 残留由 stop_all.sh 删除（下次启动也会自动清陈旧锁）。

#### 3. 控制器加载慢（WSL 性能）
**问题**: WSL 下 Gazebo 启动到控制器 active 需要 30-60s，而非文档写的 10-15s
**解决**: 启动后使用 `ros2 control list_controllers` 轮询直到全部 active，不能抢跑

#### 4. Joint_Base_to_Body1 限位
**问题**: 限位 [-0.3, 0.3] rad，初始位置就是 0.3，正向力矩推不动
**解决**: 调试时先用负向力矩离开限位

#### 5. effort 命令需要持续发送
**问题**: ForwardCommandController 虽然保持最后值，但单次 `--once` 在重力/碰撞下可能不足以产生可见运动
**解决**: 用 `--rate 20` 持续发送

#### 6. JointState 解析
**问题**: `ros2 topic echo /joint_states` 输出格式为 YAML-like 多行文本，pipe 给 grep 不容易提取
**解决**: 用 Python 写 rclpy 节点直接订阅解析，或使用 `ros2 topic echo --field data`

#### 7. 棱柱关节 NaN
**问题**: GazeboSystem 在 spawn 暂停阶段将 prismatic joint 位置初始化为 NaN，导致 `TF_NAN` 刷屏
**解决**: 感知层 `joint_state_receiver` 节点将 NaN → 0.0（等长校验 + NaN→0 + 断流告警，批次 2 由物理层 `joint_state_filter` 改造归位），发布 `/perception/sensors/joint_states`；物理层 rsp / plant_sim 与转译层 translate 均改订阅该清洗后话题（感知层须并入，见陷阱 19）

#### 8. ros2 topic echo 输出被文件重定向截断
**问题**: `timeout 3 ros2 topic echo /joint_states --once > file.txt 2>&1` 可能输出空文件，因为 timeout 在消息到达前就结束了
**解决**: timeout 给足 5s，或使用 `ros2 topic echo --once` 不加 timeout（在后台运行时使用）

#### 9. gzserver 与 spawner 的时序
**问题**: spawner 在 Gazebo 完全加载前启动会连不上 controller_manager 服务
**解决**: launch 文件中使用 `TimerAction(period=8.0)` 延迟 spawner 启动

#### 10. 算法层/控制层建议经 launch 启动
**问题**: 算法层节点参数为头文件 `Params` 编译期常量（无 yaml），直接 `ros2 run` 单起节点可运行，但无控制层回发 feedback → 不闭环；控制层必须与算法层一起经 launch 启动
**解决**: 算法层 `ros2 launch ylr1d_algorithm sim_controller.launch.py`（**composition 版**：arm/chassis 两容器捆绑算法层 5 节点 + 控制层 2 节点；独立节点版 `sim_controller_separate.launch.py` 保留调试）；控制层 `ros2 launch ylr1d_control control.launch.py`（已内嵌算法层，composition 默认；独立节点版 `control_separate.launch.py` 单起控制层 2 节点，需配算法层独立版闭环）。改参数（kp/ki/kd/output_limit 等）运行期经 `/ctrl/<组>/command` service 的 `set_param` 即时生效（或控制层聚合 `/control/<节点名>/command`），**无 `ros2 param set` 通道**；改头文件 `Params` 默认值需重编并重启节点；改限位改 description `joint_config.hpp` 后重编。**注意**：算法层反馈来自感知层 `/perception/joint_state`（控制层经 A-lite 模型封装回发 `/ctrl/<组>/feedback`），脱离感知层/物理层（如 `ros2 run` 单起）无反馈、以首帧期望作 feedback 输出（误差 0，不闭环）——验证闭环须带物理层 + 感知层

#### 11. pkill -f 会自匹配（WSL bash）
**问题**: `pkill -f chassis_simulate` 会匹配到 bash 自身命令行里的同名模式，把执行 shell 杀掉（exit 15）
**解决**: 用字符类技巧 `pkill -f "[c]hassis_simulate"`，或先 `ps` 确认 PID 再精确 kill。**注意字符类只保护模式文本本身**：若同一脚本还 `exec` 了目标二进制（如内联清理 `pkill -f '[g]zserver'; exec gzserver ...`），命令行里仍有真实字面量 "gzserver"，`-f` 照样匹配自己 → 改用 `pkill -x gzserver`（精确匹配进程名 comm，bash 的 comm 是 "bash"，不会自匹配）——gazebo_position.launch.py P2 清理即此实现。

#### 12. ylr1d_hmi 静态配置单一来源
**约定**: `ylr1d_hmi` 的关节定义统一在 `include/ylr1d_hmi/config/joint_defs.hpp`（30 关节原子 + 控制/监视/转译三组视图），传感器话题统一在 `config/sensor_topics.hpp`。改限位/话题只改这两处，并对照 `ylr1d_description/config/limits.yaml` 语境；action 发送公共逻辑在 `common/action_sender.hpp`。

#### 13. 静态配置单一来源（description joint_config + 算法层 Params）
**约定**: 分组与工作限位唯一在 `ylr1d_description/include/ylr1d_description/config/joint_config.hpp`（`jointLimitFor(name)` + `kJointGroups` 组注册表，命名空间 `ylr1d_description`；**三阶段起从算法层迁入**，算法层/控制层/物理层 plant_sim/感知层 estimator 均 include 引用），pid / 输出限幅在 `{组}_node.hpp` 的 `Params` 编译期常量。命令话题常量已移交物理层 `ylr1d_plant/config/command_topics.hpp`（算法层不持有）。**service key 分隔符为点**：统一 `controller.<控制器>.<参数>` 命名，控制器名 `whole`（位置组 P=1 前馈，速度组无）/ `joints_list`（逐关节广播，多控制器聚合组加 `_list` 后缀）/ `joints_<j>`（逐关节覆盖）/ `gripper_list`（臂夹爪 j=7/8）+ 裸 `controller.output_limit`（组合控制器自身，基准取限位表 max_accel；2026-08-13 调参后取**低于**物理限幅：位置组 2.0/夹爪 0.5/轮子 5.0，因物理层 SimPlant 对加速度不钳制，限幅=物理值会饱和极限环致剧烈震荡）。默认值 = 头文件 `Params`。改限位/分组/pid 默认值 = 改头文件后重新编译并重启节点；运行期经 `/ctrl/<组>/command` service `set_param` 即时生效（或控制层聚合 `/control/<节点名>/command`；**无 `ros2 param set` 通道**）。改限位前对照 `ylr1d_description/config/limits.yaml` 语境。

#### 14. 算法层 .so 的头文件传递（ament_export_targets）
**问题**: `ament_export_libraries(ylr1d_algorithm)` 只导出链接名、不导出 CMake target，消费方 `target_link_libraries(foo ylr1d_algorithm)` 会当裸库名链接，include 目录与编译特性**传不过去**（fatal: ylr1d_algorithm/control_law/pid.hpp: No such file）
**解决**: 算法层必须 `install(TARGETS ... EXPORT <名>)` + `ament_export_targets(<名> HAS_LIBRARY_TARGET)`（名一致），消费方链接**命名空间 target** `ylr1d_algorithm::ylr1d_algorithm_core`（三阶段起核心库改名 `ylr1d_algorithm_core`，规避与 rosidl 接口 target 同名冲突，见陷阱 23）。改了 export 后若仍报错，删对应包的 `build/` + `install/` 强制重配（colcon 缓存旧 config）。**变体**：算法层 export 的 ThirdParty Eigen include 只在构建树内（BUILD_INTERFACE），install 后消费方拿不到——`ylr1d_plant` 的 plant_sim 需自行把 `<WS>/ThirdParty/Eigen` 加入 include（物理层 CMakeLists 已处理）。

#### 15. 算法层纯控制器话题与启动
**约定**: 算法层 5 个**纯控制器**节点（`steering_sim`/`wheels_sim`/`torso_sim`/`left_arm_sim`/`right_arm_sim`）各自组合控制器+节点写在同一个 `{组}_node.hpp`，实现 `src/nodes/{组}_node.cpp` 带 main + `src/controller/{组}_controller.cpp` + `src/sim_node_base.cpp`，固定框架 `输入(期望/反馈/反馈速度) → 组合控制器(位置组 P=1 前馈 → 逐关节 PID，D 项 -kd·v 速度阻尼) → 输出加速度 u`，经 `/ctrl/<组>/{input,feedback,output,param,state}` + `/ctrl/<组>/command` srv 与控制层通信（组定义在 description `joint_config.hpp` `kJointGroups`；input/output 为 `Float64MultiArray`，feedback 为感知层 `JointGroupEstimate`，param/state 为组类别 `*Params/*State`，command 为 `ControllerCommand` srv）。核心层：统一接口 `interface/controller.hpp`（含默认虚方法 `compute_with_velocity`）、单元控制器模板 `controller/controller_impl.hpp`（`ControllerT<LawT>`，算法=Law+`Params`）、`controller/control_law/pid.hpp`。换单元算法=新增 Law 换模板实参；换组合控制器=直接改该节点头文件+实现，节点框架与控制层零改动。节点**只等首帧输入**即初始化（feedback = 期望 → 误差 0，无瞬态），不依赖反馈首帧；**feedback 由控制层从感知层 `/perception/joint_state` 提取、经独立 A-lite 模型预测器封装回发**（`ModelPredictor`，`predictor/`，由 `GroupForwarder` 组合持有，100Hz 积分 + (p,v) 独立互补校正，见控制层 README 6.7），算法层节点不自行回环；被控对象（SimPlant 积分）在物理层 `ylr1d_plant` plant_sim 组件（初始化源感知层 `/perception/sensors/joint_states`，限位单一来源 description joint_config.hpp）。运行期调参经 `/ctrl/<组>/command` service（无 `ros2 param set`）。**双轨运行**：独立节点（`sim_controller_separate.launch.py`，5 独立进程，main 保留）与 **composition 组件**（`src/components/{组}_node_component.cpp` 注册宏 + 每组件一个 SHARED 库——因 `{组}_node.cpp` 各含一个 main 合库会符号冲突；`sim_controller.launch.py` 用 `ComposableNodeContainer` 把算法层 5 节点 + 控制层 2 节点（`arm_control`/`chassis_control`）捆绑进 arm / chassis 两容器，容器内共享 executor 规避 WSL Fast DDS 跨进程时序抖动）。组件节点名取自构造函数（`SimNodeBase` 硬编码），与独立版一致，依赖节点名的 launch/测试/HMI 零改动；组件库手写 ament index（`rclcpp_components` 资源）注册，不生成单节点容器 EXECUTABLE。层间时序：`bringup_control.launch.py` 已按「物理层 + 感知层 → 控制层（内嵌算法层）→ HMI」聚合；`control.launch.py` 内嵌算法层 composition，不再单独起算法层。**转译层同样双轨**：`translate.launch.py`（默认跨包 composition：include `sim_controller.launch.py` + `translate_container` 捆绑 translate_server 组件）与 `translate_separate.launch.py`（独立节点）；节点名 hardcode `translate_server`，两形态一致。

#### 16. 改 bringup/launch/测试模块后须重编对应包（install 旧版静默生效）
**问题**: 改了 `ylr1d_bringup` 的 launch（如给 bringup_control/bringup_translate 加/删 include）后，若只 `colcon build --packages-select` 部分包（不含 bringup），install 里仍是旧 launch，`ros2 launch ylr1d_bringup` 加载旧版——症状是控制链路断（如 bringup_control 不启动算法层容器，控制层收不到 `/ctrl/<组>/output` 只发 input 不转发 `/plant/<组>/cmd`，机器人不动）但源码看起来完全正常。**Python 测试模块同样中招**：改了 `ylr1d_test/ylr1d_test/*.py`（如给 control 加健康话题断言）后不重编 `ylr1d_test`，`run_tests.sh` 跑的是 install 里旧副本——症状是**新增断言不执行、测试照样 PASS**（比 FAIL 更隐蔽）。
**解决**: 改动任何包的 launch/config/Python 模块后重编该包（`colcon build --packages-select ylr1d_bringup` 或 `--packages-select ylr1d_test`），或直接全量 `colcon build`。怀疑 install 过旧时对比 `install/<pkg>/share/<pkg>/launch/` 与 `src/<pkg>/launch/`（Python 模块对比 `install/<pkg>/lib/python3.10/site-packages/`）。

#### 17. ros2 daemon 僵尸导致 node/param 查询故障
**问题**: WSL 经 `wsl.exe bash -c` 调用时，`ros2 node list` / `ros2 param get` 持续报 `xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">`（daemon 与 CLI 进程 context 失配），与代码无关；症状是无故超时或空结果。**另一个易混现象**：WSL 负载高时 `ros2 param get` 子进程可能超时（control 测试 `get_param` 内部 timeout 15s）返回全 None，导致参数断言假 FAIL——与代码无关，重跑（可先 `ros2 daemon stop`）即恢复
**解决**: `ros2 daemon stop`（会以 fresh context 自动重启），此后 `ros2 node list` 正常。rclpy 测试节点不受影响，数据流验证优先用 rclpy 脚本而非 ros2 CLI。

#### 18. 健康话题统一约定（X1 + plant_sim + perception，A 档）
**约定**: 三层各发布 `std_msgs/String` 健康状态（500ms）：
- 控制层 `/health/{chassis_control,arm_control}`：`OK` / `STALE:<组>(<秒>),...`（算法层输出断流，已停止转发 `/plant/<组>/cmd`）/ `NO_CMD_SUB:<组>,...`（输出新鲜但 `/plant/<组>/cmd` 无人订阅）/ `AWAITING_ACCEL`（未收到算法层输出）；
- 物理层 `/health/plant_sim`：`OK` / `AWAITING_FEEDBACK`（未收到初始化测量）/ `AWAITING_ACCEL:<组>`（已初始化但从未收到控制层加速度，控制层未起）/ `E_STOP:<组>`（曾收到加速度后断流急停）；
- 感知层 `/perception/health`：`OK` / `STALE:<标签>(<秒>),...` / `<节点名>(unavailable)`（某接收节点片段缺失）。
统一解析在 `ylr1d_hmi/common/health_parse.hpp`（`parseHealthState` / `healthStateDetail`），monitor 消费端与测试断言共用同一语义。**改健康状态语义须同步**：各发布端（控制两节点 `health_state()`、plant_sim `publish_health()`、health_aggregator）、`health_parse.hpp`、`monitor_panel.cpp`（`refreshHealth()` / `updateNotables()`）、`control_test.py` / `perception.py` 断言——五处一致。

#### 19. bringup 固定时延分层启动（启动时序）
**约定**: 四个 bringup launch（`bringup_control` / `bringup_translate` / `bringup_plan_nav` / `bringup_plan_moveit`）用 `TimerAction` **固定时延分层**启动，不抽公共框架、不用事件驱动。既有两栈时序：0s 物理层（Gazebo + rviz2 + 传感器面板）+ **感知层**（反馈源 `/perception/joint_state`，控制层反馈依赖，须与物理层同起）→ 30s 控制层（含算法层 composition；bringup_translate 为 30s 跨包 composition 一次捆绑控制层 + 算法层 + 转译层——translate 随控制层提前起无碍：/joint_states 未就绪时开环发布期望，STEER_TIMEOUT 仅转向阶段计 30s 且只对"曾收到后断流"判流）→ 40s HMI。plan 两栈分层：0s 物理层（`plant_rviz:=false`）+ 感知层（plan_nav 另含定位）→ 30s 转译层跨包 composition → 40s nav2/moveit（plan_moveit 为 move_group + bridge）→ 50s 规划 HMI（hmi_plan / hmi_moveit）。rviz2 0s 启动（日志不可回放、节点状态可恢复，见物理层 README P5）。物理层 controller_manager 就绪实测波动 22-63s（若已按陷阱 22 `export ROS_LOCALHOST_ONLY=1`，就绪可缩至秒级，30s 时延仍保守适用）——30s 只覆盖快场景，慢场景（>30s）下控制层会短暂 NO_CMD_SUB / AWAITING（就绪后自动恢复，HMI 40s 起仍可用）；这是固定时延取快（减少等待）的取舍，曾用 70s 覆盖慢场景后应用户要求收紧。**改动时序**：加/删分层或调整时延须同步改四个 bringup launch 的注释、bringup README"启动时序"节、本节——且注意改 bringup launch 后须重编 `ylr1d_bringup`（陷阱 16）。

#### 20. 自建 msg 包的 member_of_group 必须放 package.xml 根节点（不是 `<export>` 内）
**问题**: 接口包（`rosidl_generate_interfaces`）构建报 `Packages installing interfaces must include '<member_of_group>rosidl_interface_packages</member_of_group>'`，即使 package.xml 已写。原因：catkin_pkg 解析 `member_of_group` 用 `_get_nodes()` 只查**根节点直接子节点**，`<export>` 内的子标签解析不到（实测 `package_xml_2_cmake.py` 输出 `set(<pkg>_MEMBER_OF_GROUPS )` 空值）。
**解决**: 参照 std_msgs 写法，把 `<member_of_group>rosidl_interface_packages</member_of_group>` 放 `<export>` **外**、根节点下。验证：`python3 /opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py <pkg>/package.xml /tmp/x.cmake && grep MEMBER_OF /tmp/x.cmake`。

#### 21. 接口包构建：include 导出、同包 msg 链接、include 嵌套
**约定**（ylr1d_perception 建成后的经验，属陷阱 14 的接口包变体）:
- **同包节点链接自建 msg**：`rosidl_generate_interfaces` 生成的 `${PROJECT_NAME}` 是 UTILITY custom target，`target_link_libraries(节点 ${PROJECT_NAME})` 报 "Target ... of type UTILITY may not be linked"。用 `rosidl_get_typesupport_target(<var> ${PROJECT_NAME} "rosidl_typesupport_cpp")` 取实际接口库再链接。
- **include 目录导出**：`install(DIRECTORY include DESTINATION include)` 会把目录本身拷成 `include/include/` 嵌套（须带尾部斜杠 `install(DIRECTORY include/ DESTINATION include)`）。加 `ament_export_include_directories(include)` 供消费方引用。
- **消费方拿自定义头文件**：接口包导出了 rosidl `_TARGETS`，`ament_target_dependencies(目标 ... "<接口包>")` 走 modern target 分支只传 msg 库、**忽略 `_INCLUDE_DIRS`**，自定义头文件（如 `config/sensor_topics.hpp`）仍找不到——消费方需显式 `target_include_directories(目标 SYSTEM PUBLIC ${<接口包>_INCLUDE_DIRS})`（ylr1d_hmi 即此处理）。
- **改 CMakeLists 加/删 install/export 后**：增量 build 可能不重新 configure 导致 install 未更新，须删 `build/<pkg>` + `install/<pkg>` 强制重配（陷阱 14 同因）。

#### 22. WSL 下物理层组件容器加载鲁棒性差（load_node 超时）
**问题**: 独立（后台）启动 `gazebo_position.launch.py` 时，`plant_container` 加载完 robot_state_publisher 后的组件（`plant_sim`，批次 2 前还有 `joint_state_filter`，2026-08-12 已迁感知层）报 `failed to send response to /plant_container/_container/load_node (timeout)`，组件加载失败 → 物理层 rsp / plant_sim 订阅的 `/perception/sensors/joint_states` 无发布者（感知层未并入时）；更严重时 gazebo_ros2_control 连 controller_manager 都不加载（`ros2 control list_controllers` → "No controllers are currently loaded!"），`/joint_states` 也无数据。2026-08-09 实跑连续两次复现（rviz:=false 后台启动场景）。
**影响**: 任何依赖 `/perception/sensors/joint_states` 的链路（物理层 rsp / plant_sim、感知层 `joint_state_estimator`、转译层 translate）无数据源 → 感知层 `/perception/joint_state` 无输出；实跑验证感知层 TD 输出会因此失败，但**非感知层问题**（Tier1 假数据冒烟全通过、sensor_proxy 真实转发验证通过）。
**解决/规避**: **根因（2026-08-13 定位）**：WSL 下 Fast DDS multicast 发现极慢，`load_node` 与 `gazebo_ros2_control` 的 `get_parameters` 等**同步 service 调用**等 discovery 超时/卡死；感知层节点能正常起是因为只用异步 topic 通信。**根治：`export ROS_LOCALHOST_ONLY=1`**（单机仿真只走 loopback，绕开 multicast，同步 service 立即响应——实测 controller_manager 从 240s 超时 → 0s 全 active；实跑验证脚本 `/tmp/run_verify.sh` 已内置）。此开关须对 launch 的**所有子进程一致生效**——2026-08-14（A5）起已在四个 bringup 顶层与 plant 两个 launch 内 `SetEnvironmentVariable` 注入（bringup 树内 rviz2/感知/控制/算法/HMI 与物理层同组）。**注意：从终端手动跑 `ros2 node list` / `ros2 control` 等 CLI 时，该终端也须 `export ROS_LOCALHOST_ONLY=1`（或先 `ros2 daemon stop`），否则 CLI 与节点不在同一 DDS 组、`node list` 只看到 `/gazebo`。**辅助兜底（run_verify.sh 经验）：rsp 组件缺失时起**独立 rsp**（`robot_description` 用 `--params-file` 传，`-p` 直接覆盖含空格 XML 会解析失败）；launch 内 spawner 在 controller_manager 就绪前超时挂掉时用 **spawner CLI 逐个补齐**（`ros2 run controller_manager spawner <名> --controller-manager-timeout 45`，自带 load→configure→activate 状态机；rclpy 手搓 `SwitchController` 在 WSL 下 25s 无响应不可靠）；plant_sim 组件缺失时 `ros2 component load /plant_container ylr1d_plant ylr1d_plant::PlantSim`（**必须 3 个位置参数**，漏 plugin 报 `required: plugin_name`）。遇 load_node timeout 也可重启一次（偶发成功）。

**22.5 ROS_LOCALHOST_ONLY 分组一致性定论（2026-08-18，用户怀疑 + 实测证实）**：`ROS_LOCALHOST_ONLY=1` 把节点限制在 loopback DDS 组——**单独启动的进程若不设同样的环境变量，与 bringup 树不在同一组、互相看不见**（实测：A 设 / B 不设，两个 rclpy 节点互发 topic 均收不到）。**坑**：hmi 各 launch（hmi_control/hmi_plan/hmi_moveit/hmi_algorithm/hmi_sensor/hmi_translate）原本只注入 `LIBGL_ALWAYS_SOFTWARE`，随 bringup 起时继承环境变量无碍，但**单独 `ros2 launch ylr1d_hmi hmi_moveit.launch.py` 起（用户习惯）就出组** → 症状"hmi 发指令 bridge 收不到、看不到 result、完全无法控制"，与 moveit/bridge 代码无关（`/plan/moveit/moveit_goal` 显示 Publisher=1 Subscription=0 是跨组证据）。**修复：6 个 hmi launch 全部补 `SetEnvironmentVariable("ROS_LOCALHOST_ONLY","1")`**（与 bringup 一致）。同理，任何要单独起的 ROS2 进程（CLI、脚本、独立 launch）都要带同一环境变量。对照实验脚本思路：两个 rclpy 节点一个 export 一个不 export，互发 topic 验证可见性。

#### 23. 三阶段接口重构（算法层 ↔ 控制层六接口 + 反馈源切感知层 + service 调参）实施记录
**约定**（方案见 `notes/archive/algorithm-control-interface-refactor.md`，2026-08-10 已实施）：
- **每组六接口**：`/ctrl/<组>/{input,feedback,output,param,state}` + `/ctrl/<组>/command` srv。input/output 为 `std_msgs/Float64MultiArray`（input=期望、output=加速度 u）；feedback 为感知层 `ylr1d_perception/JointGroupEstimate`（**A-lite 模型状态**：100Hz 内部积分 + 感知层测量（p,v）独立互补校正 αp=0.2/αv=0.1，2026-08-13 起取代 TD 直通、2026-08-14 解耦为独立 `ModelPredictor`，见控制层 README 6.7）；param/state 为算法层自建 `{Steering,Wheel,Torso,Arm}Params/State`（10Hz 只读展示）；command 为 `ylr1d_algorithm/ControllerCommand` srv（`get_param/set_param/get_state/set_state/reset_state/initialize`）。
- **算法层改成接口包**：`rosidl_generate_interfaces` 的 target **必须等于 `PROJECT_NAME`**（运行期类型支持按消息包名找库 `libylr1d_algorithm__rosidl_typesupport_fastrtps_c.so`），核心共享库改名 `ylr1d_algorithm_core` 规避 target 同名冲突；5 节点/组件用 `rosidl_get_typesupport_target` 链接（陷阱 21）。消费方（控制层）`ament_target_dependencies` 传算法层 msg/srv + 感知层 msg + description config。
- **反馈源切感知层**：控制层 `GroupForwarder` 订阅 `/perception/joint_state`（`JointStateEstimate`）按组提取，经 **A-lite 模型预测器**（独立纯算法类 `ModelPredictor`，`predictor/`；100Hz 虚拟对象积分 + 感知层测量（p,v）独立互补校正，2026-08-13 引入、2026-08-14 解耦）封装回发 `/ctrl/<组>/feedback`，不再从 `/joint_states_filtered` 提取；plant_sim 初始化源原改回物理层自有 `/joint_states_filtered`（plant 不依赖感知层），**批次 2（2026-08-12）又改订阅感知层 `/perception/sensors/joint_states`**（`joint_state_receiver` 输出，数据源收敛感知层，须感知层并入）。bringup 与 `test_stack.launch.py` 均 0s 并入感知层（不入并则一键闭环断链，算法层以首帧期望作 feedback 误差 0 不运动）。
- **运行期调参仅 service**：节点参数声明与 `add_on_set_parameters_callback` 已删（无 `ros2 param set`）；经 `/ctrl/<组>/command` 或控制层聚合 `/control/<节点名>/command`（带 `group` 字段路由）。param topic 只读展示。
- **PID D 项用反馈速度**：`Controller::compute_with_velocity(input, feedback, velocity, dt)` 默认虚方法回落 `compute`；位置组 override 用反馈速度（**A-lite 模型速度**，100Hz 平滑积分 + TD 速度低频校正（αv），贴真实被控对象速度，2026-08-13 起）做 `-kd·v` 阻尼；wheels（速度组）走默认 compute。
- **joint_config.hpp 迁至 description**：分组/限位单一来源从算法层移到 `ylr1d_description/config/joint_config.hpp`（命名空间 `ylr1d_description`），感知层依赖单向无环（感知层→无算法层；算法层→感知层 msg + description config）。改接口/话题/启动时序须同步改两 bringup launch + 各包 README + 本节（陷阱 16）。

#### 24. wsl.exe bash -c 内嵌复杂命令输出异常，用脚本文件替代
**问题**: `wsl.exe -d Ubuntu-22.04 bash -c 'nohup ros2 launch ... & PID=$!; sleep; ros2 node list | grep _sim; ...'` 这类内嵌**管道/后台/分号**的长命令，stdout 会部分丢失或乱码（`node list` 等结果消失，退出码异常），并混入无害的 stty 警告 `screen size is bogus. expect trouble`（WSL 无 tty 启动时 COLUMNS 被设成大值，可忽略）。
**解决**: 复杂核查/验证序列写成 `/tmp/xxx.sh`（Write 工具直接写进 WSL 路径 `\\wsl.localhost\...\tmp\xxx.sh`），再 `wsl.exe bash -c 'bash /tmp/xxx.sh'` 执行，输出稳定。另注意 pkill 自匹配的延伸坑：**脚本文件名含目标名也会自匹配**（如 `pkill -f "[s]ensor_panel"` 会杀掉运行 `sensor_panel_check.sh` 的 bash，因为脚本路径含真实字面量 "sensor_panel"）——清理脚本/命令里用 `pkill -x 可执行名`（精确 comm）规避。

#### 25. create_subscription 返回值必须持有（SharedPtr 临时析构即销毁订阅）
**问题**: 感知层批次 3 `health_aggregator.cpp` 用 `create_subscription(...)` 后**不保存返回值**，临时 `SharedPtr` 表达式结束即析构 → 订阅被销毁。症状隐蔽：节点正常启动、定时器正常发布，但 `ros2 topic info -v <话题>` 显示 `Subscription count: 0`，聚合输出恒 `unavailable`（Tier1 测试里 health 断言恰好靠字符串巧合通过，未暴露）。
**解决**: 订阅句柄存成员（如 `std::vector<rclcpp::Subscription<T>::SharedPtr> subs_`），循环创建时 `subs_.push_back(create_subscription(...))`。排查手段：`ros2 topic info -v` 查对端订阅/发布计数，比看日志更快定位"发布了但没人收到"。订阅创建完应自查对端计数（诊断脚本实证片段 'OK' 到达后再断言聚合）。

#### 26. 轮式里程计 AtA 奇异（批次 5 C1）+ steering/wheels 单路即可
**问题**: 感知层批次 5 `wheel_odometry` 解超定最小二乘 `X=(AᵀA)⁻¹Aᵀb`，四轮 steering 角一致（纯平移）时 A 的 sinα 列无约束 → AtA 奇异 → 除零 NaN（Tier1 纯平移场景直接 NaN）。另注意：**steering 与 wheels 同在一个 JointState 消息**（`joint_state_receiver` 输出），C1 **单路订阅**即可天然时间对齐，**无需 message_filters 双流同步**——D1 同步框架仍留待真正多路汇聚需求，别为 C1 加多余同步。
**解决**: AtA 加 ridge 正则化 εI（ε=1e-6，相对 AtA 量级 ~0.1~4 可忽略）得最小范数解，不可观测方向（纯平移的 Vy）给近零解，避免除零。C1 正确性已由 Tier1 假数据 3 场景验证（纯平移/纯旋转/侧移），真实数值验证被 URDF 轮子打滑（无接触摩擦）+ LF offset 阻塞（物理层遗留，非 C1）。

#### 27. 鲁棒性编号注册表（避免跨层撞号）
**约定**: 各层鲁棒性编号（字母+序号）只在**本层文档/代码注释**内使用，跨层引用必须写明层级前缀：
- 控制层：`C1` 输出新鲜度门限（500ms）/ `C2` 断流停止转发 / `C3` 下游订阅检测 / `C4` loop_hz 校验 / `X1` 健康话题；
- 转译层：`T1` 转向硬超时（30s）/ `T2` 数据流断流检测（2s，仅"曾收到后断流"）；
- 物理层：`P1` spawner 90s 超时 / `P2` 启动前预清理 / `P5` rviz 可选；`E_STOP` 急停；
- 感知层：`D1` LatestTimeSync 同步框架 / `D2` Throttler / `D3` 核查（维度/数值/范围/新鲜度）/ `E1` 内部前缀约定；**`C1`（轮式里程计）是感知层用法**，与控制层 C1 不同——引用须写"感知层 C1"；
- HMI：`H1` 传感器独立判活 / `H2` list_controllers 失败升级 / `H3` 新鲜度显示；
- 测试：`Tier0/1/2`、`D3/D4`（跨层配置一致性，与感知层 D3 不同）、`E1-E6`（full_flow 证据链，与感知层 E1 不同）；
- 方案/审查文档：`notes/archive/robustness-improvement-plan.md` 的 P0/P1/P2 是**实施优先级**（非物理层 P 系）；`notes/archive/robustness-audit.md` 编号是**历史审计编号**（与现行 README 不完全对应）。
增删/引用编号时对照本表，禁止在其它层复用已占用的字母序号。

---

#### 28. 规划层 P2 nav2 实跑经验（2026-08-15/16，bringup_plan + nav.launch.py）
**问题背景**：nav2 导航栈在 WSL 实跑踩坑多且症状隐蔽（lifecycle 卡住/导航必失败/机器人不动）。以下为逐条根因与修复，其中 2/3/4/6 已被 `ylr1d_test` 的 **Tier1 plan 测试静态校验捕获**（`--tests plan`，无 Gazebo，20s）：
1. **残留进程污染发现（最隐蔽）**：上次被 kill 的 bringup/nav2 子进程没死透（`ros2 launch` 被杀时孙进程成孤儿），多实例同名节点并存 → 新 lifecycle_manager 的 service client 抢到旧实例的 `/controller_server/get_state`（永不响应）→ 卡 `Waiting for service ...`；bridge 也找不到 `/chassis_move`。**排查先 `ps aux | grep -E "nav2_|gzserver|cmd_vel_bridge"` 清残留**。注意 **`pkill -x` 对 >15 字符进程名无效**（comm 截断：`lifecycle_manager`→`lifecycle_manag`、`controller_server`→`controller_serve`），须 `pkill -f "[n]av2_lifecycle_manager/lifecycle_manager"`（路径模式 + 字符类防自匹配）。
2. **bt_navigator plugin_lib_names 必须用本 deb 集存在的库名**：`nav2_transform_availability_bt_node`（新版本名）在本机不存在 → configure 抛 `Could not load library: libnav2_transform_availability_bt_node.so` → lifecycle 激活失败 `does not have error state implemented`。**最稳：照抄官方 `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` 的 plugin_lib_names 全表**（本机 1.1.20 全部存在，含 `nav2_transform_available_condition_bt_node`、`nav2_path_expiring_timer_condition` 等）。
3. **costmap 参数须双层命名空间**：nav2 Humble 要求 `global_costmap: global_costmap: ros__parameters:`（local 同理）。单层 `global_costmap: ros__parameters:` 会被**静默忽略**用默认值（robot_base_frame=base_link → TF 报 `Invalid frame ID "base_link"`、plugins 变默认三个）。改 yaml 后重编 `ylr1d_plan_nav`（陷阱 16）。
4. **bt_navigator default_server_timeout 默认 20（毫秒！）**：BT action 节点等 goal ACK 仅 20ms，WSL 负载下 planner/behavior 响应常超 → 导航必失败（`Timed out while waiting for action server to acknowledge goal request`）。须设 `default_server_timeout: 20000`；顺带 `bt_loop_duration: 100`（10Hz tick，默认 100Hz 负载下 tick rate 超限刷屏）。
5. **cmd_vel_bridge 抢占风暴（机器人不动真凶）**：bridge 每 50ms tick 重复发相同 goal → 转译层"新 goal 抢占旧 goal"不断 abort 正在执行的 goal → 转向永远不到位、机器人不动（日志特征：`chassis_move superseded by new goal` 高频刷屏 + 转译层 `succeeded: stopped` 但车不动）。**修复：在途标志（发送后响应前不叠加）+ 同目标 500ms 重发节流**；行为已被 plan 测试断言（持续同 cmd_vel 3s 间隔 ≥300ms）。
6. **amcl initial_pose 必须等于机器人在地图中的真实坐标**：nav_test 地图生成器把 world(0,0)（机器人出生点）画在图像中心 → **机器人 map 坐标为 (0,0)** 而非 (6,4)。initial_pose 填错 → amcl 在角落 → 全局代价地图 `Robot is out of bounds of the costmap!` → 规划失败。**改 amcl.yaml 后重编 ylr1d_perception**；实跑中可用 rclpy 发 `/initialpose` 热重定位（CLI `ros2 topic pub` 在负载下 context 错不可用）。
7. **WSL 后台进程随 wsl.exe 会话退出被杀**：`wsl.exe bash -c 'nohup ros2 launch ... &'` 会话一结束子进程全没（日志 0 字节）。**须用持久后台任务**（pwsh `run_in_background: true` 保持 wsl 会话存活）跑 bringup/nav2/goal 发送。
8. **重负载下 sim 时间 ~1/30 实速**（load ~19 时）：一切 sim 计时在真实时间极慢，action 超时、等待都要放宽；导航 1.5m 可能要 2+ 分钟真实时间。
9. **ros2 CLI 在负载下不可靠**：`topic echo/hz/param get` 空结果或 `context is not valid`（陷阱 17 强化）——**数据流验证一律用 rclpy 脚本**（写 /tmp/*.py）。
10. **P26 摩擦修复**（2026-08-16）：URDF 四滚动轮加 `<surface><friction><ode><mu>1.0` 后，轮式里程计 spurious vy 从 ~0.12 降到 ~0.05 m/s，EKF 漂移从 5.3m/43s 降到 0.4m/10s——**改善了但未根除**（mesh 碰撞接触仍不完美）；amcl 会反向补偿（map→odom 反向），全局导航可用，局部代价地图/长时间漂移仍受影响（后续可换 cylinder 碰撞原语或调 EKF 只融 vx）。**注意该改动改变了物理动态**，此前在打滑条件下调好的控制参数可能需要复核。
11. **nav 场景禁用相机渲染（YLR1D_DISABLE_CAMERAS=1）**：nav 测试不需要相机，bringup_plan 顶层注入环境变量，plant launch 的 xacro transforms 把相机类 sensor 的 update_rate 置 0（Gazebo 不渲染）→ **gzserver CPU 从 ~1286%（12 核软渲染 9 相机+3 点云）降到 ~233%**。代价：感知层 image/pointcloud 接收节点无数据（health 报警），导航核心链路（joint_states/odom/scan/imu）不受影响。bringup_control 不注入该变量（HMI 传感器面板需要相机）。
12. **感知层 estimator 偶发启动时序问题**：bringup 偶发 `control_feedback_guard: upstream input stale`（estimator 全程没发布，guard 不发布 /perception/joint_state → 控制层开环 → 算法层输出 0 → 底盘不动）。**kill 掉 estimator 进程手动 `ros2 run ylr1d_perception joint_state_estimator` 重启即恢复**（手动起能正常发现 receiver，bringup 同批起的偶发发现失败——WSL Fast DDS 长期运行发现不可靠）。下次 bringup 重启通常自愈。
13. **端到端导航（NavigateToPose）实跑结论（2026-08-16）**：**链路已完全打通，机器人实际移动并接近目标**。修复链：① wheels 平移不驱动是**感知层 estimator 启动时序问题**（guard stale → 控制开环 → 输出 0）——kill 重启 estimator 或重启 bringup 自愈；② **T1 转向超时（10s 太紧）**：nav2 的连续 cmd_vel 经 bridge 分段时 ROTATE/TRANSLATE 交替会重置转向 → STEER_TIMEOUT 放宽到 30s；③ **bridge 还需两级防抖**（缺则 22 次 steer timeout）：mode 切换滞回（20 tick=1s real）+ **ROTATE 内 speed 变化不重发**（nav2 wz 连续变化若重发 → 转向反复重置）。最终验证（run16）：steer timeout 0、机器人实际移动（y 0.925→1.11、yaw 转向完成）、目标 (0,1.5) 最终偏差 ~0.43m——**偏差来自 EKF 静止漂移（P26，nav2 基于 odom 判定到位）**，属物理层遗留；nav2 层（bridge/参数/配置/静态测试 13 项）已做扎实。
14. **感知层传感器 frame 修正（2026-08-16，用户反馈 LaserScan 与地图对不上）**：**gazebo_ros 插件的 `<frame_name>` 在本机未生效**——Gazebo 输出所有传感器话题的 header.frame_id 均为父链 `Link_Base`（雷达应 Link_RadarSensor、IMU 应 Link_IMUSensor 等）→ rviz 里 scan 显示在车体中心（错位）、local costmap 传感器数据位置错。**修复**：description `sensor_topics.hpp` 加传感器 TF 静态帧常量（kCameraFrames/kRadarFrame/kSonarFrames/kImuFrame，与 xacro link 名一致，TF 静态帧均存在）；感知层 `SensorTopicSpec` 加 `frame` 字段，`sensor_receiver.hpp` 转发时把 header.frame_id 修正为 spec.frame（接收层一处修，分发层/消费方无需改）。**注意 `/tf_static` 是 transient_local**，volatile 订阅看不到（陷阱 6）——TF 里 sensor frames 一直在。
15. **规划 rviz2 重构（2026-08-16，用户反馈）**：以用户调整的 `nav_display.rviz` 布局为基础，存为 `ylr1d_description/rviz/nav_display.rviz`（随包装，nav.launch.py 指向它）：去 Navigation 2 面板（纯观测无按钮）、去 RobotModel（省算力，用 AMCL Pose/粒子云表示位姿）、Odometry 改 Keep=1（去拖尾）、显示项标注坐标系/话题（Global Costmap (map 系) 等）、加 Perception Health 文本显示（/perception/health 系统观测）。**bringup_plan 默认 rviz:=true**（用户要求；导航场景相机已禁用，rviz 软渲染需 LIBGL_ALWAYS_SOFTWARE=1——nav.launch.py 的 rviz Node 已注入）。plan 测试校验更新（8 显示项 + 无按钮 + 无 RobotModel + 无拖尾）。**rviz2 闪退修复（2026-08-19，用户实测）**：软渲染下 **Map（/map 静态地图）显示项渲染致 rviz2 闪退**——`nav_display.rviz` 的 Map 显示项 `Enabled` 改 false（地图信息由 Global/Local Costmap 显示承载），plan 测试加断言 `Map Enabled=false` 防回归；旧 `nav_test.rviz` 已删（死配置：仅 localization `loc_rviz` 引用、从未启用），`localization.launch.py` 的 loc_rviz 改指 `nav_display.rviz`。


16. **launch include 参数名冲突污染（2026-08-16，用户反馈"重复打开 rviz / 布局没生效"）**：`IncludeLaunchDescription` **共享同一个 context**，被 include 的子 launch 里 `DeclareLaunchArgument("rviz", ...)` 会**覆盖父层同名参数**。bringup_plan 声明 rviz（默认 true，透传给 nav 规划 rviz）后：include plant 传 `("rviz","false")` 把父层 rviz 覆盖成 false → 规划 rviz 不启动；include localization 时它的 rviz（默认 false）又被父层 rviz=true 覆盖 → 多起一个 rviz（当时为 nav_test.rviz，2026-08-19 已删，localization loc_rviz 现指 nav_display.rviz）。**修复：plant 参数改名 `plant_rviz`、localization 改名 `loc_rviz`，只有 nav 的 `rviz` 由 bringup_plan 透传**。plan 测试已加断言防回归。**推广：include 子 launch 时，任何会透传/同名覆盖的参数都要改名隔离，别用通用名 rviz/world。**
17. **雷达安装 rpy=-2.9674rad（-170°）+ 高度两难（2026-08-16，用户反馈"LaserScan 与地图障碍物不一致"）**：Gazebo ray sensor 的 scan 0° 指向 sensor **+x**，而 xacro `Joint_Base_to_RadarSensor` 的 rpy 原为 `0 0 -2.9674`（-170°）→ scan 0° 与车体 +x 偏差 170°，amcl 补偿到错误位姿（实测定位到 box_a(-3,1.8)、yaw 漂移）→ rviz 里 scan 相对地图整体偏转。**修复：rpy 改 `0 0 0`**（scan 0°=车体 +x）。**雷达高度两难**（实测）：z=0.236 前方开阔（0° 能看到东墙 5.7m）但侧后（-60°~-180°）被轮子/转向架/底盘挡（自身点 0.9-2.7m）；z=0.55 侧后干净但正前方 ±40° 被躯干 Body2 挡（0.1m 盲区，box_c/cyl_d 全在盲区）——**保持 z=0.2364**，侧后自身点靠 amcl `laser_min_range` 过滤 + rviz 里属车体自身（视觉上已知）。改传感器安装 rpy/xyz 前先想清 ray 0° 指向 +x 与车体系的关系。
18. **amcl/EKF 时间戳与漂移三件套（2026-08-16，scan 与地图对齐的剩余修复）**：① amcl `transform_tolerance` 语义是"把发布的 map→odom TF 时间戳帖到 scan_stamp+tolerance"，默认 1.0 会帖到未来 1s，与 EKF 的 odom→Link_Base（输入时间戳）不同步 → 到处丢帧（earlier than all data in transform cache）；**设 0.1**。② EKF 输出 TF 滞后 scan ~40ms（joint_states→wheel_odometry→EKF 处理链）→ 按 scan.stamp 查 map→base 需未来外推失败；**设 `transform_time_offset: 0.1`** 补偿。③ amcl 静止时 `update_min_d=0.25` 不更新粒子 → EKF 漂移（P26）不被 scan 修正，map 系位置漂 1m+/min；**设 update_min_a/d=0.05** 每帧修正 + **laser_min_range=0.5** 忽略车体自身近点（0.1m）干扰匹配。效果：amcl 定位正确稳定（map→Link_Base≈(0,0)、yaw≈0、漂移 <0.03m/20s），box_a/cyl_b/西墙/南墙命中。**这些参数都改了 install 版，重编 ylr1d_perception（陷阱 16）。**

19. **旋转时 scan 与 odom 转速不一致、转一周不回原位（2026-08-16，用户实机旋转观察）**：rviz 里 LaserScan 的旋转速度与 odometry 不一致、scan 与障碍物不重合、旋转 360° 后 scan 不闭合。**定性**：这是**里程计 yaw 标定 + EKF 融合权重**问题，不是规划层框架问题——wheel_odometry 的 vyaw 解算存在比例误差（全向轮几何/摩擦 P26），EKF 融合后 yaw 速率跟随该误差 → 累计航向漂移；amcl 虽 update_min=0.05 每帧修正，但旋转时 scan 特征快速变化、匹配滞后，不能完全拉回。**后期收敛方向（按易到难）**：① 增大 EKF `imu0` 的 vyaw 权重（IMU 角速度比轮式里程计准，改 ekf.yaml，最容易）；② 标定 wheel_odometry vyaw 比例因子（感知层 C1，中等）；③ P26 根因（换 cylinder 碰撞原语/调摩擦）。**影响面**：对规划层框架搭建无影响（架构已通、端到端导航已达标），主要影响转向精度与长时间航向漂移，属于后续精度调优项。

20. **cmd_vel_bridge 状态机抽象为 command_converter 模块（2026-08-16，解耦重构）**：把原来耦合在 cmd_vel_bridge 里的决策逻辑抽成独立模块 `include/ylr1d_plan_nav/command_converter.hpp` + `src/command_converter.cpp`，与 ROS 通信彻底解耦。分层：`NavVelCommand`（输入，车体系 vx/vy/wz）→ `CommandConverter` 抽象接口（update/decide/note_sent/request_stop）→ `SegmentedMotionConverter` 三态分段实现 → `MotionCommand`（输出，enum mode+direction+speed+duration）。bridge 节点只保留 ROS 通信骨架（订阅 cmd_vel、发 action、goal 生命周期、wz 取反），持有 `unique_ptr<CommandConverter>`。**改复杂转换逻辑（边转边平移分解、多段路径、自定义分段）只改 converter 实现 + bridge 构造函数里 `make_unique<XxxConverter>()` 一处，ROS 通信零改动**。旧 `cmd_vel_state_machine.hpp` 已删，`--logic-test` 18 条断言（原 13 条语义不变 + request_stop 1 条 + 2026-08-20 新增 R0 暂停后同向重启重发 / R1 ROTATE 反号与大幅变化重发 / R2 reset_sent 后重发 4 条）随模块独立。
21. **STOP 语义修正（2026-08-16，用户反馈"状态机喜欢发 STOP"）**：转译层 `MODE_STOP`=转向到停车角并**立即 succeeded（goal 结束、转向复位）**，`MODE_TRANSLATE`+speed=0=保持方向、轮速 0、**goal 不结束（可恢复）**。原状态机 `cmd_vel≈0 → STOP`，nav2 中间减速/重规划一输出 0 就发 STOP → 转向复位、下次重新转向（打断导航）。**修正：cmd_vel≈0 → TRANSLATE+direction=0+speed=0（暂停）；STOP 改为显式 `request_stop()` 触发**（接口方法，仅导航结束/上层停止信号/hmi 停止按钮时调用一次；bridge 暂未接自动触发，"结束"信号源后续接 nav2 goal result 或 hmi）。中间暂停用 TRANSLATE speed=0 保持姿态可随时恢复移动。
22. **bringup_plan 拆分为 bringup_plan_nav + bringup_plan_moveit（2026-08-16，用户要求单独测试互不干扰）**：`bringup_plan.launch.py` 已删，替换为两个并列 launch（受同一把 bringup 互斥锁保护，不能同时起；单独测试各自独立）：
    - `bringup_plan_nav.launch.py`：物理 + 感知 + 定位 + 转译 + nav2 + bridge（原 bringup_plan 内容，导航测试用，`world:=nav_test.world`）；
    - `bringup_plan_moveit.launch.py`：物理 + 感知 + 转译（**无定位/nav2**，机械臂规划测试用，P3 在此追加 moveit_config + moveit_bridge + moveit rviz）。
    改 launch 后重编 ylr1d_bringup（陷阱 16）；**删 src 里的 launch 后 install 旧副本残留**（colcon 增量 build 不删 install 旧文件）——须手动 `rm install/<pkg>/share/<pkg>/launch/旧文件` 或删 install 重装。

23. **rviz Nav 状态面板 + hmi 运动可视化（2026-08-16，用户要求完善 rviz 插件与 hmi）**：
    - **NavStatusWidget**（`ylr1d_hmi/panels/nav_status_widget.{hpp,cpp}`）：展示三类节点"基本状况"（在线 get_node_names 2s 轮询 + 关键话题新鲜度 TopicStatus）——① nav2 本体（planner_server/controller_server/bt_navigator/behavior_server，lifecycle 节点以在线判定）；② plan 层指令转换（cmd_vel_bridge 在线 + /cmd_vel 数据）；③ perception nav（map_server/amcl/ekf_filter_node 在线 + /amcl_pose//odom 新鲜度 + /perception/health）。
    - **NavStatusPanel**（rviz Panel 插件，照 MonitorRvizPanel 模式）：宿主 NavStatusWidget，注册在 plugin_description.xml（追加 class），随 nav_display.rviz 的 Panels 段预加载。**rviz 自定义面板不能 GUI 手动保存（会空），必须直接改 YAML 的 Panels 列表**（用户强调，与之前 rviz 加插件一致）。
    - **ChassisDirectionWidget 抽取**（`chassis_direction_widget.hpp`）：原内嵌于 translate_panel.hpp 的底盘运动可视化（平移箭头/旋转弧/停车方块），抽成独立头文件供 translate 面板与 plan 面板共用；plan 面板把 nav2 /cmd_vel 映射成 mode/direction/speed（同 command_converter 三态）后 setParams。
    - **PlanPanel 完善**：嵌入 NavStatusWidget + ChassisDirectionWidget（发 NavigateToPose + 取消 + goal 状态 + 位姿 + 运动可视化 + 节点状态）。
    - **注意**：NavStatusWidget 订阅 nav_msgs/Odometry + geometry_msgs，ylr1d_hmi_monitor_panel 库依赖需加 `geometry_msgs nav_msgs`（原无），ylr1d_hmi_plan 目标加 `std_msgs`；新增源文件 chassis_direction_widget.cpp/nav_status_widget.cpp 要同时加进两个目标。

24. **rviz Panel 订阅与 display 同话题冲突（2026-08-16，NavStatusPanel 加载失败根因）**：rviz Panel 里创建的 rclcpp::Node 若用**全局 context**，且订阅话题与 rviz 的 display 重叠（如 NavStatusWidget 订阅 /amcl_pose、/odom，而 rviz 的 AMCL Pose/Odometry 显示也订阅），rcl 报 `create_subscription ... incompatible type`（typesupport 来源不同：rviz display 静态 vs Panel 动态加载）→ rviz "Could not load display config"、面板不显示。**修复：Panel 的 node 用独立 context**（`auto c=std::make_shared<rclcpp::Context>(); c->init(0,nullptr); NodeOptions.context(c)`）。MonitorRvizPanel 无此问题：其订阅话题（/health、/perception/health 等）与 rviz display 不重叠，仍用全局 context。**规律：rviz Panel 订阅的每个话题都要确认不与 rviz display 重叠，重叠就必须独立 context。** 顺带：规划 HMI 已并入 bringup_plan_nav 的 50s 分层（`_include("ylr1d_hmi","hmi_plan.launch.py")`，依赖 nav2 40s 激活），无需单独起 hmi_plan。

25. **rviz2/hmi 职责划分 + rviz 布局（2026-08-16，用户明确定位）**：
    - **rviz2 = 系统稳定性监视**：NavStatusPanel 只关注 **nav 相关功能节点**（nav2 本体 planner/controller/bt/behavior + 指令转换 cmd_vel_bridge + 高级感知 map_server/amcl/ekf），**不含具体传感器健康**（/perception/health 已从 NavStatusWidget 移除——那是具体传感器层面，不是 nav 相关）。
    - **hmi = 控制过程可观**：PlanPanel 只关注**控制指令 + 时间**（目标 x/y/yaw + 执行时长 + goal 状态 + 位姿 + 运动箭头），**不含节点状态**（NavStatusWidget 已从 PlanPanel 移除）。
    - **rviz 布局**（插件右侧、视图中间、左侧隐藏）：`Hide Left Dock: true` + `Hide Right Dock: false` + QMainWindow State 里面板名放右侧。**坑**：用户 GUI 手动保存 rviz 会留下旧面板名的 QMainWindow State 残留（本机 nav_display.rviz 残留 "Navigation 2"）——须直接改 YAML，用 Python 字节级替换 QMainWindow State 十六进制里的 UTF-16 面板名（格式：4 字节长度前缀 + UTF-16LE 数据；"Navigation 2"=00000018+UTF16，"NavStatusPanel"=0000001c+UTF16；替换后整体变长，Qt 按长度前缀读取能正确继续解析）。
    - 定位一句话：**仅对 plan 层 nav 部分负责；rviz2 管系统稳定性（是否正常运行），hmi 管控制（指令/时间/执行过程）。**

26. **rviz 面板节点全 offline + hmi 指令记录（2026-08-16）**：① `get_node_names()` 返回**带前导斜杠的全限定名**（`/planner_server`），硬编码比较名须补 `"/"`（`online.count("/"+name)`）——漏掉则所有节点判 offline。② hmi（PlanPanel）加**指令下发记录**（`QPlainTextEdit` 只读 + `appendLog`，时间戳用构造起运行秒数 `[xx.xs]`）：发送 NavigateToPose 记录目标、cmd_vel mode 变化记录底盘指令（平移/旋转/暂停 + dir/speed，用 `last_mode_` 追踪）、goal 结果记录"导航结束: succeeded"——只显示不落盘，让控制过程清晰。定位：hmi 管控制（指令/时间/执行过程），rviz2 管系统稳定性。

27. **MoveIt2 P3 框架（2026-08-17，用户要求：脚本从 xacro 生成 moveit_config + 静态测试 + 与 nav 平级）**：
    - **gen_moveit_config.py**（`ylr1d_plan_moveit/scripts/`，绕开 MSA GUI 卡死 + 手头 URDF 版本不明）：复用项目 xacro 处理逻辑（resolve_yaml_refs + `xacro.process_file`，文件内 `<xacro:ylr1d prefix=""/>` 自调用）得到**与当前 xacro 严格一致的纯 URDF**，再按硬编码规划组关节名生成 SRDF（torso 4 / left_arm 7 / right_arm 7 关节）+ 末端执行器（Link_LeftArm7/RightArm7）+ 相邻碰撞矩阵 + kinematics.yaml(KDL)/joint_limits.yaml/ompl_planning.yaml/pilz_cartesian_limits.yaml/moveit_controllers.yaml。**三个坑**：① `${controllers_yaml_path}` 占位符（gazebo 插件参数）须 replace 成空；② ros2_control 块里的 `<joint name=...>`（command_interface，无 parent/child）会被 `iter("joint")` 一起遍历——parse 时跳过无 parent/child 的；③ kinematics.yaml 生成时 block 要缩进（否则 `kinematics_solver` 跑到顶层，yaml 解析成组为 null）。
    - **MoveItConfigsBuilder 默认期望的文件**：pilz_cartesian_limits.yaml + `*_controllers.yaml`（缺则 to_moveit_configs() 报错/告警）——脚本都要生成占位（执行不走 FollowJointTrajectory，走转译层 ArmMove）。
    - **moveit_bridge**（`ylr1d_plan_moveit`，与 `ylr1d_plan_nav` 平级、对称）：订阅 /moveit_goal（PoseStamped）→ MoveGroupInterface 规划 → **TrajectoryDiscretizer 模块**（StepDiscretizer 等距采样，接口可换）离散 → 逐 waypoint 发 ArmMove（等 succeeded 再发下一个）。MoveIt 只负责"算"，执行走转译层，不绕开控制闭环。
    - **moveit_test.py**（`ylr1d_test`，与 plan_test 平级，Tier1 静态无 DDS）：校验安装副本的 SRDF（3 组关节与 URDF 一致、末端执行器 frame、**夹爪关节不进组**）、kinematics(KDL)/joint_limits/ompl 覆盖、纯 URDF 含规划组关节。注册进 run_all.py（id=moveit，DEFAULT_TESTS 加 moveit）。
    - **夹爪**（用户约定）：不进规划组，由 gripper_goal（GripperGoal part+open）独立开合，与关节规划解耦。/moveit_goal 是单纯臂移动（不涉夹爪）。grasp_goal 序列状态机（GripperActionSequence）已于 2026-08-18 删除。
    - **nav 测试已有**：plan_test（--logic-test 14 条 + 静态配置校验），moveit 与其平级。
    - **结构对齐（2026-08-17 用户要求）**：新增 `launch/moveit.launch.py` 一次起 move_group + moveit_bridge（与 nav.launch.py 对称），bringup_plan_moveit 40s 层只 `_include` 它（原 bridge 裸 Node 已删）。**端到端已跑通**：/moveit_goal 左臂移动 6.28 rad、/grasp_goal 抓取序列 APPROACH→OPEN→GRASP→CLOSE→RETREAT→DONE 完整执行，moveit_test 14 静态 + 6 状态机全 PASS。实跑踩坑（bad_weak_ptr/CHOMP 回退/planning frame/服务端 IK/current_state_monitor 时钟/use_sim_time/spin_thread/header-only tf2/TimePointZero/WSL 编译纪律）见包 README。
    - **抓取分步原子接口（2026-08-17 用户要求"能在某步停留"；2026-08-18 grasp_goal 删除）**：`/gripper_goal`（GripperGoal part+open，夹爪开合），与 `/moveit_goal`（臂位姿）组成原子接口供**决策层编排**（想在哪步停就在哪步停）；grasp_goal 序列宏已删（决策层自行编排）。两接口：/moveit_goal(臂位姿) + /gripper_goal(夹爪)。
    - **P4 多组 + 服务 API（2026-08-17；2026-08-18 位姿统一 Link_Base 系 + grasp_goal 删除）**：三规划组（torso/left_arm/right_arm）接口带 part（0=躯干 1=左臂 2=右臂，与转译层 ArmMove 一致），话题统一 `/plan/moveit/` 前缀；msg：MoveItGoal（part+pose / part+joint_positions）、GripperGoal（ylr1d_plan_moveit 变接口包）。**位姿目标统一 Link_Base 系（机器人基座系），不再用 Link_Body2 臂基座系**（hmi 输入/显示/msg 注释一致）。**规划走 move_group 底层服务**（/compute_ik avoid_collisions 求无碰撞解 + /plan_kinematic_path 关节目标）**全异步回调**——踩坑链：MoveGroupInterface 多实例内部 executor 冲突（plan 卡死）、wait_for 不处理响应（超时）、MultiThreaded 线程池被阻塞 wait 耗尽、spin_until 嵌套 spin 卡死，**全异步回调链是正解**。抓取状态机 GripperActionSequence 加 part。端到端 PASS：左臂 2.44 rad / 右臂 2.16 rad / 躯干 0.083 / 夹爪开合；多帧 goal 有在途标志（处理中忽略新目标）。rviz2/hmi 面板 MoveitStatusPanel + MoveitPanel（hmi_moveit 可执行）新增。
    - **topic→action 改造（2026-08-20，用户要求：nav 桥路用 topic 是因为外面有 nav2 框架；moveit 无外部框架，决策层应走 action）**：对外主入口从"hmi/决策层直接发 `/plan/moveit/moveit_goal` topic"改为 **`/plan/moveit/moveit_move` action**（`MoveItMove.action`：part + pose/joint_positions + gripper_mode(-1/0/1)），新增 **moveit_goal_server** 节点做**组合编排（先臂后爪）**：内部拆成既有原子 topic（moveit_goal + gripper_goal）下发 bridge，订阅 `/plan/moveit/result`（原 std_msgs/String 改**结构化 PlanMoveResult**：kind 0=臂 1=夹爪 + success + message）跟踪两段完成；夹爪失败整单 ABORT（message 注明"臂已到位、夹爪失败"）、忙碌 REJECT、客户端 cancel → 发 `/plan/moveit/control` cancel → bridge 中止臂链 + 取消在途夹爪 goal（bridge 新增 `chain_id_` 目标链代数：新目标/取消自增，在途回调持捕获值比对失配丢弃——取消后旧链回调不再续链）。**躯干瞄准（part=0）**：`torso_aim.hpp` 纯 C++ 解算器（硬编码 xacro 链常量 + **多起点 LM**（方位角引导 yaw ±0.6 + ±π，取 ray_error 最小），限位单一来源 description joint_config.hpp），求相机 Link_GlobalCameraSensor +X 光轴穿过目标点的 4 关节解（3 方向 DOF：yaw + 两个非共面 tilt）。**v2（2026-08-20 实跑定论）解算器直出 4 关节角**走既有关节规划链——v1 输出相机位姿走 IK 不可用：① 相机在 torso 组外（固定关节下游）→ `kinematics_solver_ik_links` 指相机则 KDL 插件初始化报 "Could not find tip name in joint group 'torso'"；去掉后 tip=组内 Link_Body4 插件能初始化，但解位形 yaw 与当前位相差可达 2.5 rad，KDL NR 局部收敛跨不过去，/compute_ik 恒 NO_IK_SOLUTION(-31)；OMPL 无种子依赖，直接规划关节目标。`--fk-self-test` 模式与 move_group `/compute_fk`（KDL）对拍（e2e 兜底防链常量笔误，7 样本 pos_err=0）；另加 `--aim-self-test`（纯数学解算器自检，5 目标 ≥3 可行）。**坑**：goal_server 躯干分支必须先 `mg.pose = g->pose` 再读瞄准点——漏赋值则 aim=(0,0,0)，解算恒收敛到同一角落解（距离=aim_distance，e2e 排查定位）。**注意 GetPositionFK 请求无 `fk_request` 包装**（header/fk_link_names/robot_state 平铺）；`create_server(node,name,goal_cb,cancel_cb,accepted_cb)` 参数顺序是 **goal→cancel→accepted**（传错编译报 CancelCallback 不匹配）。hmi moveit_panel 改 action 客户端（ACCEPTED/EXECUTING/SUCCEEDED/ABORTED/REJECTED 状态 + 取消按钮 + 仅夹爪勾选，独立夹爪按钮删除）；moveit_e2e 重写为 action 客户端 7 用例（FK 对拍 / 组合编排序 / 只臂 / 只夹爪 / 忙碌 REJECT / 取消 CANCELED / 躯干瞄准成功率≥3/5）。底层原子 topic 保留供决策层编排用。
    - **hmi 默认填充两个坑（2026-08-20 真机复现用户"躯干总失败/右臂假成功"）**：① part=0 默认把 x/y/z 填成**相机当前位置**——瞄准点=相机原点（t≈0 退化）必失败；**修复：填相机前方 0.5m（相机位姿 + 0.5·光轴方向）**，默认发送即可用。② part=1/2 默认填**当前末端位姿**——不改直接发 = 无动作目标（IK 返回当前位形），0.2s 假成功且关节不动（用户"右臂不管发什么都直接成功"）；**修复：bridge 成功消息带 waypoint 数**（"all waypoints done (N)" / "no motion (start==goal, 0 waypoint)"），hmi 显示让无动作可见。③ bridge 服务等待 1s→5s（bringup 后 move_group 加载 ~100s，"You can start planning now" 才就绪，首个目标不再因 wait_for_service 1s 失败）。真机验证（torso 归零后）：右臂/左臂编辑位姿 + 躯干编辑瞄准点均真实到位（31-35s），代码链路无问题。

#### 29. P2 实跑调试流程弯路复盘（2026-08-16，方法论——不是技术难度，是流程纪律）
本轮从"nav2 起不来"到"机器人不动"排障浪费了大量轮次，复盘下来多数是**流程性错误**，下次严格照办：

1. **WSL 文件操作只用 pwsh，不用 write/edit 工具**：`write`/`edit` 对 `\\wsl.localhost` 9P 挂载必失败（ENOTSUP / GetFileSecurityW EIO）。写文件一律 `pwsh` + `[System.IO.File]::WriteAllText(路径, 内容, UTF8无BOM)`；改文件用 ReadAllText + Replace + WriteAllText。本轮在 write/edit 上反复尝试浪费了 3+ 轮。
2. **任何含管道/引号/中文的命令一律先写 `/tmp/*.sh`（或 `*.py`）再 `wsl.exe bash -c 'bash /tmp/x.sh'`**：内联 `bash -c '...'` 复杂命令必被 PowerShell/wsl 引号层吃掉（`Creating: command not found`、`Using: command not found` 这类）。陷阱 24 早写了，本轮又犯了 5+ 次（grep 管道、tf2_echo、`--once` 组合全中招）——**内联命令只允许无引号无管道的极简形式**。
3. **长驻进程必须用持久后台任务**（pwsh `run_in_background: true` 保持 wsl 会话存活）：`wsl.exe bash -c 'nohup ros2 launch ... &'` 会话一退子进程全没（日志 0 字节）。本轮 bringup、goal 发送、tf 长监听各栽一次——**凡是"启动后还要等/还要查"的进程一律后台任务**。
4. **排障第一件事查残留进程，第二件事才是看代码**：`ps aux | grep -E "nav2_|gzserver|cmd_vel_bridge|component_container"`。上次 kill 的 launch 孙进程成孤儿 → 多实例同名节点污染 DDS 发现（lifecycle 卡 `Waiting for service`、bridge 找不到 action server），本轮在此浪费最多轮次。注意：`pkill -x` 对 >15 字符进程名无效（comm 截断），用 `pkill -f "[n]av2_lifecycle_manager/lifecycle_manager"` 路径模式；**清理后必须 `ps` 复查计数**，别假设杀干净；pkill 前先看进程清单再动手（本轮宽泛 pkill 误杀过 localization 的 lifecycle_manager——它与 nav 的共用同一二进制路径）。
5. **数据流验证一律 rclpy 脚本，ros2 CLI 只用于快查**：负载/daemon 异常下 `topic echo --once`、`topic hz`、`param get` 会空结果或报 `context is not valid`（陷阱 17/28.9 已写，本轮仍反复栽：`/perception/odometry` "没发布"实际 48 msg/15s）。写 `/tmp/*.py` 订阅 15s 打印计数+内容，比 CLI 可靠且一次看全链路。
6. **"话题无数据 / TF 缺失"先确认 QoS 与对端计数再下结论**：`/map NO MESSAGE` 是 volatile 订阅收不到 transient_local replay（并非没发布）；`tf2_echo` 找不到 transform 可能是低频 TF（0.33Hz）过期而非缺失——用 rclpy 直接订阅 `/tf` 数边（统计 `('odom','Link_Base')` 出现次数）判断。**单个 CLI 负结果不构成结论**。
7. **先静态后动态**：实跑前先跑 `--tests plan`（静态校验能直接抓住插件库名/costmap 双层结构/default_server_timeout 三类问题，20s）；改配置/参数后**先静态测试再实跑**，别把静态可查的错拖进 3 分钟一次的实跑循环（本轮把 bt 库名、costmap 结构、timeout 三个静态问题全带进了实跑才发现）。
8. **每层症状都要数据证据，改一个修一个验一个**："机器人不动"至少叠了 4 层原因（残留进程→costmap 越界→amcl 位姿→bridge 抢占风暴），每一层都曾被误当最终根因。排障逐层剥离：先健康话题（`/health/*`）+ 数据流计数（rclpy）→ 再日志关键行（grep -vE 噪声）→ 最后才改代码；**修复后立刻补断言/复验闭环**（bridge 风暴修完才补 plan 测试，晚了半程——正确做法是修完当场加断言）。
9. **注意 sim 时间尺度**：负载高时 sim 时间 ~1/30 实速（消息时间戳远小于墙钟是正常的，别当异常）；action 超时、等待、导航时长都按真实时间放大。
10. **WSL 网络异常（2026-08-16）与"零 DDS"静态测试**：`.wslconfig` 若配 `networkingMode=Mirrored`，WSL 重启后可能进入异常网络态（eth0 DOWN、拿到 NAT IP、无 IPv4 multicast）→ **Fast DDS 跨进程发现完全失效**（连 `ros2 node list` 都只看到自己），症状是"两个进程互相看不见"（bridge 找不到 action server、测试 mock 收不到 goal）。排查顺序：`ip addr`（看 eth0 是否 UP + 有无 UP 接口）、`ip maddr`（有无 IPv4 组播组）、跨进程 topic 测试。**修复**：`.wslconfig` 注释 `networkingMode=Mirrored` 回 NAT（备份在 `.wslconfig.bak_mirrored`，`wsl --shutdown` 后 eth0 恢复 172.22.x.x，跨进程发现正常；`wsl.exe --manage --set-networking-mode` 此版本不支持）。**更重要的对策：静态/单元测试一律零 DDS**——cmd_vel_bridge 的决策逻辑已提取为纯 C++ 类 `CmdVelStateMachine`（`cmd_vel_state_machine.hpp`，无 ROS 依赖），bridge 支持 `--logic-test` 自测模式（直接跑 install 二进制，不建 ROS 图）；plan 测试的 bridge 部分用该自测（`--tests plan` 秒级 PASS），网络坏了也不影响。
## 三、工作流程

以下为本项目约定俗成的协作方式，适用于任何修改类任务（代码 / 文档 / 配置）。

### 0. 文档存放约定
- 根 `README.md`：项目**门面**（简介 / 架构 / 关键特性 / 快速开始 / 演示概览 / 文档导航），兼展示功能；
- `docs/`：**开发手册**（进版本管理）——`README.md`（索引与维护规则）、`architecture.md`（架构详解）、`api-reference.md`（接口速查）、`development.md`（开发方式）、`demo-guide.md`（演示脚本）；每份手册头部标「定位 / 读者 / 入口关系」；
- `CLAUDE.md`（本文件）与各包 README：**活文档**，随代码维护，不进 `docs/`；
- `notes/`：历史 / 过程 / 审查文档（设计稿、方案、实施记录、`analysis_*` 审查报告、`problems.md` 活日志）——**私人存档，已 gitignore，不提交**；`others/` 保留已归档源码（COLCON_IGNORE），同样不提交；
- `docs/images/`：文档插图目录（根 README 用 `docs/images/xxx.png`，docs 手册用 `images/xxx.png`）。

### 1. 了解当前情况
- **首先阅读 README**（根 README + 相关包 README），而不是通读代码。
- README 已覆盖项目的使用方式、接口与限制，读完即可对当前状态有整体把握。
- 需要更深处（类结构、内部机制）时，再由 README 指明的文件进入代码。

### 2. 分析理解任务
- 有问题**尽管提**，有不清楚的要求**尽管问**，在开始执行前把任务边界确认清楚。
- 不要在执行中途才发现理解偏差——前期多问一句的代价远小于返工。

### 3. 执行任务
- 遵守 CLAUDE.md 中的注意事项与已知陷阱，避免重蹈覆辙。
- 涉及文档内容时，务必对照代码实证，不臆造（参考用户"文档准确性"偏好）。
- **有现成资源优先复用，不从零造**：创建包用 `ros2 pkg create` 生成骨架（勿全手写）；CMake / launch / 测试等模式参考现有包写法，头文件 / 配置 / 话题表等既有单一来源直接 include 引用，避免重复定义与冗余工作量。
- **【测试及时原则，2026-08-17 用户强调】写完一个模块/功能，立即写对应测试并跑通，绝不拖到"后面统一测"**。优先静态/孤立测试（纯逻辑自测、配置文件静态校验），能零 DDS 就零 DDS（参考 cmd_vel_bridge --logic-test、moveit gen_moveit_config 静态校验）；在写这些分散测试的过程中**同步规划/沉淀测试流程**（测试入口、断言风格、零 DDS 技巧），供 ylr1d_test 包复用。改完立即回归（先静态后动态，陷阱 29.7）。


- 遵守 CLAUDE.md 中的注意事项与已知陷阱，避免重蹈覆辙。
- 涉及文档内容时，务必对照代码实证，不臆造（参考用户"文档准确性"偏好）。
- **有现成资源优先复用，不从零造**：创建包用 `ros2 pkg create` 生成骨架（勿全手写）；CMake / launch / 测试等模式参考现有包写法，头文件 / 配置 / 话题表等既有单一来源直接 include 引用，避免重复定义与冗余工作量。

### 4. 任务完成后：先文档、后备忘
- **先更新 README**：让 README 保持"读了就能用"——若本次改动改变了使用方式 / 接口 / 配置 / 限制，同步更新根 README 与对应包 README。
- **再结合具体情况更新 CLAUDE.md**：若产生了新的坑 / 注意事项 / 工作流经验，沉淀到这里。

### 5. 汇报
汇报时不仅说明做了什么，还要覆盖：
- **做了什么**：改动范围与关键内容
- **结果如何**：验证情况、是否达到预期
- **下一步打算**：有无遗留问题、待办事项
- **意见建议**：对项目的观察与改进建议
