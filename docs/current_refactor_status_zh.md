# RobotFetch 当前重构实现说明

本文档面向当前项目维护者，用来说明当前实现状态、启动方式、测试范围，以及后续真实环境联调时需要重点验证的内容。

---

## 1. 当前实现状态

当前项目已经从早期的单文件实验路径，进入统一的 runner-based workflow runtime。当前同一个 ROS 节点既支持单 phase 部分测试，也支持按 `phase_sequence` 串起更长流程，覆盖：

- workflow 状态模型：`src/workflow/types.py`
- phase runner contract 和 engine：`src/workflow/phase_runner.py`、`src/workflow/engine.py`
- 共享资源 session：`src/workflow/runtime.py`
- PID 控制器：`src/algorithms/pid.py`
- status target 选择：`src/algorithms/target_selection.py`
- status 对齐算法：`src/algorithms/status_align.py`
- BaseDetect SDK 包装：`src/algorithms/detector_gateway.py`
- phase runners：`src/runners/phases/`
- 当前 ROS 节点入口：`src/runners/pid_alignment_ros_node.py`
- turtle adapter：`src/adapters/turtle_adapter.py`
- robot adapter：`src/adapters/robot_adapter.py`
- workflow YAML 配置：`configs/workflows/`

当前可运行入口是：

```bash
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.turtle.yaml
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.yaml
```

当前默认配置仍保守使用 `phase_sequence: [STATUS_ALIGN]`。需要部分验证时可写 `[STATUS_ALIGN]` 或 `[FORWARD_APPROACH]`；需要更长机器人流程时可写 `[STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]`。不再单独维护一个 full mission runner。

---

## 2. 当前链路如何串起来

当前 runner-based flow 可以理解为：

1. `pid_alignment_ros_node.py` 加载 workflow YAML。
2. 节点创建 ROS publishers、adapter、`WorkflowResources` 和 `WorkflowEngine`，但不直接打开相机或创建 detector。
3. `WorkflowEngine` 按 `phase_sequence` 调用当前 phase runner。
4. `STATUS_ALIGN` 和 `BASE_COORD` 通过共享 `VisionSession` 懒加载并复用 OpenCV capture 和 `DetectorGateway`。
5. `FORWARD_APPROACH` 不访问视觉资源，只按固定距离/速度输出开环前进命令。
6. runner 发布 workflow 状态、算法状态、环境状态、选中目标、base 坐标和 `/cmd_vel`。
7. turtle 环境会额外通过 `TurtleAdapter` 把 workflow 横向 `linear.y` 映射为 turtlesim 的 `linear.x`，用于部分验证。
8. robot 环境通过 `RobotAdapter` 直接透传 workflow velocity，供真实机器人接线层消费。

当前仍然不是完整比赛链路：角度/朝向 PID、真实机器人现场联调和完整任务收尾仍需后续实现或验证。

---

## 3. 配置模型

当前 workflow 配置在：

- `configs/workflows/pid_alignment.turtle.yaml`
- `configs/workflows/pid_alignment.robot.yaml`

关键字段包括：

- `environment`: `turtle` 或 `robot`
- `start_phase`: 当前通常是 `STATUS_ALIGN`
- `phase_sequence`: 本次运行的任务序列，例如 `[STATUS_ALIGN]`、`[FORWARD_APPROACH]` 或 `[STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]`
- `detector.sdk_config`: BaseDetect SDK 配置路径
- `detector.status_profile`: status 检测 profile
- `detector.base_coord_profile`: base-coordinate 检测 profile
- `detector.input_source`: 摄像头索引或视频路径
- `topics.cmd_topic`: workflow velocity topic
- `topics.selected_status_topic`: 当前选中 status target topic
- `adapter.turtle_cmd_topic`: turtle 模式下的 turtlesim 命令 topic
- `status_align.target_x`: 对齐目标像素 x
- `status_align.tolerance_px`: 对齐容差
- `forward_approach.speed_mps` / `distance_m`: 开环前进阶段参数
- `base_coord.publish_topic`: base-coordinate 输出 topic

历史计划文档里出现过 `one_shot` 字段；当前代码已使用 `phase_sequence` 替代。

---

## 4. 自动化测试覆盖范围

运行全部 RobotFetch 测试：

```bash
python -m pytest -q
```

当前测试主要覆盖：

- workflow enum / phase controller 行为
- PID 输出、deadband、reset、限幅
- target selection
- status alignment 状态切换和控制输出
- detector gateway 包装层
- strict YAML config loading
- turtle / robot adapter
- runner core 行为
- `pid_alignment_ros_node.py` 的构建和入口逻辑
- BaseDetect SDK config resolution
- camera diagnostic helper

不要在文档里依赖固定的 `31 passed` 或 `33 passed` 之类结果；测试数量会随着文件增删变化，以当前 `python -m pytest -q` 输出为准。

---

## 5. 自动化测试没有充分覆盖的内容

以下内容仍需要真实环境或半真实输入验证：

- BaseDetect 模型真实推理质量
- 摄像头真实输入、曝光、丢帧和延迟
- OpenCV 输入源在现场机器上的稳定性
- ROS executor / topic 在多节点真实运行时的连通性
- turtlesim 是否按预期响应桥接后的速度命令
- 真机 `/cmd_vel` 坐标轴方向和幅值是否正确
- PID 参数在真实系统里的稳定性
- 机器人底盘约束、限速、急停、碰撞风险
- 角度/朝向 PID phase
- 完整任务收尾

也就是说，当前可以说 **统一 phase runtime、STATUS_ALIGN、FORWARD_APPROACH 和 BASE_COORD 的核心结构已经具备**，但不能说完整比赛流程已经完成。

---

## 6. 真实联调建议顺序

### 6.1 检测输出

先确认 BaseDetect status profile 能稳定输出目标：

- 是否检测到目标
- 目标 `cx` 是否随画面移动合理变化
- stable label 机制是否符合现场需求
- `Detector.ready` 是否按预期进入 ready 状态

优先排查：

- `src/algorithms/detector_gateway.py`
- BaseDetect SDK 配置和 profile
- `detector.input_source`

### 6.2 控制方向和幅值

确认：

- `error = target_x - selected_target.cx` 的符号符合预期
- PID 输出正负号和底盘横移方向一致
- `max_output`、`tolerance_px`、PID 参数不会导致震荡

优先排查：

- `src/algorithms/status_align.py`
- `src/algorithms/pid.py`
- workflow YAML 中的 `status_align` 配置

### 6.3 workflow / topic 同步

确认：

- `/workflow/phase`
- `/workflow/algo_status`
- `/workflow/env_status`
- `/cmd_vel`
- selected target topic

如果这些 topic 正常但环境没响应，问题大概率在 adapter 或真机接线层，而不是算法层。

---

## 7. 当前完成与未完成

### 已完成

- workflow 抽象模型
- 从任意 phase 启动的基础能力
- STATUS_ALIGN 算法链路
- BaseDetect gateway 包装
- config-driven runner startup
- 当前 ROS node 入口：`pid_alignment_ros_node.py`
- 懒加载并跨视觉 phase 复用 OpenCV camera/video input
- `FORWARD_APPROACH` phase runner
- `BASE_COORD` phase runner
- turtle adapter 和 turtle command bridge
- robot adapter velocity passthrough
- 单元测试和入口构建测试

### 未完成 / 待验证

- turtle pose feedback / 闭环仿真
- 真实机器人现场联调
- 真机底盘方向、限速和安全约束验证
- 角度/朝向 PID phase
- 比赛级完整流程切换

### 当前状态的一句话描述

> RobotFetch 当前已经完成统一 phase runtime、STATUS_ALIGN、FORWARD_APPROACH 和 BASE_COORD 的核心测试；后续重点是实机/仿真联调、角度/朝向 PID，以及比赛级完整流程收尾。
