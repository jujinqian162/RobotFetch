# RobotFetch 当前重构实现说明

本文档面向当前项目维护者，目标是帮助你快速回答这几个问题：

1. 现在程序已经实现到什么程度。
2. 整个程序是如何串起来的。
3. 自动化测试已经覆盖了什么，没覆盖什么。
4. 你后续在真实环境里应该怎么启动、怎么测试、怎么定位问题。
5. 当前 turtle 相关准备到了什么阶段。

---

## 1. 当前实现到什么地步了

这次重构的第一阶段已经完成了一个**最小可验证链路**，覆盖：

- workflow 状态模型
- phase 控制器
- 纯 PID 控制器
- 目标选择逻辑
- status 对齐算法模块
- detector gateway
- status-only runner core
- turtle workflow 同步 adapter stub

也就是说，现在代码已经具备了以下最小闭环：

- 用统一的 `Phase / AlgoStatus / EnvStatus` 描述系统状态
- 用 `DetectorGateway` 从 BaseDetect SDK 取 status targets
- 用 `StatusAlignStep` 根据目标位置输出横向控制量
- 用 `PidAlignmentRunnerNode` 串起 detector + status_align + workflow topic 发布
- 用 `TurtleAdapter` 消费 workflow phase，并把环境侧状态同步成 `env_status`

但是要明确：

**当前完成的是“核心抽象和最小链路验证”，不是完整可直接上真实 ROS 运行的最终版本。**

特别是：

- `src/runners/pid_alignment_runner.py` 现在还是一个**轻量 runner core**，不是完整的 `rclpy.Node` 成品。
- 测试中发布的消息目前是简化的字符串 / dict 形态，不是真正的 ROS msg 对象。
- turtle adapter 目前只是 **workflow/env_status 同步层**，还**没有接 `/cmd_vel`**，也没有驱动 turtlesim 真正运动。
- forward approach / base_coord full mission / robot adapter 都还没有进入这一轮实现范围。

所以可以把当前状态理解为：

> 架构已经从“大单文件脚本”拆成了可单独验证的核心模块；
> 最重要的 status 对齐链路已经能在单元层面被验证；
> 但真实 ROS 节点接线和真实环境联调，还需要下一步继续落地。

---

## 2. 现在的核心抽象模型

### 2.1 三层共享状态模型

当前整个流程的核心共享模型是三组状态：

- `Phase`：全局任务阶段
- `AlgoStatus`：算法侧状态
- `EnvStatus`：环境侧状态

定义在：
- `src/workflow/types.py`

当前枚举值：

#### Phase
- `READY`
- `STATUS_ALIGN`
- `FORWARD_APPROACH`
- `BASE_COORD`
- `DONE`
- `ABORT`

#### AlgoStatus
- `IDLE`
- `WAITING_FOR_PHASE`
- `RUNNING`
- `TARGET_LOST`
- `ALIGNED`
- `STEP_DONE`
- `ERROR`

#### EnvStatus
- `IDLE`
- `PREPARING`
- `READY`
- `RUNNING`
- `DONE`
- `ERROR`

你可以把它理解成：

- `Phase` 决定“现在整个系统在做哪一步”
- `AlgoStatus` 决定“算法在这一步执行到哪种情况了”
- `EnvStatus` 决定“环境侧是否准备好配合这一步了”

这套模型的意义是：

> turtle、真实机器人、算法模块都不再靠隐含逻辑猜对方状态，而是靠显式 topic/state contract 协调。

---

### 2.2 PhaseController：支持从任意阶段启动

定义在：
- `src/workflow/phase_controller.py`

它的职责很简单：

- 保存当前 phase
- 支持 `start_phase`
- 支持 `enter_phase(...)`
- 记录某个 phase 进入了多少次
- 支持 phase entry 时触发 reset callback

它的作用是保证：

> 你以后可以直接从 `STATUS_ALIGN` 启动，也可以从别的 phase 启动，而不是把流程硬编码成只能从头跑。

这点很重要，因为你的真实需求本来就是：

- 有时只测 BaseDetect + PID
- 有时只测 turtle 控制稳定性
- 有时才跑完整比赛链路

---

## 3. 现在的程序如何串联起来

当前最小链路可以用下面这条逻辑理解：

1. runner 决定当前 phase 是 `STATUS_ALIGN`
2. runner 调 detector gateway 取当前一帧的 status targets
3. runner 把 targets 喂给 `StatusAlignStep`
4. `StatusAlignStep` 内部用 target selection + PID 算出 `command_x`
5. runner 发布：
   - `/workflow/phase`
   - `/workflow/algo_status`
   - `/workflow/env_status`
   - selected target
   - 控制命令
6. turtle adapter 订阅 phase，并把环境状态同步成 `env_status`

### 3.1 DetectorGateway

定义在：
- `src/algorithms/detector_gateway.py`

职责：

- 创建并持有 BaseDetect SDK 的 `Detector`
- 支持 `switch_profile(...)`
- 提供两个统一入口：
  - `detect_status_targets(frame)`
  - `detect_base_coord_targets(frame)`

统一返回：
- `DetectionBatch(ready, targets)`

这样 runner 不需要关心 SDK 的内部细节，只关心：

- detector ready 了吗
- 当前拿到了哪些 typed targets

这是把检测系统和 workflow/runner 解耦的关键一层。

---

### 3.2 StatusAlignStep

定义在：
- `src/algorithms/status_align.py`

职责：

- 根据 status targets 选出当前应该跟踪的目标
- 判断是否已经进入 tolerance
- 若未对齐则调用 PID 输出控制量
- 若无目标则返回 `TARGET_LOST`
- 若已对齐则返回 `ALIGNED`
- 支持 `reset()` 清理内部 PID 状态

当前输入：
- `targets`
- `now_s`
- `StatusAlignConfig`

当前输出：
- `StatusAlignResult`
  - `status`
  - `command_x`
  - `selected_target`
  - `aligned`

注意这里当前是**纯算法模块**，它不知道 ROS，也不知道 turtle，也不知道真实机器人。它只做“对齐计算”。

---

### 3.3 PidAlignmentRunnerNode

定义在：
- `src/runners/pid_alignment_runner.py`

当前它是一个**runner core**，职责是把前面的模块串起来：

- 维护当前 phase（目前固定进入 `STATUS_ALIGN`）
- 调用 detector gateway
- 调用 status align step
- 根据结果发布状态和控制输出

当前 `run_status_align_once(...)` 会发布：

- `phase = STATUS_ALIGN`
- `algo_status = result.status`
- `env_status = RUNNING / READY`
- `selected_target`
- `cmd`

目前的控制输出还是测试友好的轻量 dict：

```python
{
  "linear_x": 0.0,
  "linear_y": command_x,
  "angular_z": 0.0,
}
```

这说明它现在已经完成了“流程串联验证”，但还没最终接成真实 ROS publisher/msg。

---

### 3.4 TurtleAdapter

定义在：
- `src/adapters/turtle_adapter.py`

当前它是一个非常薄的环境侧同步器：

- 输入：`phase`
- 输出：`env_status`

当前逻辑：

- `STATUS_ALIGN -> EnvStatus.READY`
- 其他 phase -> `EnvStatus.IDLE`

也就是说它现在完成的是：

> 环境侧能够“知道现在 workflow 正在要求它准备进入 status 对齐阶段”。

它**还没有**：

- 消费 `/cmd_vel`
- 驱动 turtlesim 运动
- 发布位置/姿态反馈
- 模拟真实底盘约束

所以当前 turtle 进度是：

> **workflow 同步准备好了，运动仿真还没做。**

---

## 4. 自动化测试已经测了什么

这轮 verification pass 已执行，结果：

- 共 `31 passed`

覆盖文件：

- `tests/test_workflow_types.py`
- `tests/test_phase_controller.py`
- `tests/test_pid.py`
- `tests/test_target_selection.py`
- `tests/test_status_align.py`
- `tests/test_detector_gateway.py`
- `tests/test_pid_alignment_runner.py`
- `tests/test_turtle_adapter.py`

下面按模块说测试测了什么。

### 4.1 workflow types

测了：

- `Phase / AlgoStatus / EnvStatus` 的枚举值是否稳定
- `parse_phase(...)` 是否支持字符串解析
- 非法 phase 是否报错

意义：
- 保证 workflow contract 不会被随手改坏

---

### 4.2 phase controller

测了：

- 能否从指定 start phase 启动
- phase 切换是否生效
- enter count 是否记录
- reset callback 是否在进入 phase 时被触发

意义：
- 保证“从任意 phase 启动”这个核心需求真的被实现了

---

### 4.3 PID

测了：

- 输出限幅
- deadband
- reset 后积分和历史状态是否清空

意义：
- 保证 PID 基础行为稳定，至少不会在最基本的数值逻辑上出错

---

### 4.4 target selection

测了：

- status target 是否优先 stable label
- 无匹配 label 时是否返回 `None`
- base coord target 是否取最接近 target_x 的目标

意义：
- 保证“选哪个目标进行对齐”这个关键逻辑已经被单独抽出来并可验证

---

### 4.5 status align

测了：

- 无目标时返回 `TARGET_LOST`
- 在 tolerance 内时返回 `ALIGNED`
- 偏差较大时返回 `RUNNING` 且给出控制量
- reset 是否清除内部 PID 状态

意义：
- 保证 status 对齐逻辑自身的状态切换和控制输出是可测试的

---

### 4.6 detector gateway

测了：

- 初始化 profile
- status targets 检测接口
- base coord targets 检测接口
- profile 切换是否透传到底层 detector

意义：
- 保证 SDK 包装层没有把接口做歪

---

### 4.7 pid alignment runner

测了：

- `run_status_align_once(...)` 在 `RUNNING` 状态下是否发布 phase / algo_status / env_status / cmd / selected_target
- detector not ready 时是否发布 `EnvStatus.READY`
- `PidAlignmentRunnerNode.run_once(...)` 是否把这些逻辑串起来

意义：
- 保证模块之间的“串联逻辑”是通的，而不只是单个模块正确

---

### 4.8 turtle adapter

测了：

- phase 到 env_status 的映射
- adapter 是否会发布 env_status
- node 包装层是否能把 phase message 转交给 adapter

意义：
- 保证环境侧 workflow 同步接口已经建好

---

## 5. 自动化测试没有测什么

这部分很关键，因为后续真实联调主要就在这里补。

### 5.1 没测真实 BaseDetect 模型推理

当前测试没有真正加载：

- YOLO 权重
- torch 推理
- 摄像头真实输入
- BaseDetect 真正的稳定器/时序逻辑

也就是说：

- `DetectorGateway` 的接口测了
- 但 `BaseDetect SDK + 模型 + 输入流` 的真实行为没在自动测试里验证

这个必须靠你后面真实环境或半真实输入验证。

---

### 5.2 没测真实 ROS publisher / subscriber / msg 类型

当前 runner/turtle adapter 测试主要是逻辑级测试，不是完整 ROS 集成测试。

没测的包括：

- `rclpy.Node` 生命周期
- `String / Twist / PointStamped` 真实消息对象
- 真正的 topic publish / subscribe
- ROS clock / timer

所以当前还不能说“ROS 节点联通已经测过”，只能说“节点逻辑 core 已测过”。

---

### 5.3 没测真实运动闭环

当前没测：

- `/cmd_vel` 到底盘/仿真器响应
- turtle 是否真的平移
- PID 参数在真实系统里是否稳定
- 控制方向符号是否正确
- 左右平移轴是否和坐标系一致

这部分必须靠你：

- turtle 真仿真
- 真实机器人小步测试

来验证。

---

### 5.4 没测完整比赛链路

当前没有实现/验证：

- forward approach
- base coord full runner
- full mission runner
- robot adapter
- 比赛级全流程切换

所以当前不能说“完整比赛流程已准备好”。

当前只能说：

> **status 对齐这一段的核心架构已经准备好了。**

---

## 6. 你后续真实测试时重点要测什么

建议你后面真实联调按下面顺序做。

### 第一层：检测输出正确性

你要验证：

- BaseDetect status profile 是否真能稳定输出你想要的 status target
- stable label 机制在现场是否真的符合预期
- target `cx` 是否随目标左右移动而合理变化

如果这里有问题，先不要怀疑 PID，先看 detector 输出。

排查入口：
- `src/algorithms/detector_gateway.py`
- BaseDetect SDK 原始输出

---

### 第二层：控制方向和幅值

你要验证：

- `command_x` 正负号是否和你的底盘横移方向一致
- PID 输出是否过大/过小
- 对齐误差趋近时是否震荡
- tolerance 是否合适

如果这里有问题，重点看：
- `src/algorithms/status_align.py`
- `src/algorithms/pid.py`
- 配置里的 PID 参数、target_x、tolerance

---

### 第三层：workflow 同步

你要验证：

- runner 发布的 `/workflow/phase` 是否符合预期
- turtle/真实环境侧接收到 phase 后是否进入正确状态
- `/workflow/env_status` 是否能正确反映“环境已准备”

如果这里有问题，重点看：
- `src/workflow/types.py`
- `src/workflow/phase_controller.py`
- `src/adapters/turtle_adapter.py`
- runner 的状态发布逻辑

---

### 第四层：真实环境联动

你要验证：

- 摄像头输入、检测输出、控制命令、底盘动作，这四者是否串起来
- 实机中是否存在延迟、抖动、丢帧、方向颠倒、topic 不一致

这层问题通常不是单模块 bug，而是系统集成问题。

---

## 7. 以后怎么快速定位问题

给你一个最实用的排查顺序。

### 情况 A：完全不动

先查：
1. detector 有没有目标
2. `StatusAlignStep` 是否返回 `TARGET_LOST`
3. `cmd` 是否一直是 0
4. env_status 是否不是 READY / RUNNING

优先看：
- detector 输出
- phase / env_status

---

### 情况 B：方向反了

先查：
1. `error = target_x - selected_target.cx` 的符号
2. `command_x` 的正负
3. 底盘横移方向定义

优先看：
- `src/algorithms/status_align.py`
- runner 如何把 command_x 映射到底盘控制轴

---

### 情况 C：抖动很大

先查：
1. detector 输出是否抖
2. stable label 是否生效
3. PID 参数是否过激
4. tolerance 是否太小

优先看：
- detector 输出本身
- PID 参数
- tolerance

---

### 情况 D：topic 看起来正常，但环境没响应

先查：
1. `/workflow/phase`
2. `/workflow/algo_status`
3. `/workflow/env_status`
4. `/cmd_vel`

如果这几个 topic 正常，但环境没响应，问题大概率在环境 adapter / 真机接线层，而不是算法层。

---

## 8. 现在怎么启动和测试

### 8.1 当前自动测试

在 worktree 根目录下运行：

```bash
pytest tests/test_workflow_types.py \
  tests/test_phase_controller.py \
  tests/test_pid.py \
  tests/test_target_selection.py \
  tests/test_status_align.py \
  tests/test_detector_gateway.py \
  tests/test_pid_alignment_runner.py \
  tests/test_turtle_adapter.py
```

当前验证结果是：
- `31 passed`

这一步用于确认“纯逻辑层没有被改坏”。

---

### 8.2 当前 runner 启动方式

README 里已经补了目标命令：

```bash
python3 src/runners/pid_alignment_runner.py --start-phase STATUS_ALIGN
```

但你要注意：

**当前这个入口还没有接成完整真实 ROS runner。**

它现在更适合作为：
- 架构入口说明
- 后续接 `rclpy.Node` 的骨架
- 帮你确认工作流应该怎么串

如果你现在直接拿它去跑完整 ROS 链路，还需要继续把真实 publisher/subscriber/msg/camera 部分补齐。

---

### 8.3 当前 turtle 启动方式

当前没有完整 turtle 启动脚本。

现阶段 turtle 部分只有：
- `src/adapters/turtle_adapter.py`

所以你现在**不能**说“turtle 仿真环境已经准备完毕”。

更准确的描述是：

- workflow/env_status 同步 contract 已经准备好
- 真正的 turtle 运动仿真还没接

---

## 9. 当前实现状态总结

### 已完成

- workflow 抽象模型
- 任意 phase 启动能力
- 纯 PID 模块
- 目标选择模块
- status 对齐模块
- detector gateway
- status-only runner core
- turtle env_status 同步 stub
- README 基础说明
- 自动化测试验证 pass

### 未完成

- 真实 `rclpy.Node` 版本的 `pid_alignment_runner`
- 真实 ROS msg 发布/订阅接线
- 摄像头输入接线
- `/cmd_vel` 到 turtle 的消费与运动仿真
- 真机 adapter
- forward approach
- base_coord runner / full mission runner
- 实机联调

### 对 turtle 当前状态的准确描述

> 现在 turtle 只完成了“环境状态同步层”，还没有完成“运动仿真层”。

### 对整个项目当前状态的准确描述

> 现在已经从单文件原型，进入“可模块化验证的 workflow 架构第一阶段完成”的状态；
> 最关键的 status 对齐链路已经抽出来并验证过；
> 但真实 ROS 和真实环境接线仍然是下一阶段工作。
