# RobotFetch

简化版 ROS 2 示例工程，不用 colcon 构建，直接运行 `src` 里的 Python 脚本。

## 项目背景与目标

这个项目要解决的问题是：**让机器人走到端头架前的目标端头附近，并为后续机械臂操作提供可靠的定位信息。**

当前整体任务可以理解成一条分阶段的链路：

1. 机器人先走到一个固定的观察位姿，保证摄像头能完整看到端头架
2. `BaseDetect` 在 `status` 模式下输出当前画面里的端头目标
3. 控制侧从这些目标里选择“最该跟踪的那个目标”，通常是最接近目标像素列的端头
4. PID 控制机器人做左右平移，对齐到目标端头前方
5. 对齐完成后，机器人前移固定距离
6. 然后切到 `base_coord` 模式，输出目标 base 的 3D 坐标，提供给后续机械臂或更高层任务

也就是说，这个项目并不是单纯做“检测”或“PID”某一项，而是把：

- 视觉检测
- 目标选择
- 横向对齐控制
- workflow 状态同步
- 仿真/真实环境适配

串成一套可测试、可分阶段启动的机器人执行流程。

## 为什么现在会拆成 workflow 架构

这个项目当前最重要的工程背景是：**你并不总是拥有完整比赛环境。**

实际开发里至少有三种场景：

1. 只想验证 `BaseDetect + PID` 这条最核心链路
2. 没有真实机器人时，要先用 turtlesim 验证控制不会发散、workflow 能不能串起来
3. 真正比赛或实机联调时，再接入完整机器人环境

所以现在代码会强调：

- 算法模块和环境模块解耦
- 用统一的 `Phase / AlgoStatus / EnvStatus` 表达状态
- 支持从任意 phase 启动部分测试
- 让 turtle 和真实机器人都通过 ROS topic 接入同一个 workflow contract

这样做的目的不是把项目做复杂，而是为了让“部分功能测试”和“完整比赛流程”能共存，不至于为了临时测试写出一堆最后无法合并回主链路的脚本。

## 当前核心流程

目前已经拆出来的最小闭环是：

1. runner 进入 `STATUS_ALIGN`
2. `DetectorGateway` 从 BaseDetect 读取 status targets
3. `StatusAlignStep` 选择目标并输出 PID 横向控制量
4. runner 发布：
   - `/workflow/phase`
   - `/workflow/algo_status`
   - `/workflow/env_status`
   - `/cmd_vel`
5. `TurtleWorkflowNode` 或后续真实环境 adapter 消费这些 topic
6. turtle 环境把算法侧 `linear.y` 映射为 turtlesim 的 `linear.x`，用于验证 PID 和 workflow 同步

所以当前 README 里提到的 runner、turtle bridge、BaseDetect，并不是互相独立的小脚本，而是在服务同一条机器人任务链路。

## 目录结构

```text
RobotFetch/
  BaseDetect/
  src/
    base_detect_demo_node.py
    terminal_pid_follower_node.py
  README.md
```

## 这个示例做了什么

- 从 `BaseDetect/configs/camera.yaml` 读取相机与目标参数
- 调用 `basedetect.coord3d.pixel_to_3d` 估计 3D 坐标
- 以 ROS 2 话题发布 `/robot_fetch/target_position` (`geometry_msgs/msg/PointStamped`)

## 环境准备

推荐环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- `/usr/bin/python3.12`
- 项目根目录 `.venv`，并使用 `--system-site-packages` 复用 ROS 的 Python 包

不要直接用 `conda` 里的 `python3` 运行本项目。当前代码会在同一个解释器里同时导入 `rclpy` 和 `basedetect`，最稳的做法是统一到系统 Python 3.12。

首次初始化：

```bash
./scripts/setup_dev_env.sh
```

之后每次进入项目，先激活环境：

```bash
source ./scripts/activate_dev_env.sh
```

## 运行方式

环境激活后，在 `RobotFetch/` 下直接运行：

```bash
python src/base_detect_demo_node.py
```

查看发布结果：

```bash
ros2 topic echo /robot_fetch/target_position
```

## PID 跟踪最近端头（status 模式）

`src/terminal_pid_follower_node.py` 使用 BaseDetect SDK 的双阶段流程：

1. `status` 模式：按 `|cx - target_x|` 最小选择目标，PID 仅做横向对齐。
2. 对齐后预留“前移固定距离”TODO（当前只打印日志并跳过实现）。
3. 切到 `base_coord` 模式：从检测到的 base 中取 `|cx - target_x|` 最小的目标，
   广播其 3D 坐标给机械臂。

全部参数放在 `configs/terminal_pid_follower.yaml`，命令行只保留 `--hot-reload`。
开启后每个循环重读 YAML，实时更新 PID、target_x、profile 等参数。
节点状态流转：`status_align -> forward_approach_todo -> base_coord_broadcast`。

默认行为：

- 发布速度到 `/cmd_vel`（`geometry_msgs/msg/Twist`）
- 发布当前选中的像素目标到 `/robot_fetch/selected_target_px`（`geometry_msgs/msg/PointStamped`）
- 目标丢失时自动停车

运行示例：

```bash
python src/terminal_pid_follower_node.py
```

热更新运行：

```bash
python src/terminal_pid_follower_node.py --hot-reload
```

配置文件中的关键项：

- `status_align.target_x`：状态模式下对齐目标像素 x
- `status_align.labels`：允许的端头类别
- `pid.*`：PID 参数和限幅
- `modes.status_profile` / `modes.base_coord_profile`：SDK 两个阶段 profile
- `base_coord.target_x`：base_coord 广播阶段的目标像素 x
- `forward_approach.*`：前移阶段预留参数（TODO，暂未执行）

## Status-only workflow runner

For partial testing of BaseDetect + PID lateral alignment, use:

```bash
python src/runners/pid_alignment_runner.py --start-phase STATUS_ALIGN
```

Workflow topics:
- `/workflow/phase`
- `/workflow/algo_status`
- `/workflow/env_status`

## Turtle workflow bridge node

For turtlesim-based workflow testing, run:

```bash
python src/turtle_workflow_node.py
```

This node:
- subscribes to `/workflow/phase`
- subscribes to `/cmd_vel`
- publishes `/workflow/env_status`
- publishes mapped turtle motion to `/turtle1/cmd_vel`

It maps algorithm lateral `linear.y` into turtlesim forward/backward `linear.x`, so you can validate PID behavior and workflow synchronization without a holonomic simulator.

Optional arguments:

```bash
python src/turtle_workflow_node.py \
  --phase-topic /workflow/phase \
  --cmd-topic /cmd_vel \
  --env-status-topic /workflow/env_status \
  --turtle-cmd-topic /turtle1/cmd_vel \
  --node-name turtle_workflow_node
```


```bash
python src/base_detect_demo_node.py \
  --camera-config BaseDetect/configs/camera.yaml \
  --bbox-cx 320 --bbox-cy 240 --bbox-width 120 --bbox-height 90 \
  --frame-id camera_link --hz 2.0
```

## 测试

根仓库测试只收集 `tests/` 目录，避免误把 `BaseDetect/scripts/` 下依赖重型推理栈的脚本当成 RobotFetch 单测。

```bash
python -m pytest -q
```
