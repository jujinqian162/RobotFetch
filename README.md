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

当前 runner-based workflow 已经统一为一个 ROS 节点和一套 phase runtime：

1. `src/runners/pid_alignment_ros_node.py` 创建 ROS publisher、adapter、workflow resources 和 `WorkflowEngine`
2. `phase_sequence` 决定本次运行的任务链路，单阶段测试和更长流程都走同一个节点
3. `STATUS_ALIGN` 通过共享 `VisionSession` 读取 status targets，运行图像 x 方向 PID 对齐
4. `FORWARD_APPROACH` 不访问相机，按固定速度/距离做开环前进
5. `BASE_COORD` 复用同一个 `VisionSession`，切到 `base_coord` profile 并发布 3D 坐标
6. runtime 按 phase runner 的结果发布：
   - `/workflow/phase`
   - `/workflow/algo_status`
   - `/workflow/env_status`
   - robot: `/t0x0101_robotfetch` as `std_msgs/Float32MultiArray`
   - turtle: `/cmd_vel` as `geometry_msgs/Twist`
7. turtle 环境把算法侧 `linear.y` 映射为 turtlesim 的 `linear.x`，真实机器人环境由电控订阅 `/t0x0101_robotfetch`，用于验证 PID 和 workflow 同步

所以当前 README 里提到的 runner、turtle bridge、BaseDetect，并不是互相独立的小脚本，而是在服务同一条机器人任务链路。

## 目录结构

```text
RobotFetch/
  BaseDetect/                 # BaseDetect submodule / nested project
  configs/workflows/          # runner workflow YAML configs
  scripts/                    # environment setup and diagnostics
  src/
    adapters/                 # environment-specific command adapters
    algorithms/               # detector gateway, PID, target selection, status alignment
    config/                   # strict YAML config loading
    runners/                  # current runner-based ROS node entrypoints
    workflow/                 # shared phase/status contract
    terminal_pid_follower_node.py  # legacy pre-refactor experiment path
  tests/
  README.md
```

## 环境准备

推荐环境是 Linux + ROS 2 + 系统 Python。不要把 Python 版本或 ROS 发行版写死到命令里：ROS 2 Humble、Jazzy 等发行版对应的 Python 版本可能不同，项目脚本会优先使用当前环境能找到的 ROS setup 和 `python3`。

不要直接用 `conda` 里的 `python3` 运行本项目。当前代码会在同一个解释器里同时导入 `rclpy` 和 `basedetect`，最稳的做法是使用 ROS 环境对应的系统 Python，并让虚拟环境通过 `--system-site-packages` 复用 ROS 的 Python 包。

确保 `BaseDetect/` 子项目已经存在并包含配置文件；如果是通过 git 子模块获取的仓库，先运行 `git submodule update --init --recursive`。

第一次准备环境时运行：

```bash
./scripts/setup_dev_env.sh
```

脚本会自动查找 `/opt/ros/<distro>/setup.bash` 和可用的 `python3`。如果机器上有多个 ROS 发行版，或 ROS 安装在非默认目录，可以显式指定：

```bash
ROS_SETUP=/opt/ros/<distro>/setup.bash ./scripts/setup_dev_env.sh
PYTHON_BIN=/path/to/python3 ./scripts/setup_dev_env.sh
```

之后每次进入项目，先激活环境：

```bash
source ./scripts/activate_dev_env.sh
```

## 运行方式

环境激活后，在 `RobotFetch/` 下运行当前 runner-based workflow。当前主入口是 `src/runners/pid_alignment_ros_node.py --config ...`，不是旧的单节点实验脚本。

## PID 跟踪最近端头（status 模式）

旧的 `src/terminal_pid_follower_node.py` 记录的是重构前的单节点实验路径，不再作为当前架构的启动方式。
当前 refactored workflow 的启动入口是下面的 runner-based flow：`src/runners/pid_alignment_ros_node.py --config ...`。

这个节点覆盖同一个 sequence-based workflow：

1. `status` 模式：按 `|cx - target_x|` 最小选择目标，PID 仅做横向对齐。
2. 测试范围由 workflow YAML 中的 `phase_sequence` 控制；只想验证对齐时写 `[STATUS_ALIGN]`。
3. 只验证前进阶段时写 `[FORWARD_APPROACH]`，该阶段不会初始化相机或 detector。
4. 需要对齐后前移时写 `[STATUS_ALIGN, FORWARD_APPROACH]`。前移阶段不做 PID，也不依赖视觉闭环，而是按 `forward_approach.distance_m / forward_approach.speed_mps` 计算持续时间，持续发布固定前进速度，到时后发布零速并结束该阶段。
5. 需要更长机器人流程时写 `[STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]`。`BASE_COORD` 会切到 `detector.base_coord_profile`，发布 `base_coord.publish_topic`。
6. 未来加入角度/朝向 PID 时，应作为更早的独立 phase，例如 `[ANGLE_ALIGN, STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]`，而不是拆出第二个 runner。

### `target_x` 和检测坐标到底是什么规格

`status_align.target_x` 不是类别编号，也不是 YOLO 训练/推理尺寸里的归一化坐标；它是当前输入帧的像素 x 坐标。runner 做的比较是：

```text
error_px = status_align.target_x - selected_status_target.cx
```

`selected_status_target.cx` 来自 `BaseDetect/sdk/detector.py` 中 `result.boxes.xyxy` 的框中心点。当前代码把原始 `frame` 直接传给 Ultralytics `YOLO.track(...)`；即使模型内部 letterbox/resize 到类似 640 的推理尺寸，Ultralytics 返回的 `boxes.xyxy` 也是映射回传入帧的像素坐标。因此这里的 `cx/cy/width/height` 应按输入视频帧或摄像头帧来理解。

这意味着：

- 如果输入帧是 `640x480`，画面中心 x 通常是 `320`。
- 如果输入视频是 `1280x720`，画面中心 x 通常是 `640`，继续写 `target_x: 320.0` 会让 PID 对齐到画面偏左位置。
- 如果摄像头 fallback 改成了 `800x600`，中心 x 通常是 `400`。

所以换摄像头、换视频文件、或调整 fallback 分辨率后，先看运行日志里的 `frame_shape=HxWxC`，再把 `status_align.target_x` 设置到这个宽度下的目标列。

注意 status 类别名当前是 `spearhead`、`fist`、`palm`。如果你期望选中 `palm`，但日志始终显示选中 `spearhead`，先看同一行 `targets=[...]`：runner 会列出本帧所有检测到的端头 label、`cx/cy` 和置信度，然后再按 `|cx - target_x|` 选择最近的允许类别。

当前部分验证的关键配置项在 workflow YAML 中：

- `status_align.target_x`：状态模式下对齐目标像素 x
- `status_align.max_speed`：状态对齐 PID 的速度输出上限，具体值以当前 workflow YAML 为准
- `phase_sequence`：本次要执行的阶段序列；单阶段测试可写 `[STATUS_ALIGN]`
- `forward_approach.speed_mps`：`FORWARD_APPROACH` 阶段的固定前进速度，必须大于 0
- `forward_approach.distance_m`：`FORWARD_APPROACH` 阶段的固定前进距离，必须大于 0
- `detector.status_profile`：status 检测 profile
- `detector.base_coord_profile`：base-coordinate 检测 profile
- `base_coord.publish_topic`：`BASE_COORD` 阶段的 3D 坐标输出话题
- `base_coord.complete_on_first_target`：当前实现中有至少一个坐标目标时是否立即完成 `BASE_COORD`
- `topics.cmd_topic`：命令输出 topic；robot 配置使用 `/t0x0101_robotfetch`，消息为 `std_msgs/Float32MultiArray`，`data=[linear_x, linear_y, angular_z]`
- `topics.publish_cmd_vel`：是否发布 workflow `cmd_topic`；可设为 `false` 做只看状态/检测、不向底盘发速度的测试
- `topics.selected_status_topic`：当前选中目标像素话题
- `adapter.turtle_cmd_topic`：`environment: turtle` 时，将 runner 的 `/cmd_vel` 输出桥接为 turtlesim 可执行的速度话题
- `adapter.cmd_vel_transform.*`：最终输出速度前的全局符号变换；`invert_linear_x`、`invert_linear_y`、`invert_angular_z` 可分别反转 x、y 和角速度，turtle 桥接也使用同一套变换

## Runner-Based Workflow Startup

Turtle partial test:

```bash
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.turtle.yaml
```

This uses the same runner startup path as the robot config. In turtle mode, the node publishes the workflow `/cmd_vel` `Twist` message when `topics.publish_cmd_vel: true` and also bridges commands to `adapter.turtle_cmd_topic` so turtlesim can move without a separate bridge process.

Real robot partial test:

```bash
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.yaml
```

The configured `phase_sequence` is the mission/action sequence. Use `[STATUS_ALIGN]` for status-only validation, `[FORWARD_APPROACH]` for forward-only validation, and `[STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]` for a longer robot flow. The same node handles all of these cases.
In robot mode, velocity commands are published to `/t0x0101_robotfetch` as `std_msgs/Float32MultiArray`, where indices 0, 1, and 2 are `linear_x`, `linear_y`, and `angular_z`.

## 测试

根仓库测试只收集 `tests/` 目录，避免误把 `BaseDetect/scripts/` 下依赖重型推理栈的脚本当成 RobotFetch 单测。运行测试前先激活项目环境：

```bash
source ./scripts/activate_dev_env.sh
python -m pytest -q
```
