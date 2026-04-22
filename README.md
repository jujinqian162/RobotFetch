# RobotFetch

简化版 ROS 2 示例工程，不用 colcon 构建，直接运行 `src` 里的 Python 脚本。

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

## 运行方式

先 source ROS 2 环境（以 Humble 为例）：

```bash
source /opt/ros/humble/setup.bash
```

然后在 `RobotFetch/` 下直接运行：

```bash
python3 src/base_detect_demo_node.py
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
python3 src/terminal_pid_follower_node.py
```

热更新运行：

```bash
python3 src/terminal_pid_follower_node.py --hot-reload
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
python3 src/runners/pid_alignment_runner.py --start-phase STATUS_ALIGN
```

Workflow topics:
- `/workflow/phase`
- `/workflow/algo_status`
- `/workflow/env_status`

## Turtle workflow bridge node

For turtlesim-based workflow testing, run:

```bash
python3 src/turtle_workflow_node.py
```

This node:
- subscribes to `/workflow/phase`
- subscribes to `/cmd_vel`
- publishes `/workflow/env_status`
- publishes mapped turtle motion to `/turtle1/cmd_vel`

It maps algorithm lateral `linear.y` into turtlesim forward/backward `linear.x`, so you can validate PID behavior and workflow synchronization without a holonomic simulator.

Optional arguments:

```bash
python3 src/turtle_workflow_node.py \
  --phase-topic /workflow/phase \
  --cmd-topic /cmd_vel \
  --env-status-topic /workflow/env_status \
  --turtle-cmd-topic /turtle1/cmd_vel \
  --node-name turtle_workflow_node
```


```bash
python3 src/base_detect_demo_node.py \
  --camera-config BaseDetect/configs/camera.yaml \
  --bbox-cx 320 --bbox-cy 240 --bbox-width 120 --bbox-height 90 \
  --frame-id camera_link --hz 2.0
```
