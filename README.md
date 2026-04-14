# RobotFetch

简化版 ROS 2 示例工程，不用 colcon 构建，直接运行 `src` 里的 Python 脚本。

## 目录结构

```text
RobotFetch/
  BaseDetect/
  src/
    base_detect_demo_node.py
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

## 可选参数

```bash
python3 src/base_detect_demo_node.py \
  --camera-config BaseDetect/configs/camera.yaml \
  --bbox-cx 320 --bbox-cy 240 --bbox-width 120 --bbox-height 90 \
  --frame-id camera_link --hz 2.0
```
