#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


def _add_basedetect_to_sys_path() -> Path:
    project_root = Path(__file__).resolve().parents[1]
    basedetect_root = project_root / "BaseDetect"
    if not basedetect_root.exists():
        raise FileNotFoundError(f"BaseDetect not found: {basedetect_root}")
    sys.path.insert(0, str(basedetect_root))
    return basedetect_root


BASEDETECT_ROOT = _add_basedetect_to_sys_path()

from basedetect.coord3d import load_camera_config, pixel_to_3d, scale_intrinsics


class BaseDetectDemoNode(Node):
    def __init__(
        self,
        camera_config: Path,
        bbox_cx: float,
        bbox_cy: float,
        bbox_width: float,
        bbox_height: float,
        frame_id: str,
        hz: float,
    ) -> None:
        super().__init__("base_detect_demo")
        self.intrinsics, self.target = load_camera_config(camera_config)
        self.bbox_cx = bbox_cx
        self.bbox_cy = bbox_cy
        self.bbox_width = bbox_width
        self.bbox_height = bbox_height
        self.frame_id = frame_id

        self.publisher = self.create_publisher(
            PointStamped,
            "/robot_fetch/target_position",
            10,
        )

        period = 1.0 / hz if hz > 0 else 0.5
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(f"Using camera config: {camera_config}")

    def _on_timer(self) -> None:
        intrinsics = scale_intrinsics(
            self.intrinsics,
            self.intrinsics.image_width,
            self.intrinsics.image_height,
        )
        pos = pixel_to_3d(
            intrinsics=intrinsics,
            target=self.target,
            bbox_cx_px=self.bbox_cx,
            bbox_cy_px=self.bbox_cy,
            bbox_width_px=self.bbox_width,
            bbox_height_px=self.bbox_height,
        )

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = pos.x
        msg.point.y = pos.y
        msg.point.z = pos.z
        self.publisher.publish(msg)

        self.get_logger().info(
            f"Publish target position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}"
        )


def _parse_args() -> argparse.Namespace:
    default_config = BASEDETECT_ROOT / "configs" / "camera.yaml"
    parser = argparse.ArgumentParser(description="RobotFetch BaseDetect ROS2 demo")
    parser.add_argument("--camera-config", type=Path, default=default_config)
    parser.add_argument("--bbox-cx", type=float, default=320.0)
    parser.add_argument("--bbox-cy", type=float, default=240.0)
    parser.add_argument("--bbox-width", type=float, default=120.0)
    parser.add_argument("--bbox-height", type=float, default=90.0)
    parser.add_argument("--frame-id", type=str, default="camera_link")
    parser.add_argument("--hz", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = BaseDetectDemoNode(
        camera_config=args.camera_config,
        bbox_cx=args.bbox_cx,
        bbox_cy=args.bbox_cy,
        bbox_width=args.bbox_width,
        bbox_height=args.bbox_height,
        frame_id=args.frame_id,
        hz=args.hz,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
