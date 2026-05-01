#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

WORKTREE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKTREE_ROOT / "src"
DEFAULT_CONFIG_PATH = WORKTREE_ROOT / "configs/workflows/pid_alignment.robot.yaml"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from adapters.cmd_vel_transform import CmdVelTransformAdapter
from adapters.robot_adapter import RobotAdapter
from adapters.turtle_adapter import TurtleAdapter
from config.loaders import load_pid_alignment_config
from config.models import PidAlignmentWorkflowConfig
from runners.phases.registry import build_phase_registry
from runners.phases.status_align_phase import build_status_align_step
from workflow.engine import WorkflowEngine
from workflow.runtime import (
    WorkflowContext,
    WorkflowPublishers,
    WorkflowResources,
    _resolve_input_source,
    build_capture,
    build_readable_v4l2_mjpg_capture,
    build_v4l2_mjpg_capture,
)
from workflow.types import Phase


class _StringPublisher:
    def __init__(self, *, ros_publisher: Any, message_type: type[String]) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type

    def publish(self, value: str) -> None:
        message = self._message_type()
        message.data = str(value)
        self._ros_publisher.publish(message)


class _TwistPublisher:
    def __init__(self, *, ros_publisher: Any, message_type: type[Twist]) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type

    def publish(self, payload: dict[str, float]) -> None:
        message = self._message_type()
        message.linear.x = float(payload.get("linear_x", 0.0))
        message.linear.y = float(payload.get("linear_y", 0.0))
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = float(payload.get("angular_z", 0.0))
        self._ros_publisher.publish(message)


class _WorkflowCommandPublisher:
    def __init__(
        self,
        *,
        ros_publisher: Any | None,
        message_type: type[Twist],
        adapter_cmd_handler: Callable[[Any], Any] | None = None,
        cmd_vel_transform: CmdVelTransformAdapter | None = None,
    ) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type
        self._adapter_cmd_handler = adapter_cmd_handler
        self._cmd_vel_transform = cmd_vel_transform or CmdVelTransformAdapter()
        self._workflow_publisher = (
            None
            if ros_publisher is None
            else _TwistPublisher(
                ros_publisher=ros_publisher,
                message_type=message_type,
            )
        )

    def publish(self, payload: dict[str, float]) -> None:
        payload = self._cmd_vel_transform.apply(payload)
        if self._workflow_publisher is not None:
            self._workflow_publisher.publish(payload)
        if self._adapter_cmd_handler is None:
            return
        self._adapter_cmd_handler(_build_twist_message(payload))


class _SelectedTargetPublisher:
    def __init__(
        self,
        *,
        ros_publisher: Any,
        message_type: type[PointStamped],
        clock_getter: Callable[[], Any],
    ) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type
        self._clock_getter = clock_getter

    def publish(self, payload: dict[str, object]) -> None:
        message = self._message_type()
        now = self._clock_getter().now()
        if hasattr(message, "header"):
            if hasattr(now, "to_msg"):
                message.header.stamp = now.to_msg()
            message.header.frame_id = str(payload.get("frame_id", ""))
        message.point.x = float(payload.get("cx", 0.0))
        message.point.y = 0.0
        message.point.z = 0.0
        self._ros_publisher.publish(message)


class _BaseCoordPublisher:
    def __init__(
        self,
        *,
        ros_publisher: Any,
        message_type: type[PointStamped],
        clock_getter: Callable[[], Any],
    ) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type
        self._clock_getter = clock_getter

    def publish(self, payload: dict[str, object]) -> None:
        message = self._message_type()
        now = self._clock_getter().now()
        if hasattr(message, "header"):
            if hasattr(now, "to_msg"):
                message.header.stamp = now.to_msg()
            message.header.frame_id = str(payload.get("frame_id", ""))
        message.point.x = float(payload.get("x", 0.0))
        message.point.y = float(payload.get("y", 0.0))
        message.point.z = float(payload.get("z", 0.0))
        self._ros_publisher.publish(message)


def _build_twist_message(payload: dict[str, float]) -> Any:
    return SimpleNamespace(
        linear=SimpleNamespace(
            x=float(payload.get("linear_x", 0.0)),
            y=float(payload.get("linear_y", 0.0)),
            z=0.0,
        ),
        angular=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=float(payload.get("angular_z", 0.0)),
        ),
    )


def build_adapter(
    *,
    environment: str,
    env_status_publisher: Callable[[str], None],
    turtle_cmd_publisher: Callable[[dict[str, float]], None] | None = None,
) -> Any:
    if environment == "turtle":
        return TurtleAdapter(
            env_status_publisher=env_status_publisher,
            turtle_cmd_publisher=turtle_cmd_publisher,
        )
    if environment == "robot":
        return RobotAdapter(env_status_publisher=env_status_publisher)
    raise ValueError(f"Unsupported environment: {environment}")


def _build_capture_log_message(input_source: str, capture: Any) -> str:
    import cv2

    source_type = "camera" if isinstance(_resolve_input_source(input_source), int) else "video"
    width = _capture_int_property(capture, cv2.CAP_PROP_FRAME_WIDTH)
    height = _capture_int_property(capture, cv2.CAP_PROP_FRAME_HEIGHT)
    fps = _capture_float_property(capture, cv2.CAP_PROP_FPS)
    frame_count = _capture_int_property(capture, cv2.CAP_PROP_FRAME_COUNT)
    fourcc = _decode_fourcc(_capture_int_property(capture, cv2.CAP_PROP_FOURCC))
    frame_count_text = "unknown" if frame_count <= 0 else str(frame_count)

    return _format_multiline_log(
        "capture opened",
        (
            ("source", input_source),
            ("source_type", source_type),
            ("width", width),
            ("height", height),
            ("fps", f"{fps:.3f}"),
            ("frame_count", frame_count_text),
            ("fourcc", fourcc),
        ),
    )


def _capture_float_property(capture: Any, prop_id: int) -> float:
    getter = getattr(capture, "get", None)
    if not callable(getter):
        return 0.0
    try:
        return float(getter(prop_id))
    except (TypeError, ValueError):
        return 0.0


def _capture_int_property(capture: Any, prop_id: int) -> int:
    return int(round(_capture_float_property(capture, prop_id)))


def _decode_fourcc(value: int) -> str:
    if value <= 0:
        return "unknown"
    chars = [chr((value >> (8 * index)) & 0xFF) for index in range(4)]
    if not all(char.isprintable() and char.strip() for char in chars):
        return str(value)
    return "".join(chars)


def _format_multiline_log(
    title: str,
    fields: tuple[tuple[str, object], ...],
) -> str:
    return "\n".join([title, *(f"  {key}={value}" for key, value in fields)])


def _enum_or_value(value: Any) -> str:
    enum_value = getattr(value, "value", None)
    return str(enum_value if enum_value is not None else value)


class PidAlignmentRosNode(Node):
    def __init__(self, *, cfg: PidAlignmentWorkflowConfig) -> None:
        super().__init__("pid_alignment_runner")
        self._cfg = cfg

        workflow_cmd_ros_publisher = (
            self.create_publisher(Twist, cfg.topics.cmd_topic, 10)
            if cfg.topics.publish_cmd_vel
            else None
        )
        turtle_cmd_publisher: Callable[[dict[str, float]], None] | None = None
        if cfg.environment == "turtle" and cfg.adapter.turtle_cmd_topic:
            turtle_cmd_publisher = _TwistPublisher(
                ros_publisher=self.create_publisher(Twist, cfg.adapter.turtle_cmd_topic, 10),
                message_type=Twist,
            ).publish
        self._phase_pub = _StringPublisher(
            ros_publisher=self.create_publisher(String, cfg.topics.workflow_phase_topic, 10),
            message_type=String,
        )
        self._algo_status_pub = _StringPublisher(
            ros_publisher=self.create_publisher(String, cfg.topics.algo_status_topic, 10),
            message_type=String,
        )
        self._env_status_pub = _StringPublisher(
            ros_publisher=self.create_publisher(String, cfg.topics.env_status_topic, 10),
            message_type=String,
        )
        self._selected_target_pub = _SelectedTargetPublisher(
            ros_publisher=self.create_publisher(
                PointStamped, cfg.topics.selected_status_topic, 10
            ),
            message_type=PointStamped,
            clock_getter=self.get_clock,
        )
        self._base_coord_pub = _BaseCoordPublisher(
            ros_publisher=self.create_publisher(
                PointStamped, cfg.base_coord.publish_topic, 10
            ),
            message_type=PointStamped,
            clock_getter=self.get_clock,
        )

        self._adapter = build_adapter(
            environment=cfg.environment,
            env_status_publisher=lambda _: None,
            turtle_cmd_publisher=turtle_cmd_publisher,
        )
        cmd_vel_transform = CmdVelTransformAdapter.from_config(
            cfg.adapter.cmd_vel_transform
        )
        self._cmd_pub = _WorkflowCommandPublisher(
            ros_publisher=workflow_cmd_ros_publisher,
            message_type=Twist,
            adapter_cmd_handler=(
                self._adapter.on_cmd_vel if cfg.environment == "turtle" else None
            ),
            cmd_vel_transform=cmd_vel_transform,
        )
        self._resources = WorkflowResources(cfg=cfg, logger=self.get_logger())
        self._workflow_context = WorkflowContext(
            cfg=cfg,
            resources=self._resources,
            publishers=WorkflowPublishers(
                cmd_pub=self._cmd_pub,
                phase_pub=self._phase_pub,
                algo_status_pub=self._algo_status_pub,
                env_status_pub=self._env_status_pub,
                selected_target_pub=self._selected_target_pub,
                base_coord_pub=self._base_coord_pub,
            ),
            logger=self.get_logger(),
            clock=self.get_clock(),
            adapter=self._adapter,
        )
        self._engine = WorkflowEngine(
            phase_sequence=cfg.phase_sequence,
            runners=build_phase_registry(),
            context=self._workflow_context,
        )
        self._timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info(_build_startup_log_message(cfg))

    @property
    def cmd_pub(self) -> _TwistPublisher:
        return self._cmd_pub

    @property
    def phase_pub(self) -> _StringPublisher:
        return self._phase_pub

    @property
    def algo_status_pub(self) -> _StringPublisher:
        return self._algo_status_pub

    @property
    def env_status_pub(self) -> _StringPublisher:
        return self._env_status_pub

    @property
    def selected_target_pub(self) -> _SelectedTargetPublisher:
        return self._selected_target_pub

    @property
    def base_coord_pub(self) -> _BaseCoordPublisher:
        return self._base_coord_pub

    def _is_ros_context_ok(self) -> bool:
        return rclpy.ok()

    def destroy_node(self) -> bool:
        resources = getattr(self, "_resources", None)
        if resources is not None:
            resources.release_all()
        return super().destroy_node()

    def _on_timer(self) -> None:
        result = self._engine.tick()
        self._handle_engine_stop_if_requested(result)

    def _handle_engine_stop_if_requested(self, result: Any) -> None:
        if getattr(result, "stop_reason", None) is None:
            return
        self.get_logger().info(
            _format_multiline_log(
                "phase sequence stop requested",
                (
                    ("reason", result.stop_reason),
                    ("phase", _enum_or_value(result.phase)),
                    ("algo_status", _enum_or_value(result.algo_status)),
                    ("action", "stopping_node"),
                ),
            ),
        )
        self.destroy_node()
        rclpy.try_shutdown()


def _build_startup_log_message(cfg: PidAlignmentWorkflowConfig) -> str:
    return _format_multiline_log(
        "starting pid_alignment_runner",
        (
            ("environment", cfg.environment),
            ("phase_sequence", ",".join(cfg.phase_sequence)),
            ("start_phase", cfg.start_phase),
            ("input_source", cfg.detector.input_source),
            ("status_profile", cfg.detector.status_profile),
            ("base_coord_profile", cfg.detector.base_coord_profile),
            ("cmd_topic", cfg.topics.cmd_topic),
            ("selected_status_topic", cfg.topics.selected_status_topic),
            ("base_coord_topic", cfg.base_coord.publish_topic),
            ("workflow_phase_topic", cfg.topics.workflow_phase_topic),
            ("algo_status_topic", cfg.topics.algo_status_topic),
            ("env_status_topic", cfg.topics.env_status_topic),
            ("turtle_cmd_topic", cfg.adapter.turtle_cmd_topic),
            (
                "cmd_vel_transform",
                _format_cmd_vel_transform(cfg.adapter.cmd_vel_transform),
            ),
            ("target_x", f"{cfg.target_x:.3f}"),
            ("tolerance_px", f"{cfg.tolerance_px:.3f}"),
            ("forward_approach_speed_mps", f"{cfg.forward_approach.speed_mps:.3f}"),
            ("forward_approach_distance_m", f"{cfg.forward_approach.distance_m:.3f}"),
        ),
    )


def _format_cmd_vel_transform(transform: Any) -> str:
    return (
        f"invert_linear_x={bool(getattr(transform, 'invert_linear_x', False))} "
        f"invert_linear_y={bool(getattr(transform, 'invert_linear_y', False))} "
        f"invert_angular_z={bool(getattr(transform, 'invert_angular_z', False))}"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the PID alignment workflow node")
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    node: PidAlignmentRosNode | None = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        cfg = load_pid_alignment_config(Path(args.config))
        node = PidAlignmentRosNode(cfg=cfg)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
