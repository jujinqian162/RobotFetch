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

from adapters.robot_adapter import RobotAdapter
from adapters.turtle_adapter import TurtleAdapter
from algorithms.detector_gateway import DetectorGateway
from algorithms.pid import PIDConfig
from algorithms.status_align import StatusAlignConfig, StatusAlignStep
from config.loaders import load_pid_alignment_config
from config.models import CameraFallbackConfig, PidAlignmentWorkflowConfig
from runners.pid_alignment_runner import (
    RunnerConfig,
    log_deduplicated,
    run_status_align_once,
)


ZERO_CMD_MESSAGE = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


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
    ) -> None:
        self._ros_publisher = ros_publisher
        self._message_type = message_type
        self._adapter_cmd_handler = adapter_cmd_handler
        self._workflow_publisher = (
            None
            if ros_publisher is None
            else _TwistPublisher(
                ros_publisher=ros_publisher,
                message_type=message_type,
            )
        )

    def publish(self, payload: dict[str, float]) -> None:
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


def build_status_align_step(cfg: PidAlignmentWorkflowConfig) -> StatusAlignStep:
    return StatusAlignStep(
        cfg=StatusAlignConfig(
            pid=PIDConfig(
                kp=0.006,
                ki=0.0,
                kd=0.0008,
                output_limit=cfg.max_speed,
                integral_limit=1000.0,
                deadband=0.0,
                derivative_alpha=0.35,
            ),
            target_x=cfg.target_x,
            tolerance_px=cfg.tolerance_px,
            allowed_labels={"spearhead", "fist", "palm"},
            stable_labels=set(),
            use_stable_labels=True,
        )
    )


def build_capture(input_source: str) -> Any:
    import cv2

    source = _resolve_input_source(input_source)
    capture = cv2.VideoCapture(source)
    if hasattr(capture, "isOpened") and not capture.isOpened():
        raise RuntimeError(f"Unable to open source: {input_source}")
    return capture


def build_v4l2_mjpg_capture(input_source: str, config: CameraFallbackConfig) -> Any:
    import cv2

    source = _resolve_input_source(input_source)
    if not isinstance(source, int):
        raise RuntimeError(f"V4L2 MJPG fallback requires numeric source: {input_source}")
    if config.backend != "v4l2":
        raise ValueError(f"Unsupported camera fallback backend: {config.backend}")

    capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    capture.set(cv2.CAP_PROP_FPS, config.fps)
    if hasattr(capture, "isOpened") and not capture.isOpened():
        release = getattr(capture, "release", None)
        if callable(release):
            release()
        raise RuntimeError(
            "Unable to open source with camera fallback: "
            f"{input_source} {config.describe()}"
        )
    return capture


def build_readable_v4l2_mjpg_capture(
    input_source: str,
    configs: tuple[CameraFallbackConfig, ...],
) -> tuple[Any, Any, CameraFallbackConfig]:
    last_error: Exception | None = None
    for config in configs:
        capture = None
        try:
            capture = build_v4l2_mjpg_capture(input_source, config)
            ok, frame = capture.read()
            if ok:
                return capture, frame, config
        except Exception as exc:
            last_error = exc
        if capture is not None:
            release = getattr(capture, "release", None)
            if callable(release):
                release()

    message = f"Unable to read from camera fallback source: {input_source}"
    if last_error is not None:
        message = f"{message}; last_error={last_error!r}"
    raise RuntimeError(message)


def _resolve_input_source(input_source: str) -> str | int:
    if input_source.isdigit():
        return int(input_source)
    return input_source


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


def _format_camera_fallback_log(
    title: str,
    *,
    input_source: str,
    config: CameraFallbackConfig,
) -> str:
    return _format_multiline_log(
        title,
        (
            ("source", input_source),
            ("backend", config.backend),
            ("fourcc", config.fourcc),
            ("width", config.width),
            ("height", config.height),
            ("fps", f"{config.fps:g}"),
        ),
    )


class PidAlignmentRosNode(Node):
    def __init__(self, *, cfg: PidAlignmentWorkflowConfig) -> None:
        super().__init__("pid_alignment_runner")
        self._cfg = cfg
        self._capture_released = False
        self._capture_read_fallback_attempted = False
        self._runner_cfg = RunnerConfig(
            cmd_topic=cfg.topics.cmd_topic,
            selected_status_topic=cfg.topics.selected_status_topic,
            workflow_phase_topic=cfg.topics.workflow_phase_topic,
            algo_status_topic=cfg.topics.algo_status_topic,
            env_status_topic=cfg.topics.env_status_topic,
            frame_id="camera_link",
            phase_sequence=cfg.phase_sequence,
            target_x=cfg.target_x,
        )

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

        self._adapter = build_adapter(
            environment=cfg.environment,
            env_status_publisher=lambda _: None,
            turtle_cmd_publisher=turtle_cmd_publisher,
        )
        self._cmd_pub = _WorkflowCommandPublisher(
            ros_publisher=workflow_cmd_ros_publisher,
            message_type=Twist,
            adapter_cmd_handler=(
                self._adapter.on_cmd_vel if cfg.environment == "turtle" else None
            ),
        )
        self._gateway = DetectorGateway(
            config_path=cfg.detector.sdk_config,
            initial_profile=cfg.detector.status_profile,
        )
        self._status_align = build_status_align_step(cfg)
        self._cap = build_capture(cfg.detector.input_source)
        self._timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info(_build_startup_log_message(cfg))
        self.get_logger().info(
            _build_capture_log_message(cfg.detector.input_source, self._cap)
        )

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

    def _is_ros_context_ok(self) -> bool:
        return rclpy.ok()

    def destroy_node(self) -> bool:
        if not self._capture_released:
            release = getattr(self._cap, "release", None)
            if callable(release):
                release()
            self._capture_released = True
        return super().destroy_node()

    def _on_timer(self) -> None:
        ok, frame = self._cap.read()
        if not ok:
            ok, frame = self._try_read_with_capture_fallback()
        if not ok:
            self._cmd_pub.publish(ZERO_CMD_MESSAGE)
            log_deduplicated(
                node=self,
                logger=self.get_logger(),
                level="warning",
                message=(
                    "Frame read failed; published zero velocity to "
                    f"{self._runner_cfg.cmd_topic}"
                ),
            )
            return

        self._adapter.on_phase(self._cfg.start_phase)
        cycle = run_status_align_once(
            node=self,
            frame=frame,
            detector_gateway=self._gateway,
            status_align_step=self._status_align,
            cfg=self._runner_cfg,
        )
        if cycle.stop_requested:
            log_deduplicated(
                node=self,
                logger=self.get_logger(),
                level="info",
                message=_format_multiline_log(
                    "phase sequence stop requested",
                    (
                        ("reason", cycle.stop_reason),
                        ("phase", cycle.phase),
                        ("algo_status", cycle.algo_status),
                        ("action", "stopping_node"),
                    ),
                ),
            )
            self.destroy_node()
            rclpy.try_shutdown()

    def _try_read_with_capture_fallback(self) -> tuple[bool, Any]:
        input_source = self._cfg.detector.input_source
        if self._capture_read_fallback_attempted:
            return False, None
        if not isinstance(_resolve_input_source(input_source), int):
            return False, None

        self._capture_read_fallback_attempted = True
        release = getattr(self._cap, "release", None)
        if callable(release):
            release()

        for config in self._cfg.detector.camera_fallbacks:
            log_deduplicated(
                node=self,
                logger=self.get_logger(),
                level="warning",
                message=_format_camera_fallback_log(
                    "Frame read failed; trying camera fallback",
                    input_source=input_source,
                    config=config,
                ),
            )
            try:
                self._cap, frame, selected_config = build_readable_v4l2_mjpg_capture(
                    input_source,
                    (config,),
                )
            except RuntimeError:
                continue

            shape = getattr(frame, "shape", None)
            self.get_logger().info(
                _format_multiline_log(
                    "Camera fallback selected",
                    (
                        ("source", input_source),
                        ("backend", selected_config.backend),
                        ("fourcc", selected_config.fourcc),
                        ("width", selected_config.width),
                        ("height", selected_config.height),
                        ("fps", f"{selected_config.fps:g}"),
                        ("shape", shape),
                    ),
                )
            )
            return True, frame

        log_deduplicated(
            node=self,
            logger=self.get_logger(),
            level="warning",
            message=f"All camera fallback configs failed for source={input_source}",
        )
        return False, None


def _build_startup_log_message(cfg: PidAlignmentWorkflowConfig) -> str:
    return _format_multiline_log(
        "starting pid_alignment_runner",
        (
            ("environment", cfg.environment),
            ("phase_sequence", ",".join(cfg.phase_sequence)),
            ("start_phase", cfg.start_phase),
            ("input_source", cfg.detector.input_source),
            ("status_profile", cfg.detector.status_profile),
            ("cmd_topic", cfg.topics.cmd_topic),
            ("selected_status_topic", cfg.topics.selected_status_topic),
            ("workflow_phase_topic", cfg.topics.workflow_phase_topic),
            ("algo_status_topic", cfg.topics.algo_status_topic),
            ("env_status_topic", cfg.topics.env_status_topic),
            ("turtle_cmd_topic", cfg.adapter.turtle_cmd_topic),
            ("target_x", f"{cfg.target_x:.3f}"),
            ("tolerance_px", f"{cfg.tolerance_px:.3f}"),
        ),
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
