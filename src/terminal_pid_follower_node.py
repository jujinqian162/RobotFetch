#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
import sys
from typing import Any

import cv2
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node


def _add_basedetect_to_sys_path() -> Path:
    project_root = Path(__file__).resolve().parents[1]
    basedetect_root = project_root / "BaseDetect"
    if not basedetect_root.exists():
        raise FileNotFoundError(f"BaseDetect not found: {basedetect_root}")
    if str(basedetect_root) not in sys.path:
        sys.path.insert(0, str(basedetect_root))
    return basedetect_root


BASEDETECT_ROOT = _add_basedetect_to_sys_path()
PROJECT_ROOT = BASEDETECT_ROOT.parent
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "terminal_pid_follower.yaml"

from sdk import BaseCoordTarget, Detector, StatusTarget


def _resolve_source(source: str) -> str | int:
    if source.isdigit():
        return int(source)
    path = Path(source).expanduser()
    if not path.is_absolute():
        path = (PROJECT_ROOT / path).resolve()
    return str(path)


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _as_bool(value: Any, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return default


def _as_set(value: Any) -> set[str]:
    if value is None:
        return set()
    if isinstance(value, str):
        return {chunk.strip() for chunk in value.split(",") if chunk.strip()}
    if isinstance(value, list):
        return {str(item).strip() for item in value if str(item).strip()}
    return set()


def _nested(payload: dict[str, Any], path: list[str], default: Any) -> Any:
    cur: Any = payload
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float
    deadband: float
    derivative_alpha: float


@dataclass
class NodeConfig:
    sdk_config: Path
    source: str
    hz: float
    cmd_topic: str
    selected_status_topic: str
    selected_base_topic: str
    frame_id: str

    status_profile: str
    base_coord_profile: str

    allowed_labels: set[str]
    target_x: float
    x_tolerance: float
    require_ready: bool
    use_stable_labels: bool
    control_axis: str
    control_sign: float
    stop_on_lost: bool

    base_coord_target_x: float
    base_coord_labels: set[str]
    base_coord_require_ready: bool

    forward_approach_enabled: bool
    forward_approach_distance_m: float

    log_every: int
    pid: PIDConfig


def _load_config(path: Path) -> NodeConfig:
    import yaml

    if not path.exists():
        raise FileNotFoundError(f"PID config not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        payload = yaml.safe_load(f) or {}
    if not isinstance(payload, dict):
        raise ValueError("PID config must be a YAML mapping")

    root = payload.get("terminal_pid_follower", payload)
    if not isinstance(root, dict):
        raise ValueError("terminal_pid_follower section must be a mapping")

    sdk_config_raw = str(
        _nested(root, ["sdk", "config"], "BaseDetect/configs/basedetect_sdk.yaml")
    )
    sdk_config = Path(sdk_config_raw).expanduser()
    if not sdk_config.is_absolute():
        sdk_config = (PROJECT_ROOT / sdk_config).resolve()

    source = str(_nested(root, ["input", "source"], "0"))
    hz = float(_nested(root, ["runtime", "hz"], 15.0))
    cmd_topic = str(_nested(root, ["topics", "cmd_vel"], "/cmd_vel"))
    selected_status_topic = str(
        _nested(
            root,
            ["topics", "selected_status_target"],
            "/robot_fetch/selected_target_px",
        )
    )
    selected_base_topic = str(
        _nested(root, ["topics", "selected_base_coord"], "/robot_fetch/target_position")
    )
    frame_id = str(_nested(root, ["topics", "frame_id"], "camera_link"))

    status_profile = str(
        _nested(root, ["modes", "status_profile"], "status_competition")
    )
    base_coord_profile = str(
        _nested(root, ["modes", "base_coord_profile"], "base_coord_competition")
    )

    allowed_labels = _as_set(
        _nested(root, ["status_align", "labels"], ["spearhead", "fist", "palm"])
    )
    target_x = float(_nested(root, ["status_align", "target_x"], 320.0))
    x_tolerance = abs(float(_nested(root, ["status_align", "x_tolerance"], 8.0)))
    require_ready = _as_bool(
        _nested(root, ["status_align", "require_ready"], True), True
    )
    use_stable_labels = _as_bool(
        _nested(root, ["status_align", "use_stable_labels"], True),
        True,
    )
    control_axis = str(_nested(root, ["status_align", "control_axis"], "linear_y"))
    if control_axis not in {"linear_y", "angular_z"}:
        raise ValueError("status_align.control_axis must be 'linear_y' or 'angular_z'")
    control_sign = float(_nested(root, ["status_align", "control_sign"], -1.0))
    stop_on_lost = _as_bool(_nested(root, ["safety", "stop_on_lost"], True), True)

    base_coord_target_x = float(_nested(root, ["base_coord", "target_x"], target_x))
    base_coord_labels = _as_set(_nested(root, ["base_coord", "labels"], []))
    base_coord_require_ready = _as_bool(
        _nested(root, ["base_coord", "require_ready"], True),
        True,
    )

    forward_approach_enabled = _as_bool(
        _nested(root, ["forward_approach", "enabled"], False),
        False,
    )
    forward_approach_distance_m = float(
        _nested(root, ["forward_approach", "distance_m"], 0.20)
    )

    log_every = max(1, int(_nested(root, ["runtime", "log_every"], 5)))
    pid = PIDConfig(
        kp=float(_nested(root, ["pid", "kp"], 0.006)),
        ki=float(_nested(root, ["pid", "ki"], 0.0)),
        kd=float(_nested(root, ["pid", "kd"], 0.0008)),
        output_limit=abs(float(_nested(root, ["pid", "max_speed"], 0.25))),
        integral_limit=abs(float(_nested(root, ["pid", "integral_limit"], 1000.0))),
        deadband=abs(float(_nested(root, ["pid", "deadband"], 0.0))),
        derivative_alpha=float(_nested(root, ["pid", "derivative_alpha"], 0.35)),
    )

    return NodeConfig(
        sdk_config=sdk_config,
        source=source,
        hz=hz,
        cmd_topic=cmd_topic,
        selected_status_topic=selected_status_topic,
        selected_base_topic=selected_base_topic,
        frame_id=frame_id,
        status_profile=status_profile,
        base_coord_profile=base_coord_profile,
        allowed_labels=allowed_labels,
        target_x=target_x,
        x_tolerance=x_tolerance,
        require_ready=require_ready,
        use_stable_labels=use_stable_labels,
        control_axis=control_axis,
        control_sign=control_sign,
        stop_on_lost=stop_on_lost,
        base_coord_target_x=base_coord_target_x,
        base_coord_labels=base_coord_labels,
        base_coord_require_ready=base_coord_require_ready,
        forward_approach_enabled=forward_approach_enabled,
        forward_approach_distance_m=forward_approach_distance_m,
        log_every=log_every,
        pid=pid,
    )


class PIDController:
    def __init__(self, cfg: PIDConfig) -> None:
        self._cfg = cfg
        self._integral = 0.0
        self._last_error: float | None = None
        self._last_derivative = 0.0
        self._last_time_s: float | None = None

    def set_config(self, cfg: PIDConfig) -> None:
        self._cfg = cfg

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_derivative = 0.0
        self._last_time_s = None

    def update(self, error: float, now_s: float) -> float:
        cfg = self._cfg
        if abs(error) <= cfg.deadband:
            error = 0.0

        dt = 0.0
        if self._last_time_s is not None:
            dt = max(1e-6, now_s - self._last_time_s)

        derivative = 0.0
        if self._last_error is not None and dt > 0.0:
            raw_derivative = (error - self._last_error) / dt
            alpha = _clamp(cfg.derivative_alpha, 0.0, 1.0)
            derivative = alpha * raw_derivative + (1.0 - alpha) * self._last_derivative

        p_term = cfg.kp * error
        d_term = cfg.kd * derivative

        integral_candidate = self._integral
        if dt > 0.0 and cfg.ki != 0.0:
            integral_candidate = _clamp(
                self._integral + error * dt,
                -cfg.integral_limit,
                cfg.integral_limit,
            )

        i_term = cfg.ki * integral_candidate
        output_unclamped = p_term + i_term + d_term
        output = _clamp(output_unclamped, -cfg.output_limit, cfg.output_limit)

        saturated_high = output >= cfg.output_limit and error > 0.0
        saturated_low = output <= -cfg.output_limit and error < 0.0
        if not (saturated_high or saturated_low):
            self._integral = integral_candidate

        self._last_error = error
        self._last_derivative = derivative
        self._last_time_s = now_s
        return output


class Phase(str, Enum):
    STATUS_ALIGN = "status_align"
    FORWARD_APPROACH_TODO = "forward_approach_todo"
    BASE_COORD_BROADCAST = "base_coord_broadcast"


def _select_status_target(
    *,
    targets: list[StatusTarget],
    target_x: float,
    allowed_labels: set[str],
    stable_labels: set[str],
    use_stable_labels: bool,
) -> StatusTarget | None:
    filtered = [t for t in targets if not allowed_labels or t.label in allowed_labels]
    if not filtered:
        return None

    if use_stable_labels and stable_labels:
        stable_filtered = [t for t in filtered if t.label in stable_labels]
        if stable_filtered:
            filtered = stable_filtered

    return min(filtered, key=lambda item: abs(item.cx - target_x))


def _select_base_coord_target(
    *,
    targets: list[BaseCoordTarget],
    target_x: float,
    labels: set[str],
) -> BaseCoordTarget | None:
    filtered = [t for t in targets if not labels or t.label in labels]
    if not filtered:
        return None
    return min(filtered, key=lambda item: abs(item.cx - target_x))


class TerminalPidFollowerNode(Node):
    def __init__(self, *, config_path: Path, hot_reload: bool) -> None:
        super().__init__("terminal_pid_follower")
        self._config_path = config_path
        self._hot_reload = hot_reload
        self._config = _load_config(config_path)

        self._detector = Detector(
            config=str(self._config.sdk_config),
            profile=self._config.status_profile,
        )
        if self._detector.mode != "status":
            raise ValueError(
                f"status_profile must be status mode, got mode={self._detector.mode}"
            )

        self._cap = cv2.VideoCapture(_resolve_source(self._config.source))
        if not self._cap.isOpened():
            raise RuntimeError(f"Unable to open source: {self._config.source}")

        self._cmd_pub = self.create_publisher(Twist, self._config.cmd_topic, 10)
        self._status_target_pub = self.create_publisher(
            PointStamped,
            self._config.selected_status_topic,
            10,
        )
        self._base_coord_pub = self.create_publisher(
            PointStamped,
            self._config.selected_base_topic,
            10,
        )

        self._pid = PIDController(self._config.pid)
        self._phase = Phase.STATUS_ALIGN
        self._todo_logged = False
        self._loop_count = 0
        self._last_hz = max(0.1, self._config.hz)
        self._timer = self.create_timer(1.0 / self._last_hz, self._on_timer)

        self.get_logger().info(
            f"PID follower started. config={config_path} hot_reload={self._hot_reload} "
            f"status_profile={self._config.status_profile} base_profile={self._config.base_coord_profile}"
        )

    def destroy_node(self) -> bool:
        self._publish_stop()
        self._cap.release()
        return super().destroy_node()

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _publish_status_target(self, target: StatusTarget) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "image"
        msg.point.x = target.cx
        msg.point.y = target.cy
        msg.point.z = target.width * target.height
        self._status_target_pub.publish(msg)

    def _publish_base_coord_target(self, target: BaseCoordTarget) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id
        msg.point.x = target.x
        msg.point.y = target.y
        msg.point.z = target.z
        self._base_coord_pub.publish(msg)

    def _reload_config_if_needed(self) -> None:
        if not self._hot_reload:
            return
        old = self._config
        try:
            updated = _load_config(self._config_path)
        except Exception as exc:
            if self._loop_count % max(1, self._config.log_every) == 0:
                self.get_logger().warning(f"Hot reload failed, keep old config: {exc}")
            return

        self._config = updated
        self._pid.set_config(updated.pid)

        if updated.source != old.source:
            old_cap = self._cap
            old_cap.release()
            new_cap = cv2.VideoCapture(_resolve_source(updated.source))
            if not new_cap.isOpened():
                new_cap.release()
                self._cap = cv2.VideoCapture(_resolve_source(old.source))
                self.get_logger().warning(
                    f"Hot reload source open failed: {updated.source}. Reverting to previous source."
                )
            else:
                self._cap = new_cap

        if updated.cmd_topic != old.cmd_topic:
            self._cmd_pub = self.create_publisher(Twist, updated.cmd_topic, 10)
        if updated.selected_status_topic != old.selected_status_topic:
            self._status_target_pub = self.create_publisher(
                PointStamped,
                updated.selected_status_topic,
                10,
            )
        if updated.selected_base_topic != old.selected_base_topic:
            self._base_coord_pub = self.create_publisher(
                PointStamped,
                updated.selected_base_topic,
                10,
            )

        new_hz = max(0.1, updated.hz)
        if abs(new_hz - self._last_hz) > 1e-9:
            self._timer.cancel()
            self._timer = self.create_timer(1.0 / new_hz, self._on_timer)
            self._last_hz = new_hz

        if (
            self._phase == Phase.STATUS_ALIGN
            and self._detector.profile_name != updated.status_profile
        ):
            self._detector.switch_profile(updated.status_profile)
            self._pid.reset()
        if (
            self._phase == Phase.BASE_COORD_BROADCAST
            and self._detector.profile_name != updated.base_coord_profile
        ):
            self._detector.switch_profile(updated.base_coord_profile)

    def _to_base_coord_phase(self) -> None:
        if self._detector.profile_name != self._config.base_coord_profile:
            self._detector.switch_profile(self._config.base_coord_profile)
        if self._detector.mode != "base_coord":
            raise ValueError(
                f"base_coord_profile must be base_coord mode, got mode={self._detector.mode}"
            )
        self._phase = Phase.BASE_COORD_BROADCAST
        self._publish_stop()
        self.get_logger().info(
            f"Switched to base coord mode. profile={self._config.base_coord_profile}"
        )

    def _handle_status_align(self, frame: Any) -> None:
        stable_labels_raw = self._detector.detect(frame)
        status_targets = self._detector.latest_status_targets()

        if self._config.require_ready and not self._detector.ready:
            self._pid.reset()
            if self._config.stop_on_lost:
                self._publish_stop()
            return

        target = _select_status_target(
            targets=status_targets,
            target_x=self._config.target_x,
            allowed_labels=self._config.allowed_labels,
            stable_labels=set(stable_labels_raw),
            use_stable_labels=self._config.use_stable_labels,
        )
        if target is None:
            self._pid.reset()
            if self._config.stop_on_lost:
                self._publish_stop()
            return

        self._publish_status_target(target)
        error_x = target.cx - self._config.target_x
        aligned = abs(error_x) <= self._config.x_tolerance

        cmd = Twist()
        if aligned:
            self._pid.reset()
            self._publish_stop()
            self._phase = Phase.FORWARD_APPROACH_TODO
            self.get_logger().info(
                f"Status alignment done: label={target.label} cx={target.cx:.1f} err_x={error_x:.1f}"
            )
            return

        now_s = self.get_clock().now().nanoseconds * 1e-9
        speed = self._config.control_sign * self._pid.update(error_x, now_s)
        if self._config.control_axis == "linear_y":
            cmd.linear.y = float(speed)
        else:
            cmd.angular.z = float(speed)
        self._cmd_pub.publish(cmd)

        if self._loop_count % self._config.log_every == 0:
            self.get_logger().info(
                f"status target={target.label} cx={target.cx:.1f} err_x={error_x:.1f} "
                f"cmd(vy={cmd.linear.y:.3f} wz={cmd.angular.z:.3f})"
            )

    def _handle_forward_approach_todo(self) -> None:
        self._publish_stop()
        if not self._todo_logged:
            if self._config.forward_approach_enabled:
                self.get_logger().info(
                    "TODO: forward approach is reserved but not implemented yet. "
                    f"requested_distance={self._config.forward_approach_distance_m:.3f}m"
                )
            else:
                self.get_logger().info(
                    "Forward approach disabled, skip to base coord mode"
                )
            self._todo_logged = True
        self._to_base_coord_phase()

    def _handle_base_coord_broadcast(self, frame: Any) -> None:
        self._publish_stop()
        self._detector.detect(frame)
        if self._config.base_coord_require_ready and not self._detector.ready:
            return

        targets = self._detector.latest_base_coord_targets()
        target = _select_base_coord_target(
            targets=targets,
            target_x=self._config.base_coord_target_x,
            labels=self._config.base_coord_labels,
        )
        if target is None:
            return

        self._publish_base_coord_target(target)
        if self._loop_count % self._config.log_every == 0:
            self.get_logger().info(
                f"base target={target.label} cx={target.cx:.1f} "
                f"xyz=({target.x:+.3f},{target.y:.3f},{target.z:+.3f})"
            )

    def _on_timer(self) -> None:
        self._reload_config_if_needed()

        ok, frame = self._cap.read()
        if not ok:
            self._pid.reset()
            self._publish_stop()
            self.get_logger().warning("Frame read failed; stop command published")
            return

        if self._phase == Phase.STATUS_ALIGN:
            self._handle_status_align(frame)
        elif self._phase == Phase.FORWARD_APPROACH_TODO:
            self._handle_forward_approach_todo()
        else:
            self._handle_base_coord_broadcast(frame)

        self._loop_count += 1


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Terminal PID follower with YAML config (status -> base_coord)"
    )
    parser.add_argument(
        "--hot-reload",
        action="store_true",
        help="Reload YAML config on every control loop",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = TerminalPidFollowerNode(
        config_path=DEFAULT_CONFIG_PATH, hot_reload=args.hot_reload
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
