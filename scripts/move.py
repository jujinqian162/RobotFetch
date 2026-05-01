#!/usr/bin/env python3
"""Publish a short electric-control command for checking robot axis directions."""

from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass
from typing import Sequence


DEFAULT_TOPIC = "/t0x0101_robotfetch"
DEFAULT_DURATION_SEC = 0.1
DEFAULT_RATE_HZ = 20.0
DEFAULT_STARTUP_DELAY_SEC = 0.2
DEFAULT_STOP_COUNT = 3


@dataclass(frozen=True)
class MoveCommand:
    x_vel: float = 0.0
    y_vel: float = 0.0
    angular_z: float = 0.0
    duration_sec: float = DEFAULT_DURATION_SEC
    topic: str = DEFAULT_TOPIC
    rate_hz: float = DEFAULT_RATE_HZ
    startup_delay_sec: float = DEFAULT_STARTUP_DELAY_SEC
    stop_count: int = DEFAULT_STOP_COUNT


KEY_ALIASES = {
    "t": "duration_sec",
    "duration": "duration_sec",
    "duration_sec": "duration_sec",
    "x_vel": "x_vel",
    "y_vel": "y_vel",
    "angular_z": "angular_z",
    "yaw_vel": "angular_z",
    "topic": "topic",
    "rate_hz": "rate_hz",
    "startup_delay": "startup_delay_sec",
    "startup_delay_sec": "startup_delay_sec",
    "stop_count": "stop_count",
}


def usage() -> str:
    return "\n".join(
        [
            "Usage: scripts/move.py key=value [key=value ...]",
            "",
            "Examples:",
            "  scripts/move.py x_vel=0.1 t=0.1",
            "  scripts/move.py y_vel=0.1 t=0.1",
            "  scripts/move.py x_vel=-0.1 t=0.2 topic=/t0x0101_robotfetch",
            "",
            "Keys:",
            "  x_vel, y_vel              linear velocity in m/s",
            "  angular_z, yaw_vel        angular z velocity in rad/s",
            "  t, duration               command duration in seconds",
            "  topic                     Float32MultiArray topic, default /t0x0101_robotfetch",
            "  rate_hz                   publish rate, default 20",
        ]
    )


def parse_key_value_args(argv: Sequence[str]) -> MoveCommand:
    if not argv:
        raise ValueError("provide at least one key=value argument")

    values: dict[str, object] = {
        "x_vel": 0.0,
        "y_vel": 0.0,
        "angular_z": 0.0,
        "duration_sec": DEFAULT_DURATION_SEC,
        "topic": DEFAULT_TOPIC,
        "rate_hz": DEFAULT_RATE_HZ,
        "startup_delay_sec": DEFAULT_STARTUP_DELAY_SEC,
        "stop_count": DEFAULT_STOP_COUNT,
    }
    for arg in argv:
        if "=" not in arg:
            raise ValueError(f"expected key=value argument, got {arg!r}")
        raw_key, raw_value = arg.split("=", 1)
        key = KEY_ALIASES.get(raw_key)
        if key is None:
            known = ", ".join(sorted(KEY_ALIASES))
            raise ValueError(f"unknown key {raw_key!r}; known keys: {known}")
        if raw_value == "":
            raise ValueError(f"{raw_key!r} cannot be empty")
        if key == "topic":
            values[key] = raw_value
        elif key == "stop_count":
            values[key] = _parse_int(raw_key, raw_value)
        else:
            values[key] = _parse_float(raw_key, raw_value)

    command = MoveCommand(**values)
    _validate_command(command)
    return command


def _parse_float(key: str, value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise ValueError(f"{key!r} must be a number, got {value!r}") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{key!r} must be finite, got {value!r}")
    return parsed


def _parse_int(key: str, value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise ValueError(f"{key!r} must be an integer, got {value!r}") from exc
    return parsed


def _validate_command(command: MoveCommand) -> None:
    if not command.topic:
        raise ValueError("'topic' cannot be empty")
    if command.duration_sec <= 0:
        raise ValueError("'t'/'duration' must be greater than 0")
    if command.rate_hz <= 0:
        raise ValueError("'rate_hz' must be greater than 0")
    if command.startup_delay_sec < 0:
        raise ValueError("'startup_delay' cannot be negative")
    if command.stop_count < 1:
        raise ValueError("'stop_count' must be at least 1")


def build_float32_multi_array(
    message_type: type,
    command: MoveCommand,
    *,
    stop: bool = False,
):
    message = message_type()
    if stop:
        message.data = [0.0, 0.0, 0.0]
        return message

    message.data = [command.x_vel, command.y_vel, command.angular_z]
    return message


def run_ros_move(command: MoveCommand) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
    except ImportError as exc:
        print(
            "Failed to import ROS 2 Python packages. "
            "Activate the project/ROS environment first.",
            file=sys.stderr,
        )
        print(f"Import error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("robotfetch_move")
    publisher = node.create_publisher(Float32MultiArray, command.topic, 10)
    try:
        _spin_sleep(rclpy, node, command.startup_delay_sec)
        _publish_for_duration(rclpy, node, publisher, Float32MultiArray, command)
        _publish_stop(rclpy, node, publisher, Float32MultiArray, command)
        node.get_logger().info(
            "published Float32MultiArray to "
            f"{command.topic}: data=[{command.x_vel}, {command.y_vel}, {command.angular_z}] "
            f"angular_z={command.angular_z} duration={command.duration_sec}s"
        )
        return 0
    except KeyboardInterrupt:
        _publish_stop(rclpy, node, publisher, Float32MultiArray, command)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _publish_for_duration(
    rclpy_module,
    node,
    publisher,
    message_type: type,
    command: MoveCommand,
) -> None:
    period_sec = 1.0 / command.rate_hz
    end_time = time.monotonic() + command.duration_sec
    sent_count = 0
    while sent_count == 0 or time.monotonic() < end_time:
        publisher.publish(build_float32_multi_array(message_type, command))
        sent_count += 1
        remaining = end_time - time.monotonic()
        if remaining > 0:
            _spin_sleep(rclpy_module, node, min(period_sec, remaining))


def _publish_stop(
    rclpy_module,
    node,
    publisher,
    message_type: type,
    command: MoveCommand,
) -> None:
    period_sec = 1.0 / command.rate_hz
    for _ in range(command.stop_count):
        publisher.publish(build_float32_multi_array(message_type, command, stop=True))
        _spin_sleep(rclpy_module, node, period_sec)


def _spin_sleep(rclpy_module, node, duration_sec: float) -> None:
    deadline = time.monotonic() + duration_sec
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        rclpy_module.spin_once(node, timeout_sec=min(0.02, remaining))


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if any(arg in {"-h", "--help", "help"} for arg in args):
        print(usage())
        return 0
    try:
        command = parse_key_value_args(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        print(usage(), file=sys.stderr)
        return 2
    return run_ros_move(command)


if __name__ == "__main__":
    raise SystemExit(main())
