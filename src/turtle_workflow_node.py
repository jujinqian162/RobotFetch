#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

WORKTREE_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = WORKTREE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from adapters.turtle_adapter import TurtleAdapter, TurtleVelocityCommand


class TurtleWorkflowNode(Node):
    def __init__(
        self,
        *,
        node_name: str,
        phase_topic: str,
        cmd_topic: str,
        env_status_topic: str,
        turtle_cmd_topic: str,
    ) -> None:
        super().__init__(node_name)
        self._string_msg_type = String
        self._twist_msg_type = Twist
        self._env_status_pub = self.create_publisher(String, env_status_topic, 10)
        self._turtle_cmd_pub = self.create_publisher(Twist, turtle_cmd_topic, 10)
        self._adapter = TurtleAdapter(env_status_publisher=lambda _: None)

        self.create_subscription(String, phase_topic, self._on_phase, 10)
        self.create_subscription(Twist, cmd_topic, self._on_cmd_vel, 10)

    def _on_phase(self, msg: String) -> None:
        env_status = self._adapter.on_phase(msg.data)
        status_msg = self._string_msg_type()
        status_msg.data = env_status.value
        self._env_status_pub.publish(status_msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        command = self._adapter.on_cmd_vel(msg)
        self._turtle_cmd_pub.publish(self._build_turtle_twist(command))

    def _build_turtle_twist(self, command: TurtleVelocityCommand) -> Twist:
        twist = self._twist_msg_type()
        twist.linear.x = command.linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = command.angular_z
        return twist


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge workflow topics to turtlesim")
    parser.add_argument("--node-name", default="turtle_workflow_node")
    parser.add_argument("--phase-topic", default="/workflow/phase")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--env-status-topic", default="/workflow/env_status")
    parser.add_argument("--turtle-cmd-topic", default="/turtle1/cmd_vel")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TurtleWorkflowNode(
        node_name=args.node_name,
        phase_topic=args.phase_topic,
        cmd_topic=args.cmd_topic,
        env_status_topic=args.env_status_topic,
        turtle_cmd_topic=args.turtle_cmd_topic,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
