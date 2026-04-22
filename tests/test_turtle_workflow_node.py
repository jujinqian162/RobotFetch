from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from turtle_workflow_node import TurtleWorkflowNode
from workflow.types import EnvStatus


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeAdapter:
    def on_phase(self, phase: str) -> EnvStatus:
        assert phase == "STATUS_ALIGN"
        return EnvStatus.READY

    def on_cmd_vel(self, cmd_vel: object) -> object:
        return SimpleNamespace(linear_x=0.2, angular_z=0.1)


class FakeString:
    def __init__(self, data: str = "") -> None:
        self.data = data


class FakeTwist:
    def __init__(self) -> None:
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)



def test_on_phase_publishes_env_status_string_message():
    node = TurtleWorkflowNode.__new__(TurtleWorkflowNode)
    node._adapter = FakeAdapter()
    node._env_status_pub = FakePublisher()
    node._string_msg_type = FakeString

    node._on_phase(FakeString("STATUS_ALIGN"))

    assert len(node._env_status_pub.messages) == 1
    assert isinstance(node._env_status_pub.messages[0], FakeString)
    assert node._env_status_pub.messages[0].data == EnvStatus.READY.value



def test_on_cmd_vel_publishes_turtle_twist_message():
    node = TurtleWorkflowNode.__new__(TurtleWorkflowNode)
    node._adapter = FakeAdapter()
    node._turtle_cmd_pub = FakePublisher()
    node._twist_msg_type = FakeTwist

    incoming = FakeTwist()
    incoming.linear.y = 0.2
    incoming.angular.z = 0.1

    node._on_cmd_vel(incoming)

    assert len(node._turtle_cmd_pub.messages) == 1
    published = node._turtle_cmd_pub.messages[0]
    assert isinstance(published, FakeTwist)
    assert published.linear.x == 0.2
    assert published.linear.y == 0.0
    assert published.angular.z == 0.1
