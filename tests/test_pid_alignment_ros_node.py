from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import runners.pid_alignment_ros_node as ros_node
from config.models import AdapterConfig, DetectorConfig, PidAlignmentWorkflowConfig, TopicConfig
from workflow.types import AlgoStatus, Phase


def make_cfg(*, environment: str = "turtle", one_shot: bool = True) -> PidAlignmentWorkflowConfig:
    return PidAlignmentWorkflowConfig(
        environment=environment,
        start_phase=Phase.STATUS_ALIGN.value,
        one_shot=one_shot,
        target_x=320.0,
        tolerance_px=8.0,
        topics=TopicConfig(
            cmd_topic="/cmd_vel",
            workflow_phase_topic="/workflow/phase",
            algo_status_topic="/workflow/algo_status",
            env_status_topic="/workflow/env_status",
            selected_status_topic="/robot_fetch/selected_target_px",
        ),
        detector=DetectorConfig(
            sdk_config=Path("BaseDetect/configs/basedetect_sdk.yaml"),
            status_profile="status_competition",
            input_source="0",
        ),
        adapter=AdapterConfig(turtle_cmd_topic="/turtle1/cmd_vel"),
    )


def import_with_fake_ros(monkeypatch):
    class FakeNode:
        def __init__(self, node_name: str) -> None:
            self.node_name = node_name

        def destroy_node(self) -> bool:
            return True

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    class FakeString:
        def __init__(self) -> None:
            self.data = ""

    class FakeTwist:
        def __init__(self) -> None:
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class FakePointStamped:
        def __init__(self) -> None:
            self.header = SimpleNamespace(stamp=None, frame_id="")
            self.point = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda: None
    fake_rclpy.shutdown = lambda: None
    fake_rclpy.spin = lambda node: None

    fake_rclpy_node = types.ModuleType("rclpy.node")
    fake_rclpy_node.Node = FakeNode

    fake_geometry = types.ModuleType("geometry_msgs")
    fake_geometry_msg = types.ModuleType("geometry_msgs.msg")
    fake_geometry_msg.PointStamped = FakePointStamped
    fake_geometry_msg.Twist = FakeTwist

    fake_std = types.ModuleType("std_msgs")
    fake_std_msg = types.ModuleType("std_msgs.msg")
    fake_std_msg.String = FakeString

    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", fake_rclpy_node)
    monkeypatch.setitem(sys.modules, "geometry_msgs", fake_geometry)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", fake_geometry_msg)
    monkeypatch.setitem(sys.modules, "std_msgs", fake_std)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", fake_std_msg)
    sys.modules.pop("runners.pid_alignment_ros_node", None)
    return importlib.import_module("runners.pid_alignment_ros_node")


def test_build_adapter_returns_turtle_adapter_for_turtle_environment():
    adapter = ros_node.build_adapter(
        environment="turtle",
        env_status_publisher=lambda _: None,
    )

    assert adapter.__class__.__name__ == "TurtleAdapter"


def test_build_adapter_returns_robot_adapter_for_robot_environment():
    adapter = ros_node.build_adapter(
        environment="robot",
        env_status_publisher=lambda _: None,
    )

    assert adapter.__class__.__name__ == "RobotAdapter"


def test_on_timer_runs_one_shot_cycle_and_destroys_node(monkeypatch):
    calls: dict[str, object] = {}

    def fake_run_status_align_once(**kwargs):
        calls.update(kwargs)
        return SimpleNamespace(
            stop_requested=True,
            phase=Phase.STATUS_ALIGN.value,
            algo_status=AlgoStatus.ALIGNED.value,
        )

    monkeypatch.setattr(ros_node, "run_status_align_once", fake_run_status_align_once)

    class FakeCapture:
        def read(self):
            return True, "frame-1"

    class FakeAdapter:
        def __init__(self) -> None:
            self.phases: list[str] = []

        def on_phase(self, phase: str) -> None:
            self.phases.append(phase)

    class FakeLogger:
        def __init__(self) -> None:
            self.messages: list[str] = []

        def warning(self, message: str) -> None:
            self.messages.append(message)

    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._cfg = make_cfg(one_shot=True)
    node._runner_cfg = SimpleNamespace(one_shot=True)
    node._cap = FakeCapture()
    node._adapter = FakeAdapter()
    node._gateway = object()
    node._status_align = object()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.destroyed = False
    node.destroy_node = lambda: setattr(node, "destroyed", True) or True

    node._on_timer()

    assert node._adapter.phases == [Phase.STATUS_ALIGN.value]
    assert calls["frame"] == "frame-1"
    assert calls["node"] is node
    assert calls["detector_gateway"] is node._gateway
    assert calls["status_align_step"] is node._status_align
    assert calls["cfg"] is node._runner_cfg
    assert calls["one_shot"] is True
    assert node.destroyed is True


def test_pid_alignment_ros_node_does_not_route_adapter_env_status_to_topic(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    captured: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class FakeCapture:
        def read(self):
            return False, None

        def release(self) -> None:
            return None

    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_publisher",
        lambda self, msg_type, topic, qos_depth: FakeRosPublisher(),
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_timer",
        lambda self, period_s, callback: SimpleNamespace(period_s=period_s, callback=callback),
        raising=False,
    )
    monkeypatch.setattr(fake_ros_node, "build_capture", lambda input_source: FakeCapture())
    monkeypatch.setattr(
        fake_ros_node,
        "DetectorGateway",
        lambda **kwargs: SimpleNamespace(**kwargs),
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_status_align_step",
        lambda cfg: SimpleNamespace(cfg=cfg),
    )

    def fake_build_adapter(*, environment, env_status_publisher):
        captured["environment"] = environment
        captured["env_status_publisher"] = env_status_publisher
        return SimpleNamespace(on_phase=lambda phase: None)

    monkeypatch.setattr(fake_ros_node, "build_adapter", fake_build_adapter)

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))
    captured["env_status_publisher"]("READY")

    assert node.env_status_pub._ros_publisher.messages == []


def test_parse_args_defaults_config_path_from_worktree_root():
    args = ros_node.parse_args([])

    assert Path(args.config) == (
        ros_node.WORKTREE_ROOT / "configs/workflows/pid_alignment.robot.yaml"
    )


def test_main_loads_config_and_spins_node(monkeypatch):
    calls: list[tuple[str, object]] = []
    fake_cfg = make_cfg(environment="robot")
    created_nodes: list[object] = []

    monkeypatch.setattr(ros_node.rclpy, "init", lambda: calls.append(("init", None)))
    monkeypatch.setattr(ros_node.rclpy, "shutdown", lambda: calls.append(("shutdown", None)))
    monkeypatch.setattr(
        ros_node.rclpy,
        "spin",
        lambda node: calls.append(("spin", node)),
    )
    monkeypatch.setattr(
        ros_node,
        "load_pid_alignment_config",
        lambda path: calls.append(("load", Path(path))) or fake_cfg,
    )

    class FakePidAlignmentRosNode:
        def __init__(self, *, cfg):
            calls.append(("node_init", cfg))
            created_nodes.append(self)

        def destroy_node(self):
            calls.append(("destroy", None))
            return True

    monkeypatch.setattr(ros_node, "PidAlignmentRosNode", FakePidAlignmentRosNode)

    ros_node.main(["--config", "configs/workflows/pid_alignment.turtle.yaml"])

    assert calls == [
        ("init", None),
        ("load", Path("configs/workflows/pid_alignment.turtle.yaml")),
        ("node_init", fake_cfg),
        ("spin", created_nodes[0]),
        ("destroy", None),
        ("shutdown", None),
    ]
