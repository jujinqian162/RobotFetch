from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import runners.pid_alignment_ros_node as ros_node
from config.models import AdapterConfig, DetectorConfig, PidAlignmentWorkflowConfig, TopicConfig
from workflow.types import AlgoStatus, Phase


def make_cfg(
    *,
    environment: str = "turtle",
    phase_sequence: tuple[str, ...] = (Phase.STATUS_ALIGN.value,),
) -> PidAlignmentWorkflowConfig:
    return PidAlignmentWorkflowConfig(
        environment=environment,
        start_phase=Phase.STATUS_ALIGN.value,
        phase_sequence=phase_sequence,
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
        adapter=AdapterConfig(
            turtle_cmd_topic="/turtle1/cmd_vel" if environment == "turtle" else None
        ),
    )


def import_with_fake_ros(monkeypatch):
    class FakeNode:
        def __init__(self, node_name: str) -> None:
            self.node_name = node_name
            self.logger = SimpleNamespace(info_messages=[], warning_messages=[])

        def destroy_node(self) -> bool:
            return True

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

        def get_logger(self):
            def info(message: str) -> None:
                self.logger.info_messages.append(message)

            def warning(message: str) -> None:
                self.logger.warning_messages.append(message)

            return SimpleNamespace(info=info, warning=warning)

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
    fake_rclpy.try_shutdown = lambda: None
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


def test_build_capture_keeps_default_numeric_camera_open(monkeypatch):
    calls: dict[str, object] = {}

    class FakeCapture:
        def __init__(self, source, backend=None) -> None:
            calls["source"] = source
            calls["backend"] = backend
            self.set_calls: list[tuple[int, object]] = []

        def isOpened(self) -> bool:
            return True

        def set(self, prop_id: int, value: object) -> None:
            self.set_calls.append((prop_id, value))

    fake_cv2 = types.SimpleNamespace(
        CAP_V4L2=200,
        CAP_PROP_FOURCC=6,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        VideoCapture=FakeCapture,
        VideoWriter_fourcc=lambda *chars: "FOURCC-" + "".join(chars),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    capture = ros_node.build_capture("0")

    assert calls == {"source": 0, "backend": None}
    assert capture.set_calls == []


def test_on_timer_stops_when_phase_sequence_requests_stop(monkeypatch):
    calls: dict[str, object] = {}
    shutdown_calls: list[object] = []

    def fake_run_status_align_once(**kwargs):
        calls.update(kwargs)
        return SimpleNamespace(
            stop_requested=True,
            stop_reason="phase_sequence_complete",
            phase=Phase.STATUS_ALIGN.value,
            algo_status=AlgoStatus.ALIGNED.value,
        )

    monkeypatch.setattr(ros_node, "run_status_align_once", fake_run_status_align_once)
    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda **kwargs: shutdown_calls.append(kwargs),
    )

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
            self.info_messages: list[str] = []
            self.warning_messages: list[str] = []

        def info(self, message: str) -> None:
            self.info_messages.append(message)

        def warning(self, message: str) -> None:
            self.warning_messages.append(message)

    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._cfg = make_cfg()
    node._runner_cfg = SimpleNamespace(phase_sequence=(Phase.STATUS_ALIGN.value,))
    node._cap = FakeCapture()
    node._adapter = FakeAdapter()
    node._gateway = object()
    node._status_align = object()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._is_ros_context_ok = lambda: True
    node.destroyed = False
    node.destroy_node = lambda: setattr(node, "destroyed", True) or True

    node._on_timer()

    assert node._adapter.phases == [Phase.STATUS_ALIGN.value]
    assert calls["frame"] == "frame-1"
    assert calls["node"] is node
    assert calls["detector_gateway"] is node._gateway
    assert calls["status_align_step"] is node._status_align
    assert calls["cfg"] is node._runner_cfg
    assert node.destroyed is True
    assert shutdown_calls == [{}]
    assert node._logger.info_messages == [
        (
            "phase sequence stop requested reason=phase_sequence_complete "
            "phase=STATUS_ALIGN algo_status=ALIGNED; stopping node"
        )
    ]


def test_on_timer_publishes_zero_command_when_frame_read_fails():
    class FakeCapture:
        def read(self):
            return False, None

    class FakeLogger:
        def __init__(self) -> None:
            self.warning_messages: list[str] = []

        def warning(self, message: str) -> None:
            self.warning_messages.append(message)

    class FakeCmdPublisher:
        def __init__(self) -> None:
            self.messages: list[dict[str, float]] = []

        def publish(self, message: dict[str, float]) -> None:
            self.messages.append(message)

    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._cfg = make_cfg()
    node._cap = FakeCapture()
    node._cmd_pub = FakeCmdPublisher()
    node._runner_cfg = SimpleNamespace(cmd_topic="/cmd_vel")
    node._capture_read_fallback_attempted = True
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._is_ros_context_ok = lambda: True

    node._on_timer()

    assert node._cmd_pub.messages == [{"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}]
    assert node._logger.warning_messages == [
        "Frame read failed; published zero velocity to /cmd_vel"
    ]


def test_on_timer_retries_numeric_camera_with_v4l2_mjpg_fallback(monkeypatch):
    calls: dict[str, object] = {}

    def fake_run_status_align_once(**kwargs):
        calls.update(kwargs)
        return SimpleNamespace(
            stop_requested=False,
            stop_reason="",
            phase=Phase.STATUS_ALIGN.value,
            algo_status=AlgoStatus.RUNNING.value,
        )

    class InitialCapture:
        released = False

        def read(self):
            return False, None

        def release(self) -> None:
            self.released = True

    class FallbackCapture:
        set_calls: list[tuple[int, object]]

        def __init__(self, source, backend=None) -> None:
            calls["fallback_source"] = source
            calls["fallback_backend"] = backend
            self.set_calls = []

        def isOpened(self) -> bool:
            return True

        def set(self, prop_id: int, value: object) -> None:
            self.set_calls.append((prop_id, value))

        def read(self):
            return True, "fallback-frame"

    class FakeAdapter:
        def __init__(self) -> None:
            self.phases: list[str] = []

        def on_phase(self, phase: str) -> None:
            self.phases.append(phase)

    class FakeCmdPublisher:
        def __init__(self) -> None:
            self.messages: list[dict[str, float]] = []

        def publish(self, message: dict[str, float]) -> None:
            self.messages.append(message)

    class FakeLogger:
        def __init__(self) -> None:
            self.warning_messages: list[str] = []

        def warning(self, message: str) -> None:
            self.warning_messages.append(message)

    fake_cv2 = types.SimpleNamespace(
        CAP_V4L2=200,
        CAP_PROP_FOURCC=6,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        VideoCapture=FallbackCapture,
        VideoWriter_fourcc=lambda *chars: "FOURCC-" + "".join(chars),
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)
    monkeypatch.setattr(ros_node, "run_status_align_once", fake_run_status_align_once)

    initial_capture = InitialCapture()
    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._cfg = make_cfg()
    node._runner_cfg = SimpleNamespace(
        cmd_topic="/cmd_vel",
        phase_sequence=(Phase.STATUS_ALIGN.value,),
    )
    node._cap = initial_capture
    node._capture_read_fallback_attempted = False
    node._cmd_pub = FakeCmdPublisher()
    node._adapter = FakeAdapter()
    node._gateway = object()
    node._status_align = object()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._is_ros_context_ok = lambda: True

    node._on_timer()

    assert initial_capture.released is True
    assert calls["fallback_source"] == 0
    assert calls["fallback_backend"] == fake_cv2.CAP_V4L2
    assert node._cap.set_calls == [
        (fake_cv2.CAP_PROP_FOURCC, "FOURCC-MJPG"),
        (fake_cv2.CAP_PROP_FRAME_WIDTH, 640),
        (fake_cv2.CAP_PROP_FRAME_HEIGHT, 480),
        (fake_cv2.CAP_PROP_FPS, 30),
    ]
    assert calls["frame"] == "fallback-frame"
    assert node._cmd_pub.messages == []
    assert node._logger.warning_messages == [
        "Frame read failed; retrying camera 0 with V4L2 MJPG fallback"
    ]


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

    def fake_build_adapter(*, environment, env_status_publisher, turtle_cmd_publisher=None):
        captured["environment"] = environment
        captured["env_status_publisher"] = env_status_publisher
        captured["turtle_cmd_publisher"] = turtle_cmd_publisher
        return SimpleNamespace(on_phase=lambda phase: None)

    monkeypatch.setattr(fake_ros_node, "build_adapter", fake_build_adapter)

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))
    captured["env_status_publisher"]("READY")

    assert node.env_status_pub._ros_publisher.messages == []
    assert captured["turtle_cmd_publisher"] is None
    assert node.logger.info_messages == [
        (
            "starting pid_alignment_runner environment=robot "
            "phase_sequence=STATUS_ALIGN "
            "start_phase=STATUS_ALIGN input_source=0 status_profile=status_competition "
            "cmd_topic=/cmd_vel selected_status_topic=/robot_fetch/selected_target_px "
            "workflow_phase_topic=/workflow/phase algo_status_topic=/workflow/algo_status "
            "env_status_topic=/workflow/env_status turtle_cmd_topic=None "
            "target_x=320.000 tolerance_px=8.000"
        )
    ]


def test_pid_alignment_ros_node_bridges_runner_cmd_to_turtle_topic(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class FakeCapture:
        def read(self):
            return False, None

        def release(self) -> None:
            return None

    def fake_create_publisher(self, msg_type, topic, qos_depth):
        publisher = FakeRosPublisher(topic)
        publishers[topic] = publisher
        return publisher

    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_publisher",
        fake_create_publisher,
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

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="turtle"))
    node.cmd_pub.publish(
        {
            "linear_x": 0.0,
            "linear_y": 0.18,
            "angular_z": 0.25,
        }
    )

    raw_messages = publishers["/cmd_vel"].messages
    turtle_messages = publishers["/turtle1/cmd_vel"].messages

    assert len(raw_messages) == 1
    assert raw_messages[0].linear.x == 0.0
    assert raw_messages[0].linear.y == 0.18
    assert raw_messages[0].angular.z == 0.25

    assert len(turtle_messages) == 1
    assert turtle_messages[0].linear.x == 0.18
    assert turtle_messages[0].linear.y == 0.0
    assert turtle_messages[0].angular.z == 0.25


def test_parse_args_defaults_config_path_from_worktree_root():
    args = ros_node.parse_args([])

    assert Path(args.config) == (
        ros_node.WORKTREE_ROOT / "configs/workflows/pid_alignment.robot.yaml"
    )


def test_readme_mentions_pid_alignment_runner_startup():
    readme = Path("README.md").read_text(encoding="utf-8")

    assert "python src/runners/pid_alignment_ros_node.py --config" in readme
    assert "python src/terminal_pid_follower_node.py" not in readme
    assert "python src/terminal_pid_follower_node.py --hot-reload" not in readme


def test_main_loads_config_and_spins_node(monkeypatch):
    calls: list[tuple[str, object]] = []
    fake_cfg = make_cfg(environment="robot")
    created_nodes: list[object] = []

    monkeypatch.setattr(
        ros_node.rclpy,
        "init",
        lambda **kwargs: calls.append(("init", kwargs)),
    )
    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda: calls.append(("try_shutdown", None)),
    )
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
        ("init", {"signal_handler_options": ros_node.SignalHandlerOptions.NO}),
        ("load", Path("configs/workflows/pid_alignment.turtle.yaml")),
        ("node_init", fake_cfg),
        ("spin", created_nodes[0]),
        ("destroy", None),
        ("try_shutdown", None),
    ]


def test_main_handles_keyboard_interrupt_without_traceback(monkeypatch):
    calls: list[tuple[str, object]] = []
    fake_cfg = make_cfg(environment="robot")

    monkeypatch.setattr(
        ros_node.rclpy,
        "init",
        lambda **kwargs: calls.append(("init", kwargs)),
    )
    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda: calls.append(("try_shutdown", None)),
    )
    monkeypatch.setattr(
        ros_node.rclpy,
        "spin",
        lambda node: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(
        ros_node,
        "load_pid_alignment_config",
        lambda path: calls.append(("load", Path(path))) or fake_cfg,
    )

    class FakePidAlignmentRosNode:
        def __init__(self, *, cfg):
            calls.append(("node_init", cfg))

        def destroy_node(self):
            calls.append(("destroy", None))
            return True

    monkeypatch.setattr(ros_node, "PidAlignmentRosNode", FakePidAlignmentRosNode)

    ros_node.main(["--config", "configs/workflows/pid_alignment.robot.yaml"])

    assert calls == [
        ("init", {"signal_handler_options": ros_node.SignalHandlerOptions.NO}),
        ("load", Path("configs/workflows/pid_alignment.robot.yaml")),
        ("node_init", fake_cfg),
        ("destroy", None),
        ("try_shutdown", None),
    ]


def test_main_handles_rclpy_context_shutdown_during_interrupt(monkeypatch):
    calls: list[tuple[str, object]] = []
    fake_cfg = make_cfg(environment="robot")

    monkeypatch.setattr(
        ros_node.rclpy,
        "init",
        lambda **kwargs: calls.append(("init", kwargs)),
    )
    monkeypatch.setattr(ros_node.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda: calls.append(("try_shutdown", None)),
    )
    monkeypatch.setattr(
        ros_node,
        "load_pid_alignment_config",
        lambda path: calls.append(("load", Path(path))) or fake_cfg,
    )

    class FakePidAlignmentRosNode:
        def __init__(self, *, cfg):
            calls.append(("node_init", cfg))
            raise ros_node.RCLError(
                "failed to create timer: the given context is not valid"
            )

    monkeypatch.setattr(ros_node, "PidAlignmentRosNode", FakePidAlignmentRosNode)

    ros_node.main(["--config", "configs/workflows/pid_alignment.robot.yaml"])

    assert calls == [
        ("init", {"signal_handler_options": ros_node.SignalHandlerOptions.NO}),
        ("load", Path("configs/workflows/pid_alignment.robot.yaml")),
        ("node_init", fake_cfg),
        ("try_shutdown", None),
    ]
