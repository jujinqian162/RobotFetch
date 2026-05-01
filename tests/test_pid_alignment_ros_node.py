from __future__ import annotations

import importlib
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import runners.pid_alignment_ros_node as ros_node
from config.models import (
    AdapterConfig,
    BaseCoordConfig,
    CameraFallbackConfig,
    CmdVelTransformConfig,
    DetectorConfig,
    ForwardApproachConfig,
    PidAlignmentWorkflowConfig,
    TopicConfig,
)
from workflow.types import AlgoStatus, Phase


def make_cfg(
    *,
    environment: str = "turtle",
    start_phase: str = Phase.STATUS_ALIGN.value,
    phase_sequence: tuple[str, ...] = (Phase.STATUS_ALIGN.value,),
    cmd_topic: str | None = None,
    max_speed: float = 0.25,
    publish_cmd_vel: bool = True,
    cmd_vel_transform: CmdVelTransformConfig | None = None,
    camera_fallbacks: tuple[CameraFallbackConfig, ...] = (
        CameraFallbackConfig("v4l2", "MJPG", 640, 360, 270.0),
        CameraFallbackConfig("v4l2", "MJPG", 800, 600, 190.0),
        CameraFallbackConfig("v4l2", "MJPG", 1024, 768, 190.0),
    ),
) -> PidAlignmentWorkflowConfig:
    resolved_cmd_topic = cmd_topic or (
        "/cmd_vel" if environment == "turtle" else "/t0x0101_robotfetch"
    )
    return PidAlignmentWorkflowConfig(
        environment=environment,
        start_phase=start_phase,
        phase_sequence=phase_sequence,
        target_x=320.0,
        tolerance_px=8.0,
        max_speed=max_speed,
        forward_approach=ForwardApproachConfig(speed_mps=0.12, distance_m=0.3),
        topics=TopicConfig(
            cmd_topic=resolved_cmd_topic,
            publish_cmd_vel=publish_cmd_vel,
            workflow_phase_topic="/workflow/phase",
            algo_status_topic="/workflow/algo_status",
            env_status_topic="/workflow/env_status",
            selected_status_topic="/robot_fetch/selected_target_px",
        ),
        detector=DetectorConfig(
            sdk_config=Path("BaseDetect/configs/basedetect_sdk.yaml"),
            status_profile="status_competition",
            base_coord_profile="base_coord_competition",
            input_source="0",
            camera_fallbacks=camera_fallbacks,
        ),
        base_coord=BaseCoordConfig(
            publish_topic="/robot_fetch/base_coord_targets",
            frame_id="camera_link",
            complete_on_first_target=True,
        ),
        adapter=AdapterConfig(
            turtle_cmd_topic="/turtle1/cmd_vel" if environment == "turtle" else None,
            cmd_vel_transform=cmd_vel_transform or CmdVelTransformConfig(),
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

    class FakeFloat32MultiArray:
        def __init__(self) -> None:
            self.data: list[float] = []

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
    fake_std_msg.Float32MultiArray = FakeFloat32MultiArray
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


def test_build_status_align_step_uses_configured_max_speed():
    step = ros_node.build_status_align_step(make_cfg(max_speed=0.12))

    assert step._cfg.pid.output_limit == 0.12


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


def test_build_capture_log_message_reports_camera_properties(monkeypatch):
    class FakeCapture:
        def get(self, prop_id: int) -> float:
            values = {
                fake_cv2.CAP_PROP_FRAME_WIDTH: 1280.0,
                fake_cv2.CAP_PROP_FRAME_HEIGHT: 720.0,
                fake_cv2.CAP_PROP_FPS: 30.0,
                fake_cv2.CAP_PROP_FRAME_COUNT: -1.0,
                fake_cv2.CAP_PROP_FOURCC: float(
                    ord("M")
                    | (ord("J") << 8)
                    | (ord("P") << 16)
                    | (ord("G") << 24)
                ),
            }
            return values[prop_id]

    fake_cv2 = types.SimpleNamespace(
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        CAP_PROP_FRAME_COUNT=7,
        CAP_PROP_FOURCC=6,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    message = ros_node._build_capture_log_message("0", FakeCapture())

    assert message == (
        "capture opened\n"
        "  source=0\n"
        "  source_type=camera\n"
        "  width=1280\n"
        "  height=720\n"
        "  fps=30.000\n"
        "  frame_count=unknown\n"
        "  fourcc=MJPG"
    )


def test_build_capture_log_message_reports_video_properties(monkeypatch):
    class FakeCapture:
        def get(self, prop_id: int) -> float:
            values = {
                fake_cv2.CAP_PROP_FRAME_WIDTH: 1920.0,
                fake_cv2.CAP_PROP_FRAME_HEIGHT: 1080.0,
                fake_cv2.CAP_PROP_FPS: 59.94,
                fake_cv2.CAP_PROP_FRAME_COUNT: 300.0,
                fake_cv2.CAP_PROP_FOURCC: 0.0,
            }
            return values[prop_id]

    fake_cv2 = types.SimpleNamespace(
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        CAP_PROP_FRAME_COUNT=7,
        CAP_PROP_FOURCC=6,
    )
    monkeypatch.setitem(sys.modules, "cv2", fake_cv2)

    message = ros_node._build_capture_log_message("test-assets/test6.mp4", FakeCapture())

    assert message == (
        "capture opened\n"
        "  source=test-assets/test6.mp4\n"
        "  source_type=video\n"
        "  width=1920\n"
        "  height=1080\n"
        "  fps=59.940\n"
        "  frame_count=300\n"
        "  fourcc=unknown"
    )


def test_on_timer_calls_workflow_engine_tick_and_stops_on_terminal_result(monkeypatch):
    calls: list[str] = []
    shutdown_calls: list[object] = []

    class FakeEngine:
        def tick(self):
            calls.append("tick")
            return SimpleNamespace(
                done=True,
                stop_reason="phase_sequence_complete",
                phase=Phase.STATUS_ALIGN,
                algo_status=AlgoStatus.ALIGNED,
            )

    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda **kwargs: shutdown_calls.append(kwargs),
    )

    class FakeLogger:
        def __init__(self) -> None:
            self.info_messages: list[str] = []

        def info(self, message: str) -> None:
            self.info_messages.append(message)

    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._engine = FakeEngine()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node._is_ros_context_ok = lambda: True
    node.destroyed = False
    node.destroy_node = lambda: setattr(node, "destroyed", True) or True

    node._on_timer()

    assert calls == ["tick"]
    assert node.destroyed is True
    assert shutdown_calls == [{}]
    assert node._logger.info_messages == [
        (
            "phase sequence stop requested\n"
            "  reason=phase_sequence_complete\n"
            "  phase=STATUS_ALIGN\n"
            "  algo_status=ALIGNED\n"
            "  action=stopping_node"
        )
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
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_status_align_step",
        lambda cfg: SimpleNamespace(cfg=cfg),
        raising=False,
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
            "starting pid_alignment_runner\n"
            "  environment=robot\n"
            "  phase_sequence=STATUS_ALIGN\n"
            "  start_phase=STATUS_ALIGN\n"
            "  input_source=0\n"
            "  status_profile=status_competition\n"
            "  base_coord_profile=base_coord_competition\n"
            "  cmd_topic=/t0x0101_robotfetch\n"
            "  selected_status_topic=/robot_fetch/selected_target_px\n"
            "  base_coord_topic=/robot_fetch/base_coord_targets\n"
            "  workflow_phase_topic=/workflow/phase\n"
            "  algo_status_topic=/workflow/algo_status\n"
            "  env_status_topic=/workflow/env_status\n"
            "  turtle_cmd_topic=None\n"
            "  cmd_vel_transform=invert_linear_x=False invert_linear_y=False invert_angular_z=False\n"
            "  target_x=320.000\n"
            "  tolerance_px=8.000\n"
            "  forward_approach_speed_mps=0.120\n"
            "  forward_approach_distance_m=0.300"
        ),
    ]


def test_pid_alignment_ros_node_builds_engine_without_opening_capture_or_detector(
    monkeypatch,
):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    calls: dict[str, int] = {
        "capture": 0,
        "gateway": 0,
        "status_align": 0,
    }

    class FakeRosPublisher:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

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

    def fail_build_capture(input_source):
        calls["capture"] += 1
        raise AssertionError("ROS node startup must not open capture directly")

    def fail_gateway(**kwargs):
        calls["gateway"] += 1
        raise AssertionError("ROS node startup must not build detector gateway directly")

    def fail_status_align(cfg):
        calls["status_align"] += 1
        raise AssertionError("ROS node startup must not build status align directly")

    monkeypatch.setattr(fake_ros_node, "build_capture", fail_build_capture, raising=False)
    monkeypatch.setattr(fake_ros_node, "DetectorGateway", fail_gateway, raising=False)
    monkeypatch.setattr(
        fake_ros_node,
        "build_status_align_step",
        fail_status_align,
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_adapter",
        lambda **kwargs: SimpleNamespace(on_cmd_vel=lambda message: None, on_phase=lambda phase: None),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))

    assert calls == {"capture": 0, "gateway": 0, "status_align": 0}
    assert hasattr(node, "_engine")
    assert node.logger.info_messages == [
        (
            "starting pid_alignment_runner\n"
            "  environment=robot\n"
            "  phase_sequence=STATUS_ALIGN\n"
            "  start_phase=STATUS_ALIGN\n"
            "  input_source=0\n"
            "  status_profile=status_competition\n"
            "  base_coord_profile=base_coord_competition\n"
            "  cmd_topic=/t0x0101_robotfetch\n"
            "  selected_status_topic=/robot_fetch/selected_target_px\n"
            "  base_coord_topic=/robot_fetch/base_coord_targets\n"
            "  workflow_phase_topic=/workflow/phase\n"
            "  algo_status_topic=/workflow/algo_status\n"
            "  env_status_topic=/workflow/env_status\n"
            "  turtle_cmd_topic=None\n"
            "  cmd_vel_transform=invert_linear_x=False invert_linear_y=False invert_angular_z=False\n"
            "  target_x=320.000\n"
            "  tolerance_px=8.000\n"
            "  forward_approach_speed_mps=0.120\n"
            "  forward_approach_distance_m=0.300"
        ),
    ]


def test_pid_alignment_ros_node_publishes_robot_cmd_as_float32_multi_array(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, *, topic: str, msg_type: type[object]) -> None:
            self.topic = topic
            self.msg_type = msg_type
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    def fake_create_publisher(self, msg_type, topic, qos_depth):
        publisher = FakeRosPublisher(topic=topic, msg_type=msg_type)
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
    monkeypatch.setattr(
        fake_ros_node,
        "build_adapter",
        lambda **kwargs: SimpleNamespace(on_cmd_vel=lambda message: None, on_phase=lambda phase: None),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))
    node.cmd_pub.publish(
        {
            "linear_x": 0.1,
            "linear_y": -0.2,
            "angular_z": 0.3,
        }
    )

    publisher = publishers["/t0x0101_robotfetch"]
    assert publisher.msg_type.__name__ == "FakeFloat32MultiArray"
    assert len(publisher.messages) == 1
    assert publisher.messages[0].data == [0.1, -0.2, 0.3]


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
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_status_align_step",
        lambda cfg: SimpleNamespace(cfg=cfg),
        raising=False,
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


def test_pid_alignment_ros_node_applies_cmd_vel_transform_before_turtle_bridge(
    monkeypatch,
):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

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

    cfg = make_cfg(
        environment="turtle",
        cmd_vel_transform=CmdVelTransformConfig(
            invert_linear_x=True,
            invert_linear_y=True,
            invert_angular_z=True,
        ),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=cfg)
    node.cmd_pub.publish(
        {
            "linear_x": 0.1,
            "linear_y": 0.18,
            "angular_z": 0.25,
        }
    )

    raw_messages = publishers["/cmd_vel"].messages
    turtle_messages = publishers["/turtle1/cmd_vel"].messages

    assert len(raw_messages) == 1
    assert raw_messages[0].linear.x == -0.1
    assert raw_messages[0].linear.y == -0.18
    assert raw_messages[0].angular.z == -0.25

    assert len(turtle_messages) == 1
    assert turtle_messages[0].linear.x == -0.18
    assert turtle_messages[0].linear.y == 0.0
    assert turtle_messages[0].angular.z == -0.25


def test_pid_alignment_ros_node_can_disable_workflow_cmd_vel_publish(monkeypatch):
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
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_status_align_step",
        lambda cfg: SimpleNamespace(cfg=cfg),
        raising=False,
    )

    node = fake_ros_node.PidAlignmentRosNode(
        cfg=make_cfg(environment="turtle", publish_cmd_vel=False)
    )
    node.cmd_pub.publish(
        {
            "linear_x": 0.0,
            "linear_y": 0.18,
            "angular_z": 0.25,
        }
    )

    assert "/cmd_vel" not in publishers
    assert len(publishers["/turtle1/cmd_vel"].messages) == 1


def test_pid_alignment_ros_node_applies_cmd_vel_transform_when_workflow_publish_disabled(
    monkeypatch,
):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

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

    cfg = make_cfg(
        environment="turtle",
        publish_cmd_vel=False,
        cmd_vel_transform=CmdVelTransformConfig(
            invert_linear_x=False,
            invert_linear_y=True,
            invert_angular_z=True,
        ),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=cfg)
    node.cmd_pub.publish(
        {
            "linear_x": 0.0,
            "linear_y": 0.18,
            "angular_z": 0.25,
        }
    )

    assert "/cmd_vel" not in publishers
    assert len(publishers["/turtle1/cmd_vel"].messages) == 1
    assert publishers["/turtle1/cmd_vel"].messages[0].linear.x == -0.18
    assert publishers["/turtle1/cmd_vel"].messages[0].angular.z == -0.25


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
