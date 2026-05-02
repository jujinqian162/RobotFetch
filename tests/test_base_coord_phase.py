from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace

from runners.phases.base_coord_phase import BaseCoordPhaseRunner
from workflow.runtime import FrameReadResult, WorkflowPublishers
from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass
class FakeBaseCoordTarget:
    label: str
    cx: float
    x: float
    y: float
    z: float
    cy: float = 0.0
    conf: float = 0.0
    id: int | None = None


@dataclass
class FakeDetectionBatch:
    ready: bool
    targets: list[FakeBaseCoordTarget]


@dataclass
class FakePublisher:
    messages: list[object] = field(default_factory=list)

    def publish(self, message: object) -> None:
        self.messages.append(message)


class FakeVisionSession:
    def __init__(self, *, frame: object, detection: FakeDetectionBatch) -> None:
        self.frame = frame
        self.detection = detection
        self.ensure_profile_calls: list[str] = []
        self.read_frame_calls = 0
        self.detect_base_coord_calls: list[object] = []

    def ensure_profile(self, profile: str) -> None:
        self.ensure_profile_calls.append(profile)

    def read_frame(self) -> FrameReadResult:
        self.read_frame_calls += 1
        return FrameReadResult(ok=True, frame=self.frame)

    def detect_base_coord_targets(self, frame: object) -> FakeDetectionBatch:
        self.detect_base_coord_calls.append(frame)
        return self.detection


class FakeResources:
    def __init__(self, vision: FakeVisionSession) -> None:
        self.vision_calls = 0
        self._vision = vision

    def vision(self) -> FakeVisionSession:
        self.vision_calls += 1
        return self._vision


@dataclass
class FakeLogger:
    info_messages: list[str] = field(default_factory=list)
    warning_messages: list[str] = field(default_factory=list)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


def make_context(
    *,
    vision: FakeVisionSession,
    complete_on_first_target: bool = True,
    filled_heartbeat_frames: int = 0,
):
    heartbeat_stats = SimpleNamespace(filled_heartbeat_frames=filled_heartbeat_frames)
    return SimpleNamespace(
        cfg=SimpleNamespace(
            detector=SimpleNamespace(base_coord_profile="base_coord_competition"),
            base_coord=SimpleNamespace(
                publish_topic="/robot_fetch/base_coord_targets",
                frame_id="camera_link",
                complete_on_first_target=complete_on_first_target,
            ),
            topics=SimpleNamespace(cmd_topic="/cmd_vel"),
        ),
        resources=FakeResources(vision),
        publishers=WorkflowPublishers(
            cmd_pub=FakePublisher(),
            phase_pub=FakePublisher(),
            algo_status_pub=FakePublisher(),
            env_status_pub=FakePublisher(),
            selected_target_pub=FakePublisher(),
            base_coord_pub=FakePublisher(),
        ),
        logger=FakeLogger(),
        now_s=lambda: 30.0,
        consume_heartbeat_stats=lambda: heartbeat_stats,
    )


def test_base_coord_switches_profile_reads_frame_and_publishes_targets():
    target = FakeBaseCoordTarget(
        label="base",
        cx=315.0,
        x=0.12,
        y=0.34,
        z=0.56,
        conf=0.91,
        id=7,
    )
    frame = object()
    vision = FakeVisionSession(
        frame=frame,
        detection=FakeDetectionBatch(ready=True, targets=[target]),
    )
    context = make_context(vision=vision)
    phase = BaseCoordPhaseRunner()

    phase.on_enter(context)
    result = phase.tick(context)

    assert context.resources.vision_calls == 2
    assert vision.ensure_profile_calls == ["base_coord_competition"]
    assert vision.read_frame_calls == 1
    assert vision.detect_base_coord_calls == [frame]
    assert result.phase == Phase.BASE_COORD
    assert result.algo_status == AlgoStatus.STEP_DONE
    assert result.env_status == EnvStatus.RUNNING
    assert result.done is True
    assert result.base_coord_targets == [
        {
            "label": "base",
            "id": 7,
            "cx": 315.0,
            "x": 0.12,
            "y": 0.34,
            "z": 0.56,
            "frame_id": "camera_link",
        }
    ]
    assert context.publishers.base_coord_pub.messages == result.base_coord_targets


def test_base_coord_log_includes_target_coordinates_and_selected_target():
    first = FakeBaseCoordTarget(
        label="base",
        cx=315.0,
        x=0.12,
        y=0.34,
        z=0.56,
        id=7,
    )
    second = FakeBaseCoordTarget(
        label="base",
        cx=410.0,
        x=1.23,
        y=2.34,
        z=3.45,
        id=8,
    )
    vision = FakeVisionSession(
        frame=object(),
        detection=FakeDetectionBatch(ready=True, targets=[first, second]),
    )
    context = make_context(vision=vision, filled_heartbeat_frames=14)
    phase = BaseCoordPhaseRunner()

    phase.on_enter(context)
    phase.tick(context)

    log_message = context.logger.info_messages[-1]
    assert "  filled_heartbeat_frames=14" in log_message
    assert "  targets=[#1(x=0.120,y=0.340,z=0.560), #2(x=1.230,y=2.340,z=3.450)]" in log_message
    assert "  selected=#1" in log_message


def test_base_coord_keeps_running_without_targets():
    vision = FakeVisionSession(
        frame=object(),
        detection=FakeDetectionBatch(ready=True, targets=[]),
    )
    context = make_context(vision=vision)
    phase = BaseCoordPhaseRunner()

    phase.on_enter(context)
    result = phase.tick(context)

    assert result.algo_status == AlgoStatus.TARGET_LOST
    assert result.done is False
    assert context.publishers.base_coord_pub.messages == []
    assert "  targets=[]" in context.logger.info_messages[-1]
    assert "  selected=None" in context.logger.info_messages[-1]


def test_base_coord_waits_for_detector_ready_before_complete_on_first_target():
    target = FakeBaseCoordTarget(
        label="base",
        cx=315.0,
        x=0.12,
        y=0.34,
        z=0.56,
        conf=0.91,
        id=7,
    )
    vision = FakeVisionSession(
        frame=object(),
        detection=FakeDetectionBatch(ready=False, targets=[target]),
    )
    context = make_context(vision=vision, complete_on_first_target=True)
    phase = BaseCoordPhaseRunner()

    phase.on_enter(context)
    result = phase.tick(context)

    assert result.algo_status == AlgoStatus.RUNNING
    assert result.env_status == EnvStatus.READY
    assert result.done is False
    assert context.publishers.base_coord_pub.messages == result.base_coord_targets
