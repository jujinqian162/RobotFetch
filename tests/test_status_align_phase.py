from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace

from runners.phases.status_align_phase import StatusAlignPhaseRunner
from workflow.runtime import FrameReadResult, WorkflowPublishers
from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass
class FakeStatusTarget:
    label: str
    cx: float
    cy: float = 0.0
    conf: float = 0.0
    id: int | None = None


@dataclass
class FakeDetectionBatch:
    ready: bool
    targets: list[FakeStatusTarget]


@dataclass
class FakeStatusAlignResult:
    status: AlgoStatus
    command_x: float
    selected_target: FakeStatusTarget | None
    aligned: bool


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
        self.detect_status_calls: list[object] = []

    def ensure_profile(self, profile: str) -> None:
        self.ensure_profile_calls.append(profile)

    def read_frame(self) -> FrameReadResult:
        self.read_frame_calls += 1
        return FrameReadResult(ok=True, frame=self.frame)

    def detect_status_targets(self, frame: object) -> FakeDetectionBatch:
        self.detect_status_calls.append(frame)
        return self.detection


class FakeResources:
    def __init__(self, vision: FakeVisionSession) -> None:
        self.vision_calls = 0
        self._vision = vision

    def vision(self) -> FakeVisionSession:
        self.vision_calls += 1
        return self._vision


class FakeStatusAlignStep:
    def __init__(self, result: FakeStatusAlignResult) -> None:
        self.result = result
        self.calls: list[tuple[list[FakeStatusTarget], float]] = []

    def run(self, *, targets: list[FakeStatusTarget], now_s: float) -> FakeStatusAlignResult:
        self.calls.append((targets, now_s))
        return self.result


@dataclass
class FakeLogger:
    info_messages: list[str] = field(default_factory=list)
    warning_messages: list[str] = field(default_factory=list)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


def make_context(*, vision: FakeVisionSession, filled_heartbeat_frames: int = 0):
    heartbeat_stats = SimpleNamespace(filled_heartbeat_frames=filled_heartbeat_frames)
    return SimpleNamespace(
        cfg=SimpleNamespace(
            detector=SimpleNamespace(status_profile="status_competition"),
            topics=SimpleNamespace(
                cmd_topic="/cmd_vel",
                selected_status_topic="/robot_fetch/selected_target_px",
            ),
            target_x=320.0,
            frame_id="camera_link",
        ),
        resources=FakeResources(vision),
        publishers=WorkflowPublishers(
            cmd_pub=FakePublisher(),
            phase_pub=FakePublisher(),
            algo_status_pub=FakePublisher(),
            env_status_pub=FakePublisher(),
            selected_target_pub=FakePublisher(),
        ),
        logger=FakeLogger(),
        now_s=lambda: 21.5,
        consume_heartbeat_stats=lambda: heartbeat_stats,
    )


def test_status_align_uses_vision_session_and_status_profile():
    target = FakeStatusTarget(label="palm", cx=300.0)
    frame = SimpleNamespace(shape=(480, 640, 3))
    vision = FakeVisionSession(
        frame=frame,
        detection=FakeDetectionBatch(ready=True, targets=[target]),
    )
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.RUNNING,
            command_x=0.18,
            selected_target=target,
            aligned=False,
        )
    )
    context = make_context(vision=vision, filled_heartbeat_frames=3)
    phase = StatusAlignPhaseRunner(status_align_step=step)

    phase.on_enter(context)
    result = phase.tick(context)

    assert context.resources.vision_calls == 2
    assert vision.ensure_profile_calls == ["status_competition"]
    assert vision.read_frame_calls == 1
    assert vision.detect_status_calls == [frame]
    assert step.calls == [([target], 21.5)]
    assert result.phase == Phase.STATUS_ALIGN
    assert result.algo_status == AlgoStatus.RUNNING
    assert result.env_status == EnvStatus.RUNNING
    assert result.command == {"linear_x": 0.0, "linear_y": 0.18, "angular_z": 0.0}
    assert result.done is False
    assert context.publishers.cmd_pub.messages == [result.command]
    assert context.publishers.selected_target_pub.messages == [
        {"label": "palm", "cx": 300.0, "frame_id": "camera_link"}
    ]
    assert "\n  filled_heartbeat_frames=3" in context.logger.info_messages[-1]


def test_status_align_returns_done_when_step_reports_aligned():
    target = FakeStatusTarget(label="palm", cx=320.0)
    vision = FakeVisionSession(
        frame=object(),
        detection=FakeDetectionBatch(ready=True, targets=[target]),
    )
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.ALIGNED,
            command_x=0.0,
            selected_target=target,
            aligned=True,
        )
    )
    phase = StatusAlignPhaseRunner(status_align_step=step)
    context = make_context(vision=vision)

    phase.on_enter(context)
    result = phase.tick(context)

    assert result.phase == Phase.STATUS_ALIGN
    assert result.algo_status == AlgoStatus.ALIGNED
    assert result.done is True
