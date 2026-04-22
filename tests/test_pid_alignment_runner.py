from dataclasses import dataclass, field
from types import SimpleNamespace

from workflow.types import AlgoStatus, EnvStatus, Phase

from runners.pid_alignment_runner import PidAlignmentRunnerNode, RunnerConfig, run_status_align_once


@dataclass
class FakeStatusTarget:
    label: str
    cx: float


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


class FakeDetectorGateway:
    def __init__(self, batch: FakeDetectionBatch) -> None:
        self.batch = batch
        self.frames: list[object] = []

    def detect_status_targets(self, frame: object) -> FakeDetectionBatch:
        self.frames.append(frame)
        return self.batch


class FakeStatusAlignStep:
    def __init__(self, result: FakeStatusAlignResult) -> None:
        self.result = result
        self.calls: list[tuple[list[FakeStatusTarget], float]] = []

    def run(self, *, targets: list[FakeStatusTarget], now_s: float) -> FakeStatusAlignResult:
        self.calls.append((targets, now_s))
        return self.result


class FakeClock:
    def __init__(self, now_s: float) -> None:
        self._now_s = now_s

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(nanoseconds=int(self._now_s * 1e9))


@dataclass
class FakeNodeHarness:
    cmd_pub: FakePublisher = field(default_factory=FakePublisher)
    phase_pub: FakePublisher = field(default_factory=FakePublisher)
    algo_status_pub: FakePublisher = field(default_factory=FakePublisher)
    env_status_pub: FakePublisher = field(default_factory=FakePublisher)
    selected_target_pub: FakePublisher = field(default_factory=FakePublisher)
    clock: FakeClock = field(default_factory=lambda: FakeClock(12.5))

    def get_clock(self) -> FakeClock:
        return self.clock


DEFAULT_CONFIG = RunnerConfig(
    cmd_topic="/cmd_vel",
    selected_status_topic="/robot_fetch/selected_target_px",
    workflow_phase_topic="/workflow/phase",
    algo_status_topic="/workflow/algo_status",
    env_status_topic="/workflow/env_status",
    frame_id="camera_link",
)



def test_run_status_align_once_publishes_running_command_and_statuses():
    target = FakeStatusTarget(label="palm", cx=300.0)
    harness = FakeNodeHarness()
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=True, targets=[target]))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.RUNNING,
            command_x=0.2,
            selected_target=target,
            aligned=False,
        )
    )

    run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=DEFAULT_CONFIG,
    )

    assert harness.phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert harness.algo_status_pub.messages == [AlgoStatus.RUNNING.value]
    assert harness.env_status_pub.messages == [EnvStatus.RUNNING.value]
    assert harness.cmd_pub.messages == [{"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}]
    assert harness.selected_target_pub.messages == [{"label": "palm", "cx": 300.0, "frame_id": "camera_link"}]



def test_run_status_align_once_publishes_ready_env_status_when_detector_not_ready():
    harness = FakeNodeHarness()
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=False, targets=[]))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.TARGET_LOST,
            command_x=0.0,
            selected_target=None,
            aligned=False,
        )
    )

    run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=DEFAULT_CONFIG,
    )

    assert harness.phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert harness.algo_status_pub.messages == [AlgoStatus.TARGET_LOST.value]
    assert harness.env_status_pub.messages == [EnvStatus.READY.value]
    assert harness.cmd_pub.messages == [{"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}]
    assert harness.selected_target_pub.messages == []



def test_pid_alignment_runner_node_reports_running_publishers():
    node = PidAlignmentRunnerNode.__new__(PidAlignmentRunnerNode)
    node._cmd_pub = FakePublisher()
    node._phase_pub = FakePublisher()
    node._algo_status_pub = FakePublisher()
    node._env_status_pub = FakePublisher()
    node._selected_target_pub = FakePublisher()
    node._frame_id = "camera_link"
    node.get_clock = lambda: FakeClock(3.0)

    target = FakeStatusTarget(label="palm", cx=315.0)
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=True, targets=[target]))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.RUNNING,
            command_x=-0.1,
            selected_target=target,
            aligned=False,
        )
    )

    node._detector_gateway = detector
    node._status_align_step = step
    node._runner_cfg = DEFAULT_CONFIG
    node._phase_controller = SimpleNamespace(enter_phase=lambda phase: phase)

    node.run_once(frame=object())

    assert node._phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert node._algo_status_pub.messages == [AlgoStatus.RUNNING.value]
    assert node._env_status_pub.messages == [EnvStatus.RUNNING.value]
