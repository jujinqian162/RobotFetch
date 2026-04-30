from dataclasses import dataclass, field
import inspect
from types import SimpleNamespace

from workflow.types import AlgoStatus, EnvStatus, Phase

from runners.pid_alignment_runner import (
    DeduplicatingLogCache,
    PidAlignmentRunnerNode,
    RunnerConfig,
    run_status_align_once,
)


@dataclass
class FakeStatusTarget:
    label: str
    cx: float
    cy: float = 0.0
    conf: float = 0.0
    id: int | None = None
    width: float = 0.0
    height: float = 0.0


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


@dataclass
class FakeLogger:
    info_messages: list[str] = field(default_factory=list)
    warning_messages: list[str] = field(default_factory=list)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)


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
    logger: FakeLogger = field(default_factory=FakeLogger)

    def get_clock(self) -> FakeClock:
        return self.clock

    def get_logger(self) -> FakeLogger:
        return self.logger


DEFAULT_CONFIG = RunnerConfig(
    cmd_topic="/cmd_vel",
    selected_status_topic="/robot_fetch/selected_target_px",
    workflow_phase_topic="/workflow/phase",
    algo_status_topic="/workflow/algo_status",
    env_status_topic="/workflow/env_status",
    frame_id="camera_link",
    phase_sequence=(Phase.STATUS_ALIGN.value,),
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

    cycle = run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=DEFAULT_CONFIG,
    )

    assert cycle.phase == Phase.STATUS_ALIGN.value
    assert cycle.algo_status == AlgoStatus.RUNNING.value
    assert cycle.env_status == EnvStatus.RUNNING.value
    assert cycle.command_x == 0.2
    assert cycle.stop_requested is False
    assert cycle.stop_reason is None
    assert harness.phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert harness.algo_status_pub.messages == [AlgoStatus.RUNNING.value]
    assert harness.env_status_pub.messages == [EnvStatus.RUNNING.value]
    assert harness.cmd_pub.messages == [{"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}]
    assert harness.selected_target_pub.messages == [{"label": "palm", "cx": 300.0, "frame_id": "camera_link"}]
    assert len(harness.logger.info_messages) == 1
    assert "status_align cycle" in harness.logger.info_messages[0]
    assert "\n  detector_ready=True" in harness.logger.info_messages[0]
    assert "\n  target_count=1" in harness.logger.info_messages[0]
    assert "\n  selected_label=palm" in harness.logger.info_messages[0]
    assert "\n  selected_cx=300.000" in harness.logger.info_messages[0]
    assert "\n  error_px=20.000" in harness.logger.info_messages[0]
    assert "\n  cmd_topic=/cmd_vel" in harness.logger.info_messages[0]
    assert "\n  linear_y=0.200000" in harness.logger.info_messages[0]
    assert "\n  algo_status=RUNNING" in harness.logger.info_messages[0]



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

    cycle = run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=DEFAULT_CONFIG,
    )

    assert cycle.phase == Phase.STATUS_ALIGN.value
    assert cycle.algo_status == AlgoStatus.TARGET_LOST.value
    assert cycle.env_status == EnvStatus.READY.value
    assert cycle.command_x == 0.0
    assert cycle.stop_requested is False
    assert cycle.stop_reason is None
    assert harness.phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert harness.algo_status_pub.messages == [AlgoStatus.TARGET_LOST.value]
    assert harness.env_status_pub.messages == [EnvStatus.READY.value]
    assert harness.cmd_pub.messages == [{"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}]
    assert harness.selected_target_pub.messages == []
    assert len(harness.logger.info_messages) == 1
    assert "\n  detector_ready=False" in harness.logger.info_messages[0]
    assert "\n  target_count=0" in harness.logger.info_messages[0]
    assert "\n  selected_label=None" in harness.logger.info_messages[0]
    assert "\n  algo_status=TARGET_LOST" in harness.logger.info_messages[0]


def test_run_status_align_once_logs_error_from_configured_target_x():
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
    cfg = RunnerConfig(
        cmd_topic="/cmd_vel",
        selected_status_topic="/robot_fetch/selected_target_px",
        workflow_phase_topic="/workflow/phase",
        algo_status_topic="/workflow/algo_status",
        env_status_topic="/workflow/env_status",
        frame_id="camera_link",
        phase_sequence=(Phase.STATUS_ALIGN.value,),
        target_x=400.0,
    )

    run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=cfg,
    )

    assert "\n  error_px=100.000" in harness.logger.info_messages[0]


def test_run_status_align_once_logs_frame_shape_and_all_status_targets():
    targets = [
        FakeStatusTarget(label="spearhead", cx=210.0, cy=111.0, conf=0.82, id=4),
        FakeStatusTarget(label="palm", cx=640.0, cy=220.0, conf=0.91, id=5),
    ]
    harness = FakeNodeHarness()
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=True, targets=targets))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.RUNNING,
            command_x=0.2,
            selected_target=targets[1],
            aligned=False,
        )
    )
    frame = SimpleNamespace(shape=(720, 1280, 3))

    run_status_align_once(
        node=harness,
        frame=frame,
        detector_gateway=detector,
        status_align_step=step,
        cfg=DEFAULT_CONFIG,
    )

    log = harness.logger.info_messages[0]
    assert "\n  frame_shape=720x1280x3" in log
    assert "\n  targets=[" in log
    assert "\n    spearhead#4(cx=210.000,cy=111.000,conf=0.820)" in log
    assert "\n    palm#5(cx=640.000,cy=220.000,conf=0.910)" in log
    assert "\n  ]" in log


def test_deduplicating_log_cache_prints_repeated_count_without_full_log():
    stream = SimpleNamespace(
        chunks=[],
        write=lambda chunk: stream.chunks.append(chunk),
        flush=lambda: None,
    )
    cache = DeduplicatingLogCache(stream=stream)
    logger = FakeLogger()

    cache.log(logger=logger, level="info", message="same message")
    cache.log(logger=logger, level="info", message="same message")
    cache.log(logger=logger, level="info", message="same message")

    assert logger.info_messages == ["same message"]
    assert stream.chunks == ["\rsame message (+1)", "\rsame message (+2)"]


def test_deduplicating_log_cache_starts_new_line_before_changed_log():
    stream = SimpleNamespace(
        chunks=[],
        write=lambda chunk: stream.chunks.append(chunk),
        flush=lambda: None,
    )
    cache = DeduplicatingLogCache(stream=stream)
    logger = FakeLogger()

    cache.log(logger=logger, level="info", message="same message")
    cache.log(logger=logger, level="info", message="same message")
    cache.log(logger=logger, level="info", message="different message")

    assert logger.info_messages == ["same message", "different message"]
    assert stream.chunks == ["\rsame message (+1)", "\n"]


def test_deduplicating_log_cache_allows_warning_then_info():
    class RclpyLikeLogger(FakeLogger):
        def __init__(self) -> None:
            super().__init__()
            self.severity_by_caller: dict[tuple[str, int], str] = {}

        def _record_call(self, level: str, message: str) -> None:
            caller = inspect.currentframe().f_back.f_back
            caller_id = (caller.f_code.co_name, caller.f_lineno)
            previous_level = self.severity_by_caller.get(caller_id)
            if previous_level is not None and previous_level != level:
                raise ValueError("Logger severity cannot be changed between calls.")
            self.severity_by_caller[caller_id] = level
            if level == "info":
                self.info_messages.append(message)
            elif level == "warning":
                self.warning_messages.append(message)

        def info(self, message: str) -> None:
            self._record_call("info", message)

        def warning(self, message: str) -> None:
            self._record_call("warning", message)

    logger = RclpyLikeLogger()
    cache = DeduplicatingLogCache(
        stream=SimpleNamespace(write=lambda chunk: None, flush=lambda: None)
    )

    cache.log(logger=logger, level="warning", message="camera retry")
    cache.log(logger=logger, level="info", message="status cycle")

    assert logger.warning_messages == ["camera retry"]
    assert logger.info_messages == ["status cycle"]


def test_log_deduplicated_skips_when_node_context_is_not_ok():
    node = SimpleNamespace(_is_ros_context_ok=lambda: False)
    logger = FakeLogger()

    from runners.pid_alignment_runner import log_deduplicated

    log_deduplicated(
        node=node,
        logger=logger,
        level="info",
        message="status cycle",
    )

    assert logger.info_messages == []
    assert not hasattr(node, "_log_deduplicator")


def test_run_status_align_once_deduplicates_consecutive_cycle_logs():
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
    stream = SimpleNamespace(
        chunks=[],
        write=lambda chunk: stream.chunks.append(chunk),
        flush=lambda: None,
    )
    harness._log_deduplicator = DeduplicatingLogCache(stream=stream)

    for frame in (object(), object(), object()):
        run_status_align_once(
            node=harness,
            frame=frame,
            detector_gateway=detector,
            status_align_step=step,
            cfg=DEFAULT_CONFIG,
        )

    assert len(harness.logger.info_messages) == 1
    assert "status_align cycle" in harness.logger.info_messages[0]
    assert stream.chunks == [
        "\rstatus_align cycle (+1)",
        "\rstatus_align cycle (+2)",
    ]



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

    cycle = node.run_once(frame=object())

    assert cycle.phase == Phase.STATUS_ALIGN.value
    assert cycle.algo_status == AlgoStatus.RUNNING.value
    assert cycle.env_status == EnvStatus.RUNNING.value
    assert cycle.command_x == -0.1
    assert cycle.stop_requested is False
    assert node._phase_pub.messages == [Phase.STATUS_ALIGN.value]
    assert node._algo_status_pub.messages == [AlgoStatus.RUNNING.value]
    assert node._env_status_pub.messages == [EnvStatus.RUNNING.value]


def test_pid_alignment_runner_node_stops_when_status_align_sequence_is_complete():
    node = PidAlignmentRunnerNode.__new__(PidAlignmentRunnerNode)
    node._cmd_pub = FakePublisher()
    node._phase_pub = FakePublisher()
    node._algo_status_pub = FakePublisher()
    node._env_status_pub = FakePublisher()
    node._selected_target_pub = FakePublisher()
    node._frame_id = "camera_link"
    node.get_clock = lambda: FakeClock(4.0)

    target = FakeStatusTarget(label="palm", cx=320.0)
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=True, targets=[target]))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.ALIGNED,
            command_x=0.0,
            selected_target=target,
            aligned=True,
        )
    )

    node._detector_gateway = detector
    node._status_align_step = step
    node._runner_cfg = RunnerConfig(
        cmd_topic="/cmd_vel",
        selected_status_topic="/robot_fetch/selected_target_px",
        workflow_phase_topic="/workflow/phase",
        algo_status_topic="/workflow/algo_status",
        env_status_topic="/workflow/env_status",
        frame_id="camera_link",
        phase_sequence=(Phase.STATUS_ALIGN.value,),
    )
    node._phase_controller = SimpleNamespace(enter_phase=lambda phase: phase)

    cycle = node.run_once(frame=object())

    assert cycle.phase == Phase.STATUS_ALIGN.value
    assert cycle.algo_status == AlgoStatus.ALIGNED.value
    assert cycle.env_status == EnvStatus.RUNNING.value
    assert cycle.command_x == 0.0
    assert cycle.stop_requested is True
    assert cycle.stop_reason == "phase_sequence_complete"


def test_status_align_reports_unimplemented_next_sequence_phase():
    harness = FakeNodeHarness()
    target = FakeStatusTarget(label="palm", cx=320.0)
    detector = FakeDetectorGateway(FakeDetectionBatch(ready=True, targets=[target]))
    step = FakeStatusAlignStep(
        FakeStatusAlignResult(
            status=AlgoStatus.ALIGNED,
            command_x=0.0,
            selected_target=target,
            aligned=True,
        )
    )
    cfg = RunnerConfig(
        cmd_topic="/cmd_vel",
        selected_status_topic="/robot_fetch/selected_target_px",
        workflow_phase_topic="/workflow/phase",
        algo_status_topic="/workflow/algo_status",
        env_status_topic="/workflow/env_status",
        frame_id="camera_link",
        phase_sequence=(Phase.STATUS_ALIGN.value, Phase.FORWARD_APPROACH.value),
    )

    cycle = run_status_align_once(
        node=harness,
        frame=object(),
        detector_gateway=detector,
        status_align_step=step,
        cfg=cfg,
    )

    assert cycle.stop_requested is True
    assert cycle.stop_reason == "next_phase_not_implemented:FORWARD_APPROACH"
