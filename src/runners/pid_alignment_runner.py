from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, TextIO

import sys

PROJECT_ROOT = Path(__file__).resolve().parents[5]
WORKTREE_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = WORKTREE_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from algorithms.detector_gateway import DetectorGateway
from algorithms.pid import PIDConfig
from algorithms.status_align import StatusAlignConfig, StatusAlignStep
from workflow.phase_controller import PhaseController
from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass(frozen=True)
class RunnerConfig:
    cmd_topic: str
    selected_status_topic: str
    workflow_phase_topic: str
    algo_status_topic: str
    env_status_topic: str
    frame_id: str
    phase_sequence: tuple[str, ...] = (Phase.STATUS_ALIGN.value,)
    target_x: float = 320.0
    forward_approach_speed_mps: float = 0.1
    forward_approach_distance_m: float = 0.2


@dataclass(frozen=True)
class StatusAlignCycleResult:
    phase: str
    algo_status: str
    env_status: str
    command_x: float
    stop_requested: bool
    stop_reason: str | None
    next_phase: str | None = None


@dataclass
class DeduplicatingLogCache:
    stream: TextIO = field(default_factory=lambda: sys.stderr)
    _last_key: tuple[str, str] | None = None
    _repeat_count: int = 0
    _repeat_line_open: bool = False

    def log(self, *, logger: Any, level: str, message: str) -> None:
        key = (level, message)
        if key == self._last_key:
            self._repeat_count += 1
            self.stream.write(
                f"\r{_repeat_summary(message)} (+{self._repeat_count})"
            )
            self.stream.flush()
            self._repeat_line_open = True
            return

        if self._repeat_line_open:
            self.stream.write("\n")
            self.stream.flush()

        _emit_log(logger=logger, level=level, message=message)
        self._last_key = key
        self._repeat_count = 0
        self._repeat_line_open = False


def _emit_log(*, logger: Any, level: str, message: str) -> None:
    if level == "info":
        logger.info(message)
        return
    if level == "warning":
        logger.warning(message)
        return
    raise ValueError(f"Unsupported log level: {level}")


def stop_reason_after_status_align(
    *, phase_sequence: tuple[str, ...], algo_status: AlgoStatus
) -> str | None:
    if algo_status not in {AlgoStatus.ALIGNED, AlgoStatus.ERROR}:
        return None
    try:
        status_index = phase_sequence.index(Phase.STATUS_ALIGN.value)
    except ValueError:
        return "phase_sequence_missing_status_align"
    next_index = status_index + 1
    if next_index >= len(phase_sequence):
        return "phase_sequence_complete"
    next_phase = phase_sequence[next_index]
    if next_phase == Phase.FORWARD_APPROACH.value:
        return None
    return f"next_phase_not_implemented:{next_phase}"


def next_phase_after_status_align(
    *, phase_sequence: tuple[str, ...], algo_status: AlgoStatus
) -> str | None:
    if algo_status != AlgoStatus.ALIGNED:
        return None
    try:
        status_index = phase_sequence.index(Phase.STATUS_ALIGN.value)
    except ValueError:
        return None
    next_index = status_index + 1
    if next_index >= len(phase_sequence):
        return None
    next_phase = phase_sequence[next_index]
    if next_phase == Phase.FORWARD_APPROACH.value:
        return next_phase
    return None


def run_status_align_once(
    *,
    node: Any,
    frame: Any,
    detector_gateway: Any,
    status_align_step: Any,
    cfg: RunnerConfig,
) -> StatusAlignCycleResult:
    detection = detector_gateway.detect_status_targets(frame)
    now_s = node.get_clock().now().nanoseconds * 1e-9
    result = status_align_step.run(targets=detection.targets, now_s=now_s)
    env_status = EnvStatus.RUNNING.value if detection.ready else EnvStatus.READY.value
    cmd_message = _build_cmd_message(result.command_x)

    node.cmd_pub.publish(cmd_message)
    node.phase_pub.publish(Phase.STATUS_ALIGN.value)
    node.algo_status_pub.publish(result.status.value)
    node.env_status_pub.publish(env_status)
    if result.selected_target is not None:
        node.selected_target_pub.publish(
            {
                "label": result.selected_target.label,
                "cx": result.selected_target.cx,
                "frame_id": cfg.frame_id,
            }
        )
    _log_status_align_cycle(
        node=node,
        frame=frame,
        cfg=cfg,
        detection=detection,
        result=result,
        env_status=env_status,
        cmd_message=cmd_message,
    )
    stop_reason = stop_reason_after_status_align(
        phase_sequence=cfg.phase_sequence,
        algo_status=result.status,
    )
    next_phase = next_phase_after_status_align(
        phase_sequence=cfg.phase_sequence,
        algo_status=result.status,
    )
    return StatusAlignCycleResult(
        phase=Phase.STATUS_ALIGN.value,
        algo_status=result.status.value,
        env_status=env_status,
        command_x=result.command_x,
        stop_requested=stop_reason is not None,
        stop_reason=stop_reason,
        next_phase=next_phase,
    )


def stop_reason_after_forward_approach(
    *, phase_sequence: tuple[str, ...], algo_status: AlgoStatus
) -> str | None:
    if algo_status != AlgoStatus.STEP_DONE:
        return None
    try:
        forward_index = phase_sequence.index(Phase.FORWARD_APPROACH.value)
    except ValueError:
        return "phase_sequence_missing_forward_approach"
    next_index = forward_index + 1
    if next_index >= len(phase_sequence):
        return "phase_sequence_complete"
    return f"next_phase_not_implemented:{phase_sequence[next_index]}"


def run_forward_approach_once(
    *,
    node: Any,
    cfg: RunnerConfig,
) -> StatusAlignCycleResult:
    now_s = node.get_clock().now().nanoseconds * 1e-9
    start_s = getattr(node, "_forward_approach_started_s", None)
    if start_s is None:
        start_s = now_s
        setattr(node, "_forward_approach_started_s", start_s)

    duration_s = cfg.forward_approach_distance_m / cfg.forward_approach_speed_mps
    elapsed_s = max(0.0, now_s - float(start_s))
    done = elapsed_s >= duration_s
    command_x = 0.0 if done else cfg.forward_approach_speed_mps
    cmd_message = {
        "linear_x": command_x,
        "linear_y": 0.0,
        "angular_z": 0.0,
    }
    algo_status = AlgoStatus.STEP_DONE if done else AlgoStatus.RUNNING
    env_status = EnvStatus.DONE if done else EnvStatus.RUNNING

    node.cmd_pub.publish(cmd_message)
    node.phase_pub.publish(Phase.FORWARD_APPROACH.value)
    node.algo_status_pub.publish(algo_status.value)
    node.env_status_pub.publish(env_status.value)
    _log_forward_approach_cycle(
        node=node,
        cfg=cfg,
        duration_s=duration_s,
        elapsed_s=elapsed_s,
        remaining_s=max(0.0, duration_s - elapsed_s),
        cmd_message=cmd_message,
        algo_status=algo_status,
        env_status=env_status,
    )
    stop_reason = stop_reason_after_forward_approach(
        phase_sequence=cfg.phase_sequence,
        algo_status=algo_status,
    )
    return StatusAlignCycleResult(
        phase=Phase.FORWARD_APPROACH.value,
        algo_status=algo_status.value,
        env_status=env_status.value,
        command_x=command_x,
        stop_requested=stop_reason is not None,
        stop_reason=stop_reason,
    )


class PidAlignmentRunnerNode:
    def __init__(
        self,
        *,
        detector_gateway: DetectorGateway,
        status_align_step: StatusAlignStep,
        runner_cfg: RunnerConfig,
    ) -> None:
        self._detector_gateway = detector_gateway
        self._status_align_step = status_align_step
        self._runner_cfg = runner_cfg
        self._phase_controller = PhaseController(start_phase=Phase.STATUS_ALIGN)
        self._active_phase = Phase.STATUS_ALIGN
        self._cmd_pub: Any = None
        self._phase_pub: Any = None
        self._algo_status_pub: Any = None
        self._env_status_pub: Any = None
        self._selected_target_pub: Any = None

    @property
    def cmd_pub(self) -> Any:
        return self._cmd_pub

    @property
    def phase_pub(self) -> Any:
        return self._phase_pub

    @property
    def algo_status_pub(self) -> Any:
        return self._algo_status_pub

    @property
    def env_status_pub(self) -> Any:
        return self._env_status_pub

    @property
    def selected_target_pub(self) -> Any:
        return self._selected_target_pub

    def run_once(self, *, frame: Any) -> StatusAlignCycleResult:
        active_phase = getattr(self, "_active_phase", Phase.STATUS_ALIGN)
        if active_phase == Phase.FORWARD_APPROACH:
            self._phase_controller.enter_phase(Phase.FORWARD_APPROACH)
            return run_forward_approach_once(node=self, cfg=self._runner_cfg)

        self._phase_controller.enter_phase(Phase.STATUS_ALIGN)
        cycle = run_status_align_once(
            node=self,
            frame=frame,
            detector_gateway=self._detector_gateway,
            status_align_step=self._status_align_step,
            cfg=self._runner_cfg,
        )
        if cycle.next_phase == Phase.FORWARD_APPROACH.value:
            self._active_phase = Phase.FORWARD_APPROACH
        return cycle


def _build_cmd_message(command_x: float) -> dict[str, float]:
    return {
        "linear_x": 0.0,
        "linear_y": float(command_x),
        "angular_z": 0.0,
    }


def _log_status_align_cycle(
    *,
    node: Any,
    frame: Any,
    cfg: RunnerConfig,
    detection: Any,
    result: Any,
    env_status: str,
    cmd_message: dict[str, float],
) -> None:
    get_logger = getattr(node, "get_logger", None)
    if not callable(get_logger):
        return
    logger = get_logger()
    info = getattr(logger, "info", None)
    if not callable(info):
        return

    selected = result.selected_target
    selected_label = getattr(selected, "label", None) if selected is not None else None
    selected_cx = getattr(selected, "cx", None) if selected is not None else None
    error_px = None if selected_cx is None else cfg.target_x - float(selected_cx)
    log_deduplicated(
        node=node,
        logger=logger,
        level="info",
        message=_format_status_align_cycle_log(
            frame=frame,
            cfg=cfg,
            detection=detection,
            result=result,
            env_status=env_status,
            cmd_message=cmd_message,
            selected_label=selected_label,
            selected_cx=selected_cx,
            error_px=error_px,
        ),
    )


def _log_forward_approach_cycle(
    *,
    node: Any,
    cfg: RunnerConfig,
    duration_s: float,
    elapsed_s: float,
    remaining_s: float,
    cmd_message: dict[str, float],
    algo_status: AlgoStatus,
    env_status: EnvStatus,
) -> None:
    get_logger = getattr(node, "get_logger", None)
    if not callable(get_logger):
        return
    logger = get_logger()
    info = getattr(logger, "info", None)
    if not callable(info):
        return

    log_deduplicated(
        node=node,
        logger=logger,
        level="info",
        message="\n".join(
            [
                "forward_approach cycle",
                f"  phase={Phase.FORWARD_APPROACH.value}",
                f"  speed_mps={cfg.forward_approach_speed_mps:.3f}",
                f"  distance_m={cfg.forward_approach_distance_m:.3f}",
                f"  duration_s={duration_s:.3f}",
                f"  elapsed_s={elapsed_s:.3f}",
                f"  remaining_s={remaining_s:.3f}",
                f"  cmd_topic={cfg.cmd_topic}",
                f"  linear_x={cmd_message['linear_x']:.6f}",
                f"  linear_y={cmd_message['linear_y']:.6f}",
                f"  angular_z={cmd_message['angular_z']:.6f}",
                f"  algo_status={algo_status.value}",
                f"  env_status={env_status.value}",
            ]
        ),
    )


def log_deduplicated(*, node: Any, logger: Any, level: str, message: str) -> None:
    is_ros_context_ok = getattr(node, "_is_ros_context_ok", None)
    if callable(is_ros_context_ok) and not is_ros_context_ok():
        return

    cache = getattr(node, "_log_deduplicator", None)
    if cache is None:
        cache = DeduplicatingLogCache()
        setattr(node, "_log_deduplicator", cache)
    cache.log(logger=logger, level=level, message=message)


def _format_optional_float(value: object) -> str:
    if value is None:
        return "None"
    return f"{float(value):.3f}"


def _repeat_summary(message: str) -> str:
    first_line = message.splitlines()[0] if message.splitlines() else message
    return first_line or "<empty log>"


def _format_status_align_cycle_log(
    *,
    frame: Any,
    cfg: RunnerConfig,
    detection: Any,
    result: Any,
    env_status: str,
    cmd_message: dict[str, float],
    selected_label: object,
    selected_cx: object,
    error_px: object,
) -> str:
    return "\n".join(
        [
            "status_align cycle",
            f"  phase={Phase.STATUS_ALIGN.value}",
            f"  detector_ready={bool(detection.ready)}",
            f"  frame_shape={_format_frame_shape(frame)}",
            f"  target_count={len(detection.targets)}",
            "  targets=[",
            *_format_status_target_lines(detection.targets),
            "  ]",
            f"  target_x={cfg.target_x:.3f}",
            f"  selected_label={selected_label if selected_label is not None else 'None'}",
            f"  selected_cx={_format_optional_float(selected_cx)}",
            f"  error_px={_format_optional_float(error_px)}",
            f"  cmd_topic={cfg.cmd_topic}",
            f"  linear_x={cmd_message['linear_x']:.6f}",
            f"  linear_y={cmd_message['linear_y']:.6f}",
            f"  angular_z={cmd_message['angular_z']:.6f}",
            f"  algo_status={result.status.value}",
            f"  env_status={env_status}",
            f"  selected_status_topic={cfg.selected_status_topic}",
        ]
    )


def _format_frame_shape(frame: Any) -> str:
    shape = getattr(frame, "shape", None)
    if shape is None:
        return "unknown"
    try:
        return "x".join(str(int(value)) for value in shape)
    except (TypeError, ValueError):
        return str(shape)


def _format_status_target_lines(targets: list[Any]) -> list[str]:
    return [f"    {_format_status_target(target)}" for target in targets]


def _format_status_target(target: Any) -> str:
    label = getattr(target, "label", "unknown")
    target_id = getattr(target, "id", None)
    id_suffix = "" if target_id is None else f"#{target_id}"
    return (
        f"{label}{id_suffix}("
        f"cx={_format_optional_float(getattr(target, 'cx', None))},"
        f"cy={_format_optional_float(getattr(target, 'cy', None))},"
        f"conf={_format_optional_float(getattr(target, 'conf', None))}"
        ")"
    )


def build_default_status_align_step() -> StatusAlignStep:
    return StatusAlignStep(
        cfg=StatusAlignConfig(
            pid=PIDConfig(
                kp=0.006,
                ki=0.0,
                kd=0.0008,
                output_limit=0.25,
                integral_limit=1000.0,
                deadband=0.0,
                derivative_alpha=0.35,
            ),
            target_x=320.0,
            tolerance_px=8.0,
            allowed_labels={"spearhead", "fist", "palm"},
            stable_labels=set(),
            use_stable_labels=True,
        )
    )


def build_default_runner_config() -> RunnerConfig:
    return RunnerConfig(
        cmd_topic="/cmd_vel",
        selected_status_topic="/robot_fetch/selected_target_px",
        workflow_phase_topic="/workflow/phase",
        algo_status_topic="/workflow/algo_status",
        env_status_topic="/workflow/env_status",
        frame_id="camera_link",
        phase_sequence=(Phase.STATUS_ALIGN.value,),
    )
