from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

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
    one_shot: bool = False
    target_x: float = 320.0


@dataclass(frozen=True)
class StatusAlignCycleResult:
    phase: str
    algo_status: str
    env_status: str
    command_x: float
    stop_requested: bool


def should_stop_after_status_align(*, one_shot: bool, algo_status: AlgoStatus) -> bool:
    if not one_shot:
        return False
    return algo_status in {AlgoStatus.ALIGNED, AlgoStatus.TARGET_LOST, AlgoStatus.ERROR}


def run_status_align_once(
    *,
    node: Any,
    frame: Any,
    detector_gateway: Any,
    status_align_step: Any,
    cfg: RunnerConfig,
    one_shot: bool = False,
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
        cfg=cfg,
        detection=detection,
        result=result,
        env_status=env_status,
        cmd_message=cmd_message,
    )
    return StatusAlignCycleResult(
        phase=Phase.STATUS_ALIGN.value,
        algo_status=result.status.value,
        env_status=env_status,
        command_x=result.command_x,
        stop_requested=should_stop_after_status_align(
            one_shot=one_shot,
            algo_status=result.status,
        ),
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
        self._phase_controller.enter_phase(Phase.STATUS_ALIGN)
        return run_status_align_once(
            node=self,
            frame=frame,
            detector_gateway=self._detector_gateway,
            status_align_step=self._status_align_step,
            cfg=self._runner_cfg,
            one_shot=self._runner_cfg.one_shot,
        )


def _build_cmd_message(command_x: float) -> dict[str, float]:
    return {
        "linear_x": 0.0,
        "linear_y": float(command_x),
        "angular_z": 0.0,
    }


def _log_status_align_cycle(
    *,
    node: Any,
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
    info(
        "status_align cycle "
        f"phase={Phase.STATUS_ALIGN.value} "
        f"detector_ready={bool(detection.ready)} "
        f"target_count={len(detection.targets)} "
        f"selected_label={selected_label if selected_label is not None else 'None'} "
        f"selected_cx={_format_optional_float(selected_cx)} "
        f"error_px={_format_optional_float(error_px)} "
        f"cmd_topic={cfg.cmd_topic} "
        f"linear_x={cmd_message['linear_x']:.6f} "
        f"linear_y={cmd_message['linear_y']:.6f} "
        f"angular_z={cmd_message['angular_z']:.6f} "
        f"algo_status={result.status.value} "
        f"env_status={env_status} "
        f"selected_status_topic={cfg.selected_status_topic}"
    )


def _format_optional_float(value: object) -> str:
    if value is None:
        return "None"
    return f"{float(value):.3f}"


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
        one_shot=False,
    )
