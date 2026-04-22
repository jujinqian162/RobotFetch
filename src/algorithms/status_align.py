from __future__ import annotations

from dataclasses import dataclass

from algorithms.pid import PIDConfig, PIDController
from algorithms.target_selection import HasLabelAndCx, select_status_target
from workflow.types import AlgoStatus


@dataclass(frozen=True)
class StatusAlignConfig:
    pid: PIDConfig
    target_x: float
    tolerance_px: float
    allowed_labels: set[str]
    stable_labels: set[str]
    use_stable_labels: bool


@dataclass(frozen=True)
class StatusAlignResult:
    status: AlgoStatus
    command_x: float
    selected_target: HasLabelAndCx | None
    aligned: bool


class StatusAlignStep:
    def __init__(self, cfg: StatusAlignConfig) -> None:
        self._cfg = cfg
        self._pid = PIDController(cfg.pid)

    def reset(self) -> None:
        self._pid.reset()

    def run(self, *, targets: list[HasLabelAndCx], now_s: float) -> StatusAlignResult:
        return _run_status_align_step(
            pid=self._pid,
            targets=targets,
            now_s=now_s,
            cfg=self._cfg,
        )


def run_status_align_step(
    *,
    targets: list[HasLabelAndCx],
    now_s: float,
    cfg: StatusAlignConfig,
) -> StatusAlignResult:
    return _run_status_align_step(
        pid=PIDController(cfg.pid),
        targets=targets,
        now_s=now_s,
        cfg=cfg,
    )


def _run_status_align_step(
    *,
    pid: PIDController,
    targets: list[HasLabelAndCx],
    now_s: float,
    cfg: StatusAlignConfig,
) -> StatusAlignResult:
    selected_target = select_status_target(
        targets=targets,
        target_x=cfg.target_x,
        allowed_labels=cfg.allowed_labels,
        stable_labels=cfg.stable_labels,
        use_stable_labels=cfg.use_stable_labels,
    )
    if selected_target is None:
        return StatusAlignResult(
            status=AlgoStatus.TARGET_LOST,
            command_x=0.0,
            selected_target=None,
            aligned=False,
        )

    error = cfg.target_x - selected_target.cx
    if abs(error) <= cfg.tolerance_px:
        return StatusAlignResult(
            status=AlgoStatus.ALIGNED,
            command_x=0.0,
            selected_target=selected_target,
            aligned=True,
        )

    return StatusAlignResult(
        status=AlgoStatus.RUNNING,
        command_x=pid.update(error=error, now_s=now_s),
        selected_target=selected_target,
        aligned=False,
    )
