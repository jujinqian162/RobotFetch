from __future__ import annotations

from typing import Any

from algorithms.pid import PIDConfig
from algorithms.status_align import StatusAlignConfig, StatusAlignStep
from runners.phases.heartbeat_stats import filled_heartbeat_frames
from workflow.phase_runner import PhaseTickResult
from workflow.types import AlgoStatus, EnvStatus, Phase


ZERO_CMD_MESSAGE = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


class StatusAlignPhaseRunner:
    phase = Phase.STATUS_ALIGN

    def __init__(self, *, status_align_step: Any | None = None) -> None:
        self._status_align_step = status_align_step

    def on_enter(self, context: Any) -> None:
        context.resources.vision().ensure_profile(context.cfg.detector.status_profile)
        if self._status_align_step is None:
            self._status_align_step = build_status_align_step(context.cfg)

    def tick(self, context: Any) -> PhaseTickResult:
        vision = context.resources.vision()
        frame_result = vision.read_frame()
        if not frame_result.ok:
            context.publishers.cmd_pub.publish(ZERO_CMD_MESSAGE)
            warning = getattr(context.logger, "warning", None)
            if callable(warning):
                warning(
                    "Frame read failed; published zero velocity to "
                    f"{context.cfg.topics.cmd_topic}"
                )
            return PhaseTickResult(
                phase=self.phase,
                algo_status=AlgoStatus.TARGET_LOST,
                env_status=EnvStatus.ERROR,
                command=ZERO_CMD_MESSAGE,
                done=False,
            )

        detection = vision.detect_status_targets(frame_result.frame)
        result = self._status_align_step.run(
            targets=detection.targets,
            now_s=context.now_s(),
        )
        env_status = EnvStatus.RUNNING if detection.ready else EnvStatus.READY
        command = {
            "linear_x": 0.0,
            "linear_y": float(result.command_x),
            "angular_z": 0.0,
        }
        selected_target = _selected_target_payload(context, result.selected_target)

        _publish_cycle_state(
            context=context,
            phase=self.phase,
            algo_status=result.status,
            env_status=env_status,
            command=command,
            selected_target=selected_target,
        )
        _log_status_align_cycle(
            context=context,
            frame=frame_result.frame,
            detection=detection,
            result=result,
            env_status=env_status,
            command=command,
        )
        return PhaseTickResult(
            phase=self.phase,
            algo_status=result.status,
            env_status=env_status,
            command=command,
            selected_target=selected_target,
            done=result.status == AlgoStatus.ALIGNED,
        )

    def on_exit(self, context: Any) -> None:
        return None


def build_status_align_step(cfg: Any) -> StatusAlignStep:
    return StatusAlignStep(
        cfg=StatusAlignConfig(
            pid=PIDConfig(
                kp=0.006,
                ki=0.0,
                kd=0.0008,
                output_limit=cfg.max_speed,
                integral_limit=1000.0,
                deadband=0.0,
                derivative_alpha=0.35,
            ),
            target_x=cfg.target_x,
            tolerance_px=cfg.tolerance_px,
            allowed_labels={"spearhead", "fist", "palm"},
            stable_labels=set(),
            use_stable_labels=True,
        )
    )


def _publish_cycle_state(
    *,
    context: Any,
    phase: Phase,
    algo_status: AlgoStatus,
    env_status: EnvStatus,
    command: dict[str, float],
    selected_target: dict[str, object] | None,
) -> None:
    adapter = getattr(context, "adapter", None)
    on_phase = getattr(adapter, "on_phase", None)
    if callable(on_phase):
        on_phase(phase.value)
    context.publishers.cmd_pub.publish(command)
    context.publishers.phase_pub.publish(phase.value)
    context.publishers.algo_status_pub.publish(algo_status.value)
    context.publishers.env_status_pub.publish(env_status.value)
    if selected_target is not None:
        context.publishers.selected_target_pub.publish(selected_target)


def _selected_target_payload(
    context: Any,
    selected_target: Any | None,
) -> dict[str, object] | None:
    if selected_target is None:
        return None
    return {
        "label": selected_target.label,
        "cx": selected_target.cx,
        "frame_id": getattr(context.cfg, "frame_id", "camera_link"),
    }


def _log_status_align_cycle(
    *,
    context: Any,
    frame: Any,
    detection: Any,
    result: Any,
    env_status: EnvStatus,
    command: dict[str, float],
) -> None:
    info = getattr(context.logger, "info", None)
    if not callable(info):
        return
    selected = result.selected_target
    selected_label = getattr(selected, "label", None) if selected is not None else None
    selected_cx = getattr(selected, "cx", None) if selected is not None else None
    error_px = None if selected_cx is None else context.cfg.target_x - float(selected_cx)
    info(
        "\n".join(
            [
                "status_align cycle",
                f"  phase={Phase.STATUS_ALIGN.value}",
                f"  filled_heartbeat_frames={filled_heartbeat_frames(context)}",
                f"  detector_ready={bool(detection.ready)}",
                f"  frame_shape={_format_frame_shape(frame)}",
                f"  target_count={len(detection.targets)}",
                "  targets=[",
                *_format_status_target_lines(detection.targets),
                "  ]",
                f"  target_x={context.cfg.target_x:.3f}",
                f"  selected_label={selected_label if selected_label is not None else 'None'}",
                f"  selected_cx={_format_optional_float(selected_cx)}",
                f"  error_px={_format_optional_float(error_px)}",
                f"  cmd_topic={context.cfg.topics.cmd_topic}",
                f"  linear_x={command['linear_x']:.6f}",
                f"  linear_y={command['linear_y']:.6f}",
                f"  angular_z={command['angular_z']:.6f}",
                f"  algo_status={result.status.value}",
                f"  env_status={env_status.value}",
                f"  selected_status_topic={context.cfg.topics.selected_status_topic}",
            ]
        )
    )


def _format_optional_float(value: object) -> str:
    if value is None:
        return "None"
    return f"{float(value):.3f}"


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
