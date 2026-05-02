from __future__ import annotations

from typing import Any

from runners.phases.heartbeat_stats import filled_heartbeat_frames
from workflow.phase_runner import PhaseTickResult
from workflow.types import AlgoStatus, EnvStatus, Phase


ZERO_CMD_MESSAGE = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


class BaseCoordPhaseRunner:
    phase = Phase.BASE_COORD

    def on_enter(self, context: Any) -> None:
        context.resources.vision().ensure_profile(context.cfg.detector.base_coord_profile)

    def tick(self, context: Any) -> PhaseTickResult:
        vision = context.resources.vision()
        frame_result = vision.read_frame()
        context.publishers.cmd_pub.publish(ZERO_CMD_MESSAGE)
        if not frame_result.ok:
            warning = getattr(context.logger, "warning", None)
            if callable(warning):
                warning(
                    "Frame read failed during base_coord; published zero velocity to "
                    f"{context.cfg.topics.cmd_topic}"
                )
            return PhaseTickResult(
                phase=self.phase,
                algo_status=AlgoStatus.TARGET_LOST,
                env_status=EnvStatus.ERROR,
                command=ZERO_CMD_MESSAGE,
                base_coord_targets=[],
                done=False,
            )

        detection = vision.detect_base_coord_targets(frame_result.frame)
        targets = [_base_coord_target_payload(context, target) for target in detection.targets]
        for target in targets:
            base_coord_pub = context.publishers.base_coord_pub
            if base_coord_pub is not None:
                base_coord_pub.publish(target)

        complete_on_target = (
            bool(targets)
            and detection.ready
            and context.cfg.base_coord.complete_on_first_target
        )
        algo_status = (
            AlgoStatus.STEP_DONE
            if complete_on_target
            else AlgoStatus.RUNNING
            if targets
            else AlgoStatus.TARGET_LOST
        )
        env_status = EnvStatus.RUNNING if detection.ready else EnvStatus.READY
        done = algo_status == AlgoStatus.STEP_DONE
        _publish_cycle_state(
            context=context,
            phase=self.phase,
            algo_status=algo_status,
            env_status=env_status,
        )
        _log_base_coord_cycle(
            context=context,
            detection=detection,
            targets=targets,
            algo_status=algo_status,
            env_status=env_status,
        )
        return PhaseTickResult(
            phase=self.phase,
            algo_status=algo_status,
            env_status=env_status,
            command=ZERO_CMD_MESSAGE,
            base_coord_targets=targets,
            done=done,
        )

    def on_exit(self, context: Any) -> None:
        return None


def _publish_cycle_state(
    *,
    context: Any,
    phase: Phase,
    algo_status: AlgoStatus,
    env_status: EnvStatus,
) -> None:
    adapter = getattr(context, "adapter", None)
    on_phase = getattr(adapter, "on_phase", None)
    if callable(on_phase):
        on_phase(phase.value)
    context.publishers.phase_pub.publish(phase.value)
    context.publishers.algo_status_pub.publish(algo_status.value)
    context.publishers.env_status_pub.publish(env_status.value)


def _base_coord_target_payload(context: Any, target: Any) -> dict[str, object]:
    return {
        "label": target.label,
        "id": getattr(target, "id", None),
        "cx": float(getattr(target, "cx", 0.0)),
        "x": float(target.x),
        "y": float(target.y),
        "z": float(target.z),
        "frame_id": context.cfg.base_coord.frame_id,
    }


def _log_base_coord_cycle(
    *,
    context: Any,
    detection: Any,
    targets: list[dict[str, object]],
    algo_status: AlgoStatus,
    env_status: EnvStatus,
) -> None:
    info = getattr(context.logger, "info", None)
    if not callable(info):
        return
    info(
        "\n".join(
            [
                "base_coord cycle",
                f"  phase={Phase.BASE_COORD.value}",
                f"  filled_heartbeat_frames={filled_heartbeat_frames(context)}",
                f"  detector_ready={bool(detection.ready)}",
                f"  target_count={len(targets)}",
                f"  targets={_format_base_coord_targets(targets)}",
                f"  selected={_format_selected_base_coord_target(targets)}",
                f"  publish_topic={context.cfg.base_coord.publish_topic}",
                f"  frame_id={context.cfg.base_coord.frame_id}",
                f"  algo_status={algo_status.value}",
                f"  env_status={env_status.value}",
            ]
        )
    )


def _format_base_coord_targets(targets: list[dict[str, object]]) -> str:
    return "[" + ", ".join(
        _format_base_coord_target(index=index, target=target)
        for index, target in enumerate(targets, start=1)
    ) + "]"


def _format_base_coord_target(*, index: int, target: dict[str, object]) -> str:
    return (
        f"#{index}("
        f"x={_format_coord_value(target.get('x'))},"
        f"y={_format_coord_value(target.get('y'))},"
        f"z={_format_coord_value(target.get('z'))}"
        ")"
    )


def _format_selected_base_coord_target(targets: list[dict[str, object]]) -> str:
    if not targets:
        return "None"
    return "#1"


def _format_coord_value(value: object) -> str:
    if value is None:
        return "None"
    return f"{float(value):.3f}"
