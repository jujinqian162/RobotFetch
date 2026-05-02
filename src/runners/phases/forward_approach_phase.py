from __future__ import annotations

from typing import Any

from runners.phases.heartbeat_stats import filled_heartbeat_frames
from workflow.phase_runner import PhaseTickResult
from workflow.types import AlgoStatus, EnvStatus, Phase


class ForwardApproachPhaseRunner:
    phase = Phase.FORWARD_APPROACH

    def __init__(self) -> None:
        self._started_s: float | None = None

    def on_enter(self, context: Any) -> None:
        self._started_s = context.now_s()

    def tick(self, context: Any) -> PhaseTickResult:
        if self._started_s is None:
            self.on_enter(context)

        speed_mps = float(context.cfg.forward_approach.speed_mps)
        distance_m = float(context.cfg.forward_approach.distance_m)
        duration_s = distance_m / speed_mps
        now_s = context.now_s()
        elapsed_s = max(0.0, now_s - float(self._started_s))
        done = elapsed_s >= duration_s
        command = {
            "linear_x": 0.0 if done else speed_mps,
            "linear_y": 0.0,
            "angular_z": 0.0,
        }
        algo_status = AlgoStatus.STEP_DONE if done else AlgoStatus.RUNNING
        env_status = EnvStatus.DONE if done else EnvStatus.RUNNING

        _publish_cycle_state(
            context=context,
            phase=self.phase,
            algo_status=algo_status,
            env_status=env_status,
            command=command,
        )
        _log_forward_approach_cycle(
            context=context,
            duration_s=duration_s,
            elapsed_s=elapsed_s,
            remaining_s=max(0.0, duration_s - elapsed_s),
            command=command,
            algo_status=algo_status,
            env_status=env_status,
        )

        return PhaseTickResult(
            phase=self.phase,
            algo_status=algo_status,
            env_status=env_status,
            command=command,
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
    command: dict[str, float],
) -> None:
    adapter = getattr(context, "adapter", None)
    on_phase = getattr(adapter, "on_phase", None)
    if callable(on_phase):
        on_phase(phase.value)
    context.publishers.cmd_pub.publish(command)
    context.publishers.phase_pub.publish(phase.value)
    context.publishers.algo_status_pub.publish(algo_status.value)
    context.publishers.env_status_pub.publish(env_status.value)


def _log_forward_approach_cycle(
    *,
    context: Any,
    duration_s: float,
    elapsed_s: float,
    remaining_s: float,
    command: dict[str, float],
    algo_status: AlgoStatus,
    env_status: EnvStatus,
) -> None:
    info = getattr(context.logger, "info", None)
    if not callable(info):
        return
    info(
        "\n".join(
            [
                "forward_approach cycle",
                f"  phase={Phase.FORWARD_APPROACH.value}",
                f"  filled_heartbeat_frames={filled_heartbeat_frames(context)}",
                f"  speed_mps={float(context.cfg.forward_approach.speed_mps):.3f}",
                f"  distance_m={float(context.cfg.forward_approach.distance_m):.3f}",
                f"  duration_s={duration_s:.3f}",
                f"  elapsed_s={elapsed_s:.3f}",
                f"  remaining_s={remaining_s:.3f}",
                f"  cmd_topic={context.cfg.topics.cmd_topic}",
                f"  linear_x={command['linear_x']:.6f}",
                f"  linear_y={command['linear_y']:.6f}",
                f"  angular_z={command['angular_z']:.6f}",
                f"  algo_status={algo_status.value}",
                f"  env_status={env_status.value}",
            ]
        )
    )
