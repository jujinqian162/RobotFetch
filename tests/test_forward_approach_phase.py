from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace

from runners.phases.forward_approach_phase import ForwardApproachPhaseRunner
from workflow.runtime import WorkflowPublishers
from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass
class FakePublisher:
    messages: list[object] = field(default_factory=list)

    def publish(self, message: object) -> None:
        self.messages.append(message)


class VisionForbiddenResources:
    def vision(self):
        raise AssertionError("forward approach must not access vision resources")


class FakeClock:
    def __init__(self, now_s: float) -> None:
        self.now_s = now_s

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.now_s * 1e9))


@dataclass
class FakeLogger:
    info_messages: list[str] = field(default_factory=list)

    def info(self, message: str) -> None:
        self.info_messages.append(message)


def make_context(*, now_s: float = 10.0):
    cfg = SimpleNamespace(
        forward_approach=SimpleNamespace(speed_mps=0.5, distance_m=1.0),
        topics=SimpleNamespace(cmd_topic="/cmd_vel"),
    )
    return SimpleNamespace(
        cfg=cfg,
        resources=VisionForbiddenResources(),
        publishers=WorkflowPublishers(
            cmd_pub=FakePublisher(),
            phase_pub=FakePublisher(),
            algo_status_pub=FakePublisher(),
            env_status_pub=FakePublisher(),
            selected_target_pub=FakePublisher(),
        ),
        logger=FakeLogger(),
        clock=FakeClock(now_s),
        now_s=lambda: now_s,
    )


def test_forward_approach_publishes_forward_command_without_vision_access():
    context = make_context(now_s=12.0)
    phase = ForwardApproachPhaseRunner()

    phase.on_enter(context)
    result = phase.tick(context)

    assert result.phase == Phase.FORWARD_APPROACH
    assert result.algo_status == AlgoStatus.RUNNING
    assert result.env_status == EnvStatus.RUNNING
    assert result.command == {"linear_x": 0.5, "linear_y": 0.0, "angular_z": 0.0}
    assert result.done is False
    assert context.publishers.cmd_pub.messages == [result.command]
    assert context.publishers.phase_pub.messages == [Phase.FORWARD_APPROACH.value]
    assert context.publishers.algo_status_pub.messages == [AlgoStatus.RUNNING.value]
    assert context.publishers.env_status_pub.messages == [EnvStatus.RUNNING.value]
    assert "forward_approach cycle" in context.logger.info_messages[0]
    assert "\n  linear_x=0.500000" in context.logger.info_messages[0]


def test_forward_approach_publishes_zero_command_and_done_after_duration():
    context = make_context(now_s=10.0)
    phase = ForwardApproachPhaseRunner()
    phase.on_enter(context)

    context.now_s = lambda: 12.1
    result = phase.tick(context)

    assert result.phase == Phase.FORWARD_APPROACH
    assert result.algo_status == AlgoStatus.STEP_DONE
    assert result.env_status == EnvStatus.DONE
    assert result.command == {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}
    assert result.done is True
    assert context.publishers.cmd_pub.messages == [result.command]
