from __future__ import annotations

from dataclasses import dataclass, field

from workflow.engine import WorkflowEngine
from workflow.phase_runner import PhaseTickResult
from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass
class FakePhaseRunner:
    phase: Phase
    done_after_ticks: int
    enter_calls: int = 0
    tick_calls: int = 0
    exit_calls: int = 0
    contexts: list[object] = field(default_factory=list)

    def on_enter(self, context: object) -> None:
        self.enter_calls += 1
        self.contexts.append(context)

    def tick(self, context: object) -> PhaseTickResult:
        self.tick_calls += 1
        self.contexts.append(context)
        return PhaseTickResult(
            phase=self.phase,
            algo_status=AlgoStatus.STEP_DONE
            if self.tick_calls >= self.done_after_ticks
            else AlgoStatus.RUNNING,
            env_status=EnvStatus.DONE
            if self.tick_calls >= self.done_after_ticks
            else EnvStatus.RUNNING,
            done=self.tick_calls >= self.done_after_ticks,
        )

    def on_exit(self, context: object) -> None:
        self.exit_calls += 1
        self.contexts.append(context)


def test_engine_enters_first_phase_once_and_ticks_until_done():
    context = object()
    status = FakePhaseRunner(Phase.STATUS_ALIGN, done_after_ticks=2)
    forward = FakePhaseRunner(Phase.FORWARD_APPROACH, done_after_ticks=1)
    engine = WorkflowEngine(
        phase_sequence=(Phase.STATUS_ALIGN, Phase.FORWARD_APPROACH),
        runners={
            Phase.STATUS_ALIGN: status,
            Phase.FORWARD_APPROACH: forward,
        },
        context=context,
    )

    first = engine.tick()
    second = engine.tick()

    assert first.phase == Phase.STATUS_ALIGN
    assert first.done is False
    assert second.phase == Phase.STATUS_ALIGN
    assert second.done is True
    assert status.enter_calls == 1
    assert status.tick_calls == 2
    assert status.exit_calls == 1
    assert forward.enter_calls == 0
    assert set(status.contexts) == {context}


def test_engine_advances_to_next_phase_after_done_result():
    context = object()
    status = FakePhaseRunner(Phase.STATUS_ALIGN, done_after_ticks=1)
    forward = FakePhaseRunner(Phase.FORWARD_APPROACH, done_after_ticks=1)
    engine = WorkflowEngine(
        phase_sequence=(Phase.STATUS_ALIGN, Phase.FORWARD_APPROACH),
        runners={
            Phase.STATUS_ALIGN: status,
            Phase.FORWARD_APPROACH: forward,
        },
        context=context,
    )

    status_result = engine.tick()
    forward_result = engine.tick()

    assert status_result.phase == Phase.STATUS_ALIGN
    assert status_result.stop_reason is None
    assert forward_result.phase == Phase.FORWARD_APPROACH
    assert forward.enter_calls == 1
    assert forward.tick_calls == 1
    assert forward.exit_calls == 1


def test_engine_returns_phase_sequence_complete_after_last_phase():
    context = object()
    forward = FakePhaseRunner(Phase.FORWARD_APPROACH, done_after_ticks=1)
    engine = WorkflowEngine(
        phase_sequence=(Phase.FORWARD_APPROACH,),
        runners={Phase.FORWARD_APPROACH: forward},
        context=context,
    )

    result = engine.tick()
    later = engine.tick()

    assert result.phase == Phase.FORWARD_APPROACH
    assert result.done is True
    assert result.stop_reason == "phase_sequence_complete"
    assert later.phase == Phase.DONE
    assert later.done is True
    assert later.stop_reason == "phase_sequence_complete"
