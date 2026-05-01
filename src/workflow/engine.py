from __future__ import annotations

from dataclasses import replace
from typing import Any, Mapping

from workflow.phase_runner import PhaseRunner, PhaseTickResult
from workflow.types import AlgoStatus, EnvStatus, Phase, parse_phase


class WorkflowEngine:
    def __init__(
        self,
        *,
        phase_sequence: tuple[Phase | str, ...],
        runners: Mapping[Phase | str, PhaseRunner],
        context: Any,
    ) -> None:
        if not phase_sequence:
            raise ValueError("phase_sequence must not be empty")

        self._phase_sequence = tuple(parse_phase(phase) for phase in phase_sequence)
        self._runners = {parse_phase(phase): runner for phase, runner in runners.items()}
        self._context = context
        self._phase_index = 0
        self._entered = False
        self._complete = False

        missing = [
            phase.value for phase in self._phase_sequence if phase not in self._runners
        ]
        if missing:
            raise ValueError(f"Missing phase runners for: {', '.join(missing)}")

    @property
    def current_phase(self) -> Phase:
        if self._complete:
            return Phase.DONE
        return self._phase_sequence[self._phase_index]

    def tick(self) -> PhaseTickResult:
        if self._complete:
            return _phase_sequence_complete_result()

        phase = self._phase_sequence[self._phase_index]
        runner = self._runners[phase]
        if not self._entered:
            runner.on_enter(self._context)
            self._entered = True

        result = runner.tick(self._context)
        if not result.done:
            return result

        runner.on_exit(self._context)
        is_last_phase = self._phase_index + 1 >= len(self._phase_sequence)
        if is_last_phase:
            self._complete = True
            if result.stop_reason is None:
                return replace(result, stop_reason="phase_sequence_complete")
            return result

        self._phase_index += 1
        self._entered = False
        return result


def _phase_sequence_complete_result() -> PhaseTickResult:
    return PhaseTickResult(
        phase=Phase.DONE,
        algo_status=AlgoStatus.STEP_DONE,
        env_status=EnvStatus.DONE,
        done=True,
        stop_reason="phase_sequence_complete",
    )
