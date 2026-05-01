from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass(frozen=True)
class PhaseTickResult:
    phase: Phase
    algo_status: AlgoStatus
    env_status: EnvStatus
    command: dict[str, float] | None = None
    selected_target: dict[str, object] | None = None
    base_coord_targets: list[dict[str, object]] | None = None
    done: bool = False
    stop_reason: str | None = None


class PhaseRunner(Protocol):
    @property
    def phase(self) -> Phase:
        ...

    def on_enter(self, context: Any) -> None:
        ...

    def tick(self, context: Any) -> PhaseTickResult:
        ...

    def on_exit(self, context: Any) -> None:
        ...
