from __future__ import annotations

from collections import Counter
from typing import Callable

from .types import Phase, parse_phase

ResetCallback = Callable[[], None]


class PhaseController:
    def __init__(
        self,
        start_phase: Phase | str,
        reset_callbacks: dict[Phase, ResetCallback] | None = None,
    ) -> None:
        self._current_phase = parse_phase(start_phase)
        self._reset_callbacks = dict(reset_callbacks or {})
        self._enter_counts: Counter[Phase] = Counter({self._current_phase: 1})

    @property
    def current_phase(self) -> Phase:
        return self._current_phase

    def enter_phase(self, phase: Phase | str) -> Phase:
        next_phase = parse_phase(phase)
        self._current_phase = next_phase
        self._enter_counts[next_phase] += 1
        callback = self._reset_callbacks.get(next_phase)
        if callback is not None:
            callback()
        return next_phase

    def enter_count(self, phase: Phase | str) -> int:
        return self._enter_counts[parse_phase(phase)]
