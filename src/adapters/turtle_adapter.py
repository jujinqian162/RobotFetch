from __future__ import annotations

from typing import Callable

from workflow.types import EnvStatus, Phase, parse_phase


def update_env_status_from_phase(phase: Phase | str) -> EnvStatus:
    current_phase = parse_phase(phase)
    if current_phase is Phase.STATUS_ALIGN:
        return EnvStatus.READY
    return EnvStatus.IDLE


class TurtleAdapter:
    def __init__(self, *, env_status_publisher: Callable[[str], None]) -> None:
        self._env_status_publisher = env_status_publisher
        self._current_env_status = EnvStatus.IDLE

    @property
    def current_env_status(self) -> EnvStatus:
        return self._current_env_status

    def on_phase(self, phase: Phase | str) -> EnvStatus:
        self._current_env_status = update_env_status_from_phase(phase)
        self._env_status_publisher(self._current_env_status.value)
        return self._current_env_status


class TurtleAdapterNode:
    def on_phase_message(self, phase_value: str) -> EnvStatus:
        return self._adapter.on_phase(phase_value)
