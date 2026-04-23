from __future__ import annotations

from typing import Any, Callable

from workflow.types import EnvStatus, Phase, parse_phase

from .base import AdapterCommand


class RobotAdapter:
    def __init__(self, *, env_status_publisher: Callable[[str], None]) -> None:
        self._env_status_publisher = env_status_publisher
        self._current_env_status = EnvStatus.IDLE

    def on_phase(self, phase: Phase | str) -> EnvStatus:
        current_phase = parse_phase(phase)
        self._current_env_status = (
            EnvStatus.READY if current_phase is Phase.STATUS_ALIGN else EnvStatus.IDLE
        )
        self._env_status_publisher(self._current_env_status.value)
        return self._current_env_status

    def on_cmd_vel(self, cmd_vel: Any) -> AdapterCommand:
        return AdapterCommand(
            linear_x=float(cmd_vel.linear.x),
            linear_y=float(cmd_vel.linear.y),
            angular_z=float(cmd_vel.angular.z),
        )
