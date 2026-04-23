from __future__ import annotations

from typing import Any, Callable

from workflow.types import EnvStatus, Phase, parse_phase

from .base import AdapterCommand

TurtleVelocityCommand = AdapterCommand


def update_env_status_from_phase(phase: Phase | str) -> EnvStatus:
    current_phase = parse_phase(phase)
    if current_phase is Phase.STATUS_ALIGN:
        return EnvStatus.READY
    return EnvStatus.IDLE


def map_cmd_vel_to_turtle_command(cmd_vel: Any) -> AdapterCommand:
    return AdapterCommand(
        linear_x=float(cmd_vel.linear.y),
        linear_y=0.0,
        angular_z=float(cmd_vel.angular.z),
    )


class TurtleAdapter:
    def __init__(
        self,
        *,
        env_status_publisher: Callable[[str], None],
        turtle_cmd_publisher: Callable[[dict[str, float]], None] | None = None,
    ) -> None:
        self._env_status_publisher = env_status_publisher
        self._turtle_cmd_publisher = turtle_cmd_publisher
        self._current_env_status = EnvStatus.IDLE

    @property
    def current_env_status(self) -> EnvStatus:
        return self._current_env_status

    def on_phase(self, phase: Phase | str) -> EnvStatus:
        self._current_env_status = update_env_status_from_phase(phase)
        self._env_status_publisher(self._current_env_status.value)
        return self._current_env_status

    def on_cmd_vel(self, cmd_vel: Any) -> AdapterCommand:
        command = map_cmd_vel_to_turtle_command(cmd_vel)
        if self._turtle_cmd_publisher is not None:
            self._turtle_cmd_publisher(
                {
                    "linear_x": command.linear_x,
                    "angular_z": command.angular_z,
                }
            )
        return command


class TurtleAdapterNode:
    def on_phase_message(self, phase_value: str) -> EnvStatus:
        return self._adapter.on_phase(phase_value)

    def on_cmd_vel_message(self, cmd_vel: Any) -> AdapterCommand:
        return self._adapter.on_cmd_vel(cmd_vel)
