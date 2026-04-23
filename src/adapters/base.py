from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol

from workflow.types import EnvStatus, Phase


@dataclass(frozen=True)
class AdapterCommand:
    linear_x: float
    linear_y: float
    angular_z: float


class WorkflowAdapter(Protocol):
    def on_phase(self, phase: Phase | str) -> EnvStatus:
        pass

    def on_cmd_vel(self, cmd_vel: Any) -> AdapterCommand:
        pass
