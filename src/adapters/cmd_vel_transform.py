from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class CmdVelTransformAdapter:
    invert_linear_x: bool = False
    invert_linear_y: bool = False
    invert_angular_z: bool = False

    @classmethod
    def from_config(cls, config: Any) -> CmdVelTransformAdapter:
        return cls(
            invert_linear_x=bool(getattr(config, "invert_linear_x", False)),
            invert_linear_y=bool(getattr(config, "invert_linear_y", False)),
            invert_angular_z=bool(getattr(config, "invert_angular_z", False)),
        )

    def apply(self, payload: Mapping[str, float]) -> dict[str, float]:
        linear_x = float(payload.get("linear_x", 0.0))
        linear_y = float(payload.get("linear_y", 0.0))
        angular_z = float(payload.get("angular_z", 0.0))
        return {
            **payload,
            "linear_x": -linear_x if self.invert_linear_x else linear_x,
            "linear_y": -linear_y if self.invert_linear_y else linear_y,
            "angular_z": -angular_z if self.invert_angular_z else angular_z,
        }
