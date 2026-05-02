from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class TopicConfig:
    cmd_topic: str
    publish_cmd_vel: bool
    workflow_phase_topic: str
    algo_status_topic: str
    env_status_topic: str
    selected_status_topic: str


@dataclass(frozen=True)
class CameraFallbackConfig:
    backend: str
    fourcc: str
    width: int
    height: int
    fps: float

    def describe(self) -> str:
        return (
            f"backend={self.backend} fourcc={self.fourcc} "
            f"width={self.width} height={self.height} fps={self.fps:g}"
        )


DEFAULT_CAMERA_FALLBACKS = (
    CameraFallbackConfig("v4l2", "MJPG", 640, 360, 270.0),
    CameraFallbackConfig("v4l2", "MJPG", 800, 600, 190.0),
    CameraFallbackConfig("v4l2", "MJPG", 1024, 768, 190.0),
)


@dataclass(frozen=True)
class DetectorConfig:
    sdk_config: Path
    status_profile: str
    base_coord_profile: str
    input_source: str
    camera_fallbacks: tuple[CameraFallbackConfig, ...]


@dataclass(frozen=True)
class CmdVelTransformConfig:
    invert_linear_x: bool = False
    invert_linear_y: bool = False
    invert_angular_z: bool = False


@dataclass(frozen=True)
class AdapterConfig:
    turtle_cmd_topic: str | None
    cmd_vel_transform: CmdVelTransformConfig = field(
        default_factory=CmdVelTransformConfig
    )


@dataclass(frozen=True)
class ForwardApproachConfig:
    speed_mps: float
    distance_m: float


@dataclass(frozen=True)
class BaseCoordConfig:
    publish_topic: str
    frame_id: str
    complete_on_first_target: bool


@dataclass(frozen=True)
class RuntimeConfig:
    workflow_hz: float
    command_publish_hz: float
    command_timeout_s: float


@dataclass(frozen=True)
class PidAlignmentWorkflowConfig:
    environment: str
    start_phase: str
    phase_sequence: tuple[str, ...]
    target_x: float
    tolerance_px: float
    max_speed: float
    runtime: RuntimeConfig
    forward_approach: ForwardApproachConfig
    base_coord: BaseCoordConfig
    topics: TopicConfig
    detector: DetectorConfig
    adapter: AdapterConfig
