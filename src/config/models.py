from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TopicConfig:
    cmd_topic: str
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
    input_source: str
    camera_fallbacks: tuple[CameraFallbackConfig, ...]


@dataclass(frozen=True)
class AdapterConfig:
    turtle_cmd_topic: str | None


@dataclass(frozen=True)
class PidAlignmentWorkflowConfig:
    environment: str
    start_phase: str
    phase_sequence: tuple[str, ...]
    target_x: float
    tolerance_px: float
    topics: TopicConfig
    detector: DetectorConfig
    adapter: AdapterConfig
