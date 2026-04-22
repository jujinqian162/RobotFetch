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
class DetectorConfig:
    sdk_config: Path
    status_profile: str
    input_source: str


@dataclass(frozen=True)
class AdapterConfig:
    turtle_cmd_topic: str | None


@dataclass(frozen=True)
class PidAlignmentWorkflowConfig:
    environment: str
    start_phase: str
    one_shot: bool
    target_x: float
    tolerance_px: float
    topics: TopicConfig
    detector: DetectorConfig
    adapter: AdapterConfig
