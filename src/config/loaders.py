from __future__ import annotations

from pathlib import Path

import yaml

from .models import (
    AdapterConfig,
    DetectorConfig,
    PidAlignmentWorkflowConfig,
    TopicConfig,
)


def load_pid_alignment_config(path: Path) -> PidAlignmentWorkflowConfig:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    root = payload["pid_alignment_workflow"]

    return PidAlignmentWorkflowConfig(
        environment=str(root["environment"]),
        start_phase=str(root.get("start_phase", "STATUS_ALIGN")),
        one_shot=bool(root.get("one_shot", False)),
        target_x=float(root["status_align"]["target_x"]),
        tolerance_px=float(root["status_align"]["tolerance_px"]),
        topics=TopicConfig(**root["topics"]),
        detector=DetectorConfig(
            sdk_config=Path(root["detector"]["sdk_config"]),
            status_profile=str(root["detector"]["status_profile"]),
            input_source=str(root["detector"]["input_source"]),
        ),
        adapter=AdapterConfig(
            turtle_cmd_topic=root["adapter"].get("turtle_cmd_topic"),
        ),
    )
