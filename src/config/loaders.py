from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import (
    AdapterConfig,
    DetectorConfig,
    PidAlignmentWorkflowConfig,
    TopicConfig,
)


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def load_pid_alignment_config(
    path: Path, *, project_root: Path = PROJECT_ROOT
) -> PidAlignmentWorkflowConfig:
    config_path = Path(path)
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    payload_mapping = _require_mapping(payload, "config")
    root = _require_mapping_field(payload_mapping, "pid_alignment_workflow")
    topics = _require_mapping_field(root, "topics", parent="pid_alignment_workflow")
    detector = _require_mapping_field(root, "detector", parent="pid_alignment_workflow")
    adapter = _require_mapping_field(root, "adapter", parent="pid_alignment_workflow")
    status_align = _require_mapping_field(
        root, "status_align", parent="pid_alignment_workflow"
    )

    return PidAlignmentWorkflowConfig(
        environment=_require_str_field(root, "environment", parent="pid_alignment_workflow"),
        start_phase=_require_defaulted_str_field(
            root,
            "start_phase",
            default="STATUS_ALIGN",
            parent="pid_alignment_workflow",
        ),
        one_shot=_require_optional_bool_field(
            root,
            "one_shot",
            default=False,
            parent="pid_alignment_workflow",
        ),
        target_x=_require_float_field(
            status_align, "target_x", parent="pid_alignment_workflow.status_align"
        ),
        tolerance_px=_require_float_field(
            status_align, "tolerance_px", parent="pid_alignment_workflow.status_align"
        ),
        topics=TopicConfig(
            cmd_topic=_require_str_field(
                topics, "cmd_topic", parent="pid_alignment_workflow.topics"
            ),
            workflow_phase_topic=_require_str_field(
                topics,
                "workflow_phase_topic",
                parent="pid_alignment_workflow.topics",
            ),
            algo_status_topic=_require_str_field(
                topics,
                "algo_status_topic",
                parent="pid_alignment_workflow.topics",
            ),
            env_status_topic=_require_str_field(
                topics, "env_status_topic", parent="pid_alignment_workflow.topics"
            ),
            selected_status_topic=_require_str_field(
                topics,
                "selected_status_topic",
                parent="pid_alignment_workflow.topics",
            ),
        ),
        detector=DetectorConfig(
            sdk_config=_require_path_field(
                detector,
                "sdk_config",
                parent="pid_alignment_workflow.detector",
                project_root=project_root,
            ),
            status_profile=_require_str_field(
                detector,
                "status_profile",
                parent="pid_alignment_workflow.detector",
            ),
            input_source=_require_str_field(
                detector,
                "input_source",
                parent="pid_alignment_workflow.detector",
            ),
        ),
        adapter=AdapterConfig(
            turtle_cmd_topic=_require_optional_str_field(
                adapter,
                "turtle_cmd_topic",
                default=None,
                parent="pid_alignment_workflow.adapter",
            ),
        ),
    )


def _require_mapping(value: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TypeError(f"{field_name} must be a mapping")
    return value


def _require_mapping_field(
    mapping: dict[str, Any], key: str, *, parent: str = "config"
) -> dict[str, Any]:
    value = _require_field(mapping, key, parent=parent)
    if not isinstance(value, dict):
        raise TypeError(f"{parent}.{key} must be a mapping")
    return value


def _require_field(mapping: dict[str, Any], key: str, *, parent: str) -> Any:
    if key not in mapping:
        raise KeyError(f"Missing required config field: {parent}.{key}")
    return mapping[key]


def _require_str_field(mapping: dict[str, Any], key: str, *, parent: str) -> str:
    value = _require_field(mapping, key, parent=parent)
    if not isinstance(value, str):
        raise TypeError(f"{parent}.{key} must be a string")
    return value


def _require_optional_str_field(
    mapping: dict[str, Any],
    key: str,
    *,
    default: str | None,
    parent: str,
) -> str | None:
    if key not in mapping:
        return default
    value = mapping[key]
    if value is None:
        return None
    if not isinstance(value, str):
        raise TypeError(f"{parent}.{key} must be a string or null")
    return value


def _require_defaulted_str_field(
    mapping: dict[str, Any], key: str, *, default: str, parent: str
) -> str:
    if key not in mapping:
        return default
    value = mapping[key]
    if not isinstance(value, str):
        raise TypeError(f"{parent}.{key} must be a string")
    return value


def _require_optional_bool_field(
    mapping: dict[str, Any], key: str, *, default: bool, parent: str
) -> bool:
    if key not in mapping:
        return default
    value = mapping[key]
    if not isinstance(value, bool):
        raise TypeError(f"{parent}.{key} must be a bool")
    return value


def _require_float_field(mapping: dict[str, Any], key: str, *, parent: str) -> float:
    value = _require_field(mapping, key, parent=parent)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{parent}.{key} must be a number")
    return float(value)


def _require_path_field(
    mapping: dict[str, Any], key: str, *, parent: str, project_root: Path
) -> Path:
    raw_path = _require_str_field(mapping, key, parent=parent)
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate
