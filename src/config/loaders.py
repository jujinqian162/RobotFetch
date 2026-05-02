from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .models import (
    DEFAULT_CAMERA_FALLBACKS,
    AdapterConfig,
    BaseCoordConfig,
    CameraFallbackConfig,
    CmdVelTransformConfig,
    DetectorConfig,
    ForwardApproachConfig,
    PidAlignmentWorkflowConfig,
    RuntimeConfig,
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
    runtime = _optional_mapping_field(
        root,
        "runtime",
        parent="pid_alignment_workflow",
    )
    topics = _require_mapping_field(root, "topics", parent="pid_alignment_workflow")
    detector = _require_mapping_field(root, "detector", parent="pid_alignment_workflow")
    adapter = _require_mapping_field(root, "adapter", parent="pid_alignment_workflow")
    cmd_vel_transform = _optional_mapping_field(
        adapter,
        "cmd_vel_transform",
        parent="pid_alignment_workflow.adapter",
    )
    status_align = _require_mapping_field(
        root, "status_align", parent="pid_alignment_workflow"
    )
    forward_approach = _optional_mapping_field(
        root, "forward_approach", parent="pid_alignment_workflow"
    )
    base_coord = _optional_mapping_field(
        root, "base_coord", parent="pid_alignment_workflow"
    )

    start_phase = _require_defaulted_str_field(
        root,
        "start_phase",
        default="STATUS_ALIGN",
        parent="pid_alignment_workflow",
    )
    phase_sequence = _require_phase_sequence_field(
        root,
        "phase_sequence",
        start_phase=start_phase,
        parent="pid_alignment_workflow",
    )

    runtime_config = RuntimeConfig(
        workflow_hz=_require_positive_defaulted_float_field(
            runtime,
            "workflow_hz",
            default=10.0,
            parent="pid_alignment_workflow.runtime",
        ),
        command_publish_hz=_require_minimum_defaulted_float_field(
            runtime,
            "command_publish_hz",
            default=30.0,
            minimum=20.0,
            parent="pid_alignment_workflow.runtime",
        ),
        command_timeout_s=_require_positive_defaulted_float_field(
            runtime,
            "command_timeout_s",
            default=0.25,
            parent="pid_alignment_workflow.runtime",
        ),
    )
    _validate_runtime_config(runtime_config)

    return PidAlignmentWorkflowConfig(
        environment=_require_str_field(root, "environment", parent="pid_alignment_workflow"),
        start_phase=start_phase,
        phase_sequence=phase_sequence,
        target_x=_require_float_field(
            status_align, "target_x", parent="pid_alignment_workflow.status_align"
        ),
        tolerance_px=_require_float_field(
            status_align, "tolerance_px", parent="pid_alignment_workflow.status_align"
        ),
        max_speed=abs(
            _require_defaulted_float_field(
                status_align,
                "max_speed",
                default=0.25,
                parent="pid_alignment_workflow.status_align",
            )
        ),
        runtime=runtime_config,
        forward_approach=ForwardApproachConfig(
            speed_mps=_require_positive_defaulted_float_field(
                forward_approach,
                "speed_mps",
                default=0.1,
                parent="pid_alignment_workflow.forward_approach",
            ),
            distance_m=_require_positive_defaulted_float_field(
                forward_approach,
                "distance_m",
                default=0.2,
                parent="pid_alignment_workflow.forward_approach",
            ),
        ),
        base_coord=BaseCoordConfig(
            publish_topic=_require_defaulted_str_field(
                base_coord,
                "publish_topic",
                default="/robot_fetch/base_coord_targets",
                parent="pid_alignment_workflow.base_coord",
            ),
            frame_id=_require_defaulted_str_field(
                base_coord,
                "frame_id",
                default="camera_link",
                parent="pid_alignment_workflow.base_coord",
            ),
            complete_on_first_target=_require_defaulted_bool_field(
                base_coord,
                "complete_on_first_target",
                default=True,
                parent="pid_alignment_workflow.base_coord",
            ),
        ),
        topics=TopicConfig(
            cmd_topic=_require_str_field(
                topics, "cmd_topic", parent="pid_alignment_workflow.topics"
            ),
            publish_cmd_vel=_require_defaulted_bool_field(
                topics,
                "publish_cmd_vel",
                default=True,
                parent="pid_alignment_workflow.topics",
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
            base_coord_profile=_require_defaulted_str_field(
                detector,
                "base_coord_profile",
                default="base_coord_competition",
                parent="pid_alignment_workflow.detector",
            ),
            input_source=_require_str_field(
                detector,
                "input_source",
                parent="pid_alignment_workflow.detector",
            ),
            camera_fallbacks=_require_camera_fallbacks_field(
                detector,
                "camera_fallbacks",
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
            cmd_vel_transform=CmdVelTransformConfig(
                invert_linear_x=_require_defaulted_bool_field(
                    cmd_vel_transform,
                    "invert_linear_x",
                    default=False,
                    parent="pid_alignment_workflow.adapter.cmd_vel_transform",
                ),
                invert_linear_y=_require_defaulted_bool_field(
                    cmd_vel_transform,
                    "invert_linear_y",
                    default=False,
                    parent="pid_alignment_workflow.adapter.cmd_vel_transform",
                ),
                invert_angular_z=_require_defaulted_bool_field(
                    cmd_vel_transform,
                    "invert_angular_z",
                    default=False,
                    parent="pid_alignment_workflow.adapter.cmd_vel_transform",
                ),
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


def _optional_mapping_field(
    mapping: dict[str, Any], key: str, *, parent: str = "config"
) -> dict[str, Any]:
    if key not in mapping:
        return {}
    value = mapping[key]
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


def _require_defaulted_bool_field(
    mapping: dict[str, Any], key: str, *, default: bool, parent: str
) -> bool:
    if key not in mapping:
        return default
    value = mapping[key]
    if not isinstance(value, bool):
        raise TypeError(f"{parent}.{key} must be a boolean")
    return value


def _require_defaulted_float_field(
    mapping: dict[str, Any], key: str, *, default: float, parent: str
) -> float:
    if key not in mapping:
        return default
    value = mapping[key]
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{parent}.{key} must be a number")
    return float(value)


def _require_positive_defaulted_float_field(
    mapping: dict[str, Any], key: str, *, default: float, parent: str
) -> float:
    value = _require_defaulted_float_field(
        mapping,
        key,
        default=default,
        parent=parent,
    )
    if value <= 0.0:
        raise ValueError(f"{parent}.{key} must be greater than 0")
    return value


def _require_minimum_defaulted_float_field(
    mapping: dict[str, Any],
    key: str,
    *,
    default: float,
    minimum: float,
    parent: str,
) -> float:
    value = _require_defaulted_float_field(
        mapping,
        key,
        default=default,
        parent=parent,
    )
    if value < minimum:
        raise ValueError(f"{parent}.{key} must be >= {minimum:g}")
    return value


def _validate_runtime_config(runtime: RuntimeConfig) -> None:
    workflow_period_s = 1.0 / runtime.workflow_hz
    if runtime.command_timeout_s <= workflow_period_s:
        raise ValueError(
            "pid_alignment_workflow.runtime.command_timeout_s must be greater "
            "than one workflow tick period"
        )


def _require_phase_sequence_field(
    mapping: dict[str, Any],
    key: str,
    *,
    start_phase: str,
    parent: str,
) -> tuple[str, ...]:
    if key not in mapping:
        return (start_phase,)
    value = mapping[key]
    if not isinstance(value, list):
        raise TypeError(f"{parent}.{key} must be a list of strings")
    if not value:
        raise ValueError(f"{parent}.{key} must not be empty")
    for item in value:
        if not isinstance(item, str):
            raise TypeError(f"{parent}.{key} must be a list of strings")
    if value[0] != start_phase:
        raise ValueError(f"{parent}.{key} must start with start_phase {start_phase}")
    return tuple(value)


def _require_int_field(mapping: dict[str, Any], key: str, *, parent: str) -> int:
    value = _require_field(mapping, key, parent=parent)
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{parent}.{key} must be an integer")
    return value


def _require_float_field(mapping: dict[str, Any], key: str, *, parent: str) -> float:
    value = _require_field(mapping, key, parent=parent)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{parent}.{key} must be a number")
    return float(value)


def _require_camera_fallbacks_field(
    mapping: dict[str, Any], key: str, *, parent: str
) -> tuple[CameraFallbackConfig, ...]:
    if key not in mapping:
        return DEFAULT_CAMERA_FALLBACKS
    value = mapping[key]
    if not isinstance(value, list):
        raise TypeError(f"{parent}.{key} must be a list of camera fallback mappings")

    fallbacks: list[CameraFallbackConfig] = []
    for index, item in enumerate(value):
        item_parent = f"{parent}.{key}[{index}]"
        if not isinstance(item, dict):
            raise TypeError(f"{item_parent} must be a mapping")
        fallbacks.append(
            CameraFallbackConfig(
                backend=_require_str_field(item, "backend", parent=item_parent),
                fourcc=_require_str_field(item, "fourcc", parent=item_parent),
                width=_require_int_field(item, "width", parent=item_parent),
                height=_require_int_field(item, "height", parent=item_parent),
                fps=_require_float_field(item, "fps", parent=item_parent),
            )
        )
    return tuple(fallbacks)


def _require_path_field(
    mapping: dict[str, Any], key: str, *, parent: str, project_root: Path
) -> Path:
    raw_path = _require_str_field(mapping, key, parent=parent)
    candidate = Path(raw_path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate
