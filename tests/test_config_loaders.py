from pathlib import Path

import pytest

from config.loaders import load_pid_alignment_config
from config.models import DEFAULT_CAMERA_FALLBACKS, CameraFallbackConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_load_pid_alignment_config_reads_turtle_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.turtle.yaml")
    )

    assert cfg.environment == "turtle"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.phase_sequence == ("STATUS_ALIGN",)
    assert cfg.target_x == 320.0
    assert cfg.tolerance_px == 8.0
    assert cfg.topics.cmd_topic == "/cmd_vel"
    assert cfg.topics.workflow_phase_topic == "/workflow/phase"
    assert cfg.adapter.turtle_cmd_topic == "/turtle1/cmd_vel"
    assert cfg.detector.sdk_config == (
        PROJECT_ROOT / "BaseDetect/configs/basedetect_sdk.yaml"
    )
    assert cfg.detector.status_profile == "status_competition"
    assert isinstance(cfg.detector.sdk_config, Path)
    assert cfg.detector.camera_fallbacks == DEFAULT_CAMERA_FALLBACKS


def test_load_pid_alignment_config_reads_robot_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.robot.yaml")
    )

    assert cfg.environment == "robot"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.phase_sequence == ("STATUS_ALIGN",)
    assert cfg.target_x == 320.0
    assert cfg.tolerance_px == 8.0
    assert cfg.topics.algo_status_topic == "/workflow/algo_status"
    assert cfg.topics.env_status_topic == "/workflow/env_status"
    assert cfg.topics.selected_status_topic == "/robot_fetch/selected_target_px"
    assert cfg.adapter.turtle_cmd_topic is None
    assert cfg.detector.sdk_config == (
        PROJECT_ROOT / "BaseDetect/configs/basedetect_sdk.yaml"
    )
    assert cfg.detector.camera_fallbacks == DEFAULT_CAMERA_FALLBACKS


def test_load_pid_alignment_config_resolves_sdk_config_independent_of_cwd(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    monkeypatch.chdir(tmp_path)

    cfg = load_pid_alignment_config(
        PROJECT_ROOT / "configs/workflows/pid_alignment.turtle.yaml"
    )

    assert cfg.detector.sdk_config == (
        PROJECT_ROOT / "BaseDetect/configs/basedetect_sdk.yaml"
    )


def test_load_pid_alignment_config_rejects_string_for_phase_sequence(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: turtle
  start_phase: STATUS_ALIGN
  phase_sequence: STATUS_ALIGN
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(TypeError, match="phase_sequence"):
        load_pid_alignment_config(config_path)


def test_load_pid_alignment_config_defaults_phase_sequence_to_start_phase(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.default_sequence.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: turtle
  start_phase: STATUS_ALIGN
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    cfg = load_pid_alignment_config(config_path)

    assert cfg.phase_sequence == ("STATUS_ALIGN",)
    assert cfg.detector.camera_fallbacks == DEFAULT_CAMERA_FALLBACKS


def test_load_pid_alignment_config_reads_ordered_camera_fallbacks(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.camera_fallbacks.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  phase_sequence: [STATUS_ALIGN]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
    camera_fallbacks:
      - backend: v4l2
        fourcc: MJPG
        width: 1024
        height: 768
        fps: 190.0
      - backend: v4l2
        fourcc: MJPG
        width: 800
        height: 600
        fps: 190.0
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    cfg = load_pid_alignment_config(config_path)

    assert cfg.detector.camera_fallbacks == (
        CameraFallbackConfig("v4l2", "MJPG", 1024, 768, 190.0),
        CameraFallbackConfig("v4l2", "MJPG", 800, 600, 190.0),
    )


def test_load_pid_alignment_config_allows_empty_camera_fallbacks(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.no_fallbacks.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  phase_sequence: [STATUS_ALIGN]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
    camera_fallbacks: []
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    cfg = load_pid_alignment_config(config_path)

    assert cfg.detector.camera_fallbacks == ()


def test_load_pid_alignment_config_rejects_invalid_camera_fallbacks(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid_camera_fallbacks.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  phase_sequence: [STATUS_ALIGN]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
    camera_fallbacks:
      - backend: v4l2
        fourcc: MJPG
        width: "1024"
        height: 768
        fps: 190.0
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(TypeError, match="camera_fallbacks"):
        load_pid_alignment_config(config_path)


def test_load_pid_alignment_config_rejects_sequence_that_does_not_start_with_start_phase(
    tmp_path: Path,
):
    config_path = tmp_path / "pid_alignment.invalid_sequence.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: turtle
  start_phase: STATUS_ALIGN
  phase_sequence: [FORWARD_APPROACH]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="must start with start_phase"):
        load_pid_alignment_config(config_path)


def test_load_pid_alignment_config_rejects_non_string_environment(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: 123
  start_phase: STATUS_ALIGN
  phase_sequence: [STATUS_ALIGN]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(TypeError, match="environment"):
        load_pid_alignment_config(config_path)


def test_load_pid_alignment_config_rejects_null_start_phase(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: turtle
  start_phase: null
  phase_sequence: [STATUS_ALIGN]
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(TypeError, match="start_phase"):
        load_pid_alignment_config(config_path)
