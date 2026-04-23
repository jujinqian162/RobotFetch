from pathlib import Path

import pytest

from config.loaders import load_pid_alignment_config


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def test_load_pid_alignment_config_reads_turtle_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.turtle.yaml")
    )

    assert cfg.environment == "turtle"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.one_shot is True
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


def test_load_pid_alignment_config_reads_robot_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.robot.yaml")
    )

    assert cfg.environment == "robot"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.one_shot is True
    assert cfg.target_x == 320.0
    assert cfg.tolerance_px == 8.0
    assert cfg.topics.algo_status_topic == "/workflow/algo_status"
    assert cfg.topics.env_status_topic == "/workflow/env_status"
    assert cfg.topics.selected_status_topic == "/robot_fetch/selected_target_px"
    assert cfg.adapter.turtle_cmd_topic is None
    assert cfg.detector.sdk_config == (
        PROJECT_ROOT / "BaseDetect/configs/basedetect_sdk.yaml"
    )


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


def test_load_pid_alignment_config_rejects_string_for_one_shot(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: turtle
  start_phase: STATUS_ALIGN
  one_shot: "false"
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

    with pytest.raises(TypeError, match="one_shot"):
        load_pid_alignment_config(config_path)


def test_load_pid_alignment_config_rejects_non_string_environment(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.invalid.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: 123
  start_phase: STATUS_ALIGN
  one_shot: true
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
  one_shot: true
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
