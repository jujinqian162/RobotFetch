from pathlib import Path

from config.loaders import load_pid_alignment_config


def test_load_pid_alignment_config_reads_turtle_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.turtle.yaml")
    )

    assert cfg.environment == "turtle"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.one_shot is True
    assert cfg.adapter.turtle_cmd_topic == "/turtle1/cmd_vel"
    assert cfg.detector.status_profile == "status_competition"
