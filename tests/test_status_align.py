from dataclasses import dataclass

from algorithms.pid import PIDConfig
from algorithms.status_align import StatusAlignConfig, StatusAlignStep, run_status_align_step


@dataclass
class FakeStatusTarget:
    label: str
    cx: float


DEFAULT_PID = PIDConfig(
    kp=0.1,
    ki=0.0,
    kd=0.0,
    output_limit=1.0,
    integral_limit=10.0,
    deadband=0.0,
    derivative_alpha=0.5,
)

DEFAULT_CONFIG = StatusAlignConfig(
    pid=DEFAULT_PID,
    target_x=320.0,
    tolerance_px=10.0,
    allowed_labels={"palm", "fist"},
    stable_labels={"palm"},
    use_stable_labels=True,
)


def test_status_align_waits_when_no_targets_are_available():
    step = run_status_align_step(
        targets=[],
        now_s=1.0,
        cfg=DEFAULT_CONFIG,
    )

    assert step.status == "TARGET_LOST"
    assert step.command_x == 0.0
    assert step.selected_target is None
    assert step.aligned is False



def test_status_align_reports_aligned_within_tolerance():
    target = FakeStatusTarget(label="palm", cx=326.0)

    step = run_status_align_step(
        targets=[target],
        now_s=1.0,
        cfg=DEFAULT_CONFIG,
    )

    assert step.status == "ALIGNED"
    assert step.command_x == 0.0
    assert step.selected_target is target
    assert step.aligned is True



def test_status_align_runs_pid_when_target_is_outside_tolerance():
    target = FakeStatusTarget(label="palm", cx=300.0)

    step = run_status_align_step(
        targets=[target],
        now_s=1.0,
        cfg=DEFAULT_CONFIG,
    )

    assert step.status == "RUNNING"
    assert step.command_x == 1.0
    assert step.selected_target is target
    assert step.aligned is False



def test_status_align_reset_clears_pid_history():
    target = FakeStatusTarget(label="palm", cx=300.0)
    state = StatusAlignStep(cfg=DEFAULT_CONFIG)

    first = state.run(targets=[target], now_s=1.0)
    second = state.run(targets=[target], now_s=2.0)
    state.reset()
    after_reset = state.run(targets=[target], now_s=3.0)

    assert first.command_x == 1.0
    assert second.command_x == 1.0
    assert after_reset.command_x == 1.0
