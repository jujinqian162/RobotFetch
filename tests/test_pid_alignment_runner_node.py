from workflow.types import AlgoStatus

from runners.pid_alignment_runner import stop_reason_after_status_align


def test_stop_reason_after_status_align_when_single_phase_sequence_aligned():
    assert stop_reason_after_status_align(
        phase_sequence=("STATUS_ALIGN",),
        algo_status=AlgoStatus.ALIGNED,
    ) == "phase_sequence_complete"


def test_stop_reason_after_status_align_keeps_running_when_target_lost():
    assert (
        stop_reason_after_status_align(
            phase_sequence=("STATUS_ALIGN",),
            algo_status=AlgoStatus.TARGET_LOST,
        )
        is None
    )
