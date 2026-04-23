from workflow.types import AlgoStatus

from runners.pid_alignment_runner import should_stop_after_status_align


def test_should_stop_after_status_align_when_one_shot_and_aligned():
    assert should_stop_after_status_align(
        one_shot=True,
        algo_status=AlgoStatus.ALIGNED,
    ) is True
