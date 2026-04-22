from workflow.phase_controller import PhaseController
from workflow.types import Phase


def test_phase_controller_uses_start_phase():
    controller = PhaseController(start_phase=Phase.STATUS_ALIGN)
    assert controller.current_phase is Phase.STATUS_ALIGN


def test_phase_controller_records_phase_transition():
    controller = PhaseController(start_phase=Phase.READY)
    controller.enter_phase(Phase.BASE_COORD)
    assert controller.current_phase is Phase.BASE_COORD
    assert controller.enter_count(Phase.BASE_COORD) == 1


def test_phase_controller_runs_reset_callbacks_on_entry():
    calls: list[str] = []

    def reset_status_align() -> None:
        calls.append("status")

    controller = PhaseController(
        start_phase=Phase.READY,
        reset_callbacks={Phase.STATUS_ALIGN: reset_status_align},
    )

    controller.enter_phase(Phase.STATUS_ALIGN)
    assert calls == ["status"]


def test_phase_controller_does_not_call_missing_reset_callback():
    calls: list[str] = []

    def reset_status_align() -> None:
        calls.append("status")

    controller = PhaseController(
        start_phase=Phase.READY,
        reset_callbacks={Phase.STATUS_ALIGN: reset_status_align},
    )
    controller.enter_phase(Phase.DONE)
    assert controller.enter_count(Phase.DONE) == 1
    assert calls == []
