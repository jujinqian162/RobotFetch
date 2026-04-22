from workflow.types import AlgoStatus, EnvStatus, Phase, parse_phase


def test_phase_values_are_stable():
    assert Phase.READY.value == "READY"
    assert Phase.STATUS_ALIGN.value == "STATUS_ALIGN"
    assert Phase.FORWARD_APPROACH.value == "FORWARD_APPROACH"
    assert Phase.BASE_COORD.value == "BASE_COORD"
    assert Phase.DONE.value == "DONE"
    assert Phase.ABORT.value == "ABORT"


def test_algo_status_values_are_stable():
    assert AlgoStatus.IDLE.value == "IDLE"
    assert AlgoStatus.WAITING_FOR_PHASE.value == "WAITING_FOR_PHASE"
    assert AlgoStatus.RUNNING.value == "RUNNING"
    assert AlgoStatus.TARGET_LOST.value == "TARGET_LOST"
    assert AlgoStatus.ALIGNED.value == "ALIGNED"
    assert AlgoStatus.STEP_DONE.value == "STEP_DONE"
    assert AlgoStatus.ERROR.value == "ERROR"


def test_env_status_values_are_stable():
    assert EnvStatus.IDLE.value == "IDLE"
    assert EnvStatus.PREPARING.value == "PREPARING"
    assert EnvStatus.READY.value == "READY"
    assert EnvStatus.RUNNING.value == "RUNNING"
    assert EnvStatus.DONE.value == "DONE"
    assert EnvStatus.ERROR.value == "ERROR"


def test_parse_phase_accepts_normalized_string_values():
    assert parse_phase(" STATUS_ALIGN ") is Phase.STATUS_ALIGN
    assert parse_phase("base_coord") is Phase.BASE_COORD


def test_parse_phase_passes_phase_instances_through():
    assert parse_phase(Phase.DONE) is Phase.DONE


def test_parse_phase_rejects_unknown_phase():
    try:
        parse_phase("NOT_A_PHASE")
    except ValueError as exc:
        assert "Unknown phase" in str(exc)
    else:
        raise AssertionError("parse_phase should reject unknown names")
