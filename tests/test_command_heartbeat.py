import pytest

from runners.command_heartbeat import BufferedCommandPublisher, ZERO_COMMAND


def test_heartbeat_repeats_latest_command_until_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)

    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    first = buffer.snapshot_for_heartbeat(timeout_s=0.25)
    assert first.command == {"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}
    assert first.stop_requested is False
    assert first.timed_out is False

    now = 10.10
    second = buffer.snapshot_for_heartbeat(timeout_s=0.25)
    assert second.command == {"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}
    assert second.repeat_count_since_workflow_tick == 2


def test_heartbeat_uses_zero_after_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)

    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})
    now = 10.30

    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == ZERO_COMMAND
    assert snapshot.timed_out is True


def test_explicit_stop_signal_overrides_latest_command_without_waiting_for_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    now = 10.01
    buffer.request_stop(reason="phase_sequence_complete")
    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == ZERO_COMMAND
    assert snapshot.stop_requested is True
    assert snapshot.stop_reason == "phase_sequence_complete"
    assert snapshot.timed_out is False


def test_new_workflow_command_clears_stop_signal():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.request_stop(reason="frame_read_failed")

    now = 10.02
    buffer.publish({"linear_x": 0.0, "linear_y": -0.1, "angular_z": 0.0})
    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == {"linear_x": 0.0, "linear_y": -0.1, "angular_z": 0.0}
    assert snapshot.stop_requested is False
    assert snapshot.stop_reason is None


def test_consume_workflow_cycle_stats_reports_filled_heartbeat_frames_once():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    buffer.snapshot_for_heartbeat(timeout_s=0.25)
    buffer.snapshot_for_heartbeat(timeout_s=0.25)

    stats = buffer.consume_workflow_cycle_stats()
    assert stats.filled_heartbeat_frames == 2
    assert stats.command_age_s == 0.0
    assert stats.stop_requested is False
    assert stats.stop_reason is None

    next_stats = buffer.consume_workflow_cycle_stats()
    assert next_stats.filled_heartbeat_frames == 0


def test_request_stop_refreshes_latest_zero_command_and_stats():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.publish({"linear_x": 0.1, "linear_y": 0.2, "angular_z": 0.3})

    now = 10.05
    buffer.request_stop(reason="phase_sequence_complete")

    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)
    assert snapshot.command == ZERO_COMMAND
    assert snapshot.stop_requested is True
    assert snapshot.stop_reason == "phase_sequence_complete"
    assert snapshot.timed_out is False
    assert snapshot.command_age_s == 0.0

    stats = buffer.consume_workflow_cycle_stats()
    assert stats.command_age_s == 0.0
    assert stats.stop_requested is True
    assert stats.stop_reason == "phase_sequence_complete"


def test_snapshot_timeout_must_be_passed_by_keyword():
    buffer = BufferedCommandPublisher(now_s=lambda: 10.0)

    with pytest.raises(TypeError):
        buffer.snapshot_for_heartbeat(0.25)


def test_workflow_cycle_stats_counts_only_filled_heartbeat_frames():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)

    buffer.snapshot_for_heartbeat(timeout_s=0.25)
    startup_stats = buffer.consume_workflow_cycle_stats()
    assert startup_stats.filled_heartbeat_frames == 0

    buffer.publish({"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0})
    buffer.snapshot_for_heartbeat(timeout_s=0.25)
    filled_stats = buffer.consume_workflow_cycle_stats()
    assert filled_stats.filled_heartbeat_frames == 1

    now = 10.30
    buffer.snapshot_for_heartbeat(timeout_s=0.25)
    timeout_stats = buffer.consume_workflow_cycle_stats()
    assert timeout_stats.filled_heartbeat_frames == 0
