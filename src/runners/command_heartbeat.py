from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable

ZERO_COMMAND = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


def normalize_command(payload: dict[str, float]) -> dict[str, float]:
    return {
        "linear_x": float(payload.get("linear_x", 0.0)),
        "linear_y": float(payload.get("linear_y", 0.0)),
        "angular_z": float(payload.get("angular_z", 0.0)),
    }


@dataclass(frozen=True)
class CommandHeartbeatSnapshot:
    command: dict[str, float]
    stop_requested: bool
    stop_reason: str | None
    timed_out: bool
    repeat_count_since_workflow_tick: int
    command_age_s: float | None


@dataclass(frozen=True)
class WorkflowCycleHeartbeatStats:
    filled_heartbeat_frames: int
    stop_requested: bool
    stop_reason: str | None
    command_age_s: float | None


class BufferedCommandPublisher:
    def __init__(self, *, now_s: Callable[[], float]) -> None:
        self._now_s = now_s
        self._lock = threading.Lock()
        self._latest_command = dict(ZERO_COMMAND)
        self._latest_command_at_s: float | None = None
        self._stop_requested = True
        self._stop_reason: str | None = "startup"
        self._repeat_count_since_workflow_tick = 0
        self._filled_count_since_workflow_tick = 0

    def publish(self, payload: dict[str, float]) -> None:
        normalized = normalize_command(payload)
        now = self._now_s()
        with self._lock:
            self._latest_command = normalized
            self._latest_command_at_s = now
            self._stop_requested = False
            self._stop_reason = None

    def request_stop(self, *, reason: str) -> None:
        now = self._now_s()
        with self._lock:
            self._latest_command = dict(ZERO_COMMAND)
            self._latest_command_at_s = now
            self._stop_requested = True
            self._stop_reason = reason

    def snapshot_for_heartbeat(self, *, timeout_s: float) -> CommandHeartbeatSnapshot:
        now = self._now_s()
        with self._lock:
            command_age_s = self._command_age_s_locked(now)
            if self._stop_requested:
                timed_out = False
            else:
                timed_out = (
                    self._latest_command_at_s is None
                    or command_age_s is None
                    or command_age_s > timeout_s
                )
            if self._stop_requested or timed_out:
                command = dict(ZERO_COMMAND)
            else:
                command = dict(self._latest_command)
                self._filled_count_since_workflow_tick += 1
            self._repeat_count_since_workflow_tick += 1
            return CommandHeartbeatSnapshot(
                command=command,
                stop_requested=self._stop_requested,
                stop_reason=self._stop_reason,
                timed_out=timed_out,
                repeat_count_since_workflow_tick=self._repeat_count_since_workflow_tick,
                command_age_s=command_age_s,
            )

    def consume_workflow_cycle_stats(self) -> WorkflowCycleHeartbeatStats:
        now = self._now_s()
        with self._lock:
            stats = WorkflowCycleHeartbeatStats(
                filled_heartbeat_frames=self._filled_count_since_workflow_tick,
                stop_requested=self._stop_requested,
                stop_reason=self._stop_reason,
                command_age_s=self._command_age_s_locked(now),
            )
            self._repeat_count_since_workflow_tick = 0
            self._filled_count_since_workflow_tick = 0
            return stats

    def _command_age_s_locked(self, now: float) -> float | None:
        if self._latest_command_at_s is None:
            return None
        return max(0.0, now - self._latest_command_at_s)
