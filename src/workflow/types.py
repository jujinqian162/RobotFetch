from __future__ import annotations

from enum import Enum


class Phase(str, Enum):
    READY = "READY"
    STATUS_ALIGN = "STATUS_ALIGN"
    FORWARD_APPROACH = "FORWARD_APPROACH"
    BASE_COORD = "BASE_COORD"
    DONE = "DONE"
    ABORT = "ABORT"


class AlgoStatus(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_PHASE = "WAITING_FOR_PHASE"
    RUNNING = "RUNNING"
    TARGET_LOST = "TARGET_LOST"
    ALIGNED = "ALIGNED"
    STEP_DONE = "STEP_DONE"
    ERROR = "ERROR"


class EnvStatus(str, Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    READY = "READY"
    RUNNING = "RUNNING"
    DONE = "DONE"
    ERROR = "ERROR"


def parse_phase(raw: str | Phase) -> Phase:
    if isinstance(raw, Phase):
        return raw
    try:
        return Phase(str(raw).strip().upper())
    except ValueError as exc:
        raise ValueError(f"Unknown phase: {raw}") from exc
