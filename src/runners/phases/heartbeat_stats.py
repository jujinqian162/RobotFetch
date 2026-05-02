from __future__ import annotations

from typing import Any


def filled_heartbeat_frames(context: Any) -> int:
    consumer = getattr(context, "consume_heartbeat_stats", None)
    if not callable(consumer):
        return 0
    stats = consumer()
    try:
        return max(0, int(getattr(stats, "filled_heartbeat_frames", 0)))
    except (TypeError, ValueError):
        return 0
