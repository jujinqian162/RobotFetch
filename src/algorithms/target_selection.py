from __future__ import annotations

from typing import Protocol, TypeVar


class HasLabelAndCx(Protocol):
    label: str
    cx: float


TargetT = TypeVar("TargetT", bound=HasLabelAndCx)


def select_status_target(
    *,
    targets: list[TargetT],
    target_x: float,
    allowed_labels: set[str],
    stable_labels: set[str],
    use_stable_labels: bool,
) -> TargetT | None:
    filtered = [t for t in targets if not allowed_labels or t.label in allowed_labels]
    if not filtered:
        return None

    if use_stable_labels and stable_labels:
        stable_filtered = [t for t in filtered if t.label in stable_labels]
        if stable_filtered:
            filtered = stable_filtered

    return min(filtered, key=lambda item: abs(item.cx - target_x))


def select_base_coord_target(
    *,
    targets: list[TargetT],
    target_x: float,
    labels: set[str],
) -> TargetT | None:
    filtered = [t for t in targets if not labels or t.label in labels]
    if not filtered:
        return None
    return min(filtered, key=lambda item: abs(item.cx - target_x))
