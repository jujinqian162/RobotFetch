from dataclasses import dataclass

from algorithms.target_selection import select_base_coord_target, select_status_target


@dataclass
class FakeStatusTarget:
    label: str
    cx: float


@dataclass
class FakeBaseCoordTarget:
    label: str
    cx: float


def test_select_status_target_uses_stable_labels_when_available():
    targets = [
        FakeStatusTarget(label="fist", cx=100.0),
        FakeStatusTarget(label="palm", cx=320.0),
    ]

    selected = select_status_target(
        targets=targets,
        target_x=300.0,
        allowed_labels={"fist", "palm"},
        stable_labels={"palm"},
        use_stable_labels=True,
    )

    assert selected is targets[1]


def test_select_status_target_returns_none_when_no_allowed_labels_match():
    targets = [FakeStatusTarget(label="other", cx=300.0)]

    selected = select_status_target(
        targets=targets,
        target_x=300.0,
        allowed_labels={"fist"},
        stable_labels=set(),
        use_stable_labels=False,
    )

    assert selected is None


def test_select_base_coord_target_picks_nearest_matching_label():
    targets = [
        FakeBaseCoordTarget(label="base", cx=350.0),
        FakeBaseCoordTarget(label="base", cx=305.0),
    ]

    selected = select_base_coord_target(
        targets=targets,
        target_x=300.0,
        labels={"base"},
    )

    assert selected is targets[1]
