from dataclasses import dataclass
from pathlib import Path

from algorithms.detector_gateway import DetectionBatch, DetectorGateway


@dataclass(frozen=True)
class StatusTarget:
    id: int | None
    label: str
    conf: float
    cx: float
    cy: float
    width: float
    height: float


@dataclass(frozen=True)
class BaseCoordTarget:
    id: int | None
    label: str
    conf: float
    cx: float
    cy: float
    width: float
    height: float
    x: float
    y: float
    z: float


class FakeDetector:
    def __init__(self, *, config: str | Path, profile: str | None = None) -> None:
        self.config = config
        self.profile = profile
        self.ready = False
        self.detect_calls: list[object] = []
        self.switch_calls: list[str | None] = []
        self.status_targets: list[StatusTarget] = []
        self.base_coord_targets: list[BaseCoordTarget] = []

    def switch_profile(self, profile: str | None) -> None:
        self.switch_calls.append(profile)
        self.profile = profile

    def detect(self, frame: object) -> list[object]:
        self.detect_calls.append(frame)
        return []

    def latest_status_targets(self) -> list[StatusTarget]:
        return list(self.status_targets)

    def latest_base_coord_targets(self) -> list[BaseCoordTarget]:
        return list(self.base_coord_targets)


class FakeFrame:
    pass


def test_detector_gateway_builds_detector_with_initial_profile():
    gateway = DetectorGateway(
        config_path=Path("fake.yaml"),
        initial_profile="status_competition",
        detector_factory=FakeDetector,
    )

    assert gateway.profile_name == "status_competition"



def test_detector_gateway_detect_status_targets_returns_ready_and_targets():
    gateway = DetectorGateway(
        config_path=Path("fake.yaml"),
        initial_profile="status_competition",
        detector_factory=FakeDetector,
    )
    frame = FakeFrame()
    target = StatusTarget(
        id=1,
        label="palm",
        conf=0.9,
        cx=320.0,
        cy=120.0,
        width=40.0,
        height=50.0,
    )
    gateway._detector.ready = True
    gateway._detector.status_targets = [target]

    batch = gateway.detect_status_targets(frame)

    assert batch == DetectionBatch(ready=True, targets=[target])
    assert gateway._detector.detect_calls == [frame]



def test_detector_gateway_detect_base_coord_targets_returns_ready_and_targets():
    gateway = DetectorGateway(
        config_path=Path("fake.yaml"),
        initial_profile="base_coord_competition",
        detector_factory=FakeDetector,
    )
    frame = FakeFrame()
    target = BaseCoordTarget(
        id=2,
        label="base",
        conf=0.8,
        cx=280.0,
        cy=100.0,
        width=30.0,
        height=45.0,
        x=1.0,
        y=2.0,
        z=3.0,
    )
    gateway._detector.ready = False
    gateway._detector.base_coord_targets = [target]

    batch = gateway.detect_base_coord_targets(frame)

    assert batch == DetectionBatch(ready=False, targets=[target])
    assert gateway._detector.detect_calls == [frame]



def test_detector_gateway_switch_profile_forwards_to_detector():
    gateway = DetectorGateway(
        config_path=Path("fake.yaml"),
        initial_profile="status_competition",
        detector_factory=FakeDetector,
    )

    gateway.switch_profile("base_coord_competition")

    assert gateway.profile_name == "base_coord_competition"
    assert gateway._detector.switch_calls == ["base_coord_competition"]
