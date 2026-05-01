from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

from config.models import CameraFallbackConfig
from workflow.runtime import VisionSession, WorkflowResources


@dataclass
class FakeCapture:
    reads: list[object]
    released: bool = False

    def read(self):
        if not self.reads:
            return False, None
        return self.reads.pop(0)

    def release(self) -> None:
        self.released = True


class FakeDetectorGateway:
    def __init__(self, *, config_path: Path, initial_profile: str | None) -> None:
        self.config_path = config_path
        self.profile_name = initial_profile
        self.switch_calls: list[str | None] = []

    def switch_profile(self, profile: str | None) -> None:
        self.switch_calls.append(profile)
        self.profile_name = profile


def make_cfg() -> SimpleNamespace:
    return SimpleNamespace(
        detector=SimpleNamespace(
            sdk_config=Path("BaseDetect/configs/basedetect_sdk.yaml"),
            input_source="0",
            camera_fallbacks=(),
        )
    )


def test_workflow_resources_creates_vision_session_lazily_and_reuses_it():
    capture = FakeCapture(reads=[])
    capture_calls: list[str] = []
    detector_calls: list[tuple[Path, str | None]] = []

    def capture_factory(input_source: str):
        capture_calls.append(input_source)
        return capture

    def detector_gateway_factory(*, config_path: Path, initial_profile: str | None):
        detector_calls.append((config_path, initial_profile))
        return FakeDetectorGateway(
            config_path=config_path,
            initial_profile=initial_profile,
        )

    resources = WorkflowResources(
        cfg=make_cfg(),
        logger=SimpleNamespace(info=lambda message: None, warning=lambda message: None),
        capture_factory=capture_factory,
        detector_gateway_factory=detector_gateway_factory,
    )

    assert capture_calls == []
    assert detector_calls == []

    first = resources.vision()
    second = resources.vision()
    resources.release_all()

    assert isinstance(first, VisionSession)
    assert first is second
    assert capture_calls == ["0"]
    assert detector_calls == [
        (Path("BaseDetect/configs/basedetect_sdk.yaml"), None)
    ]
    assert capture.released is True


def test_vision_session_switches_profiles_only_when_needed():
    capture = FakeCapture(reads=[])
    gateway = FakeDetectorGateway(
        config_path=Path("BaseDetect/configs/basedetect_sdk.yaml"),
        initial_profile=None,
    )
    session = VisionSession(
        cfg=make_cfg(),
        logger=SimpleNamespace(info=lambda message: None, warning=lambda message: None),
        capture_factory=lambda input_source: capture,
        detector_gateway_factory=lambda **kwargs: gateway,
    )

    session.ensure_profile("status_competition")
    session.ensure_profile("status_competition")
    session.ensure_profile("base_coord_competition")

    assert gateway.switch_calls == [
        "status_competition",
        "base_coord_competition",
    ]
    assert gateway.profile_name == "base_coord_competition"


def test_vision_session_retries_numeric_camera_with_configured_fallbacks():
    initial_capture = FakeCapture(reads=[(False, None)])
    fallback_capture = FakeCapture(reads=[])
    fallback_calls: list[tuple[str, tuple[CameraFallbackConfig, ...]]] = []
    info_messages: list[str] = []
    warning_messages: list[str] = []
    selected_frame = SimpleNamespace(shape=(600, 800, 3))
    cfg = SimpleNamespace(
        detector=SimpleNamespace(
            sdk_config=Path("BaseDetect/configs/basedetect_sdk.yaml"),
            input_source="0",
            camera_fallbacks=(
                CameraFallbackConfig("v4l2", "MJPG", 1024, 768, 190.0),
                CameraFallbackConfig("v4l2", "MJPG", 800, 600, 190.0),
            ),
        )
    )

    def readable_fallback_factory(
        input_source: str,
        configs: tuple[CameraFallbackConfig, ...],
    ):
        fallback_calls.append((input_source, configs))
        config = configs[0]
        if config.width == 1024:
            raise RuntimeError("first fallback failed")
        return fallback_capture, selected_frame, config

    session = VisionSession(
        cfg=cfg,
        logger=SimpleNamespace(
            info=info_messages.append,
            warning=warning_messages.append,
        ),
        capture_factory=lambda input_source: initial_capture,
        detector_gateway_factory=lambda **kwargs: FakeDetectorGateway(**kwargs),
        readable_fallback_factory=readable_fallback_factory,
    )

    result = session.read_frame()

    assert result.ok is True
    assert result.frame is selected_frame
    assert initial_capture.released is True
    assert session.capture is fallback_capture
    assert fallback_calls == [
        ("0", (CameraFallbackConfig("v4l2", "MJPG", 1024, 768, 190.0),)),
        ("0", (CameraFallbackConfig("v4l2", "MJPG", 800, 600, 190.0),)),
    ]
    assert warning_messages == [
        (
            "Frame read failed; trying camera fallback\n"
            "  source=0\n"
            "  backend=v4l2\n"
            "  fourcc=MJPG\n"
            "  width=1024\n"
            "  height=768\n"
            "  fps=190"
        ),
        (
            "Frame read failed; trying camera fallback\n"
            "  source=0\n"
            "  backend=v4l2\n"
            "  fourcc=MJPG\n"
            "  width=800\n"
            "  height=600\n"
            "  fps=190"
        ),
    ]
    assert info_messages == [
        (
            "capture opened\n"
            "  source=0\n"
            "  source_type=camera\n"
            "  width=0\n"
            "  height=0\n"
            "  fps=0.000\n"
            "  frame_count=unknown\n"
            "  fourcc=unknown"
        ),
        (
            "Camera fallback selected\n"
            "  source=0\n"
            "  backend=v4l2\n"
            "  fourcc=MJPG\n"
            "  width=800\n"
            "  height=600\n"
            "  fps=190\n"
            "  shape=(600, 800, 3)"
        )
    ]
