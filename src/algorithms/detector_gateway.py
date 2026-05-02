from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Generic, TypeVar

PROJECT_ROOT = Path(__file__).resolve().parents[5]
BASEDETECT_ROOT = PROJECT_ROOT / "BaseDetect"

import sys

if str(BASEDETECT_ROOT) not in sys.path:
    sys.path.insert(0, str(BASEDETECT_ROOT))

TargetT = TypeVar("TargetT")
DetectorFactory = Callable[..., Any]


@dataclass(frozen=True)
class DetectionBatch(Generic[TargetT]):
    ready: bool
    targets: list[TargetT]


class DetectorGateway:
    def __init__(
        self,
        *,
        config_path: Path,
        initial_profile: str | None = None,
        detector_factory: DetectorFactory | None = None,
        debug: bool | None = None,
        export_debug_video_path: str | Path | None = None,
    ) -> None:
        factory = detector_factory or _default_detector_factory
        self._profile_name = initial_profile
        self._detector = factory(
            config=config_path,
            profile=initial_profile,
            debug=debug,
            export_video=export_debug_video_path,
        )

    @property
    def profile_name(self) -> str | None:
        return self._profile_name

    def switch_profile(self, profile: str | None) -> None:
        self._detector.switch_profile(profile)
        self._profile_name = profile

    def detect_status_targets(self, frame: Any) -> DetectionBatch[Any]:
        self._detector.detect(frame)
        return DetectionBatch(
            ready=self._detector.ready,
            targets=self._detector.latest_status_targets(),
        )

    def detect_base_coord_targets(self, frame: Any) -> DetectionBatch[Any]:
        self._detector.detect(frame)
        return DetectionBatch(
            ready=self._detector.ready,
            targets=self._detector.latest_base_coord_targets(),
        )

    def release(self) -> None:
        release = getattr(self._detector, "release", None)
        if callable(release):
            release()
            return
        close = getattr(self._detector, "close", None)
        if callable(close):
            close()


def _default_detector_factory(
    *,
    config: Path,
    profile: str | None,
    debug: bool | None = None,
    export_video: str | Path | None = None,
) -> Any:
    from sdk import Detector

    return Detector(
        config=config,
        profile=profile,
        debug=debug,
        export_video=export_video,
    )
