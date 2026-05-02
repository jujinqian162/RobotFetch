from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BASEDETECT_ROOT = PROJECT_ROOT / "BaseDetect"
if str(BASEDETECT_ROOT) not in sys.path:
    sys.path.insert(0, str(BASEDETECT_ROOT))


def test_detector_disables_ultralytics_verbose_output(monkeypatch, tmp_path):
    from sdk import detector as detector_module

    track_calls: list[dict[str, object]] = []

    class FakeYOLO:
        names = {0: "palm"}

        def __init__(self, weights: str) -> None:
            self.weights = weights

        def track(self, *args, **kwargs):
            track_calls.append(kwargs)
            return [SimpleNamespace(boxes=None, names=self.names)]

    monkeypatch.setattr(detector_module, "YOLO", FakeYOLO)
    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )

    config_path = tmp_path / "basedetect_sdk.yaml"
    config_path.write_text(
        yaml.safe_dump(
            {
                "runtime": {
                    "device": "auto",
                    "queue_size": 1,
                    "warmup_frames": 1,
                    "debug": False,
                    "grayscale_input": False,
                },
                "active_profile": "status_test",
                "profiles": {
                    "status_test": {
                        "mode": "status",
                        "weights": "fake-model",
                        "conf": 0.25,
                        "vote_threshold": 1,
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    detector = detector_module.Detector(config=config_path, profile="status_test")
    detector.detect(np.zeros((4, 4, 3), dtype=np.uint8))

    assert track_calls
    assert track_calls[0]["verbose"] is False


def test_detector_exports_debug_overlay_video_when_enabled(monkeypatch, tmp_path):
    from sdk import detector as detector_module

    written_frames: list[np.ndarray] = []
    writer_instances: list[object] = []

    class FakeVideoWriter:
        def __init__(self, path: str, fourcc: int, fps: float, size: tuple[int, int]):
            self.path = path
            self.fourcc = fourcc
            self.fps = fps
            self.size = size
            self.released = False
            writer_instances.append(self)

        def write(self, frame: np.ndarray) -> None:
            written_frames.append(frame.copy())

        def release(self) -> None:
            self.released = True

    class FakeResult:
        boxes = None
        names = {0: "palm"}

        def plot(self) -> np.ndarray:
            return np.zeros((6, 8, 3), dtype=np.uint8)

    class FakeYOLO:
        names = {0: "palm"}

        def __init__(self, weights: str) -> None:
            self.weights = weights

        def track(self, *args, **kwargs):
            return [FakeResult()]

    monkeypatch.setattr(detector_module, "YOLO", FakeYOLO)
    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )
    monkeypatch.setattr(detector_module.cv2, "VideoWriter", FakeVideoWriter)
    monkeypatch.setattr(detector_module.cv2, "VideoWriter_fourcc", lambda *args: 1234)

    config_path = tmp_path / "basedetect_sdk.yaml"
    export_path = tmp_path / "debug" / "basedetect.mp4"
    config_path.write_text(
        yaml.safe_dump(
            {
                "runtime": {
                    "device": "auto",
                    "queue_size": 1,
                    "warmup_frames": 1,
                    "debug": False,
                    "grayscale_input": False,
                },
                "active_profile": "status_test",
                "profiles": {
                    "status_test": {
                        "mode": "status",
                        "weights": "fake-model",
                        "conf": 0.25,
                        "vote_threshold": 1,
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    detector = detector_module.Detector(
        config=config_path,
        profile="status_test",
        debug=True,
        export_video=export_path,
    )
    detector.detect(np.zeros((4, 4, 3), dtype=np.uint8))
    detector.release()

    assert len(writer_instances) == 1
    writer = writer_instances[0]
    assert writer.path == str(export_path)
    assert writer.fourcc == 1234
    assert writer.fps == 30.0
    assert writer.size == (8, 6)
    assert len(written_frames) == 1
    assert writer.released is True


def test_detector_does_not_export_debug_video_when_debug_disabled(
    monkeypatch,
    tmp_path,
):
    from sdk import detector as detector_module

    writer_instances: list[object] = []

    class FakeVideoWriter:
        def __init__(self, *args, **kwargs):
            writer_instances.append(self)

        def write(self, frame: np.ndarray) -> None:
            raise AssertionError("debug video writer must not receive frames")

        def release(self) -> None:
            return None

    class FakeResult:
        boxes = None
        names = {0: "palm"}

        def plot(self) -> np.ndarray:
            raise AssertionError("debug overlay must not be rendered when disabled")

    class FakeYOLO:
        names = {0: "palm"}

        def __init__(self, weights: str) -> None:
            self.weights = weights

        def track(self, *args, **kwargs):
            return [FakeResult()]

    monkeypatch.setattr(detector_module, "YOLO", FakeYOLO)
    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )
    monkeypatch.setattr(detector_module.cv2, "VideoWriter", FakeVideoWriter)

    config_path = tmp_path / "basedetect_sdk.yaml"
    export_path = tmp_path / "debug" / "basedetect.mp4"
    config_path.write_text(
        yaml.safe_dump(
            {
                "runtime": {
                    "device": "auto",
                    "queue_size": 1,
                    "warmup_frames": 1,
                    "debug": True,
                    "grayscale_input": False,
                },
                "active_profile": "status_test",
                "profiles": {
                    "status_test": {
                        "mode": "status",
                        "weights": "fake-model",
                        "conf": 0.25,
                        "vote_threshold": 1,
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    detector = detector_module.Detector(
        config=config_path,
        profile="status_test",
        debug=False,
        export_video=export_path,
    )
    detector.detect(np.zeros((4, 4, 3), dtype=np.uint8))
    detector.release()

    assert writer_instances == []
    assert not export_path.parent.exists()
    assert detector.debug_overlay() is None
    assert detector.debug_info() == {}


def test_detector_disables_debug_export_after_writer_open_failure(
    monkeypatch,
    tmp_path,
):
    from sdk import detector as detector_module

    writer_instances: list[object] = []

    class FakeVideoWriter:
        def __init__(self, *args, **kwargs):
            writer_instances.append(self)
            self.released = False

        def isOpened(self) -> bool:
            return False

        def write(self, frame: np.ndarray) -> None:
            raise AssertionError("closed writer must not receive frames")

        def release(self) -> None:
            self.released = True

    class FakeResult:
        boxes = None
        names = {0: "palm"}

        def plot(self) -> np.ndarray:
            return np.zeros((6, 8, 3), dtype=np.uint8)

    class FakeYOLO:
        names = {0: "palm"}

        def __init__(self, weights: str) -> None:
            self.weights = weights

        def track(self, *args, **kwargs):
            return [FakeResult()]

    monkeypatch.setattr(detector_module, "YOLO", FakeYOLO)
    monkeypatch.setattr(
        detector_module.torch.cuda,
        "is_available",
        lambda: False,
    )
    monkeypatch.setattr(detector_module.cv2, "VideoWriter", FakeVideoWriter)
    monkeypatch.setattr(detector_module.cv2, "VideoWriter_fourcc", lambda *args: 1234)

    config_path = tmp_path / "basedetect_sdk.yaml"
    export_path = tmp_path / "debug" / "basedetect.mp4"
    config_path.write_text(
        yaml.safe_dump(
            {
                "runtime": {
                    "device": "auto",
                    "queue_size": 1,
                    "warmup_frames": 1,
                    "debug": False,
                    "grayscale_input": False,
                },
                "active_profile": "status_test",
                "profiles": {
                    "status_test": {
                        "mode": "status",
                        "weights": "fake-model",
                        "conf": 0.25,
                        "vote_threshold": 1,
                    }
                },
            }
        ),
        encoding="utf-8",
    )

    detector = detector_module.Detector(
        config=config_path,
        profile="status_test",
        debug=True,
        export_video=export_path,
    )

    assert detector.detect(np.zeros((4, 4, 3), dtype=np.uint8)) == []
    assert detector.detect(np.zeros((4, 4, 3), dtype=np.uint8)) == []
    detector.release()

    assert len(writer_instances) == 1
    assert writer_instances[0].released is True
