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
