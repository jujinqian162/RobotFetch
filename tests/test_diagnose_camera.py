import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "diagnose_camera.py"


def load_module():
    spec = importlib.util.spec_from_file_location("diagnose_camera", SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_v4l2_formats_extracts_fourcc_sizes_and_fps():
    module = load_module()
    text = """
        [0]: 'MJPG' (Motion-JPEG, compressed)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                        Interval: Discrete 0.067s (15.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
        [1]: 'YUYV' (YUYV 4:2:2)
                Size: Discrete 640x480
                        Interval: Discrete 0.100s (10.000 fps)
    """

    formats = module.parse_v4l2_formats(text)

    assert formats == [
        module.CameraMode("MJPG", 640, 480, 30.0),
        module.CameraMode("MJPG", 640, 480, 15.0),
        module.CameraMode("MJPG", 1280, 720, 30.0),
        module.CameraMode("YUYV", 640, 480, 10.0),
    ]


def test_candidate_modes_prefer_supported_mjpg_low_resolution_then_yuyv():
    module = load_module()
    supported = [
        module.CameraMode("YUYV", 640, 480, 30.0),
        module.CameraMode("MJPG", 1280, 720, 30.0),
        module.CameraMode("MJPG", 640, 480, 30.0),
        module.CameraMode("MJPG", 640, 480, 15.0),
    ]

    candidates = module.build_candidate_modes(supported)

    assert candidates[:3] == [
        module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0),
        module.CaptureConfig("v4l2", "MJPG", 640, 480, 15.0),
        module.CaptureConfig("v4l2", "YUYV", 640, 480, 30.0),
    ]


def test_recommendation_reports_working_config_and_select_timeout_context():
    module = load_module()
    results = [
        module.ProbeResult(index=0, config=module.DEFAULT_CONFIG, opened=True, read_ok=False, frame_shape=None, elapsed_sec=10.0, error=None),
        module.ProbeResult(index=0, config=module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0), opened=True, read_ok=True, frame_shape=(480, 640, 3), elapsed_sec=0.2, error=None),
    ]

    recommendation = module.build_recommendation(results)

    assert "source 0" in recommendation
    assert "backend=v4l2 fourcc=MJPG width=640 height=480 fps=30" in recommendation
    assert "default capture opened but read failed" in recommendation
    assert "select timeout" in recommendation
