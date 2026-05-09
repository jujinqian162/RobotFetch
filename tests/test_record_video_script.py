import importlib.util
import sys
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "record_video.py"


def load_module():
    spec = importlib.util.spec_from_file_location("record_video", SCRIPT_PATH)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_output_path_uses_timestamped_mp4_in_cwd():
    module = load_module()

    output_path = module.default_output_path(
        now=datetime(2026, 5, 9, 21, 30, 45),
        cwd=Path("/tmp/robotfetch"),
    )

    assert output_path == Path("/tmp/robotfetch/record-20260509-213045.mp4")


def test_parse_args_rejects_non_mp4_output_path():
    module = load_module()

    try:
        module.parse_args(["record.avi"], default_device=lambda: 0)
    except ValueError as exc:
        assert "must end with .mp4" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_parse_args_accepts_device_index_and_dev_video_path():
    module = load_module()

    by_index = module.parse_args(["out.mp4", "--device", "2"], default_device=lambda: 0)
    by_path = module.parse_args(["out.mp4", "--device", "/dev/video3"], default_device=lambda: 0)

    assert by_index.device_id == 2
    assert by_path.device_id == 3


def test_format_choices_reuse_diagnose_camera_candidate_order(monkeypatch):
    module = load_module()
    modes = [
        module.CameraMode("YUYV", 640, 480, 30.0),
        module.CameraMode("MJPG", 1280, 720, 30.0),
        module.CameraMode("MJPG", 640, 480, 30.0),
    ]

    monkeypatch.setattr(module, "v4l2_formats_for_index", lambda index: (modes, ""))

    choices = module.load_format_choices(0)

    assert choices[:2] == [
        module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0),
        module.CaptureConfig("v4l2", "YUYV", 640, 480, 30.0),
    ]


def test_format_choices_do_not_add_generic_fallbacks_when_device_reports_modes(monkeypatch):
    module = load_module()
    modes = [module.CameraMode("MJPG", 640, 480, 30.0)]

    monkeypatch.setattr(module, "v4l2_formats_for_index", lambda index: (modes, ""))

    choices = module.load_format_choices(0)

    assert choices == [module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0)]


def test_menu_rows_render_cursor_and_format_as_two_columns():
    module = load_module()
    choices = [
        module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0),
        module.CaptureConfig("v4l2", "YUYV", 320, 240, 15.0),
    ]

    rows = module.build_menu_rows(choices, selected_index=1)

    assert rows == [
        (" ", "MJPG 640x480 30fps"),
        (">", "YUYV 320x240 15fps"),
    ]


def test_selection_wraps_with_up_and_down():
    module = load_module()

    assert module.move_selection(selected_index=0, delta=-1, choice_count=3) == 2
    assert module.move_selection(selected_index=2, delta=1, choice_count=3) == 0


class FakeFrame:
    shape = (480, 640, 3)


class FakeCapture:
    def __init__(self):
        self.props = []
        self.released = False
        self.reads = 0

    def set(self, prop, value):
        self.props.append((prop, value))
        return True

    def get(self, prop):
        return 0.0

    def isOpened(self):
        return True

    def read(self):
        self.reads += 1
        return True, FakeFrame()

    def release(self):
        self.released = True


class FakeWriter:
    def __init__(self, path, fourcc, fps, size):
        self.path = path
        self.fourcc = fourcc
        self.fps = fps
        self.size = size
        self.frames = []
        self.released = False

    def isOpened(self):
        return True

    def write(self, frame):
        self.frames.append(frame)

    def release(self):
        self.released = True


class FakeCv2:
    CAP_V4L2 = 200
    CAP_PROP_FOURCC = 6
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FPS = 5
    CAP_PROP_OPEN_TIMEOUT_MSEC = 53
    CAP_PROP_READ_TIMEOUT_MSEC = 54

    def __init__(self):
        self.capture = FakeCapture()
        self.writer = None

    def VideoCapture(self, device_id, backend=None):
        self.capture_args = (device_id, backend)
        return self.capture

    def VideoWriter_fourcc(self, *letters):
        return "".join(letters)

    def VideoWriter(self, path, fourcc, fps, size):
        self.writer = FakeWriter(path, fourcc, fps, size)
        return self.writer


def test_recording_session_configures_capture_and_writes_mp4_frames(tmp_path):
    module = load_module()
    fake_cv2 = FakeCv2()
    output_path = tmp_path / "record.mp4"
    session = module.RecordingSession(
        device_id=2,
        config=module.CaptureConfig("v4l2", "MJPG", 640, 480, 30.0),
        output_path=output_path,
        cv2_module=fake_cv2,
    )

    session.start()
    session.write_next_frame()
    session.stop()

    assert fake_cv2.capture_args == (2, fake_cv2.CAP_V4L2)
    assert (fake_cv2.CAP_PROP_FOURCC, "MJPG") in fake_cv2.capture.props
    assert (fake_cv2.CAP_PROP_FRAME_WIDTH, 640) in fake_cv2.capture.props
    assert (fake_cv2.CAP_PROP_FRAME_HEIGHT, 480) in fake_cv2.capture.props
    assert (fake_cv2.CAP_PROP_FPS, 30.0) in fake_cv2.capture.props
    assert fake_cv2.writer.path == str(output_path)
    assert fake_cv2.writer.fourcc == "mp4v"
    assert fake_cv2.writer.fps == 30.0
    assert fake_cv2.writer.size == (640, 480)
    assert len(fake_cv2.writer.frames) == 2
    assert fake_cv2.capture.released
    assert fake_cv2.writer.released
