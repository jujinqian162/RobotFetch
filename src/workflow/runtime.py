from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any, Callable

from algorithms.detector_gateway import DetectorGateway
from config.models import CameraFallbackConfig, PidAlignmentWorkflowConfig


CaptureFactory = Callable[[str], Any]
DetectorGatewayFactory = Callable[..., Any]
ReadableFallbackFactory = Callable[
    [str, tuple[CameraFallbackConfig, ...]],
    tuple[Any, Any, CameraFallbackConfig],
]


@dataclass(frozen=True)
class FrameReadResult:
    ok: bool
    frame: Any = None


@dataclass(frozen=True)
class WorkflowPublishers:
    cmd_pub: Any
    phase_pub: Any
    algo_status_pub: Any
    env_status_pub: Any
    selected_target_pub: Any
    base_coord_pub: Any | None = None


@dataclass(frozen=True)
class WorkflowContext:
    cfg: PidAlignmentWorkflowConfig
    resources: "WorkflowResources"
    publishers: WorkflowPublishers
    logger: Any
    clock: Any
    adapter: Any | None = None
    heartbeat_stats: Callable[[], Any] | None = None

    def now_s(self) -> float:
        now = self.clock.now()
        return float(now.nanoseconds) * 1e-9

    def consume_heartbeat_stats(self) -> Any:
        if self.heartbeat_stats is None:
            return SimpleNamespace(filled_heartbeat_frames=0)
        return self.heartbeat_stats()


class WorkflowResources:
    def __init__(
        self,
        *,
        cfg: PidAlignmentWorkflowConfig,
        logger: Any,
        capture_factory: CaptureFactory = None,
        detector_gateway_factory: DetectorGatewayFactory = None,
        readable_fallback_factory: ReadableFallbackFactory = None,
    ) -> None:
        self._cfg = cfg
        self._logger = logger
        self._capture_factory = capture_factory or build_capture
        self._detector_gateway_factory = detector_gateway_factory or DetectorGateway
        self._readable_fallback_factory = (
            readable_fallback_factory or build_readable_v4l2_mjpg_capture
        )
        self._vision: VisionSession | None = None

    def vision(self) -> "VisionSession":
        if self._vision is None:
            self._vision = VisionSession(
                cfg=self._cfg,
                logger=self._logger,
                capture_factory=self._capture_factory,
                detector_gateway_factory=self._detector_gateway_factory,
                readable_fallback_factory=self._readable_fallback_factory,
            )
        return self._vision

    def release_all(self) -> None:
        if self._vision is not None:
            self._vision.release()


class VisionSession:
    def __init__(
        self,
        *,
        cfg: PidAlignmentWorkflowConfig,
        logger: Any,
        capture_factory: CaptureFactory = None,
        detector_gateway_factory: DetectorGatewayFactory = None,
        readable_fallback_factory: ReadableFallbackFactory = None,
    ) -> None:
        self._cfg = cfg
        self._logger = logger
        self._capture_factory = capture_factory or build_capture
        self._readable_fallback_factory = (
            readable_fallback_factory or build_readable_v4l2_mjpg_capture
        )
        gateway_factory = detector_gateway_factory or DetectorGateway
        self._capture = self._capture_factory(cfg.detector.input_source)
        _log(
            self._logger,
            "info",
            _build_capture_log_message(cfg.detector.input_source, self._capture),
        )
        try:
            debug_cfg = getattr(cfg, "debug", None)
            debug_enabled = bool(getattr(debug_cfg, "enable", False))
            export_debug_video_path = (
                getattr(debug_cfg, "export_basedetect_video", None)
                if debug_enabled
                else None
            )
            self._detector_gateway = gateway_factory(
                config_path=cfg.detector.sdk_config,
                initial_profile=None,
                debug=debug_enabled,
                export_debug_video_path=export_debug_video_path,
            )
        except Exception:
            release = getattr(self._capture, "release", None)
            if callable(release):
                release()
            raise
        self._active_profile = getattr(self._detector_gateway, "profile_name", None)
        self._capture_read_fallback_attempted = False
        self._released = False

    @property
    def capture(self) -> Any:
        return self._capture

    @property
    def detector_gateway(self) -> Any:
        return self._detector_gateway

    def ensure_profile(self, profile: str | None) -> None:
        if profile == self._active_profile:
            return
        self._detector_gateway.switch_profile(profile)
        self._active_profile = profile

    def read_frame(self) -> FrameReadResult:
        ok, frame = self._capture.read()
        if ok:
            return FrameReadResult(ok=True, frame=frame)
        ok, frame = self._try_read_with_capture_fallback()
        return FrameReadResult(ok=ok, frame=frame)

    def detect_status_targets(self, frame: Any) -> Any:
        return self._detector_gateway.detect_status_targets(frame)

    def detect_base_coord_targets(self, frame: Any) -> Any:
        return self._detector_gateway.detect_base_coord_targets(frame)

    def release(self) -> None:
        if self._released:
            return
        try:
            detector_release = getattr(self._detector_gateway, "release", None)
            if callable(detector_release):
                detector_release()
            else:
                detector_close = getattr(self._detector_gateway, "close", None)
                if callable(detector_close):
                    detector_close()
        finally:
            release = getattr(self._capture, "release", None)
            if callable(release):
                release()
            self._released = True

    def _try_read_with_capture_fallback(self) -> tuple[bool, Any]:
        input_source = self._cfg.detector.input_source
        if self._capture_read_fallback_attempted:
            return False, None
        if not isinstance(_resolve_input_source(input_source), int):
            return False, None

        self._capture_read_fallback_attempted = True
        release = getattr(self._capture, "release", None)
        if callable(release):
            release()

        for config in self._cfg.detector.camera_fallbacks:
            _log(
                self._logger,
                "warning",
                _format_camera_fallback_log(
                    "Frame read failed; trying camera fallback",
                    input_source=input_source,
                    config=config,
                ),
            )
            try:
                self._capture, frame, selected_config = self._readable_fallback_factory(
                    input_source,
                    (config,),
                )
            except RuntimeError:
                continue

            _log(
                self._logger,
                "info",
                _format_multiline_log(
                    "Camera fallback selected",
                    (
                        ("source", input_source),
                        ("backend", selected_config.backend),
                        ("fourcc", selected_config.fourcc),
                        ("width", selected_config.width),
                        ("height", selected_config.height),
                        ("fps", f"{selected_config.fps:g}"),
                        ("shape", getattr(frame, "shape", None)),
                    ),
                ),
            )
            return True, frame

        _log(
            self._logger,
            "warning",
            f"All camera fallback configs failed for source={input_source}",
        )
        return False, None


def build_capture(input_source: str) -> Any:
    import cv2

    source = _resolve_input_source(input_source)
    capture = cv2.VideoCapture(source)
    if hasattr(capture, "isOpened") and not capture.isOpened():
        raise RuntimeError(f"Unable to open source: {input_source}")
    return capture


def build_v4l2_mjpg_capture(input_source: str, config: CameraFallbackConfig) -> Any:
    import cv2

    source = _resolve_input_source(input_source)
    if not isinstance(source, int):
        raise RuntimeError(f"V4L2 MJPG fallback requires numeric source: {input_source}")
    if config.backend != "v4l2":
        raise ValueError(f"Unsupported camera fallback backend: {config.backend}")

    capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
    capture.set(cv2.CAP_PROP_FPS, config.fps)
    if hasattr(capture, "isOpened") and not capture.isOpened():
        release = getattr(capture, "release", None)
        if callable(release):
            release()
        raise RuntimeError(
            "Unable to open source with camera fallback: "
            f"{input_source} {config.describe()}"
        )
    return capture


def build_readable_v4l2_mjpg_capture(
    input_source: str,
    configs: tuple[CameraFallbackConfig, ...],
) -> tuple[Any, Any, CameraFallbackConfig]:
    last_error: Exception | None = None
    for config in configs:
        capture = None
        try:
            capture = build_v4l2_mjpg_capture(input_source, config)
            ok, frame = capture.read()
            if ok:
                return capture, frame, config
        except Exception as exc:
            last_error = exc
        if capture is not None:
            release = getattr(capture, "release", None)
            if callable(release):
                release()

    message = f"Unable to read from camera fallback source: {input_source}"
    if last_error is not None:
        message = f"{message}; last_error={last_error!r}"
    raise RuntimeError(message)


def _resolve_input_source(input_source: str) -> str | int:
    if input_source.isdigit():
        return int(input_source)
    return input_source


def _build_capture_log_message(input_source: str, capture: Any) -> str:
    import cv2

    source_type = "camera" if isinstance(_resolve_input_source(input_source), int) else "video"
    width = _capture_int_property(capture, cv2.CAP_PROP_FRAME_WIDTH)
    height = _capture_int_property(capture, cv2.CAP_PROP_FRAME_HEIGHT)
    fps = _capture_float_property(capture, cv2.CAP_PROP_FPS)
    frame_count = _capture_int_property(capture, cv2.CAP_PROP_FRAME_COUNT)
    fourcc = _decode_fourcc(_capture_int_property(capture, cv2.CAP_PROP_FOURCC))
    frame_count_text = "unknown" if frame_count <= 0 else str(frame_count)

    return _format_multiline_log(
        "capture opened",
        (
            ("source", input_source),
            ("source_type", source_type),
            ("width", width),
            ("height", height),
            ("fps", f"{fps:.3f}"),
            ("frame_count", frame_count_text),
            ("fourcc", fourcc),
        ),
    )


def _capture_float_property(capture: Any, prop_id: int) -> float:
    getter = getattr(capture, "get", None)
    if not callable(getter):
        return 0.0
    try:
        return float(getter(prop_id))
    except (TypeError, ValueError):
        return 0.0


def _capture_int_property(capture: Any, prop_id: int) -> int:
    return int(round(_capture_float_property(capture, prop_id)))


def _decode_fourcc(value: int) -> str:
    if value <= 0:
        return "unknown"
    chars = [chr((value >> (8 * index)) & 0xFF) for index in range(4)]
    if not all(char.isprintable() and char.strip() for char in chars):
        return str(value)
    return "".join(chars)


def _format_camera_fallback_log(
    title: str,
    *,
    input_source: str,
    config: CameraFallbackConfig,
) -> str:
    return _format_multiline_log(
        title,
        (
            ("source", input_source),
            ("backend", config.backend),
            ("fourcc", config.fourcc),
            ("width", config.width),
            ("height", config.height),
            ("fps", f"{config.fps:g}"),
        ),
    )


def _format_multiline_log(
    title: str,
    fields: tuple[tuple[str, object], ...],
) -> str:
    return "\n".join([title, *(f"  {key}={value}" for key, value in fields)])


def _log(logger: Any, level: str, message: str) -> None:
    sink = getattr(logger, level, None)
    if callable(sink):
        sink(message)
