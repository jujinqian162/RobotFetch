#!/usr/bin/env python3
"""Interactively choose a camera format and record an MP4 video."""

from __future__ import annotations

import argparse
import curses
import math
import re
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from diagnose_camera import (  # noqa: E402
    CameraMode,
    CaptureConfig,
    build_candidate_modes,
    discover_video_indices,
    suppress_native_stderr,
    v4l2_formats_for_index,
)


DEFAULT_RECORD_FPS = 30.0
OUTPUT_FOURCC = "mp4v"


@dataclass(frozen=True)
class RecordArgs:
    output_path: Path
    device_id: int


def default_output_path(
    *,
    now: datetime | None = None,
    cwd: Path | None = None,
) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d-%H%M%S")
    return (cwd or Path.cwd()) / f"record-{timestamp}.mp4"


def parse_args(
    argv: Sequence[str] | None = None,
    *,
    default_device: Callable[[], int] | None = None,
    now: datetime | None = None,
    cwd: Path | None = None,
) -> RecordArgs:
    parser = argparse.ArgumentParser(
        description="Choose a V4L2 camera format and record an MP4 video."
    )
    parser.add_argument(
        "output_path",
        nargs="?",
        help="Video output path. Defaults to record-YYYYMMDD-HHMMSS.mp4.",
    )
    parser.add_argument(
        "--device",
        default=None,
        metavar="DEVICE_ID",
        help="Camera index or /dev/videoN path. Defaults to the first discovered index, or 0.",
    )
    namespace = parser.parse_args(argv)

    output_path = (
        default_output_path(now=now, cwd=cwd)
        if namespace.output_path is None
        else Path(namespace.output_path)
    )
    if output_path.suffix.lower() != ".mp4":
        raise ValueError(f"output path must end with .mp4: {output_path}")

    device_id = (
        (default_device or choose_default_device)()
        if namespace.device is None
        else parse_device_id(namespace.device)
    )
    return RecordArgs(output_path=output_path, device_id=device_id)


def choose_default_device() -> int:
    indices = discover_video_indices()
    return indices[0] if indices else 0


def parse_device_id(value: str | int) -> int:
    if isinstance(value, int):
        device_id = value
    else:
        text = value.strip()
        video_match = re.fullmatch(r"/dev/video(\d+)", text)
        if video_match:
            device_id = int(video_match.group(1))
        else:
            try:
                device_id = int(text)
            except ValueError as exc:
                raise ValueError(
                    f"--device must be an integer index or /dev/videoN path: {value!r}"
                ) from exc
    if device_id < 0:
        raise ValueError(f"--device must be non-negative: {value!r}")
    return device_id


def load_format_choices(device_id: int) -> list[CaptureConfig]:
    modes, _error = v4l2_formats_for_index(device_id)
    candidates = build_candidate_modes(modes)
    if not modes:
        return candidates

    reported = {capture_config_for_mode(mode) for mode in modes}
    return [candidate for candidate in candidates if candidate in reported]


def capture_config_for_mode(mode: CameraMode) -> CaptureConfig:
    return CaptureConfig(
        backend="v4l2",
        fourcc=mode.fourcc,
        width=mode.width,
        height=mode.height,
        fps=mode.fps,
    )


def format_label(config: CaptureConfig) -> str:
    parts: list[str] = []
    if config.fourcc:
        parts.append(config.fourcc)
    if config.width and config.height:
        parts.append(f"{config.width}x{config.height}")
    if config.fps:
        parts.append(f"{config.fps:g}fps")
    return " ".join(parts) if parts else config.describe()


def build_menu_rows(
    choices: Sequence[CaptureConfig],
    selected_index: int,
) -> list[tuple[str, str]]:
    return [
        (">" if index == selected_index else " ", format_label(choice))
        for index, choice in enumerate(choices)
    ]


def move_selection(selected_index: int, delta: int, choice_count: int) -> int:
    if choice_count <= 0:
        raise ValueError("choice_count must be greater than 0")
    return (selected_index + delta) % choice_count


@dataclass
class RecordingSession:
    device_id: int
    config: CaptureConfig
    output_path: Path
    cv2_module: object | None = None
    capture: object | None = field(init=False, default=None)
    writer: object | None = field(init=False, default=None)
    frame_count: int = field(init=False, default=0)
    started_at: float | None = field(init=False, default=None)

    def start(self) -> None:
        cv2_module = self._cv2()
        capture = open_capture(cv2_module, self.device_id, self.config)
        writer = None
        try:
            if not _is_opened(capture):
                raise RuntimeError(f"failed to open /dev/video{self.device_id}")

            ok, first_frame = capture.read()
            if not ok or first_frame is None:
                raise RuntimeError(
                    f"failed to read first frame from /dev/video{self.device_id}"
                )

            height, width = first_frame.shape[:2]
            fps = choose_writer_fps(cv2_module, capture, self.config)
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2_module.VideoWriter(
                str(self.output_path),
                cv2_module.VideoWriter_fourcc(*OUTPUT_FOURCC),
                fps,
                (width, height),
            )
            if hasattr(writer, "isOpened") and not writer.isOpened():
                raise RuntimeError(f"failed to open MP4 writer for {self.output_path}")

            writer.write(first_frame)
            self.capture = capture
            self.writer = writer
            self.frame_count = 1
            self.started_at = time.monotonic()
        except Exception:
            release_quietly(writer)
            release_quietly(capture)
            raise

    def write_next_frame(self) -> bool:
        if self.capture is None or self.writer is None:
            raise RuntimeError("recording session is not started")
        ok, frame = self.capture.read()
        if not ok or frame is None:
            return False
        self.writer.write(frame)
        self.frame_count += 1
        return True

    def stop(self) -> None:
        release_quietly(self.writer)
        release_quietly(self.capture)
        self.writer = None
        self.capture = None

    def elapsed_sec(self) -> float:
        if self.started_at is None:
            return 0.0
        return time.monotonic() - self.started_at

    def _cv2(self):
        if self.cv2_module is not None:
            return self.cv2_module
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError(
                "failed to import cv2. Activate the project environment first."
            ) from exc
        return cv2


def open_capture(cv2_module, device_id: int, config: CaptureConfig):
    with suppress_native_stderr():
        if config.backend == "v4l2" and hasattr(cv2_module, "CAP_V4L2"):
            capture = cv2_module.VideoCapture(device_id, cv2_module.CAP_V4L2)
        else:
            capture = cv2_module.VideoCapture(device_id)

        set_capture_prop(cv2_module, capture, "CAP_PROP_OPEN_TIMEOUT_MSEC", 2000)
        set_capture_prop(cv2_module, capture, "CAP_PROP_READ_TIMEOUT_MSEC", 2000)
        if config.fourcc:
            set_capture_prop(
                cv2_module,
                capture,
                "CAP_PROP_FOURCC",
                cv2_module.VideoWriter_fourcc(*config.fourcc),
            )
        set_capture_prop(cv2_module, capture, "CAP_PROP_FRAME_WIDTH", config.width)
        set_capture_prop(cv2_module, capture, "CAP_PROP_FRAME_HEIGHT", config.height)
        set_capture_prop(cv2_module, capture, "CAP_PROP_FPS", config.fps)
    return capture


def set_capture_prop(cv2_module, capture, prop_name: str, value) -> None:
    if value is None or not hasattr(capture, "set"):
        return
    prop = getattr(cv2_module, prop_name, None)
    if prop is not None:
        capture.set(prop, value)


def choose_writer_fps(cv2_module, capture, config: CaptureConfig) -> float:
    if config.fps and math.isfinite(config.fps) and config.fps > 0:
        return config.fps
    prop = getattr(cv2_module, "CAP_PROP_FPS", None)
    if prop is not None and hasattr(capture, "get"):
        fps = float(capture.get(prop) or 0.0)
        if math.isfinite(fps) and fps > 0:
            return fps
    return DEFAULT_RECORD_FPS


def _is_opened(capture) -> bool:
    return bool(capture.isOpened()) if hasattr(capture, "isOpened") else True


def release_quietly(resource) -> None:
    release = getattr(resource, "release", None)
    if callable(release):
        release()


def run_interactive_menu(
    stdscr,
    args: RecordArgs,
    choices: Sequence[CaptureConfig],
) -> int:
    if not choices:
        raise RuntimeError(f"no video formats available for /dev/video{args.device_id}")

    try:
        curses.curs_set(0)
    except curses.error:
        pass
    stdscr.keypad(True)

    selected_index = 0
    session: RecordingSession | None = None
    status = f"device=/dev/video{args.device_id} output={args.output_path}"
    try:
        while True:
            draw_menu(stdscr, choices, selected_index, status)
            if session is None:
                stdscr.nodelay(False)
                key = stdscr.getch()
                if key == curses.KEY_UP:
                    selected_index = move_selection(selected_index, -1, len(choices))
                elif key == curses.KEY_DOWN:
                    selected_index = move_selection(selected_index, 1, len(choices))
                elif is_enter_key(key):
                    session = RecordingSession(
                        device_id=args.device_id,
                        config=choices[selected_index],
                        output_path=args.output_path,
                    )
                    try:
                        session.start()
                    except RuntimeError as exc:
                        session = None
                        status = f"start failed: {exc}"
                    else:
                        status = recording_status(session)
                elif key in {ord("q"), ord("Q"), 27}:
                    return 130
            else:
                stdscr.nodelay(True)
                key = stdscr.getch()
                if is_enter_key(key):
                    session.stop()
                    return 0
                if not session.write_next_frame():
                    frame_count = session.frame_count
                    session.stop()
                    raise RuntimeError(
                        f"camera read failed after {frame_count} frame(s)"
                    )
                status = recording_status(session)
    finally:
        if session is not None:
            session.stop()


def recording_status(session: RecordingSession) -> str:
    return (
        f"recording frames={session.frame_count} "
        f"elapsed={session.elapsed_sec():.1f}s output={session.output_path}"
    )


def draw_menu(
    stdscr,
    choices: Sequence[CaptureConfig],
    selected_index: int,
    status: str,
) -> None:
    stdscr.erase()
    for row, (cursor, label) in enumerate(build_menu_rows(choices, selected_index)):
        safe_addstr(stdscr, row, 0, cursor)
        safe_addstr(stdscr, row, 2, label)
    safe_addstr(stdscr, len(choices) + 1, 0, status)
    safe_addstr(stdscr, len(choices) + 2, 0, "Enter=start/stop  Up/Down=select  q=quit")
    stdscr.refresh()


def safe_addstr(stdscr, y: int, x: int, text: str) -> None:
    max_y, max_x = stdscr.getmaxyx()
    if y < 0 or y >= max_y or x < 0 or x >= max_x:
        return
    width = max_x - x
    if width <= 0:
        return
    try:
        stdscr.addnstr(y, x, text, width - 1)
    except curses.error:
        pass


def is_enter_key(key: int) -> bool:
    return key in {10, 13, curses.KEY_ENTER}


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(sys.argv[1:] if argv is None else argv)
        choices = load_format_choices(args.device_id)
        return curses.wrapper(run_interactive_menu, args, choices)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
