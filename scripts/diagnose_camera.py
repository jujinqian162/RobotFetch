#!/usr/bin/env python3
"""Diagnose Linux/OpenCV camera capture problems for RobotFetch."""

from __future__ import annotations

import argparse
import contextlib
import glob
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class CameraMode:
    fourcc: str
    width: int
    height: int
    fps: float | None


@dataclass(frozen=True)
class CaptureConfig:
    backend: str
    fourcc: str | None = None
    width: int | None = None
    height: int | None = None
    fps: float | None = None

    def describe(self) -> str:
        parts = [f"backend={self.backend}"]
        if self.fourcc:
            parts.append(f"fourcc={self.fourcc}")
        if self.width:
            parts.append(f"width={self.width}")
        if self.height:
            parts.append(f"height={self.height}")
        if self.fps:
            fps_text = f"{self.fps:g}"
            parts.append(f"fps={fps_text}")
        return " ".join(parts)


@dataclass(frozen=True)
class ProbeResult:
    index: int
    config: CaptureConfig
    opened: bool
    read_ok: bool
    frame_shape: tuple[int, ...] | None
    elapsed_sec: float
    error: str | None


DEFAULT_CONFIG = CaptureConfig("default")
FORMAT_PRIORITY = {"MJPG": 0, "YUYV": 1}


@contextlib.contextmanager
def suppress_native_stderr():
    saved_fd = os.dup(2)
    try:
        with open(os.devnull, "w", encoding="utf-8") as devnull:
            os.dup2(devnull.fileno(), 2)
            yield
    finally:
        os.dup2(saved_fd, 2)
        os.close(saved_fd)


def run_command(args: list[str]) -> tuple[int, str, str]:
    try:
        completed = subprocess.run(args, capture_output=True, text=True, check=False)
    except FileNotFoundError as exc:
        return 127, "", str(exc)
    return completed.returncode, completed.stdout, completed.stderr


def discover_video_indices() -> list[int]:
    indices: list[int] = []
    for path in glob.glob("/dev/video*"):
        match = re.fullmatch(r"/dev/video(\d+)", path)
        if match:
            indices.append(int(match.group(1)))
    return sorted(indices)


def parse_v4l2_formats(text: str) -> list[CameraMode]:
    modes: list[CameraMode] = []
    current_fourcc: str | None = None
    current_size: tuple[int, int] | None = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        fourcc_match = re.search(r"\[\d+\]: '([^']+)'", line)
        if fourcc_match:
            current_fourcc = fourcc_match.group(1)
            current_size = None
            continue

        size_match = re.search(r"Size:\s+Discrete\s+(\d+)x(\d+)", line)
        if size_match:
            current_size = (int(size_match.group(1)), int(size_match.group(2)))
            continue

        fps_match = re.search(r"\((\d+(?:\.\d+)?)\s+fps\)", line)
        if fps_match and current_fourcc and current_size:
            modes.append(
                CameraMode(
                    fourcc=current_fourcc,
                    width=current_size[0],
                    height=current_size[1],
                    fps=float(fps_match.group(1)),
                )
            )

    return modes


def v4l2_formats_for_index(index: int) -> tuple[list[CameraMode], str]:
    code, stdout, stderr = run_command(
        ["v4l2-ctl", "-d", f"/dev/video{index}", "--list-formats-ext"]
    )
    if code != 0:
        return [], (stderr or stdout).strip()
    return parse_v4l2_formats(stdout), ""


def _mode_sort_key(mode: CameraMode) -> tuple[int, int, int, float]:
    fourcc_rank = FORMAT_PRIORITY.get(mode.fourcc, 10)
    area = mode.width * mode.height
    resolution_rank = 0 if (mode.width, mode.height) == (640, 480) else area
    fps_distance = abs((mode.fps or 30.0) - 30.0)
    return resolution_rank, fourcc_rank, area, fps_distance


def build_candidate_modes(modes: Iterable[CameraMode]) -> list[CaptureConfig]:
    seen: set[CaptureConfig] = set()
    candidates: list[CaptureConfig] = []
    for mode in sorted(modes, key=_mode_sort_key):
        config = CaptureConfig(
            backend="v4l2",
            fourcc=mode.fourcc,
            width=mode.width,
            height=mode.height,
            fps=mode.fps,
        )
        if config not in seen:
            candidates.append(config)
            seen.add(config)

    # Useful generic fallbacks when v4l2-ctl is missing or the device cannot
    # report formats, ordered to target common select() timeout workarounds.
    generic = [
        CaptureConfig("v4l2", "MJPG", 640, 480, 30.0),
        CaptureConfig("v4l2", "MJPG", 640, 480, 15.0),
        CaptureConfig("v4l2", "YUYV", 640, 480, 30.0),
        CaptureConfig("v4l2", "YUYV", 320, 240, 30.0),
    ]
    for config in generic:
        if config not in seen:
            candidates.append(config)
            seen.add(config)
    return candidates


def probe_capture(index: int, config: CaptureConfig, read_attempts: int) -> ProbeResult:
    try:
        import cv2
    except Exception as exc:  # pragma: no cover - depends on local environment
        return ProbeResult(index, config, False, False, None, 0.0, repr(exc))

    if hasattr(cv2, "setLogLevel"):
        cv2.setLogLevel(0)

    start = time.monotonic()
    cap = None
    try:
        with suppress_native_stderr():
            if config.backend == "v4l2":
                cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            else:
                cap = cv2.VideoCapture(index)

            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
            if config.fourcc:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config.fourcc))
            if config.width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
            if config.height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
            if config.fps:
                cap.set(cv2.CAP_PROP_FPS, config.fps)

            opened = bool(cap.isOpened()) if hasattr(cap, "isOpened") else True
            read_ok = False
            frame_shape = None
            if opened:
                for _ in range(read_attempts):
                    read_ok, frame = cap.read()
                    if read_ok:
                        frame_shape = tuple(frame.shape) if frame is not None else None
                        break
        return ProbeResult(
            index=index,
            config=config,
            opened=opened,
            read_ok=read_ok,
            frame_shape=frame_shape,
            elapsed_sec=time.monotonic() - start,
            error=None,
        )
    except Exception as exc:
        return ProbeResult(
            index=index,
            config=config,
            opened=False,
            read_ok=False,
            frame_shape=None,
            elapsed_sec=time.monotonic() - start,
            error=repr(exc),
        )
    finally:
        if cap is not None:
            release = getattr(cap, "release", None)
            if callable(release):
                release()


def build_recommendation(results: list[ProbeResult]) -> str:
    working = [result for result in results if result.opened and result.read_ok]
    default_failures = [
        result
        for result in results
        if result.config == DEFAULT_CONFIG and result.opened and not result.read_ok
    ]

    if working:
        best = working[0]
        lines = [
            (
                "Recommended capture setting: "
                f"source {best.index} {best.config.describe()}"
            )
        ]
        if best.config != DEFAULT_CONFIG:
            lines.append(
                "Use this as a read() fallback candidate; keep the default capture path "
                "first for robot configs."
            )
        if default_failures:
            lines.append(
                "Reason: default capture opened but read failed, which matches the "
                "common OpenCV/V4L2 select timeout pattern."
            )
        return "\n".join(lines)

    if default_failures:
        return (
            "No fallback config succeeded. Default capture opened but read failed, so "
            "this is still consistent with a select timeout. Check supported V4L2 "
            "formats, lower resolution/fps, and USB/WSL transport stability."
        )

    opened = [result for result in results if result.opened]
    if opened:
        return (
            "At least one source opened, but no read succeeded. This points to capture "
            "format negotiation or transport timeout before BaseDetect/PID runs."
        )
    return (
        "No tested camera index opened. Check /dev/video* presence, permissions, "
        "video group membership, USB attachment, and whether the index maps to a "
        "metadata-only node."
    )


def print_device_summary(indices: list[int]) -> None:
    print("== Device nodes ==")
    device_paths = sorted(glob.glob("/dev/video*") + glob.glob("/dev/media*"))
    if device_paths:
        for path in device_paths:
            try:
                stat = Path(path).stat()
                print(f"{path} mode={oct(stat.st_mode & 0o777)}")
            except OSError as exc:
                print(f"{path} stat_error={exc}")
    else:
        print("No /dev/video* or /dev/media* nodes found.")
    print(f"Candidate video indices: {indices or 'none'}")
    print()

    print("== v4l2-ctl --list-devices ==")
    code, stdout, stderr = run_command(["v4l2-ctl", "--list-devices"])
    if code == 0:
        print(stdout.rstrip() or "(empty)")
    else:
        print((stderr or stdout).strip() or "v4l2-ctl failed")
    print()


def print_probe_result(result: ProbeResult) -> None:
    status = "PASS" if result.opened and result.read_ok else "FAIL"
    shape = f" shape={result.frame_shape}" if result.frame_shape else ""
    error = f" error={result.error}" if result.error else ""
    print(
        f"[{status}] source={result.index} {result.config.describe()} "
        f"opened={result.opened} read={result.read_ok} "
        f"elapsed={result.elapsed_sec:.3f}s{shape}{error}"
    )


def diagnose(indices: list[int], read_attempts: int, max_candidates: int) -> list[ProbeResult]:
    results: list[ProbeResult] = []
    for index in indices:
        print(f"== /dev/video{index} formats ==")
        modes, error = v4l2_formats_for_index(index)
        if modes:
            for mode in modes:
                fps = "unknown" if mode.fps is None else f"{mode.fps:g}"
                print(f"{mode.fourcc} {mode.width}x{mode.height} {fps}fps")
        else:
            print(error or "No modes reported.")
        print()

        configs = [DEFAULT_CONFIG] + build_candidate_modes(modes)[:max_candidates]
        print(f"== OpenCV probe source {index} ==")
        for config in configs:
            result = probe_capture(index, config, read_attempts)
            results.append(result)
            print_probe_result(result)
        print()
    return results


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose camera index, V4L2 modes, and OpenCV settings that may "
            "avoid select() timeout."
        )
    )
    parser.add_argument(
        "--indices",
        nargs="+",
        type=int,
        help="Camera indices to probe. Defaults to discovered /dev/videoN nodes, or 0 1 2.",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=8,
        help="Maximum V4L2 fallback candidates to probe per index.",
    )
    parser.add_argument(
        "--read-attempts",
        type=int,
        default=1,
        help="read() attempts per capture config.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    indices = args.indices
    if indices is None:
        indices = discover_video_indices() or [0, 1, 2]

    print_device_summary(indices)
    results = diagnose(indices, args.read_attempts, args.max_candidates)
    print("== Recommendation ==")
    print(build_recommendation(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
