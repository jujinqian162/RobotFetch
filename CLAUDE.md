# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment and setup

- This is a lightweight ROS 2 Python project run directly from `src/`; it is not a `colcon` workspace.
- Recommended runtime is Ubuntu 24.04, ROS 2 Jazzy, and `/usr/bin/python3.12` with a project `.venv` created using `--system-site-packages` so ROS Python packages and project dependencies share one interpreter.
- Avoid conda Python for normal runs because workflow code imports both `rclpy` and `basedetect` in the same process.
- Initialize the development environment from the repository root:
  ```bash
  ./scripts/setup_dev_env.sh
  ```
- Activate the environment before running commands:
  ```bash
  source ./scripts/activate_dev_env.sh
  ```
- `setup_dev_env.sh` installs `BaseDetect` editable plus `pytest`, using `uv` if available and falling back to `venv`/`pip`.

## Common commands

- Run all RobotFetch tests:
  ```bash
  python -m pytest -q
  ```
- Run one test file or one test:
  ```bash
  python -m pytest -q tests/test_pid.py
  python -m pytest -q tests/test_pid.py::test_pid_output_is_clamped
  ```
- Start the current turtle partial workflow:
  ```bash
  python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.turtle.yaml
  ```
- Start the current robot partial workflow:
  ```bash
  python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.yaml
  ```
- Run the BaseDetect coordinate demo node:
  ```bash
  python src/base_detect_demo_node.py
  ```
- Run the coordinate demo with explicit inputs:
  ```bash
  python src/base_detect_demo_node.py \
    --camera-config BaseDetect/configs/camera.yaml \
    --bbox-cx 320 --bbox-cy 240 --bbox-width 120 --bbox-height 90 \
    --frame-id camera_link --hz 2.0
  ```
- Inspect demo output:
  ```bash
  ros2 topic echo /robot_fetch/target_position
  ```
- Run the camera diagnostic script:
  ```bash
  python scripts/diagnose_camera.py
  ```
- There is no repository-level build or lint command configured beyond direct Python execution and pytest.

## Architecture overview

RobotFetch coordinates a staged robot workflow: observe terminal/base targets, use BaseDetect status detections to select the target closest to a desired image column, run PID lateral alignment, publish workflow state and velocity commands, then later support forward approach and base-coordinate output for downstream manipulation.

The active workflow path is runner-based:

1. `src/runners/pid_alignment_ros_node.py` loads a workflow YAML, opens an OpenCV camera/video source, creates ROS publishers, constructs the adapter and detector gateway, and ticks the runner on a timer.
2. `src/runners/pid_alignment_runner.py` contains the ROS-agnostic one-cycle loop: call the detector, run status alignment, publish command/status outputs, and advance/stop according to `phase_sequence`.
3. `src/algorithms/detector_gateway.py` integrates BaseDetect by adding `BaseDetect/` to `sys.path` and importing `Detector` from the BaseDetect SDK.
4. `src/algorithms/status_align.py`, `target_selection.py`, and `pid.py` choose the nearest valid status target and compute bounded lateral control while reporting `RUNNING`, `ALIGNED`, or `TARGET_LOST`.
5. `src/workflow/types.py` and `phase_controller.py` define the shared workflow contract (`Phase`, `AlgoStatus`, `EnvStatus`) used by runners, adapters, and tests.
6. `src/adapters/robot_adapter.py` passes workflow velocity through for the robot environment; `src/adapters/turtle_adapter.py` maps workflow lateral `linear.y` to turtlesim forward `linear.x` for partial validation.

The current MVP runner implements `STATUS_ALIGN` only. `FORWARD_APPROACH` and `BASE_COORD` exist in the workflow contract for future full-mission support, but are not executed by the current partial-test configs.

## Configuration model

- Main workflow configs live in `configs/workflows/`:
  - `pid_alignment.turtle.yaml`
  - `pid_alignment.robot.yaml`
- `src/config/loaders.py` performs strict YAML loading into dataclasses from `src/config/models.py` and resolves relative BaseDetect paths from the project root.
- Important config fields include:
  - `environment`: `turtle` or `robot`
  - `start_phase`: currently `STATUS_ALIGN`
  - `phase_sequence`: current partial configs use `[STATUS_ALIGN]`
  - `detector.sdk_config`: usually `BaseDetect/configs/basedetect_sdk.yaml`
  - `detector.status_profile`: usually `status_competition`
  - `detector.input_source`: camera index such as `"0"` or a replay video path
  - `topics.cmd_topic`: workflow velocity topic, usually `/cmd_vel`
  - `topics.selected_status_topic`: selected target pixel topic
  - `adapter.turtle_cmd_topic`: turtlesim command topic for turtle mode
  - `status_align.target_x` and `status_align.tolerance_px`: pixel alignment target and tolerance

## BaseDetect integration

- `BaseDetect/` is a nested Python project that provides YOLO-based detection plus an SDK.
- RobotFetch should normally access BaseDetect through `DetectorGateway`, not by calling training or prediction scripts directly from workflow code.
- BaseDetect SDK behavior used by RobotFetch:
  - `status` profiles return ordered status targets from frames.
  - `base_coord` profiles return stable 3D targets, but this path is not part of the current MVP runner.
  - `Detector.ready` depends on configured warmup frames.
- If working inside `BaseDetect/`, prefer its documented commands from the `BaseDetect` directory:
  ```bash
  uv sync
  uv run --module basedetect
  uv run scripts/predict.py
  uv run scripts/test_cli.py
  ```
- Avoid committing generated BaseDetect artifacts, datasets, output videos, or large training weights unless explicitly intended.

## Tests

- `pytest.ini` restricts collection to `tests/`, which avoids accidentally collecting heavy BaseDetect scripts.
- Tests cover config loading, workflow enum/phase behavior, PID and target selection, status alignment, adapters, detector gateway wrapping, runner behavior, BaseDetect SDK config resolution, detector runtime behavior, and camera diagnostics.
- Many tests mock ROS/BaseDetect boundaries; for UI/robotics behavior changes, also run the relevant runner or ROS topic check when the environment is available.

## Notes for future changes

- Treat `src/terminal_pid_follower_node.py` as the older pre-refactor experiment path; the current startup path is `src/runners/pid_alignment_ros_node.py --config ...`.
- Preserve the workflow contract across runner, adapter, and tests when adding phases or topics.
- When changing turtle behavior, remember that turtle mode intentionally bridges workflow lateral motion to turtlesim forward motion for validating PID and workflow synchronization without the real robot environment.
