# Single-Node Command Heartbeat Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Start implementation on a new branch before editing code.

**Goal:** Keep the current single ROS node startup while decoupling slow vision/PID ticks from a stable command heartbeat that targets at least 20Hz command delivery to the robot.

**Architecture:** `WorkflowEngine.tick()` remains the slow perception/PID path. Phase runners write desired commands into a small thread-safe command buffer. A separate timer in the same `PidAlignmentRosNode` repeatedly publishes the latest non-stopped command at `command_publish_hz`, and immediately switches to zero when the workflow sends an explicit stop signal.

**Tech Stack:** Python, ROS 2 `rclpy`, `MultiThreadedExecutor`, callback groups, `threading.Lock`, existing RobotFetch config loaders and pytest suite.

---

## Problem Background

The robot shakes on real hardware even when PID output appears fixed because the current runner only publishes `/t0x0101_robotfetch` from the vision/PID timer. That timer is configured at 10Hz and also performs frame read plus YOLO inference, so real publish intervals can be much larger than 100ms. The existing `scripts/move.py` feels smooth because it publishes the same command continuously at 20Hz without running vision inference.

The fix must not require launching multiple nodes. It should stay in one node, but separate the responsibilities inside that node:

- workflow timer: read frame, run detector/PID, update desired command
- heartbeat timer: repeat latest desired command at 30Hz target rate
- stop signal: explicit cross-thread stop request, not just waiting for command timeout
- logs: report how many heartbeat frames were filled inside each existing phase cycle INFO; heartbeat delay warnings still identify the heartbeat thread without logging every successful publish

## Implementation Constraints

- Do not create a second ROS node or require the user to launch another process.
- Do not let the heartbeat timer call detector, camera, workflow engine, or phase runners.
- Do not make `PidAlignmentRosNode` a large unstructured file. Put command-buffer logic in a small focused module.
- Treat a zero command and a stop signal as related but distinct concepts:
  - zero command can be a normal command while workflow continues
  - stop signal means heartbeat must stop repeating stale motion immediately
- Preserve `topics.publish_cmd_vel` semantics. Disabling workflow cmd publish must not disable turtle bridge behavior.
- Preserve existing command transform semantics. `cmd_vel_transform` must still apply immediately before output publish.

## File Structure

- Create `src/runners/command_heartbeat.py`
  - Owns thread-safe command buffer, explicit stop signal, heartbeat snapshot logic, and per-workflow-cycle heartbeat counters.
  - Must not import ROS message classes.

- Modify `src/config/models.py`
  - Add `RuntimeConfig`.
  - Add `runtime: RuntimeConfig` to `PidAlignmentWorkflowConfig`.

- Modify `src/config/loaders.py`
  - Parse optional top-level `runtime`.
  - Defaults: `workflow_hz=10.0`, `command_publish_hz=30.0`, `command_timeout_s=0.25`.
  - Validate `command_publish_hz >= 20.0`.

- Modify `configs/workflows/pid_alignment.robot.yaml`
- Modify `configs/workflows/pid_alignment.robot.3.yaml`
- Modify `configs/workflows/pid_alignment.robot.forward.yaml`
- Modify `configs/workflows/pid_alignment.turtle.yaml`
  - Add explicit runtime fields so operators can tune them.

- Modify `src/runners/pid_alignment_ros_node.py`
  - Rename the current direct output publisher to `_command_output`.
  - Pass `BufferedCommandPublisher` into `WorkflowPublishers.cmd_pub`.
  - Add workflow timer and heartbeat timer with separate callback groups.
  - Use `MultiThreadedExecutor(num_threads=2)` in `main()`.
  - Add startup/runtime fields, heartbeat-warning logs, stop logs, and expose per-cycle heartbeat stats to phase cycle logs.

- Create `src/runners/phases/heartbeat_stats.py`
  - Provide a small helper for phase runners to append `filled_heartbeat_frames` to their existing cycle INFO logs.

- Modify `tests/test_config_loaders.py`
  - Add runtime config parsing and validation tests.

- Modify `tests/test_pid_alignment_ros_node.py`
  - Update fake ROS imports for callback groups and executor.
  - Update tests that currently expect immediate command publish.
  - Add heartbeat, stop signal, timeout, and logging tests.

- Create `tests/test_command_heartbeat.py`
  - Unit-test command buffer behavior without ROS.

## Task 0: Branch and Execution Mode

**Files:**
- No source edits in this task.

- [ ] **Step 1: Verify current repo state**

Run:

```bash
git status --short --branch
```

Expected: current branch and dirty files are visible. Do not discard user changes.

- [ ] **Step 2: Create a focused implementation branch**

Run:

```bash
git switch -c fix/single-node-command-heartbeat
```

Expected: new branch `fix/single-node-command-heartbeat`.

- [ ] **Step 3: Use subagent-driven implementation**

Use `superpowers:subagent-driven-development` to implement this plan task-by-task. The branch creation is intentionally separate from implementation so all code changes are isolated and reviewable.

## Task 1: Runtime Config

**Files:**
- Modify: `src/config/models.py`
- Modify: `src/config/loaders.py`
- Modify: `tests/test_config_loaders.py`
- Modify: `tests/test_pid_alignment_ros_node.py`
- Modify: `configs/workflows/pid_alignment.robot.yaml`
- Modify: `configs/workflows/pid_alignment.robot.3.yaml`
- Modify: `configs/workflows/pid_alignment.robot.forward.yaml`
- Modify: `configs/workflows/pid_alignment.turtle.yaml`

- [ ] **Step 1: Write failing config tests**

Add to `tests/test_config_loaders.py`:

```python
def test_load_pid_alignment_config_reads_runtime_heartbeat_fields(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.runtime.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  runtime:
    workflow_hz: 8.0
    command_publish_hz: 30.0
    command_timeout_s: 0.25
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    cfg = load_pid_alignment_config(config_path)

    assert cfg.runtime.workflow_hz == 8.0
    assert cfg.runtime.command_publish_hz == 30.0
    assert cfg.runtime.command_timeout_s == 0.25


def test_load_pid_alignment_config_defaults_runtime_fields(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.runtime_defaults.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    cfg = load_pid_alignment_config(config_path)

    assert cfg.runtime.workflow_hz == 10.0
    assert cfg.runtime.command_publish_hz == 30.0
    assert cfg.runtime.command_timeout_s == 0.25


def test_load_pid_alignment_config_rejects_command_publish_hz_below_20(tmp_path: Path):
    config_path = tmp_path / "pid_alignment.runtime_low_hz.yaml"
    config_path.write_text(
        """
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  runtime:
    command_publish_hz: 19.0
  detector:
    sdk_config: BaseDetect/configs/basedetect_sdk.yaml
    status_profile: status_competition
    input_source: "0"
  topics:
    cmd_topic: /cmd_vel
    workflow_phase_topic: /workflow/phase
    algo_status_topic: /workflow/algo_status
    env_status_topic: /workflow/env_status
    selected_status_topic: /robot_fetch/selected_target_px
  adapter:
    turtle_cmd_topic: null
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="runtime.command_publish_hz"):
        load_pid_alignment_config(config_path)
```

Update `make_cfg()` in `tests/test_pid_alignment_ros_node.py` after importing `RuntimeConfig`:

```python
runtime=RuntimeConfig(
    workflow_hz=10.0,
    command_publish_hz=30.0,
    command_timeout_s=0.25,
),
```

- [ ] **Step 2: Run config tests and verify failure**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_config_loaders.py::test_load_pid_alignment_config_reads_runtime_heartbeat_fields tests/test_config_loaders.py::test_load_pid_alignment_config_defaults_runtime_fields tests/test_config_loaders.py::test_load_pid_alignment_config_rejects_command_publish_hz_below_20
```

Expected: FAIL because `RuntimeConfig` and `cfg.runtime` do not exist yet.

- [ ] **Step 3: Implement runtime models**

In `src/config/models.py`, add:

```python
@dataclass(frozen=True)
class RuntimeConfig:
    workflow_hz: float
    command_publish_hz: float
    command_timeout_s: float
```

Add to `PidAlignmentWorkflowConfig`:

```python
    runtime: RuntimeConfig
```

- [ ] **Step 4: Implement runtime loader**

In `src/config/loaders.py`, import `RuntimeConfig`, then add near the top of `load_pid_alignment_config()`:

```python
    runtime = _optional_mapping_field(
        root,
        "runtime",
        parent="pid_alignment_workflow",
    )
```

Add to the `PidAlignmentWorkflowConfig(...)` constructor:

```python
        runtime=RuntimeConfig(
            workflow_hz=_require_positive_defaulted_float_field(
                runtime,
                "workflow_hz",
                default=10.0,
                parent="pid_alignment_workflow.runtime",
            ),
            command_publish_hz=_require_minimum_defaulted_float_field(
                runtime,
                "command_publish_hz",
                default=30.0,
                minimum=20.0,
                parent="pid_alignment_workflow.runtime",
            ),
            command_timeout_s=_require_positive_defaulted_float_field(
                runtime,
                "command_timeout_s",
                default=0.25,
                parent="pid_alignment_workflow.runtime",
            ),
        ),
```

Add helper:

```python
def _require_minimum_defaulted_float_field(
    mapping: dict[str, Any],
    key: str,
    *,
    default: float,
    minimum: float,
    parent: str,
) -> float:
    value = _require_defaulted_float_field(
        mapping,
        key,
        default=default,
        parent=parent,
    )
    if value < minimum:
        raise ValueError(f"{parent}.{key} must be >= {minimum:g}")
    return value
```

- [ ] **Step 5: Add YAML runtime sections**

Add this block under `start_phase` in each workflow YAML:

```yaml
  runtime:
    # Vision/PID workflow tick. This can run slower than the command heartbeat.
    workflow_hz: 10.0

    # Command heartbeat target. Keep above 20Hz to preserve margin on real hardware.
    command_publish_hz: 30.0

    # If workflow does not refresh a command before this age, heartbeat publishes zero.
    command_timeout_s: 0.25
```

- [ ] **Step 6: Run tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_config_loaders.py tests/test_pid_alignment_ros_node.py::test_build_status_align_step_uses_configured_max_speed
```

Expected: PASS.

## Task 2: Thread-Safe Command Buffer

**Files:**
- Create: `src/runners/command_heartbeat.py`
- Create: `tests/test_command_heartbeat.py`

- [ ] **Step 1: Write failing command heartbeat tests**

Create `tests/test_command_heartbeat.py`:

```python
from runners.command_heartbeat import BufferedCommandPublisher, ZERO_COMMAND


def test_heartbeat_repeats_latest_command_until_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)

    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    first = buffer.snapshot_for_heartbeat(timeout_s=0.25)
    assert first.command == {"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}
    assert first.stop_requested is False
    assert first.timed_out is False

    now = 10.10
    second = buffer.snapshot_for_heartbeat(timeout_s=0.25)
    assert second.command == {"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0}
    assert second.repeat_count_since_workflow_tick == 2


def test_heartbeat_uses_zero_after_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)

    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})
    now = 10.30

    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == ZERO_COMMAND
    assert snapshot.timed_out is True


def test_explicit_stop_signal_overrides_latest_command_without_waiting_for_timeout():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    now = 10.01
    buffer.request_stop(reason="phase_sequence_complete")
    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == ZERO_COMMAND
    assert snapshot.stop_requested is True
    assert snapshot.stop_reason == "phase_sequence_complete"
    assert snapshot.timed_out is False


def test_new_workflow_command_clears_stop_signal():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.request_stop(reason="frame_read_failed")

    now = 10.02
    buffer.publish({"linear_x": 0.0, "linear_y": -0.1, "angular_z": 0.0})
    snapshot = buffer.snapshot_for_heartbeat(timeout_s=0.25)

    assert snapshot.command == {"linear_x": 0.0, "linear_y": -0.1, "angular_z": 0.0}
    assert snapshot.stop_requested is False
    assert snapshot.stop_reason is None


def test_consume_workflow_cycle_stats_reports_filled_heartbeat_frames_once():
    now = 10.0
    buffer = BufferedCommandPublisher(now_s=lambda: now)
    buffer.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})

    buffer.snapshot_for_heartbeat(timeout_s=0.25)
    buffer.snapshot_for_heartbeat(timeout_s=0.25)

    stats = buffer.consume_workflow_cycle_stats()
    assert stats.filled_heartbeat_frames == 2
    assert stats.command_age_s == 0.0

    next_stats = buffer.consume_workflow_cycle_stats()
    assert next_stats.filled_heartbeat_frames == 0
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_command_heartbeat.py
```

Expected: FAIL because `runners.command_heartbeat` does not exist.

- [ ] **Step 3: Implement command buffer**

Create `src/runners/command_heartbeat.py`:

```python
from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable

ZERO_COMMAND = {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}


def normalize_command(payload: dict[str, float]) -> dict[str, float]:
    return {
        "linear_x": float(payload.get("linear_x", 0.0)),
        "linear_y": float(payload.get("linear_y", 0.0)),
        "angular_z": float(payload.get("angular_z", 0.0)),
    }


@dataclass(frozen=True)
class CommandHeartbeatSnapshot:
    command: dict[str, float]
    stop_requested: bool
    stop_reason: str | None
    timed_out: bool
    repeat_count_since_workflow_tick: int
    command_age_s: float | None


@dataclass(frozen=True)
class WorkflowCycleHeartbeatStats:
    filled_heartbeat_frames: int
    stop_requested: bool
    stop_reason: str | None
    command_age_s: float | None


class BufferedCommandPublisher:
    def __init__(self, *, now_s: Callable[[], float]) -> None:
        self._now_s = now_s
        self._lock = threading.Lock()
        self._latest_command = dict(ZERO_COMMAND)
        self._latest_command_at_s: float | None = None
        self._stop_requested = True
        self._stop_reason: str | None = "startup"
        self._repeat_count_since_workflow_tick = 0
        self._filled_count_since_workflow_tick = 0

    def publish(self, payload: dict[str, float]) -> None:
        normalized = normalize_command(payload)
        now = self._now_s()
        with self._lock:
            self._latest_command = normalized
            self._latest_command_at_s = now
            self._stop_requested = False
            self._stop_reason = None

    def request_stop(self, *, reason: str) -> None:
        now = self._now_s()
        with self._lock:
            self._latest_command = dict(ZERO_COMMAND)
            self._latest_command_at_s = now
            self._stop_requested = True
            self._stop_reason = reason

    def snapshot_for_heartbeat(self, *, timeout_s: float) -> CommandHeartbeatSnapshot:
        now = self._now_s()
        with self._lock:
            command_age_s = self._command_age_s_locked(now)
            if self._stop_requested:
                timed_out = False
            else:
                timed_out = (
                    self._latest_command_at_s is None
                    or command_age_s is None
                    or command_age_s > timeout_s
                )
            if self._stop_requested or timed_out:
                command = dict(ZERO_COMMAND)
            else:
                command = dict(self._latest_command)
                self._filled_count_since_workflow_tick += 1
            self._repeat_count_since_workflow_tick += 1
            return CommandHeartbeatSnapshot(
                command=command,
                stop_requested=self._stop_requested,
                stop_reason=self._stop_reason,
                timed_out=timed_out,
                repeat_count_since_workflow_tick=self._repeat_count_since_workflow_tick,
                command_age_s=command_age_s,
            )

    def consume_workflow_cycle_stats(self) -> WorkflowCycleHeartbeatStats:
        now = self._now_s()
        with self._lock:
            stats = WorkflowCycleHeartbeatStats(
                filled_heartbeat_frames=self._filled_count_since_workflow_tick,
                stop_requested=self._stop_requested,
                stop_reason=self._stop_reason,
                command_age_s=self._command_age_s_locked(now),
            )
            self._repeat_count_since_workflow_tick = 0
            self._filled_count_since_workflow_tick = 0
            return stats

    def _command_age_s_locked(self, now: float) -> float | None:
        if self._latest_command_at_s is None:
            return None
        return max(0.0, now - self._latest_command_at_s)
```

- [ ] **Step 4: Run tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_command_heartbeat.py
```

Expected: PASS.

## Task 3: Wire Buffer and Heartbeat Timer into the ROS Node

**Files:**
- Modify: `src/runners/pid_alignment_ros_node.py`
- Modify: `tests/test_pid_alignment_ros_node.py`

- [ ] **Step 1: Write failing ROS node tests**

Update fake ROS setup in `tests/test_pid_alignment_ros_node.py` so `FakeNode.create_timer` tests can capture callback groups:

```python
def fake_create_timer(self, period_s, callback, callback_group=None):
    return SimpleNamespace(
        period_s=period_s,
        callback=callback,
        callback_group=callback_group,
    )
```

Add tests:

```python
def test_pid_alignment_ros_node_uses_separate_workflow_and_command_timers(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    timers: list[object] = []

    class FakeRosPublisher:
        def publish(self, message: object) -> None:
            return None

    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_publisher",
        lambda self, msg_type, topic, qos_depth: FakeRosPublisher(),
        raising=False,
    )

    def fake_create_timer(self, period_s, callback, callback_group=None):
        timer = SimpleNamespace(
            period_s=period_s,
            callback=callback,
            callback_group=callback_group,
        )
        timers.append(timer)
        return timer

    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_timer",
        fake_create_timer,
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_adapter",
        lambda **kwargs: SimpleNamespace(on_cmd_vel=lambda message: None, on_phase=lambda phase: None),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))

    assert node._workflow_timer.period_s == 0.1
    assert round(node._command_timer.period_s, 6) == round(1.0 / 30.0, 6)
    assert len(timers) == 2
    assert timers[0].callback_group is not timers[1].callback_group


def test_runner_cmd_updates_buffer_until_heartbeat_publishes(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, topic: str, msg_type: type[object]) -> None:
            self.topic = topic
            self.msg_type = msg_type
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    def fake_create_publisher(self, msg_type, topic, qos_depth):
        publisher = FakeRosPublisher(topic=topic, msg_type=msg_type)
        publishers[topic] = publisher
        return publisher

    monkeypatch.setattr(fake_ros_node.PidAlignmentRosNode, "create_publisher", fake_create_publisher, raising=False)
    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_timer",
        lambda self, period_s, callback, callback_group=None: SimpleNamespace(period_s=period_s, callback=callback),
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_adapter",
        lambda **kwargs: SimpleNamespace(on_cmd_vel=lambda message: None, on_phase=lambda phase: None),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))
    node.cmd_pub.publish({"linear_x": 0.1, "linear_y": -0.2, "angular_z": 0.3})

    assert publishers["/t0x0101_robotfetch"].messages == []

    node._on_command_timer()

    assert len(publishers["/t0x0101_robotfetch"].messages) == 1
    assert publishers["/t0x0101_robotfetch"].messages[0].data == [0.1, -0.2, 0.3]


def test_stop_request_publishes_zero_without_waiting_for_timeout(monkeypatch):
    fake_ros_node = import_with_fake_ros(monkeypatch)
    publishers: dict[str, object] = {}

    class FakeRosPublisher:
        def __init__(self, topic: str) -> None:
            self.topic = topic
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    def fake_create_publisher(self, msg_type, topic, qos_depth):
        publisher = FakeRosPublisher(topic)
        publishers[topic] = publisher
        return publisher

    monkeypatch.setattr(fake_ros_node.PidAlignmentRosNode, "create_publisher", fake_create_publisher, raising=False)
    monkeypatch.setattr(
        fake_ros_node.PidAlignmentRosNode,
        "create_timer",
        lambda self, period_s, callback, callback_group=None: SimpleNamespace(period_s=period_s, callback=callback),
        raising=False,
    )
    monkeypatch.setattr(
        fake_ros_node,
        "build_adapter",
        lambda **kwargs: SimpleNamespace(on_cmd_vel=lambda message: None, on_phase=lambda phase: None),
    )

    node = fake_ros_node.PidAlignmentRosNode(cfg=make_cfg(environment="robot"))
    node.cmd_pub.publish({"linear_x": 0.0, "linear_y": 0.2, "angular_z": 0.0})
    node._command_buffer.request_stop(reason="test_stop")
    node._on_command_timer()

    assert publishers["/t0x0101_robotfetch"].messages[0].data == [0.0, 0.0, 0.0]
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_uses_separate_workflow_and_command_timers tests/test_pid_alignment_ros_node.py::test_runner_cmd_updates_buffer_until_heartbeat_publishes tests/test_pid_alignment_ros_node.py::test_stop_request_publishes_zero_without_waiting_for_timeout
```

Expected: FAIL because node still has one timer and direct command publishing.

- [ ] **Step 3: Import runtime pieces**

In `src/runners/pid_alignment_ros_node.py`, add:

```python
import threading
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from runners.command_heartbeat import BufferedCommandPublisher, ZERO_COMMAND
```

- [ ] **Step 4: Split direct output and buffered command publisher**

Replace `self._cmd_pub = _WorkflowCommandPublisher(...)` with:

```python
        self._command_output = _WorkflowCommandPublisher(
            ros_publisher=workflow_cmd_ros_publisher,
            message_type=workflow_cmd_message_type,
            workflow_publisher_type=workflow_publisher_type,
            adapter_cmd_handler=(
                self._adapter.on_cmd_vel if cfg.environment == "turtle" else None
            ),
            cmd_vel_transform=cmd_vel_transform,
        )
        self._command_buffer = BufferedCommandPublisher(now_s=time.monotonic)
        self._cmd_pub = self._command_buffer
```

Keep `WorkflowPublishers(cmd_pub=self._cmd_pub, ...)`.

- [ ] **Step 5: Add two timers and callback groups**

Replace:

```python
        self._timer = self.create_timer(0.1, self._on_timer)
```

With:

```python
        self._workflow_callback_group = MutuallyExclusiveCallbackGroup()
        self._command_callback_group = MutuallyExclusiveCallbackGroup()
        self._workflow_timer = self.create_timer(
            1.0 / cfg.runtime.workflow_hz,
            self._on_workflow_timer,
            callback_group=self._workflow_callback_group,
        )
        self._command_timer = self.create_timer(
            1.0 / cfg.runtime.command_publish_hz,
            self._on_command_timer,
            callback_group=self._command_callback_group,
        )
```

- [ ] **Step 6: Add workflow and command timer callbacks**

Replace `_on_timer()` with:

```python
    def _on_workflow_timer(self) -> None:
        result = self._engine.tick()
        self._handle_engine_stop_if_requested(result)

    def _on_command_timer(self) -> None:
        snapshot = self._command_buffer.snapshot_for_heartbeat(
            timeout_s=self._cfg.runtime.command_timeout_s
        )
        self._command_output.publish(snapshot.command)
```

- [ ] **Step 7: Keep test-facing compatibility**

Keep:

```python
    @property
    def cmd_pub(self):
        return self._cmd_pub
```

This property now exposes the buffer, not the direct ROS output. Tests and phase runners should treat it as the workflow-side command publisher.

- [ ] **Step 8: Run target node tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_command_heartbeat.py tests/test_pid_alignment_ros_node.py
```

Expected: the five immediate-publish tests named in Task 4 will fail until they call `_on_command_timer()` after `node.cmd_pub.publish(...)`.

## Task 4: Preserve Existing Command Semantics Through Heartbeat

**Files:**
- Modify: `tests/test_pid_alignment_ros_node.py`
- Modify: `src/runners/pid_alignment_ros_node.py`

- [ ] **Step 1: Update immediate publish tests**

For tests like `test_pid_alignment_ros_node_publishes_robot_cmd_as_float32_multi_array`, change the assertion flow from:

```python
node.cmd_pub.publish(payload)
assert len(publisher.messages) == 1
```

To:

```python
node.cmd_pub.publish(payload)
assert publisher.messages == []
node._on_command_timer()
assert len(publisher.messages) == 1
```

Apply the same pattern to turtle bridge and transform tests:

- `test_pid_alignment_ros_node_bridges_runner_cmd_to_turtle_topic`
- `test_pid_alignment_ros_node_applies_cmd_vel_transform_before_turtle_bridge`
- `test_pid_alignment_ros_node_can_disable_workflow_cmd_vel_publish`
- `test_pid_alignment_ros_node_applies_cmd_vel_transform_when_workflow_publish_disabled`

- [ ] **Step 2: Ensure transform remains output-side**

Do not apply `cmd_vel_transform` in `BufferedCommandPublisher`. Keep transform inside `_WorkflowCommandPublisher.publish()`. This preserves existing adapter/output semantics and avoids storing environment-specific output in the workflow command buffer.

- [ ] **Step 3: Run command semantic tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_publishes_robot_cmd_as_float32_multi_array tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_bridges_runner_cmd_to_turtle_topic tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_applies_cmd_vel_transform_before_turtle_bridge tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_can_disable_workflow_cmd_vel_publish tests/test_pid_alignment_ros_node.py::test_pid_alignment_ros_node_applies_cmd_vel_transform_when_workflow_publish_disabled
```

Expected: PASS.

## Task 5: Stop Signal and Shutdown Behavior

**Files:**
- Modify: `src/runners/pid_alignment_ros_node.py`
- Modify: `tests/test_pid_alignment_ros_node.py`

- [ ] **Step 1: Add failing engine-stop test**

Add to `tests/test_pid_alignment_ros_node.py`:

```python
def test_engine_stop_requests_command_stop_before_shutdown(monkeypatch):
    stop_reasons: list[str] = []
    output_payloads: list[dict[str, float]] = []
    shutdown_calls: list[object] = []

    class FakeCommandBuffer:
        def request_stop(self, *, reason: str) -> None:
            stop_reasons.append(reason)

    class FakeCommandOutput:
        def publish(self, payload: dict[str, float]) -> None:
            output_payloads.append(payload)

    monkeypatch.setattr(
        ros_node.rclpy,
        "try_shutdown",
        lambda **kwargs: shutdown_calls.append(kwargs),
    )

    class FakeLogger:
        def __init__(self) -> None:
            self.info_messages: list[str] = []

        def info(self, message: str) -> None:
            self.info_messages.append(message)

    node = ros_node.PidAlignmentRosNode.__new__(ros_node.PidAlignmentRosNode)
    node._command_buffer = FakeCommandBuffer()
    node._command_output = FakeCommandOutput()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.destroy_node = lambda: True

    node._handle_engine_stop_if_requested(
        SimpleNamespace(
            stop_reason="phase_sequence_complete",
            phase=Phase.STATUS_ALIGN,
            algo_status=AlgoStatus.ALIGNED,
        )
    )

    assert stop_reasons == ["phase_sequence_complete"]
    assert output_payloads == [{"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0}]
    assert shutdown_calls == [{}]
```

- [ ] **Step 2: Implement immediate stop publish helper**

Add to `PidAlignmentRosNode`:

```python
    def _request_command_stop(self, *, reason: str) -> None:
        self._command_buffer.request_stop(reason=reason)
        self._command_output.publish(ZERO_COMMAND)
```

Update `_handle_engine_stop_if_requested()` before `destroy_node()`:

```python
        self._request_command_stop(reason=str(result.stop_reason))
```

Update `destroy_node()`:

```python
    def destroy_node(self) -> bool:
        command_buffer = getattr(self, "_command_buffer", None)
        command_output = getattr(self, "_command_output", None)
        if command_buffer is not None and command_output is not None:
            command_buffer.request_stop(reason="destroy_node")
            command_output.publish(ZERO_COMMAND)
        resources = getattr(self, "_resources", None)
        if resources is not None:
            resources.release_all()
        return super().destroy_node()
```

- [ ] **Step 3: Run stop tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_engine_stop_requests_command_stop_before_shutdown tests/test_command_heartbeat.py::test_explicit_stop_signal_overrides_latest_command_without_waiting_for_timeout
```

Expected: PASS.

## Task 6: Heartbeat Frame Count In Existing Cycle Logs

**Files:**
- Modify: `src/runners/pid_alignment_ros_node.py`
- Modify: `src/workflow/runtime.py`
- Modify: `src/runners/phases/status_align_phase.py`
- Modify: `src/runners/phases/forward_approach_phase.py`
- Modify: `src/runners/phases/base_coord_phase.py`
- Create: `src/runners/phases/heartbeat_stats.py`
- Modify: `tests/test_pid_alignment_ros_node.py`
- Modify: `tests/test_status_align_phase.py`
- Modify: `tests/test_forward_approach_phase.py`
- Modify: `tests/test_base_coord_phase.py`

- [ ] **Step 1: Add failing phase log tests**

Add focused assertions to the phase tests so each existing cycle INFO includes the heartbeat frames filled since the previous engine tick:

```python
assert "\n  filled_heartbeat_frames=3" in context.logger.info_messages[-1]
```

- [ ] **Step 2: Expose stats through workflow context**

Add a nullable callback to `WorkflowContext`:

```python
heartbeat_stats: Callable[[], Any] | None = None

def consume_heartbeat_stats(self) -> Any:
    if self.heartbeat_stats is None:
        return SimpleNamespace(filled_heartbeat_frames=0)
    return self.heartbeat_stats()
```

When building `WorkflowContext`, pass:

```python
heartbeat_stats=self._command_buffer.consume_workflow_cycle_stats
```

Do not emit a separate heartbeat summary from `_on_workflow_timer()`.

- [ ] **Step 3: Append the count inside existing cycle INFO logs**

Add a helper:

```python
def filled_heartbeat_frames(context: Any) -> int:
    consumer = getattr(context, "consume_heartbeat_stats", None)
    if not callable(consumer):
        return 0
    stats = consumer()
    return max(0, int(getattr(stats, "filled_heartbeat_frames", 0)))
```

Append one field to each phase cycle log:

```python
f"  filled_heartbeat_frames={filled_heartbeat_frames(context)}"
```

- [ ] **Step 4: Add startup log fields**

Extend `_build_startup_log_message()` fields:

```python
            ("workflow_hz", f"{cfg.runtime.workflow_hz:.3f}"),
            ("command_publish_hz", f"{cfg.runtime.command_publish_hz:.3f}"),
            ("command_timeout_s", f"{cfg.runtime.command_timeout_s:.3f}"),
```

- [ ] **Step 5: Add heartbeat warning only for publish jitter**

Add minimal state in `PidAlignmentRosNode.__init__`:

```python
        self._last_command_publish_monotonic_s: float | None = None
```

In `_on_command_timer()`:

```python
        now_s = time.monotonic()
        if self._last_command_publish_monotonic_s is not None:
            dt_s = now_s - self._last_command_publish_monotonic_s
            max_expected_s = 1.5 / self._cfg.runtime.command_publish_hz
            if dt_s > max_expected_s:
                self.get_logger().warning(
                    _format_multiline_log(
                        "command heartbeat publish delayed",
                        (
                            ("thread_role", "heartbeat"),
                            ("thread_name", _current_thread_name()),
                            ("dt_ms", f"{dt_s * 1000.0:.3f}"),
                            ("configured_hz", f"{self._cfg.runtime.command_publish_hz:.3f}"),
                        ),
                    )
                )
        self._last_command_publish_monotonic_s = now_s
```

This warning is acceptable because it only appears when the heartbeat itself is delayed, not on every successful heartbeat frame.

- [ ] **Step 6: Run log tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_status_align_phase.py tests/test_forward_approach_phase.py tests/test_base_coord_phase.py tests/test_pid_alignment_ros_node.py::test_workflow_timer_does_not_emit_standalone_heartbeat_summary tests/test_pid_alignment_ros_node.py::test_node_context_exposes_heartbeat_stats_for_phase_cycle_logs
```

Expected: PASS.

## Task 7: MultiThreadedExecutor Startup

**Files:**
- Modify: `src/runners/pid_alignment_ros_node.py`
- Modify: `tests/test_pid_alignment_ros_node.py`

- [ ] **Step 1: Update fake ROS imports**

In `import_with_fake_ros()`, add fake modules:

```python
    fake_callback_groups = types.ModuleType("rclpy.callback_groups")
    fake_callback_groups.MutuallyExclusiveCallbackGroup = lambda: object()

    class FakeMultiThreadedExecutor:
        def __init__(self, *, num_threads: int) -> None:
            self.num_threads = num_threads
            self.nodes: list[object] = []
            self.spin_called = False

        def add_node(self, node: object) -> None:
            self.nodes.append(node)

        def spin(self) -> None:
            self.spin_called = True

        def shutdown(self) -> None:
            return None

    fake_executors = types.ModuleType("rclpy.executors")
    fake_executors.MultiThreadedExecutor = FakeMultiThreadedExecutor

    monkeypatch.setitem(sys.modules, "rclpy.callback_groups", fake_callback_groups)
    monkeypatch.setitem(sys.modules, "rclpy.executors", fake_executors)
```

- [ ] **Step 2: Update main tests to expect executor**

Replace expectations around `rclpy.spin(node)` with:

```python
("executor_init", 2)
("executor_add_node", created_nodes[0])
("executor_spin", None)
("executor_shutdown", None)
```

One way to test this without relying on the fake import helper is monkeypatch `ros_node.MultiThreadedExecutor` directly:

```python
class FakeExecutor:
    def __init__(self, *, num_threads: int) -> None:
        calls.append(("executor_init", num_threads))

    def add_node(self, node: object) -> None:
        calls.append(("executor_add_node", node))

    def spin(self) -> None:
        calls.append(("executor_spin", None))

    def shutdown(self) -> None:
        calls.append(("executor_shutdown", None))

monkeypatch.setattr(ros_node, "MultiThreadedExecutor", FakeExecutor)
```

- [ ] **Step 3: Implement executor startup**

Replace:

```python
        node = PidAlignmentRosNode(cfg=cfg)
        rclpy.spin(node)
```

With:

```python
        node = PidAlignmentRosNode(cfg=cfg)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
```

In `finally`, shut down executor if created:

```python
    executor: MultiThreadedExecutor | None = None
    ...
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
```

- [ ] **Step 4: Run main tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_main_loads_config_and_spins_node tests/test_pid_alignment_ros_node.py::test_main_handles_keyboard_interrupt_without_traceback tests/test_pid_alignment_ros_node.py::test_main_handles_rclpy_context_shutdown_during_interrupt
```

Expected: PASS.

## Task 8: Full Verification and Manual Robot Check

**Files:**
- Modify only if tests expose a missed integration issue.

- [ ] **Step 1: Run targeted tests**

Run:

```bash
.venv/bin/python -m pytest -q tests/test_command_heartbeat.py tests/test_config_loaders.py tests/test_pid_alignment_ros_node.py
```

Expected: PASS.

- [ ] **Step 2: Run full suite**

Run:

```bash
.venv/bin/python -m pytest -q
```

Expected: PASS.

- [ ] **Step 3: Manual startup check**

Run:

```bash
.venv/bin/python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.yaml
```

Expected startup log includes:

```text
workflow_hz=10.000
command_publish_hz=30.000
command_timeout_s=0.250
```

Expected workflow-cycle log includes:

```text
status_align cycle
  ...
  filled_heartbeat_frames=...
```

- [ ] **Step 4: Manual ROS topic rate check**

In another terminal:

```bash
ros2 topic hz /t0x0101_robotfetch
```

Expected: observed rate should stay near `30Hz` and should not remain below `20Hz` during normal workflow operation. If it drops below 20Hz, collect logs with `command heartbeat publish delayed` and inspect whether Python scheduling, executor setup, or DDS publish is the limiting layer.

- [ ] **Step 5: Manual command content check**

Run:

```bash
ros2 topic echo /t0x0101_robotfetch
```

Expected:

- while STATUS_ALIGN produces a nonzero command, heartbeat repeats that same command between workflow ticks
- when workflow publishes zero, heartbeat repeats zero
- when phase sequence stops, a zero command is published immediately instead of waiting for timeout
- if workflow stalls longer than `command_timeout_s`, heartbeat publishes zero

## Self-Review Checklist

- [ ] Plan preserves single-node startup and does not add a second launch requirement.
- [ ] Command heartbeat does not depend on detector/camera/engine code.
- [ ] Thread communication has both latest-command buffering and explicit stop signal.
- [ ] Stop signal causes zero publish before shutdown, not after timeout.
- [ ] Logs identify `thread_role` and `thread_name`.
- [ ] Heartbeat does not print every publish at info level.
- [ ] Workflow-cycle log reports `filled_heartbeat_frames`.
- [ ] Config exposes workflow rate, heartbeat rate, and timeout.
- [ ] `command_publish_hz < 20` is rejected.
- [ ] Existing turtle bridge and `publish_cmd_vel` behavior remain separate.
- [ ] Full verification uses `.venv/bin/python -m pytest -q`.
