# Workflow Assembly MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Assemble the existing refactored workflow, algorithm, and adapter pieces into a runnable `pid_alignment_runner` MVP that can start from `STATUS_ALIGN`, switch between `turtle` and `robot` environments, and support one-shot alignment tests.

**Architecture:** Keep the current separation between workflow, algorithm, adapter, and runner layers. Add one thin config layer plus one ROS runtime wrapper so the existing pure modules stay reusable, while the runner becomes the single startup path for partial workflow testing.

**Tech Stack:** Python 3.12, ROS 2 Jazzy, `rclpy`, BaseDetect SDK, YAML config loading, pytest

---

## File Structure

### Files to create

- `src/config/models.py`
  - Dataclasses for runner, topic, detector, input, phase, and adapter configuration.
- `src/config/loaders.py`
  - YAML loader that converts a workflow config file into typed config models.
- `src/adapters/base.py`
  - Shared adapter protocol for turtle and robot environments.
- `src/adapters/robot_adapter.py`
  - Real-robot adapter MVP that passes `cmd_vel` through unchanged and reports environment readiness.
- `src/runners/pid_alignment_ros_node.py`
  - Executable ROS node wrapper that wires config, detector gateway, status-align logic, publishers, adapter, and timer loop together.
- `configs/workflows/pid_alignment.turtle.yaml`
  - Turtle workflow scenario config.
- `configs/workflows/pid_alignment.robot.yaml`
  - Real-robot workflow scenario config.
- `tests/test_config_loaders.py`
  - Loader and config validation tests.
- `tests/test_robot_adapter.py`
  - Robot adapter tests.
- `tests/test_pid_alignment_ros_node.py`
  - Runner assembly tests around startup, one-shot behavior, adapter selection, and topic publication.

### Files to modify

- `src/adapters/turtle_adapter.py`
  - Align the turtle adapter with the shared adapter protocol.
- `src/runners/pid_alignment_runner.py`
  - Keep it pure, but add the small interfaces the ROS node wrapper needs.
- `README.md`
  - Replace the old startup narrative with the new runner-based workflow startup path.

### Files explicitly out of scope for this plan

- `src/algorithms/forward_approach.py`
- `src/algorithms/base_coord.py`
- `src/runners/full_mission_runner.py`

Those belong in the next plan after this MVP runner is working.

---

### Task 1: Add Typed Workflow Config Models And Loader

**Files:**
- Create: `src/config/models.py`
- Create: `src/config/loaders.py`
- Create: `tests/test_config_loaders.py`

- [ ] **Step 1: Write the failing config loader test**

```python
from pathlib import Path

from config.loaders import load_pid_alignment_config


def test_load_pid_alignment_config_reads_turtle_workflow_file():
    cfg = load_pid_alignment_config(
        Path("configs/workflows/pid_alignment.turtle.yaml")
    )

    assert cfg.environment == "turtle"
    assert cfg.start_phase == "STATUS_ALIGN"
    assert cfg.one_shot is True
    assert cfg.adapter.turtle_cmd_topic == "/turtle1/cmd_vel"
    assert cfg.detector.status_profile == "status_competition"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_config_loaders.py::test_load_pid_alignment_config_reads_turtle_workflow_file`

Expected: FAIL with `ModuleNotFoundError: No module named 'config'` or missing loader symbol.

- [ ] **Step 3: Write minimal config models**

```python
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TopicConfig:
    cmd_topic: str
    workflow_phase_topic: str
    algo_status_topic: str
    env_status_topic: str
    selected_status_topic: str


@dataclass(frozen=True)
class DetectorConfig:
    sdk_config: Path
    status_profile: str
    input_source: str


@dataclass(frozen=True)
class AdapterConfig:
    turtle_cmd_topic: str | None


@dataclass(frozen=True)
class PidAlignmentWorkflowConfig:
    environment: str
    start_phase: str
    one_shot: bool
    target_x: float
    tolerance_px: float
    topics: TopicConfig
    detector: DetectorConfig
    adapter: AdapterConfig
```

- [ ] **Step 4: Write minimal YAML loader**

```python
from pathlib import Path

import yaml

from .models import (
    AdapterConfig,
    DetectorConfig,
    PidAlignmentWorkflowConfig,
    TopicConfig,
)


def load_pid_alignment_config(path: Path) -> PidAlignmentWorkflowConfig:
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    root = payload["pid_alignment_workflow"]

    return PidAlignmentWorkflowConfig(
        environment=str(root["environment"]),
        start_phase=str(root.get("start_phase", "STATUS_ALIGN")),
        one_shot=bool(root.get("one_shot", False)),
        target_x=float(root["status_align"]["target_x"]),
        tolerance_px=float(root["status_align"]["tolerance_px"]),
        topics=TopicConfig(**root["topics"]),
        detector=DetectorConfig(
            sdk_config=Path(root["detector"]["sdk_config"]),
            status_profile=str(root["detector"]["status_profile"]),
            input_source=str(root["detector"]["input_source"]),
        ),
        adapter=AdapterConfig(
            turtle_cmd_topic=root["adapter"].get("turtle_cmd_topic"),
        ),
    )
```

- [ ] **Step 5: Add the two scenario YAML files**

`configs/workflows/pid_alignment.turtle.yaml`

```yaml
pid_alignment_workflow:
  environment: turtle
  start_phase: STATUS_ALIGN
  one_shot: true
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
    turtle_cmd_topic: /turtle1/cmd_vel
  status_align:
    target_x: 320.0
    tolerance_px: 8.0
```

`configs/workflows/pid_alignment.robot.yaml`

```yaml
pid_alignment_workflow:
  environment: robot
  start_phase: STATUS_ALIGN
  one_shot: true
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
```

- [ ] **Step 6: Run the loader tests**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_config_loaders.py`

Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add src/config/models.py src/config/loaders.py configs/workflows/pid_alignment.turtle.yaml configs/workflows/pid_alignment.robot.yaml tests/test_config_loaders.py
git commit -m "feat: add workflow config loader for pid alignment"
```

---

### Task 2: Introduce A Shared Adapter Contract And Add Robot Adapter

**Files:**
- Create: `src/adapters/base.py`
- Create: `src/adapters/robot_adapter.py`
- Modify: `src/adapters/turtle_adapter.py`
- Create: `tests/test_robot_adapter.py`

- [ ] **Step 1: Write the failing robot adapter test**

```python
from types import SimpleNamespace

from adapters.robot_adapter import RobotAdapter
from workflow.types import EnvStatus, Phase


def test_robot_adapter_marks_status_align_ready_and_passthroughs_cmd_vel():
    adapter = RobotAdapter(env_status_publisher=lambda _: None)

    env_status = adapter.on_phase(Phase.STATUS_ALIGN)
    command = adapter.on_cmd_vel(
        SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.12, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.05),
        )
    )

    assert env_status is EnvStatus.READY
    assert command.linear_y == 0.12
    assert command.angular_z == 0.05
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_robot_adapter.py::test_robot_adapter_marks_status_align_ready_and_passthroughs_cmd_vel`

Expected: FAIL with missing adapter module.

- [ ] **Step 3: Add a shared adapter protocol**

```python
from dataclasses import dataclass
from typing import Any, Protocol

from workflow.types import EnvStatus, Phase


@dataclass(frozen=True)
class AdapterCommand:
    linear_x: float
    linear_y: float
    angular_z: float


class WorkflowAdapter(Protocol):
    def on_phase(self, phase: Phase | str) -> EnvStatus:
        pass

    def on_cmd_vel(self, cmd_vel: Any) -> AdapterCommand:
        pass
```

- [ ] **Step 4: Implement the robot adapter**

```python
from workflow.types import EnvStatus, Phase, parse_phase

from .base import AdapterCommand


class RobotAdapter:
    def __init__(self, *, env_status_publisher):
        self._env_status_publisher = env_status_publisher
        self._current_env_status = EnvStatus.IDLE

    def on_phase(self, phase: Phase | str) -> EnvStatus:
        current_phase = parse_phase(phase)
        self._current_env_status = (
            EnvStatus.READY if current_phase is Phase.STATUS_ALIGN else EnvStatus.IDLE
        )
        self._env_status_publisher(self._current_env_status.value)
        return self._current_env_status

    def on_cmd_vel(self, cmd_vel):
        return AdapterCommand(
            linear_x=float(cmd_vel.linear.x),
            linear_y=float(cmd_vel.linear.y),
            angular_z=float(cmd_vel.angular.z),
        )
```

- [ ] **Step 5: Align the turtle adapter with the shared contract**

```python
from .base import AdapterCommand


def map_cmd_vel_to_turtle_command(cmd_vel: Any) -> AdapterCommand:
    return AdapterCommand(
        linear_x=float(cmd_vel.linear.y),
        linear_y=0.0,
        angular_z=float(cmd_vel.angular.z),
    )
```

- [ ] **Step 6: Run adapter tests**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_turtle_adapter.py tests/test_robot_adapter.py`

Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add src/adapters/base.py src/adapters/robot_adapter.py src/adapters/turtle_adapter.py tests/test_robot_adapter.py
git commit -m "feat: add workflow adapter contract and robot adapter"
```

---

### Task 3: Turn The Pure Runner Into A Reusable Assembly Core

**Files:**
- Modify: `src/runners/pid_alignment_runner.py`
- Create: `tests/test_pid_alignment_runner_node.py`

- [ ] **Step 1: Write the failing one-shot runner test**

```python
from workflow.types import AlgoStatus

from runners.pid_alignment_runner import should_stop_after_status_align


def test_should_stop_after_status_align_when_one_shot_and_aligned():
    assert should_stop_after_status_align(
        one_shot=True,
        algo_status=AlgoStatus.ALIGNED,
    ) is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_runner_node.py::test_should_stop_after_status_align_when_one_shot_and_aligned`

Expected: FAIL with missing symbol.

- [ ] **Step 3: Add the pure stop decision helper**

```python
from workflow.types import AlgoStatus


def should_stop_after_status_align(*, one_shot: bool, algo_status: AlgoStatus) -> bool:
    if not one_shot:
        return False
    return algo_status in {AlgoStatus.ALIGNED, AlgoStatus.TARGET_LOST, AlgoStatus.ERROR}
```

- [ ] **Step 4: Make `run_status_align_once` return a structured cycle result**

```python
from dataclasses import dataclass

from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass(frozen=True)
class StatusAlignCycleResult:
    phase: str
    algo_status: str
    env_status: str
    command_x: float
    stop_requested: bool


def run_status_align_once(
    *,
    node,
    frame,
    detector_gateway,
    status_align_step,
    cfg,
    one_shot: bool = False,
) -> StatusAlignCycleResult:
    detection = detector_gateway.detect_status_targets(frame)
    now_s = node.get_clock().now().nanoseconds * 1e-9
    result = status_align_step.run(targets=detection.targets, now_s=now_s)

    env_status = (
        EnvStatus.RUNNING.value if detection.ready else EnvStatus.READY.value
    )

    node.cmd_pub.publish(_build_cmd_message(result.command_x))
    node.phase_pub.publish(Phase.STATUS_ALIGN.value)
    node.algo_status_pub.publish(result.status.value)
    node.env_status_pub.publish(env_status)

    return StatusAlignCycleResult(
        phase=Phase.STATUS_ALIGN.value,
        algo_status=result.status.value,
        env_status=env_status,
        command_x=result.command_x,
        stop_requested=should_stop_after_status_align(
            one_shot=one_shot,
            algo_status=result.status,
        ),
    )
```

- [ ] **Step 5: Update tests to assert the returned cycle result**

```python
assert cycle.stop_requested is False
assert cycle.algo_status == AlgoStatus.RUNNING.value
```

- [ ] **Step 6: Run runner tests**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_runner.py tests/test_pid_alignment_runner_node.py`

Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add src/runners/pid_alignment_runner.py tests/test_pid_alignment_runner.py tests/test_pid_alignment_runner_node.py
git commit -m "feat: add one-shot runner assembly behavior"
```

---

### Task 4: Add A Real ROS Startup Node For `pid_alignment_runner`

**Files:**
- Create: `src/runners/pid_alignment_ros_node.py`
- Create: `tests/test_pid_alignment_ros_node.py`

- [ ] **Step 1: Write the failing startup-node test**

```python
from pathlib import Path

from runners.pid_alignment_ros_node import build_adapter


def test_build_adapter_returns_turtle_adapter_for_turtle_environment():
    adapter = build_adapter(environment="turtle", env_status_publisher=lambda _: None)
    assert adapter.__class__.__name__ == "TurtleAdapter"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_build_adapter_returns_turtle_adapter_for_turtle_environment`

Expected: FAIL with missing module.

- [ ] **Step 3: Add the adapter factory and ROS node shell**

```python
def build_adapter(*, environment: str, env_status_publisher):
    if environment == "turtle":
        return TurtleAdapter(env_status_publisher=env_status_publisher)
    if environment == "robot":
        return RobotAdapter(env_status_publisher=env_status_publisher)
    raise ValueError(f"Unsupported environment: {environment}")


class PidAlignmentRosNode(Node):
    def __init__(self, *, cfg: PidAlignmentWorkflowConfig) -> None:
        super().__init__("pid_alignment_runner")
        self._cfg = cfg
        self._runner_cfg = RunnerConfig(
            cmd_topic=cfg.topics.cmd_topic,
            selected_status_topic=cfg.topics.selected_status_topic,
            workflow_phase_topic=cfg.topics.workflow_phase_topic,
            algo_status_topic=cfg.topics.algo_status_topic,
            env_status_topic=cfg.topics.env_status_topic,
            frame_id="camera_link",
        )
        self._adapter = build_adapter(
            environment=cfg.environment,
            env_status_publisher=lambda _: None,
        )
        self._gateway = DetectorGateway(
            config_path=cfg.detector.sdk_config,
            initial_profile=cfg.detector.status_profile,
        )
        self._status_align = StatusAlignStep(
            cfg=StatusAlignConfig(
                pid=PIDConfig(
                    kp=0.006,
                    ki=0.0,
                    kd=0.0008,
                    output_limit=0.25,
                    integral_limit=1000.0,
                    deadband=0.0,
                    derivative_alpha=0.35,
                ),
                target_x=cfg.target_x,
                tolerance_px=cfg.tolerance_px,
                allowed_labels={"spearhead", "fist", "palm"},
                stable_labels=set(),
                use_stable_labels=True,
            )
        )
```

- [ ] **Step 4: Add the timer cycle that publishes all workflow topics**

```python
def _on_timer(self) -> None:
    ok, frame = self._cap.read()
    if not ok:
        self.get_logger().warning("Frame read failed")
        return

    self._adapter.on_phase(self._cfg.start_phase)
    cycle = run_status_align_once(
        node=self,
        frame=frame,
        detector_gateway=self._gateway,
        status_align_step=self._status_align,
        cfg=self._runner_cfg,
        one_shot=self._cfg.one_shot,
    )

    if cycle.stop_requested:
        self.destroy_node()
```

- [ ] **Step 5: Add the executable `main()` wrapper**

```python
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        default="configs/workflows/pid_alignment.robot.yaml",
    )
    args = parser.parse_args()

    rclpy.init()
    cfg = load_pid_alignment_config(Path(args.config))
    node = PidAlignmentRosNode(cfg=cfg)
    rclpy.spin(node)
    rclpy.shutdown()
```

- [ ] **Step 6: Run startup-node tests**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py`

Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add src/runners/pid_alignment_ros_node.py tests/test_pid_alignment_ros_node.py
git commit -m "feat: add runnable ros node for pid alignment workflow"
```

---

### Task 5: Publish The New Runner Startup Path And Deprecate The Old One

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Write the failing docs smoke check**

```python
from pathlib import Path


def test_readme_mentions_pid_alignment_runner_startup():
    readme = Path("README.md").read_text(encoding="utf-8")
    assert "python src/runners/pid_alignment_ros_node.py --config" in readme
```

- [ ] **Step 2: Run test to verify it fails**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_readme_mentions_pid_alignment_runner_startup`

Expected: FAIL because README still points at the old startup narrative.

- [ ] **Step 3: Replace the startup section with the runner-based flow**

````markdown
## Runner-Based Workflow Startup

Turtle partial test:

```bash
python src/runners/pid_alignment_ros_node.py \
  --config configs/workflows/pid_alignment.turtle.yaml
```

Real robot partial test:

```bash
python src/runners/pid_alignment_ros_node.py \
  --config configs/workflows/pid_alignment.robot.yaml
```

One-shot mode is controlled in YAML with `one_shot: true`.
Forward motion is not part of this MVP runner and remains deferred to the full mission runner.
````

- [ ] **Step 4: Run the docs smoke check**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q tests/test_pid_alignment_ros_node.py::test_readme_mentions_pid_alignment_runner_startup`

Expected: PASS

- [ ] **Step 5: Run the full MVP regression suite**

Run: `set +u; source /opt/ros/jazzy/setup.bash; set -u; .venv/bin/python -m pytest -q`

Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add README.md
git commit -m "docs: document runner-based workflow startup"
```

---

## Acceptance Checklist

- [ ] `pid_alignment_runner` has a real executable startup path.
- [ ] Runner startup can choose `turtle` or `robot` through config rather than code edits.
- [ ] The startup path can begin from `STATUS_ALIGN`.
- [ ] The workflow publishes `/workflow/phase`, `/workflow/algo_status`, `/workflow/env_status`, and `/cmd_vel`.
- [ ] One-shot mode stops after a single `STATUS_ALIGN` completion event.
- [ ] README explains how to launch the new runner for turtle and robot partial tests.
- [ ] Old `terminal_pid_follower_node.py` is no longer the documented startup path for the refactored architecture.

## Deferred To The Next Plan

- Implement `src/algorithms/forward_approach.py`
- Implement `src/algorithms/base_coord.py`
- Add `src/runners/full_mission_runner.py`
- Add a richer robot adapter that waits for real environment readiness rather than publishing `READY` immediately
- Add field-specific startup profiles for competition execution

## Self-Review

### Spec coverage

- Workflow phase/status model: covered by reusing `src/workflow/types.py` and `src/workflow/phase_controller.py` through Tasks 3 and 4.
- `pid_alignment_runner`: covered directly in Tasks 3 and 4.
- `turtle_adapter`: covered in Task 2 integration and Task 4 assembly.
- `robot_adapter`: covered in Task 2.
- Arbitrary phase start support: covered by Task 1 config and Task 4 startup node wiring.
- Partial turtle and real-robot testing: covered by Task 1 scenario files and Task 5 startup docs.

### Placeholder scan

- No `TODO`, `TBD`, or “implement later” steps inside the planned scope.
- Deferred items are isolated under a separate deferred section, not hidden in execution tasks.

### Type consistency

- Config root type: `PidAlignmentWorkflowConfig`
- Adapter contract command type: `AdapterCommand`
- Runner cycle output: `StatusAlignCycleResult`
- Startup entry: `PidAlignmentRosNode`
