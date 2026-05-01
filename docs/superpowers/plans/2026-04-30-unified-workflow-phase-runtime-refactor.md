# Unified Workflow Phase Runtime Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the current `pid_alignment_ros_node.py` path into a single extensible workflow node where partial tests and full mission execution are both expressed by the configured phase sequence.

**Architecture:** Keep one ROS entrypoint and one workflow runtime. Move phase-specific behavior out of the ROS node into `PhaseRunner` classes, and move shared hardware/model lifetime into explicit resource sessions. Camera and detector resources are created only when a phase needs vision, then reused across later vision phases so `STATUS_ALIGN -> FORWARD_APPROACH -> BASE_COORD` does not close and reopen the camera.

**Tech Stack:** Python dataclasses/protocols, ROS 2 `rclpy`, `geometry_msgs/Twist`, `std_msgs/String`, `geometry_msgs/PointStamped`, existing `DetectorGateway`, existing config loader/model pattern, pytest.

---

## Background And Trigger

The project started from a partial runner focused on `STATUS_ALIGN`: read camera frames, detect status targets with BaseDetect, run image-space PID alignment, and publish workflow topics plus `/cmd_vel`. The recent forward patch added `FORWARD_APPROACH` as an open-loop phase controlled by `forward_approach.speed_mps` and `forward_approach.distance_m`.

That patch exposed an architectural problem. Running:

```bash
source scripts/activate_dev_env.sh
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.forward.yaml
```

with:

```yaml
start_phase: FORWARD_APPROACH
phase_sequence: [FORWARD_APPROACH]
```

should immediately publish/log a forward command:

```text
linear_x=0.500000
```

Instead, the node originally tried to open camera index `0` during startup and failed before the first timer tick. The immediate fix made `PidAlignmentRosNode` skip detector/camera setup when `start_phase` is `FORWARD_APPROACH`, and that patch was committed as:

```text
5243dee feat: add forward approach workflow phase
```

That fix is correct behaviorally, but it is a transitional design. It still leaves phase-resource knowledge inside `PidAlignmentRosNode`. The next refactor should remove that knowledge from the ROS shell instead of adding more `if phase == ...` logic.

## Current Project State

The current runner-based path is:

```text
configs/workflows/*.yaml
  -> config.loaders.load_pid_alignment_config()
  -> PidAlignmentRosNode
  -> run_status_align_once() or run_forward_approach_once()
  -> publishers/adapters
```

Important current files:

- `src/runners/pid_alignment_ros_node.py`: ROS shell, publisher construction, adapter construction, capture construction, detector construction, timer callback, phase branching.
- `src/runners/pid_alignment_runner.py`: stateless-ish functions for `STATUS_ALIGN` and `FORWARD_APPROACH` cycles, logging helpers, `RunnerConfig`.
- `src/algorithms/detector_gateway.py`: detector abstraction, supports status and base-coordinate target retrieval and profile switching.
- `src/config/models.py` and `src/config/loaders.py`: strict YAML config parsing.
- `src/workflow/types.py`: shared `Phase`, `AlgoStatus`, `EnvStatus`.
- `src/workflow/phase_controller.py`: lightweight current-phase tracker.

Current implemented phases in the runner path:

- `STATUS_ALIGN`: vision required. Uses camera, `DetectorGateway`, status profile, and image-space lateral PID.
- `FORWARD_APPROACH`: vision not required. Uses clock, fixed speed/distance config, and command publisher.

Expected near-term phase:

- `BASE_COORD`: vision required. It should switch BaseDetect to a base-coordinate profile and publish relevant base coordinate targets.

Expected future phase:

- angle/heading PID alignment before `STATUS_ALIGN`. This should be its own phase, likely `ANGLE_ALIGN` or `HEADING_ALIGN`. It should depend on robot pose/odom/yaw, not on status-target vision by default.

## User Requirements Captured

The user does not want a patch pile. The user explicitly cares about architecture, single responsibility, clean abstractions, and avoiding noisy conditionals in the ROS node.

Specific requirements:

- Do not keep separate "test runner" and "full mission runner" concepts unless there is a hard technical need.
- Use one node. Partial tests and full mission runs should both be configured by the action/phase sequence.
- If the user wants a full mission, they should fill the sequence, for example:

```yaml
phase_sequence: [ANGLE_ALIGN, STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]
```

- If the user wants a partial test, they should shorten the same sequence:

```yaml
phase_sequence: [FORWARD_APPROACH]
```

or:

```yaml
phase_sequence: [STATUS_ALIGN]
```

- `FORWARD_APPROACH` must not require camera startup.
- `STATUS_ALIGN -> FORWARD_APPROACH -> BASE_COORD` must not close camera after `STATUS_ALIGN` and reopen it for `BASE_COORD`.
- `BASE_COORD` must be able to switch detector profile and publish coordinate output.
- Future angle PID alignment must fit the model without modifying the ROS node main loop.
- README should be updated to describe the unified node and sequence-based execution model.

## Design Principles

1. `PidAlignmentRosNode` is a ROS shell.

It owns ROS-specific construction:

```text
publishers
timer
shutdown
message conversion
adapter wiring
```

It must not know the business details of each phase.

2. A phase declares what it needs.

Each phase runner should be understandable by its dependencies:

```text
ForwardApproachPhaseRunner -> clock + command publishing via context
StatusAlignPhaseRunner -> VisionSession + StatusAlignStep + publishers via context
BaseCoordPhaseRunner -> VisionSession + coordinate publisher via context
AngleAlignPhaseRunner -> PoseSession/OdomSession + angular PID + command publisher via context
```

3. Resources are not owned by phases.

Phases request resources from a shared runtime context. The context owns resource lifetime:

```text
VisionSession
  capture
  DetectorGateway
  active_profile
```

This prevents repeated open/close/open cycles between two vision phases separated by a non-vision phase.

4. The phase sequence is the mission definition.

There is no separate "full mission runner" in the current plan. A full mission is simply a longer `phase_sequence`. A partial test is a shorter `phase_sequence`.

5. Profile switching is phase-owned, resource-executed.

`StatusAlignPhaseRunner` should request `status_profile`. `BaseCoordPhaseRunner` should request `base_coord_profile`. `VisionSession` performs the switch only when the requested profile differs from the current one.

## Proposed Architecture

### Runtime Shape

```text
PidAlignmentRosNode
  creates RosPublishers
  creates WorkflowResources
  creates PhaseRegistry
  creates WorkflowEngine
  timer -> engine.tick()

WorkflowEngine
  owns phase_sequence
  owns current phase index
  calls current PhaseRunner.on_enter() once
  calls current PhaseRunner.tick() every timer
  advances when result says STEP_DONE / ALIGNED / DONE according to phase rules
  asks resources to release on final shutdown

PhaseRunner
  phase: Phase
  on_enter(context) -> None
  tick(context) -> PhaseTickResult
  on_exit(context) -> None

WorkflowContext
  cfg
  resources
  publishers
  logger
  clock

WorkflowResources
  vision() -> VisionSession
  release_all() -> None

VisionSession
  capture
  detector_gateway
  ensure_profile(profile)
  read_frame() -> FrameReadResult
```

### Key Dependency Direction

Allowed:

```text
pid_alignment_ros_node -> workflow.engine
workflow.engine -> phase runners
phase runners -> workflow.context
workflow.context -> resources/publishers/config
resources -> DetectorGateway/build_capture
```

Not allowed:

```text
resources -> phase runners
DetectorGateway -> ROS node
phase runner -> rclpy publisher construction
PidAlignmentRosNode -> StatusAlignStep details
PidAlignmentRosNode -> BaseCoord profile switching details
```

## File Structure Plan

Create:

- `src/workflow/phase_runner.py`
  - `PhaseTickResult`
  - `PhaseRunner` protocol or abstract base
  - `PhaseCompletionPolicy` if needed

- `src/workflow/runtime.py`
  - `WorkflowContext`
  - `WorkflowResources`
  - `VisionSession`
  - `FrameReadResult`
  - shared resource lifecycle methods

- `src/workflow/engine.py`
  - `WorkflowEngine`
  - sequence advancement
  - phase enter/exit handling
  - stop reason handling

- `src/runners/phases/__init__.py`

- `src/runners/phases/forward_approach_phase.py`
  - `ForwardApproachPhaseRunner`

- `src/runners/phases/status_align_phase.py`
  - `StatusAlignPhaseRunner`

- `src/runners/phases/base_coord_phase.py`
  - `BaseCoordPhaseRunner`

- `tests/test_workflow_engine.py`

- `tests/test_workflow_runtime_resources.py`

- `tests/test_forward_approach_phase.py`

- `tests/test_status_align_phase.py`

- `tests/test_base_coord_phase.py`

Modify:

- `src/runners/pid_alignment_ros_node.py`
  - remove phase-specific timer branching
  - construct engine/resources/phase registry

- `src/runners/pid_alignment_runner.py`
  - keep compatibility temporarily or shrink after phase migration

- `src/config/models.py`
  - add `base_coord` config model
  - add `detector.base_coord_profile`
  - later add `angle_align` config

- `src/config/loaders.py`
  - load new fields strictly

- `configs/workflows/pid_alignment.robot.yaml`
  - document unified sequence model
  - keep default partial sequence conservative

- `configs/workflows/pid_alignment.turtle.yaml`
  - document unified sequence model

- `configs/workflows/pid_alignment.robot.forward.yaml`
  - remain a single-node partial-test config

- `README.md`
  - remove wording that says forward/full mission belongs to a separate future runner
  - say one node supports partial and full mission by phase sequence

## Detailed Component Design

### PhaseTickResult

The engine should not infer phase state from arbitrary strings. Each phase returns a structured result.

Proposed shape:

```python
from dataclasses import dataclass

from workflow.types import AlgoStatus, EnvStatus, Phase


@dataclass(frozen=True)
class PhaseTickResult:
    phase: Phase
    algo_status: AlgoStatus
    env_status: EnvStatus
    command: dict[str, float] | None = None
    selected_target: dict[str, object] | None = None
    base_coord_targets: list[dict[str, object]] | None = None
    done: bool = False
    stop_reason: str | None = None
```

The `done` flag means the current phase finished and the engine may advance. The `stop_reason` is reserved for terminal or error states.

### PhaseRunner Contract

```python
from typing import Protocol

from workflow.phase_runner import PhaseTickResult
from workflow.runtime import WorkflowContext
from workflow.types import Phase


class PhaseRunner(Protocol):
    @property
    def phase(self) -> Phase:
        ...

    def on_enter(self, context: WorkflowContext) -> None:
        ...

    def tick(self, context: WorkflowContext) -> PhaseTickResult:
        ...

    def on_exit(self, context: WorkflowContext) -> None:
        ...
```

Default `on_enter` and `on_exit` can be no-ops via a small base class if that keeps implementations smaller.

### WorkflowEngine

Responsibilities:

- Parse and validate `phase_sequence` into `Phase`.
- Hold current phase index.
- Call `on_enter()` exactly once per phase entry.
- Call `tick()` on every timer.
- Call `on_exit()` when a phase completes.
- Advance to the next configured phase.
- Return terminal result when sequence is complete.

It should not open camera, switch detector profile, or publish ROS messages directly.

### WorkflowResources

Responsibilities:

- Lazily create `VisionSession` only when a phase calls `context.resources.vision()`.
- Keep `VisionSession` alive until workflow shutdown or explicit release.
- Later host `PoseSession` or `OdomSession` for angle PID.

Proposed API:

```python
class WorkflowResources:
    def __init__(self, *, cfg, logger) -> None:
        self._cfg = cfg
        self._logger = logger
        self._vision: VisionSession | None = None

    def vision(self) -> VisionSession:
        if self._vision is None:
            self._vision = VisionSession(cfg=self._cfg, logger=self._logger)
        return self._vision

    def release_all(self) -> None:
        if self._vision is not None:
            self._vision.release()
```

### VisionSession

Responsibilities:

- Open camera once.
- Build `DetectorGateway` once.
- Apply camera read fallback exactly as today.
- Track active detector profile.
- Switch profile only if different.
- Read frames and return structured frame-read status.

Important behavior:

```text
FORWARD_APPROACH only -> no VisionSession is constructed
STATUS_ALIGN -> VisionSession constructed with status profile
STATUS_ALIGN -> FORWARD_APPROACH -> BASE_COORD -> same VisionSession reused
BASE_COORD after FORWARD_APPROACH-only start -> VisionSession constructed only when BASE_COORD begins
```

### StatusAlignPhaseRunner

Responsibilities:

- On enter: `context.resources.vision().ensure_profile(cfg.detector.status_profile)`.
- On tick:
  - read frame through `VisionSession`
  - detect status targets
  - run `StatusAlignStep`
  - publish/log selected target and command through context/publishers
  - return `done=True` when aligned

It should not construct `DetectorGateway` or `VideoCapture`.

### ForwardApproachPhaseRunner

Responsibilities:

- On enter: record start time.
- On tick:
  - compute `elapsed_s`
  - publish/log fixed forward command until duration is reached
  - publish/log zero command at completion
  - return `done=True` once complete

It should not call `context.resources.vision()`.

### BaseCoordPhaseRunner

Responsibilities:

- On enter: `context.resources.vision().ensure_profile(cfg.detector.base_coord_profile)`.
- On tick:
  - read frame through `VisionSession`
  - detect base coordinate targets
  - publish coordinate targets
  - report `RUNNING`, `STEP_DONE`, or `TARGET_LOST` according to configured completion policy

The first implementation should define a simple completion rule:

```text
If at least one base-coordinate target is published in the current frame, return STEP_DONE and done=True.
If no target is available, keep RUNNING or TARGET_LOST but do not terminate unless a configured timeout is reached.
```

If the current BaseDetect SDK returns stabilized detection output from `Detector.detect(frame)`, `BaseCoordPhaseRunner` should consume that stable output. If it must use `latest_base_coord_targets()`, the code should document that this is current-frame output and not temporal smoothing.

### Future AngleAlignPhaseRunner

The architecture should reserve space for a future phase without implementing it in this refactor.

Expected dependencies:

```text
Pose/Odom/Yaw provider
Angular PID controller
cmd_vel angular_z output
```

It should be a peer of `StatusAlignPhaseRunner`, not an option inside it.

Expected future sequence:

```yaml
phase_sequence: [ANGLE_ALIGN, STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD]
```

The node should not require modification to add this phase. Only config model, phase runner, and phase registry should change.

## Implementation Plan

### Task 1: Add Phase Runner Contract And Engine

**Files:**

- Create: `src/workflow/phase_runner.py`
- Create: `src/workflow/engine.py`
- Test: `tests/test_workflow_engine.py`

- [ ] **Step 1: Write failing tests for sequence execution**

Create tests that prove:

```text
engine starts at first configured phase
engine calls on_enter once
engine calls tick repeatedly while done=False
engine advances after done=True
engine returns phase_sequence_complete after last phase
```

Use fake phase runners with counters.

- [ ] **Step 2: Run failing test**

Run:

```bash
python3 -m pytest tests/test_workflow_engine.py -q
```

Expected: fails because `workflow.engine` and `workflow.phase_runner` do not exist.

- [ ] **Step 3: Implement minimal contract and engine**

Implement `PhaseTickResult`, `PhaseRunner`, and `WorkflowEngine`.

Keep the engine independent of ROS and camera resources.

- [ ] **Step 4: Run test**

Run:

```bash
python3 -m pytest tests/test_workflow_engine.py -q
```

Expected: passes.

### Task 2: Add Workflow Resources And VisionSession

**Files:**

- Create: `src/workflow/runtime.py`
- Test: `tests/test_workflow_runtime_resources.py`

- [ ] **Step 1: Write failing tests for lazy vision resource creation**

Tests must prove:

```text
constructing WorkflowResources does not open camera
calling resources.vision() opens camera once
calling resources.vision() again returns same VisionSession
release_all() releases capture once
```

Use fake capture and fake detector factories.

- [ ] **Step 2: Write failing tests for profile reuse**

Tests must prove:

```text
ensure_profile(status_profile) switches only when needed
ensure_profile(base_coord_profile) switches detector profile without rebuilding detector
```

- [ ] **Step 3: Run failing tests**

Run:

```bash
python3 -m pytest tests/test_workflow_runtime_resources.py -q
```

Expected: fails because runtime resources do not exist.

- [ ] **Step 4: Implement `WorkflowResources` and `VisionSession`**

Move capture/detector construction details out of `PidAlignmentRosNode`, but keep existing fallback behavior intact.

- [ ] **Step 5: Run tests**

Run:

```bash
python3 -m pytest tests/test_workflow_runtime_resources.py -q
```

Expected: passes.

### Task 3: Migrate Forward Approach Into A Phase Runner

**Files:**

- Create: `src/runners/phases/__init__.py`
- Create: `src/runners/phases/forward_approach_phase.py`
- Test: `tests/test_forward_approach_phase.py`

- [ ] **Step 1: Write failing tests for no vision access**

Use a context whose `resources.vision()` raises `AssertionError`. The test must show `ForwardApproachPhaseRunner.tick()` publishes/logs `linear_x` without touching vision resources.

- [ ] **Step 2: Write failing tests for completion**

Test:

```text
before duration -> linear_x=speed, done=False
after duration -> linear_x=0.0, done=True, algo_status=STEP_DONE
```

- [ ] **Step 3: Run failing tests**

Run:

```bash
python3 -m pytest tests/test_forward_approach_phase.py -q
```

Expected: fails because the phase runner does not exist.

- [ ] **Step 4: Implement `ForwardApproachPhaseRunner`**

Move behavior from `run_forward_approach_once()` into the class while preserving log fields:

```text
phase
speed_mps
distance_m
duration_s
elapsed_s
remaining_s
cmd_topic
linear_x
linear_y
angular_z
algo_status
env_status
```

- [ ] **Step 5: Run tests**

Run:

```bash
python3 -m pytest tests/test_forward_approach_phase.py -q
```

Expected: passes.

### Task 4: Migrate Status Align Into A Phase Runner

**Files:**

- Create: `src/runners/phases/status_align_phase.py`
- Test: `tests/test_status_align_phase.py`

- [ ] **Step 1: Write failing tests for vision usage**

Use a fake `VisionSession` that records:

```text
ensure_profile(status_profile)
read_frame()
detect_status_targets(frame)
```

Assert these are called through `WorkflowResources`, not constructed in the phase.

- [ ] **Step 2: Write failing tests for aligned completion**

Test that when `StatusAlignStep` returns `ALIGNED`, the phase result has:

```text
done=True
algo_status=ALIGNED
phase=STATUS_ALIGN
```

- [ ] **Step 3: Run failing tests**

Run:

```bash
python3 -m pytest tests/test_status_align_phase.py -q
```

Expected: fails because the phase runner does not exist.

- [ ] **Step 4: Implement `StatusAlignPhaseRunner`**

Preserve current target selection, command publishing, selected target publishing, and cycle log content.

- [ ] **Step 5: Run tests**

Run:

```bash
python3 -m pytest tests/test_status_align_phase.py -q
```

Expected: passes.

### Task 5: Add Base Coord Config And Phase Runner

**Files:**

- Modify: `src/config/models.py`
- Modify: `src/config/loaders.py`
- Create: `src/runners/phases/base_coord_phase.py`
- Test: `tests/test_config_loaders.py`
- Test: `tests/test_base_coord_phase.py`

- [ ] **Step 1: Write failing config loader tests**

Add YAML test coverage for:

```yaml
detector:
  status_profile: status_competition
  base_coord_profile: base_coord_competition
base_coord:
  publish_topic: /robot_fetch/base_coord_targets
  frame_id: camera_link
  complete_on_first_target: true
```

Expected loaded config should expose a structured `base_coord` config.

- [ ] **Step 2: Write failing phase tests**

Use fake `VisionSession` and fake publisher. Assert:

```text
on_enter switches to detector.base_coord_profile
tick reads from the same VisionSession
tick publishes coordinate targets
tick returns STEP_DONE when complete_on_first_target is true and targets exist
```

- [ ] **Step 3: Run failing tests**

Run:

```bash
python3 -m pytest tests/test_config_loaders.py tests/test_base_coord_phase.py -q
```

Expected: fails for missing config fields and missing phase runner.

- [ ] **Step 4: Implement config and phase runner**

Add `BaseCoordConfig` to config models and loader. Implement `BaseCoordPhaseRunner` using `context.resources.vision()`.

- [ ] **Step 5: Run tests**

Run:

```bash
python3 -m pytest tests/test_config_loaders.py tests/test_base_coord_phase.py -q
```

Expected: passes.

### Task 6: Refactor PidAlignmentRosNode Into A ROS Shell

**Files:**

- Modify: `src/runners/pid_alignment_ros_node.py`
- Test: `tests/test_pid_alignment_ros_node.py`

- [ ] **Step 1: Write failing tests for node not knowing phase internals**

Patch in a fake `WorkflowEngine` and assert:

```text
PidAlignmentRosNode._on_timer() calls engine.tick()
PidAlignmentRosNode does not call build_capture directly
PidAlignmentRosNode does not call DetectorGateway directly
PidAlignmentRosNode shuts down when engine returns terminal result
```

- [ ] **Step 2: Run failing tests**

Run:

```bash
python3 -m pytest tests/test_pid_alignment_ros_node.py -q
```

Expected: fails while node still contains direct phase/capture logic.

- [ ] **Step 3: Rewrite node construction**

Node should construct:

```text
RosPublishers
WorkflowResources
PhaseRegistry
WorkflowEngine
```

Timer should call:

```python
result = self._engine.tick()
```

The node should keep ROS lifecycle behavior and shutdown logging.

- [ ] **Step 4: Run tests**

Run:

```bash
python3 -m pytest tests/test_pid_alignment_ros_node.py -q
```

Expected: passes.

### Task 7: Update Configs And README For Unified Node

**Files:**

- Modify: `README.md`
- Modify: `configs/workflows/pid_alignment.robot.yaml`
- Modify: `configs/workflows/pid_alignment.turtle.yaml`
- Modify: `configs/workflows/pid_alignment.robot.forward.yaml`
- Test: existing README/config tests if present

- [ ] **Step 1: Update README wording**

README must say:

```text
There is one runner node: src/runners/pid_alignment_ros_node.py.
Partial tests and full mission runs use the same node.
The configured phase_sequence is the mission/action sequence.
Use [STATUS_ALIGN] for status-only validation.
Use [FORWARD_APPROACH] for forward-only validation.
Use [STATUS_ALIGN, FORWARD_APPROACH, BASE_COORD] for a longer robot flow.
Future angle PID alignment can be added as an earlier phase in the same sequence.
```

Remove wording that implies forward/base/full mission requires a separate future runner.

- [ ] **Step 2: Update config comments**

Config comments should describe sequence-based execution, not MVP-vs-full-mission split.

- [ ] **Step 3: Run focused tests**

Run:

```bash
python3 -m pytest tests/test_config_loaders.py tests/test_pid_alignment_ros_node.py -q
```

Expected: passes.

### Task 8: Compatibility Cleanup

**Files:**

- Modify: `src/runners/pid_alignment_runner.py`
- Modify: tests that import `run_status_align_once()` or `run_forward_approach_once()`

- [ ] **Step 1: Decide compatibility boundary**

Keep old functions only if they remain useful as pure cycle helpers behind phase runners. Otherwise remove them after phase runner tests cover their behavior.

- [ ] **Step 2: Move tests to phase runner tests**

Existing behavior tests should live with:

```text
tests/test_forward_approach_phase.py
tests/test_status_align_phase.py
tests/test_base_coord_phase.py
tests/test_workflow_engine.py
tests/test_workflow_runtime_resources.py
```

- [ ] **Step 3: Run full focused workflow tests**

Run:

```bash
python3 -m pytest tests/test_workflow_engine.py tests/test_workflow_runtime_resources.py tests/test_forward_approach_phase.py tests/test_status_align_phase.py tests/test_base_coord_phase.py tests/test_pid_alignment_ros_node.py tests/test_config_loaders.py -q
```

Expected: passes.

### Task 9: Full Verification

**Files:**

- No expected source changes.

- [ ] **Step 1: Run full test suite**

Run:

```bash
python3 -m pytest -q
```

Expected in a complete local environment: all tests pass.

Known current caveat: in the current environment, full suite may fail at `tests/test_basedetect_detector_runtime.py` because `torch` is not installed. If that remains true, report it explicitly and include the focused workflow test result.

- [ ] **Step 2: Run forward-only startup smoke**

Run:

```bash
source scripts/activate_dev_env.sh
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.forward.yaml
```

Expected:

```text
capture startup skipped
forward_approach cycle
linear_x=0.500000
```

- [ ] **Step 3: Run status-only startup smoke when camera is available**

Run:

```bash
source scripts/activate_dev_env.sh
python src/runners/pid_alignment_ros_node.py --config configs/workflows/pid_alignment.robot.yaml
```

Expected:

```text
capture opened
status_align cycle
```

## Acceptance Criteria

- `PidAlignmentRosNode` no longer contains phase-specific business branching for `STATUS_ALIGN`, `FORWARD_APPROACH`, or `BASE_COORD`.
- `FORWARD_APPROACH` single-phase startup does not initialize camera or detector.
- `STATUS_ALIGN -> FORWARD_APPROACH -> BASE_COORD` initializes camera once and reuses it across both vision phases.
- `BASE_COORD` switches detector profile through `VisionSession.ensure_profile()` rather than rebuilding `DetectorGateway`.
- README documents one unified node and sequence-based partial/full execution.
- Adding future `ANGLE_ALIGN` requires adding a phase runner and config, not editing the node timer logic.

## Risks And Guardrails

- Do not hide camera fallback logic inside phase runners. It belongs in `VisionSession` because it is resource behavior.
- Do not put profile switching in `PidAlignmentRosNode`. It belongs in vision-using phase runners through `VisionSession`.
- Do not split into a separate full-mission node. The unified sequence is the mission definition.
- Do not add robot-specific transport fields to generic algorithm config. Keep adapter/environment details at the ROS shell or adapter boundary.
- Do not claim BaseDetect temporal smoothing is used unless the code consumes the stabilized detector output. If `latest_base_coord_targets()` is used, document it as current-frame cache output.

## Suggested Commit Sequence

1. `refactor: add workflow phase engine`
2. `refactor: add workflow resource sessions`
3. `refactor: move forward approach into phase runner`
4. `refactor: move status align into phase runner`
5. `feat: add base coord workflow phase`
6. `refactor: make pid alignment node workflow-driven`
7. `docs: document unified workflow sequence model`

