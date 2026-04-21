# RobotFetch Workflow Refactor Design

Date: 2026-04-21

## 1. Goal

This refactor is intended to solve three practical problems in the current RobotFetch project:

1. BaseDetect + PID lateral alignment must be testable independently without requiring the full competition environment.
2. The same algorithm core must be reusable with different environments, especially turtle and the real robot.
3. The workflow must support starting from arbitrary phases so partial testing,现场调试, and full-mission execution all use the same architecture rather than separate code paths.

The design goal is not to build a heavy robot framework. The design goal is to create a small, orthogonal workflow architecture that keeps the algorithm core stable while allowing different environments and test scenarios to be plugged in.

## 2. Design Principles

The implementation should follow these principles:

- Keep environment, algorithm, and workflow responsibilities separate.
- Let every phase be entered independently without hidden dependence on previous phases.
- Let turtle testing and real-robot testing reuse the same algorithm modules.
- Keep full competition mode as an assembly of smaller validated steps rather than a separate implementation.
- Avoid heavy orchestration frameworks, behavior trees, or over-generalized abstractions in the first iteration.

## 3. Proposed Architecture

The system is split into four layers.

### 3.1 Workflow Layer

The workflow layer defines the current mission phase shared by the whole system.

Responsibilities:
- define the global phase model
- expose the current phase
- support external phase injection at startup
- support phase-local reset behavior when entering a phase

This layer does not do detection, PID control, or robot motion.

### 3.2 Algorithm Layer

The algorithm layer converts perception input into control output or mission results.

Responsibilities:
- call BaseDetect through a stable gateway
- select the active target
- run PID lateral alignment
- determine whether alignment is finished
- perform forward-approach logic
- publish or expose base-coordinate results

This layer only depends on:
- current workflow phase
- perception input
- algorithm configuration

This layer must not depend on whether the environment is turtle or the real robot.

### 3.3 Adapter Layer

The adapter layer connects the external environment to the shared workflow contract.

Examples:
- turtle adapter
- real robot adapter

Responsibilities:
- consume workflow phase
- consume velocity commands
- prepare the environment for the current phase
- publish environment execution status

This layer must not contain PID logic, target selection, or mission sequencing.

### 3.4 Runner Layer

The runner layer assembles the system for a concrete scenario.

Two runners are required in the first design:
- `pid_alignment_runner`: only runs the status-alignment workflow
- `full_mission_runner`: runs `STATUS_ALIGN -> FORWARD_APPROACH -> BASE_COORD`

The runner advances workflow phases and wires modules together, but does not implement PID or detection logic.

## 4. Workflow Model

### 4.1 Global Mission Phases

The first iteration should keep a minimal global phase set:

- `READY`
- `STATUS_ALIGN`
- `FORWARD_APPROACH`
- `BASE_COORD`
- `DONE`
- `ABORT`

These phases are system-level states shared by all participants.

### 4.2 Algorithm Status

The algorithm reports its current execution status separately from the global phase.

Suggested values:
- `IDLE`
- `WAITING_FOR_PHASE`
- `RUNNING`
- `TARGET_LOST`
- `ALIGNED`
- `STEP_DONE`
- `ERROR`

These statuses are used for debugging and phase coordination, not as replacements for the global workflow phase.

### 4.3 Environment Status

The environment adapter reports whether it is ready to cooperate with the current phase.

Suggested values:
- `IDLE`
- `PREPARING`
- `READY`
- `RUNNING`
- `DONE`
- `ERROR`

These statuses allow turtle and real-robot workflows to share the same coordination model.

## 5. ROS Communication Contract

The design uses a small shared contract rather than a heavy orchestration system.

### Required topics

- `/workflow/phase`
  - system command state: which mission phase should run now
- `/workflow/algo_status`
  - algorithm execution feedback for the current phase
- `/workflow/env_status`
  - environment execution feedback for the current phase
- `/cmd_vel`
  - control command output from the algorithm side
- `/robot_fetch/selected_target_px`
  - currently selected status target in pixel space
- `/robot_fetch/target_position`
  - published base 3D coordinate result

### Semantic rule

- `phase` is the commanded mission phase
- `algo_status` is the algorithm-side execution feedback
- `env_status` is the environment-side execution feedback

This distinction is required so all participants can know both what the system is asking for and whether each side is actually ready or finished.

## 6. Phase Entry and Arbitrary Start Support

A core architectural requirement is:

> Every phase must be independently enterable and must reset its own local context on entry.

That means:
- `STATUS_ALIGN` must reset PID state and target cache when entered
- `FORWARD_APPROACH` must not assume status alignment ran immediately before it
- `BASE_COORD` must be able to start directly and switch detector profile cleanly

This rule is what allows:
- turtle testing from a partial workflow
- real-robot partial testing without full competition coordinates
- direct debugging from any mission step
- future full-mission execution without branching code paths

## 7. File and Module Boundaries

The current monolithic `src/terminal_pid_follower_node.py` should be split into focused modules.

### 7.1 Workflow modules

- `src/workflow/types.py`
  - define `Phase`, `AlgoStatus`, `EnvStatus`
- `src/workflow/phase_controller.py`
  - phase transitions
  - startup phase injection
  - phase-local reset entrypoints

### 7.2 Algorithm modules

- `src/algorithms/pid.py`
  - pure PID controller
- `src/algorithms/target_selection.py`
  - status-target and base-target selection logic
- `src/algorithms/detector_gateway.py`
  - wrap BaseDetect profile switching and result access
- `src/algorithms/status_align.py`
  - status alignment phase logic
- `src/algorithms/forward_approach.py`
  - forward approach phase logic
- `src/algorithms/base_coord.py`
  - base-coordinate phase logic

### 7.3 Adapter modules

- `src/adapters/turtle_adapter.py`
- `src/adapters/robot_adapter.py`

These modules are environment-specific and must remain free of mission logic.

### 7.4 Runner modules

- `src/runners/pid_alignment_runner.py`
- `src/runners/full_mission_runner.py`

### 7.5 Optional config modules

To keep configuration orthogonal as scenarios grow, the design recommends:

- `src/config/models.py`
- `src/config/loaders.py`

These modules should parse YAML into structured config objects for runners and phase handlers.

## 8. Testing Workflow Design

Testing is modeled as assembly, not as special-case business logic.

The testing workflow is defined by a combination of:
- runner
- phase
- adapter
- config

### 8.1 Turtle basic control test

Goal:
- verify PID does not diverge
- verify workflow phase switching
- verify algorithm/environment synchronization

Assembly:
- runner: `pid_alignment_runner`
- adapter: `turtle_adapter`
- start phase: `STATUS_ALIGN`

This mode mainly validates control behavior and workflow synchronization, not full visual realism.

### 8.2 Real-world partial functional test

Goal:
- verify BaseDetect + PID lateral alignment on the real robot
- avoid dependence on the full competition coordinate system

Assembly:
- runner: `pid_alignment_runner`
- adapter: `robot_adapter`
- start phase: `STATUS_ALIGN`

This is the primary day-to-day development workflow.

### 8.3 Full competition mode

Goal:
- execute the complete mission after field parameters are available

Assembly:
- runner: `full_mission_runner`
- adapter: `robot_adapter`
- start phase: `READY`

This mode should be built on top of already validated partial workflows rather than used as the main development entrypoint.

## 9. Recommended Phase Coordination Flow

The recommended coordination model is intentionally lightweight.

1. runner sets `/workflow/phase`
2. adapter sees the new phase and prepares the environment
3. adapter publishes `/workflow/env_status = READY`
4. algorithm sees the phase and environment readiness, resets local state, and starts execution
5. algorithm publishes `/workflow/algo_status = RUNNING`
6. algorithm later publishes `ALIGNED` or `STEP_DONE`
7. runner decides whether to stop or move to the next phase

In this design, phase advancement belongs to the runner, while execution feedback belongs to the algorithm and environment.

## 10. Minimum Viable First Iteration

The first implementation should stop at the smallest useful architecture.

### Must be implemented

- unified workflow phase/status model
- pure PID module extracted from the current node
- extracted target selection module
- extracted status alignment module
- BaseDetect gateway wrapper
- `pid_alignment_runner`
- `turtle_adapter`
- support for starting directly from `STATUS_ALIGN`

### Explicitly deferred

The first implementation should not attempt to solve everything.

Defer:
- heavy central orchestrator logic
- behavior trees
- complex custom message schemas
- final forward-approach implementation details
- complete full-mission polishing
- unrealistic simulation completeness in turtle

## 11. Recommended Implementation Order

Implementation should proceed in this order:

1. define workflow types and topic contract
2. extract PID into `algorithms/pid.py`
3. extract target selection and status alignment logic
4. add `detector_gateway`
5. build `pid_alignment_runner`
6. connect `turtle_adapter`
7. add `robot_adapter`
8. implement `forward_approach.py`
9. build `full_mission_runner`

This order prioritizes the user’s main need: fast and reliable partial testing before full competition integration.

## 12. Expected Outcome

After this refactor:
- BaseDetect + PID alignment can be tested independently
- turtle and the real robot can share the same workflow contract
- arbitrary mission phases can be used as test entrypoints
- the competition full workflow becomes an assembly of validated modules rather than a monolithic script
- environment-specific differences stay in adapters instead of contaminating algorithm code

This should make the RobotFetch project easier to test, easier to evolve, and less likely to break when switching between turtle testing, partial real-robot testing, and full competition execution.
