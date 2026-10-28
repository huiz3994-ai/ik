# RM75 Waypoint Trajectory Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and publish a reusable manual waypoint trajectory editor, generator, persistence layer, and guarded executor for RM75-6FB.

**Architecture:** Focused Python trajectory modules own the versioned model, quintic interpolation, repository, and orchestration. FastAPI and React expose them, while real motion reuses TaskManager/JointControlAdapter and the existing FollowJointTrajectory action; simulation uses the same API without physical output.

**Tech Stack:** ROS1 Noetic, Python 3, NumPy, FastAPI/Pydantic, React 18, TypeScript, Vite, pytest/rostest/catkin.

**Spec:** `docs/superpowers/specs/2026-10-28-rm75-waypoint-trajectory-design.md`

## Global Constraints

- Preserve canonical seven-joint order and SI units.
- Default persistence is `~/.ros/rm75_sampling_gui/trajectories` with stable IDs and atomic replacement.
- All write/execute/stop routes require the existing exclusive Web control lease.
- Real motion must reuse the existing guarded action chain and MoveIt state-validity service.
- Taught joint vectors are immutable interpolation constraints; all waypoint/config edits mark dense data stale.

## Review Focus

- Stale/partial joint or TF feedback must reject recording rather than saving guessed data (Task 3 tests).
- Duplicate/unsafe trajectory IDs and path traversal must never escape the configured repository (Task 2 tests).
- Reversal/flat secants must not overshoot position limits or create non-finite derivatives (Task 1 tests).
- Lease loss/cancel during execution must terminate progress as cancelled/SAFE_STOP, not completed (Task 4 tests).
- Loading older/malformed documents must fail with actionable validation without damaging valid files (Task 2 tests).

---

### Task 1: Versioned model and quintic generator

**Files:**
- Create: `rm75_ws/src/rm75_sampling_gui/trajectory/models.py`
- Create: `rm75_ws/src/rm75_sampling_gui/trajectory/interpolation.py`
- Create: `rm75_ws/src/rm75_sampling_gui/trajectory/__init__.py`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_waypoint_trajectory.py`

**Interfaces:**
- Produces: `new_trajectory(name, ...)`, `validate_document(payload)`, `generate_quintic(waypoints, config, velocity_limits, acceleration_limits)`.

- [ ] Write failing tests for schema defaults, strict waypoint passage, C2 knot continuity, monotone pass-through, reversal velocity, dense limit compliance, and non-finite input rejection.
- [ ] Run the focused tests and confirm missing-module/behavior failures.
- [ ] Implement the model and generator with deterministic knot indexing and iterative time scaling.
- [ ] Run focused tests and refactor while green.

### Task 2: Atomic trajectory repository

**Files:**
- Create: `rm75_ws/src/rm75_sampling_gui/trajectory/repository.py`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_waypoint_trajectory.py`

**Interfaces:**
- Consumes: `validate_document(payload)`.
- Produces: `TrajectoryRepository.list/get/save/delete` keyed by stable trajectory ID.

- [ ] Write failing tests for create/overwrite/list/load/delete, restart recovery, malformed JSON isolation, stable ID validation, and atomic temp cleanup.
- [ ] Run tests and verify repository failures.
- [ ] Implement locked CRUD with `fsync` plus `os.replace`.
- [ ] Run focused tests.

### Task 3: Feedback capture and trajectory service

**Files:**
- Create: `rm75_ws/src/rm75_sampling_gui/trajectory/service.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/ros/monitoring.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/web/backend/services.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/config/gui.yaml`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_waypoint_trajectory.py`

**Interfaces:**
- Consumes: repository, generator, `WebRuntime.snapshot`, robot model limits/FK, adapter collision checker.
- Produces: trajectory CRUD/waypoint/generate/status methods and `record_current_waypoint` using fresh source timestamps and TF quaternion.

- [ ] Write failing tests for 20 single-click points matching feedback, stale/partial rejection, delete/reorder/clear invalidation, and generation collision rejection.
- [ ] Run tests and confirm failures.
- [ ] Add TF quaternion fields and implement the service/runtime composition.
- [ ] Run focused tests.

### Task 4: Existing-chain execution and progress

**Files:**
- Modify: `rm75_ws/src/rm75_sampling_gui/control/joint_control_adapter.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/task/task_manager.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/trajectory/service.py`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_waypoint_trajectory.py`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_joint_control_server.py`

**Interfaces:**
- Produces: `JointControlAdapter.validate_trajectory/execute_trajectory/to_joint_trajectory`, `TaskManager.execute_trajectory`, and ID-based background execution/cancel/status.

- [ ] Write failing tests for ROS point conversion, start mismatch rejection, sample collision/limit validation, feedback-derived progress, ID invocation, and stop.
- [ ] Run tests and confirm failures.
- [ ] Implement adapter/task/service execution using the existing action and cancellation path.
- [ ] Run focused plus existing joint-control tests.

### Task 5: FastAPI resources and simulation flow

**Files:**
- Modify: `rm75_ws/src/rm75_sampling_gui/web/backend/app.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/test/test_web_backend.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/scripts/validate_web_simulation.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/docs/web/API.md`

**Interfaces:**
- Produces: collection/resource CRUD, waypoint record/reorder, generate, execute, stop, and execution status routes under `/api/v1/trajectories`.

- [ ] Write failing API tests for lease boundaries, CRUD, 20 points, stale status, generation, execution progress, stop, and ID invocation.
- [ ] Run API tests and confirm route failures.
- [ ] Implement request models/routes and simulation validation flow.
- [ ] Run Web backend tests.

### Task 6: Path teaching React page

**Files:**
- Modify: `rm75_ws/src/rm75_sampling_gui/web/frontend/src/types.ts`
- Modify: `rm75_ws/src/rm75_sampling_gui/web/frontend/src/store.ts`
- Modify: `rm75_ws/src/rm75_sampling_gui/web/frontend/src/App.tsx`
- Modify: `rm75_ws/src/rm75_sampling_gui/web/frontend/src/styles.css`

**Interfaces:**
- Consumes: Task 5 REST resources.
- Produces: `路径示教` navigation/page with ordered waypoint table, parameters, persistence controls, generation, execution, stop, progress, and errors.

- [ ] Add TypeScript contracts and page behavior against the REST API.
- [ ] Match current panel/button/table responsive styles.
- [ ] Run typecheck and production build, fixing all errors.

### Task 7: Integration verification and GitHub publication

**Files:**
- Modify: `rm75_ws/src/rm75_sampling_gui/README.md`
- Modify: `PROJECT_FOR_CHATGPT.md`

**Interfaces:**
- Produces: user/developer documentation and published feature branch.

- [ ] Run focused and full RM75 sampling GUI pytest/rostest suites in `rm75_noetic`.
- [ ] Run simulation black-box validation, React production build, and catkin package build/tests.
- [ ] Review `git diff`, verify no cache/log/secrets and that pre-existing user changes are excluded.
- [ ] Commit as `feat(rm75): add reusable waypoint trajectory editor`.
- [ ] Push `feature/rm75-waypoint-trajectory` to `origin` without force and report commit/push evidence.
