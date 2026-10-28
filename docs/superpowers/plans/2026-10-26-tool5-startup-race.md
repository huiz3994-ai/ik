# Tool5 Startup Race Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent Tool5 startup polling from falsely latching `SAFE_STOP` while the RM75 base stack is delivering its first fresh feedback after a control-path switch.

**Architecture:** Keep policy and state handling local to `Tool5DrillWorkflow`. Treat `STARTING` with no motor output as an initialization phase, wait conditionally for fresh driver/joint/TCP/wrench telemetry before preflight, and retain every existing runtime safety check as soon as outputs begin. Serialize cancellation against continuous-motor process registration through a narrow hardware startup callback so `SAFE_STOP` remains terminal for an in-flight start.

**Tech Stack:** Python 3, threading, monotonic time, pytest

**Spec:** `docs/superpowers/specs/2026-10-26-tool5-startup-race-design.md`

## Global Constraints

- Use the existing global `feedback_timeout_s` as the maximum startup wait.
- Do not change global telemetry freshness thresholds.
- Do not remove or weaken lease, watchdog, feedback, axial-force, torque, driver-error, or tracking-error interlocks.
- Do not validate the code change by moving the real robot.
- Preserve unrelated user changes in the dirty worktree.

## Review Focus

- A fresh-feedback transition before the deadline must allow startup to proceed; covered by Task 1 startup-wait test.
- Feedback that never becomes fresh must fail with a specific timeout and latch `SAFE_STOP`; covered by Task 1 timeout test.
- WebSocket snapshots during output-free `STARTING` must not send a motor stop or call arm emergency stop; covered by Task 1 concurrency regression test.
- Loss of the browser control lease during `STARTING` must still latch `SAFE_STOP`; covered by Task 1 lease-loss test.
- Once the motor is marked running, stale telemetry must immediately invoke the existing runtime stop path even if the state is still `STARTING`; covered by Task 1 output-active test.
- Lease loss while a real `start()` call is waiting, preparing, or registering the motor must abort that call, retain `SAFE_STOP`, and prevent any continuous-motor or engagement command after the stop; covered by deterministic event-driven concurrency tests added during final review.

---

### Task 1: Make Tool5 startup initialization-safe

**Files:**
- Modify: `rm75_ws/src/rm75_sampling_gui/web/backend/tool5_drill.py`
- Modify: `rm75_ws/src/rm75_sampling_gui/web/backend/hardware.py`
- Test: `rm75_ws/src/rm75_sampling_gui/test/test_web_backend.py`

**Interfaces:**
- Consumes: `config["feedback_timeout_s"]`, `telemetry() -> Dict[str, Any]`, existing `_preflight(raw)` and `_check_runtime_safety(raw)` methods.
- Produces: `_wait_for_preflight_feedback() -> Dict[str, Any]`; unchanged public `start(depth_m)`, `snapshot()`, and `poll()` interfaces.

- [ ] **Step 1: Add failing startup race and readiness tests**

Add tests named:

- `test_tool5_starting_snapshot_does_not_stop_before_outputs_begin`
- `test_tool5_start_waits_for_fresh_robot_feedback`
- `test_tool5_start_feedback_timeout_safe_stops`
- `test_tool5_starting_still_stops_when_lease_is_lost`
- `test_tool5_starting_with_motor_output_applies_runtime_safety`

The tests must assert the state transitions, motor stop call count, arm safe-stop call count, and timeout error text described in Review Focus.

- [ ] **Step 2: Run the focused tests and verify the regression tests fail**

Run: `pytest -q rm75_ws/src/rm75_sampling_gui/test/test_web_backend.py -k 'tool5_starting or tool5_start_waits or tool5_start_feedback_timeout'`

Expected: the new race/readiness tests fail against the current implementation.

- [ ] **Step 3: Add bounded startup feedback waiting**

In `Tool5DrillWorkflow`, read `feedback_timeout_s` with a positive bounded default of `5.0`, add `_wait_for_preflight_feedback() -> Dict[str, Any]`, and have `start()` use its returned fresh snapshot before `_preflight`. Poll at a short interval with `time.monotonic()` and report which of driver/joints/wrench/TCP remained unavailable on timeout.

- [ ] **Step 4: Separate output-free initialization from runtime polling**

In `poll()`, keep the lease check active for every active state. Return without runtime telemetry checks only when `state == "STARTING"` and `_motor_running` is false. Once `_motor_running` becomes true, reuse the existing runtime safety path unchanged.

- [ ] **Step 5: Run focused Tool5 tests**

Run: `pytest -q rm75_ws/src/rm75_sampling_gui/test/test_web_backend.py -k 'tool5'`

Expected: all selected tests pass.

- [ ] **Step 6: Run related manager and field-station tests**

Run: `pytest -q rm75_ws/src/rm75_sampling_gui/test/test_task_manager.py rm75_ws/src/rm75_sampling_gui/test/test_field_station.py`

Expected: all tests pass.

- [ ] **Step 7: Run the complete backend suite**

Run: `pytest -q rm75_ws/src/rm75_sampling_gui/test`

Expected: all tests pass with no real robot commands.

- [ ] **Step 8: Commit the isolated fix if requested**

Stage only `tool5_drill.py`, `test_web_backend.py`, the approved spec, and this plan; do not stage unrelated dirty-worktree files.
