# RM75 Waypoint Trajectory Design

## Intent

Add a reusable manual waypoint-teaching workflow to the existing RM75 Web HMI. Operators record discrete real-feedback poses while moving the arm with Touch or existing controls, reorder the points, generate a smooth joint-space path that preserves every taught joint vector, persist it as JSON, and execute it through the existing guarded seven-axis action chain.

## Goals

- Record one fresh seven-joint and `base_link -> Link7` feedback sample per click, including ROS source time and quaternion pose.
- Preserve ordered waypoints exactly while generating dense, velocity/acceleration-continuous trajectory samples.
- Persist stable-ID `RM75Trajectory` resources atomically under a configurable directory.
- Validate limits and the current MoveIt collision scene before execution.
- Reuse the existing lease, SAFE_STOP, TaskManager, JointControlAdapter, FollowJointTrajectory action, and simulation boundaries.
- Provide a dedicated React page and REST API usable by later tasks by trajectory ID.

## Non-goals

Automatic waypoint capture, Cartesian interpolation, obstacle avoidance planning, reinforcement learning, a second robot command publisher, or claiming real-hardware validation.

## Safety anti-goals

- Never accept stale or partial robot feedback as a taught point.
- Never change a taught joint position during smoothing.
- Never execute through a browser-only timer or bypass the exclusive lease and existing action server.
- Never silently keep a generated trajectory valid after waypoints or timing parameters change.

## Constraints

- ROS1 Noetic and Python 3 compatibility.
- Seven joints in canonical `joint1` through `joint7` order, radians/metres/SI units.
- Existing local-only FastAPI and React/Vite architecture and UI style.
- Default storage `~/.ros/rm75_sampling_gui/trajectories`, configurable through `trajectory_library.directory`.
- Real execution remains gated by the existing joint/trajectory authorization and MoveIt state-validity service.

## Approaches considered

1. MoveIt Cartesian or OMPL replanning through poses: rejected because it may move away from taught joint vectors and therefore cannot guarantee strict passage.
2. Independent ROS trajectory controller path: rejected because it duplicates the established guarded execution architecture.
3. Joint-space piecewise quintic Hermite trajectory through fixed waypoints, with non-zero monotone knot velocities, zero velocity only per joint at reversals/endpoints, shared zero knot accelerations, and iterative duration scaling: selected because it guarantees exact waypoint interpolation and C2 continuity while fitting the existing `JointTrajectory` action path.

## Architecture

- `trajectory/models.py`: versioned data validation and JSON representation.
- `trajectory/interpolation.py`: deterministic multi-joint quintic generation and kinematic limit checks.
- `trajectory/repository.py`: stable-ID CRUD and atomic JSON replacement.
- `trajectory/service.py`: fresh-feedback recording, stale-state invalidation, collision validation, simulation execution/progress, and real adapter delegation.
- `JointControlAdapter`: accept an already sampled position/velocity/acceleration trajectory, validate every sample against limits/current MoveIt scene, submit the existing action, and publish feedback-based progress callbacks.
- FastAPI: `/api/v1/trajectories` collection/resource routes plus waypoint, generation, execution, and stop actions. All mutations and execution require the existing Web control lease; reads do not.
- React: a `路径示教` page that edits server-owned resources and shows waypoint, generation, execution, progress, and error state.

## Interpolation details

Initial segment durations are derived from per-joint displacement and configured velocity/acceleration limits at the requested scales. Per-joint internal velocities use a weighted harmonic mean when adjacent secants share direction, and zero at direction reversal or a flat side. Endpoint velocity is zero. Knot acceleration is zero, which is shared by both adjacent quintic segments and therefore gives C2 continuity without forcing intermediate velocity to zero. Each segment solves quintic Hermite coefficients from `(q, qdot, qddot)` at both ends. The generator samples at the configured period, explicitly emits each knot once, verifies exact passage, then iteratively stretches segment times until dense-sample velocity and acceleration maxima fit all limits.

## Data and lifecycle

Each document contains schema/version/identity metadata, canonical joint names, frame/tool metadata, raw waypoints, trajectory config, dense trajectory points, and trajectory status. Any waypoint edit or config change clears the dense trajectory and marks status `STALE`. Generation marks it `READY` only after limit/collision validation. Atomic writes use a same-directory temporary file, flush/fsync, and `os.replace`.

## Testing

Pure unit tests cover 20-point recording, CRUD/restart recovery, reorder/delete, strict passage, C2 knot continuity, direction reversal, limit scaling, malformed JSON, ROS message conversion, execution progress, cancellation, and ID-based invocation. Existing Web/API tests cover lease enforcement and simulation flow. Final verification includes the full relevant pytest suites, React production build, catkin package build/tests, simulation black-box validation, git diff review, commit, and non-force push.
