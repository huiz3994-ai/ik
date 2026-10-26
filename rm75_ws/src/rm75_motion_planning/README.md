# RM75 Motion Planning (offline-first, guarded commissioning)

`rm75_motion_planning` provides an offline-first ROS 1 planning library for the
RM75-6FB plus a fail-closed phase-3 execution layer. The core library and
benchmark have no hardware side effects. Explicit commissioning executables are
separate from that core:
`rm75_live_state_audit` is read-only, while
`rm75_low_speed_commissioning` is disarmed by default and can publish
`/rm_driver/JointPos` only when `_execute:=true` is supplied.
`rm75_cartesian_commissioning` adds Link7 Cartesian continuous IK,
execution-time Guard, and tracking metrics with the same disarmed default.
`rm75_gazebo_shadow_execution` is restricted to `/arm/...` Gazebo topics and
requires `/use_sim_time=true`.

## Model contract

- Model: vendor `RM75_6FB/rm_75_6fb_description.urdf.xacro` expanded with
  `link7_type:=Link7_6fb`.
- Kinematic chain: `base_link -> Link7`, moving joints `joint1..joint7`.
- The xacro link is still named `Link7`; `Link7_6fb` is the branch selector and
  mesh name. In this branch joint 7 has a 0.1612 m origin (not 0.144 m).
- Phase 3 deliberately uses `base_link -> Link7` as its complete end-reference
  contract. Tool models, TCP calibration, and Tool Frame switching are deferred
  until physical tooling is installed.
- Units are metre, radian, second.

## Pipeline

```text
base-frame Link7 target
  -> SE(3) line / circular arc / multi-waypoint sampling
  -> previous-solution-seeded adaptive DLS
  -> recursive half-step refinement
  -> optional persistent constrained-IK fallback (CasADi/Ipopt offline)
  -> path-preserving quintic S-curve time parameterization
  -> FK, dynamics, joint-limit, jump and PlanningScene collision validation
  -> 50 Hz position/velocity/acceleration/jerk samples
```

The interpolator uses linear translation and quaternion SLERP. Sample count is

```text
N = max(ceil(||p1-p0|| / max_translation_step),
        ceil(rotation_angle / max_rotation_step), 1).
```

The default steps are 5 mm and 0.05 rad. A circular arc is the unique
circumcircle through start/via/goal; collinear inputs fail explicitly, and the
via pose is emitted exactly. Multi-waypoint paths concatenate independently
sampled line segments without duplicate boundaries.

Continuous IK solves `q[k] = IK(T[k], q[k-1])`. A failed or over-jump solution
causes recursive SE(3) bisection. After the configured depth, an injected
fallback is tried; if that also fails, the entire path fails. No target point is
silently skipped. `PersistentIpoptFallback` owns one isolated Python worker,
builds the phase-one CasADi 3.7.2/Ipopt problem once, retains warm-start state,
and exposes the callback expected by `ContinuousIkSolver`. It has bounded
startup/response waits and reaps the child process on both normal destruction
and constructor failure.

`MoveitCollisionChecker` loads the RM75 URDF and SRDF into an FCL-backed MoveIt
`PlanningScene`. It checks both self collision and registered world objects at
every emitted 50 Hz trajectory sample. `addBox()` and `removeObject()` provide a
minimal environment interface; production callers must populate the scene with
measured environment geometry. Tool geometry is outside this stage.

## Jerk-limited fallback

No Ruckig package/version is available in the current container, so the built-in
dependency-free fallback uses the quintic scalar law

```text
s(u) = 10u^3 - 15u^4 + 6u^5,  u in [0,1].
```

For every joint-space segment, duration is the maximum required by the exact
derivative bounds `max(s')=15/8`, `max(|s''|)=10/sqrt(3)`, and
`max(|s'''|)=60`, then rounded upward to a 20 ms tick. Every input joint
waypoint is reached exactly with zero boundary velocity and acceleration. This
preserves the path, unlike joint low-pass filtering. The tradeoff is a full stop
at every joint waypoint; blending/Ruckig integration is a later optimization.

## Build and test

```bash
cd /root/rm75_ws
catkin build rm75_motion_planning
catkin run_tests rm75_motion_planning
```

The tests cover SE(3), 90-degree and near-180-degree rotation, random Log/Exp,
frame transforms, line/circle/waypoint paths,
continuous and near-singular IK, explicit failure, velocity/acceleration/jerk,
FK, joint margin, jump detection, real URDF/SRDF self/world collision, mandatory
collision-check enforcement, Runtime Guard, execution gating, guarded command
ownership, persistent-process protocol reuse, and real benchmark reporting.

To create an isolated Ipopt worker environment and exercise the real fallback:

```bash
rosrun rm75_motion_planning setup_ipopt_fallback.sh /tmp/rm75_motion_ipopt_venv
rosrun rm75_motion_planning rm75_fallback_smoke \
  /tmp/rm75_motion_ipopt_venv/bin/python \
  $(rospack find rm75_motion_planning)/scripts/ipopt_fallback_server.py \
  $(rospack find rm75_kinematics_optimization)/scripts/optimization_ik.py
```

Python 3.8 is pinned to NumPy 1.24.4; newer Python uses the phase-one NumPy
2.5.4 pin. Both use CasADi 3.7.2.

## Read-only hardware audit

When an RM75 driver is already connected, the following command samples the
real `/joint_states` stream without advertising any command topic:

```bash
rosrun rm75_motion_planning rm75_live_state_audit \
  _samples:=100 _minimum_margin_rad:=0.02
```

It requires exactly `joint1..joint7`, reports the observed stream rate and
maximum sample-to-sample joint change, recomputes `base_T_Link7`, compares it
with `/rm_driver/Udp_Joint_PoseEuler`, checks the URDF joint limits, and runs FCL
self/world collision on every received sample. Exit status is nonzero on
stale/invalid input, more than 0.01 rad joint-vector excursion during the
stationary audit window, insufficient joint margin, FK disagreement above 3 mm
or 2 degrees, or collision. This is a shadow diagnostic only: it has no
publisher and cannot move or stop the robot. The default scene still contains
only vendor robot geometry unless the calling application adds calibrated
tool/world objects.

## Guarded low-speed commissioning

`rm75_low_speed_commissioning` is disarmed by default. It creates a conservative
quintic round trip from the current pose by moving only joint 7, validates every
50 Hz sample against dynamics, limits, FK and FCL, and monitors live joint/force
feedback during execution. The executable hard-limits the requested excursion
to 0.02 rad and tracking error to at most 0.05 rad. Its default commissioning
limits are 0.05 rad/s, 0.10 rad/s², 0.50 rad/s³, 8 N and 0.5 N·m. A feedback
timeout, controller planning failure, collision, tracking error, or force limit
stops command streaming and requests `/rm_driver/Emergency_Stop`.

Dry-run preflight:

```bash
rosrun rm75_motion_planning rm75_low_speed_commissioning
```

Physical execution requires explicit authorization and the flag below. It must
be the only `/rm_driver/JointPos` publisher:

```bash
rosrun rm75_motion_planning rm75_low_speed_commissioning _execute:=true
```

## Reproduce offline benchmarks

```bash
source /root/rm75_ws/devel/setup.bash
rosrun rm75_motion_planning rm75_motion_benchmark \
  --count-per-scenario 1000 --seed 7506 \
  --output /root/rm75_ws/src/rm75_motion_planning/benchmark/results/recheck
python3 scripts/render_report.py benchmark/results/recheck.json \
  benchmark/results/recheck.md
```

The committed C++ result contains 20,000 complete pipeline runs: five scenes,
1,000 paths per scene, and KDL/fixed-DLS/adaptive-DLS/adaptive-DLS+joint-center
variants. The Ipopt supplement contains 500 complete paths (100 per scene),
reusing the first 100 start/goal configurations from each C++ scene. The smaller
count is because process-wide nonlinear trajectory solving is intentionally
evaluated as an offline fallback rather than a 50 Hz method. See:

- `benchmark/results/rm75_6fb_motion_1000x5.md`
- `benchmark/results/rm75_6fb_ipopt_motion_100x5.md`

These are numerical offline results, not simulation execution and not real-robot
validation.

## Real Link7 execution benchmark

`benchmark/real_execution/` contains the single-run recorder, immutable CSV
schema, and CSV/JSON/Markdown/SVG aggregator. It counts STOP and FAIL attempts
instead of requiring 100 cherry-picked successes. It rejects logs that are not
labelled `data_source=real_robot` and `reference_frame=base_link_to_Link7`.
No third-stage physical rows are currently claimed.

## Current boundary

The validator now covers FK path error, joint position limits, joint jumps,
analytic velocity/acceleration/jerk limits, and FCL self/world collision. The
real entry is disarmed by default and creates no JointPos publisher unless
`_execute:=true` is explicit. The PlanningScene still lacks a measured complete
workcell. Phase-3 Cartesian, 5x100 real-run, and stop-distance results are not
yet claimed. Gazebo evidence remains separately labelled simulation. See
`docs/phase3-real-motion-safety-20261013.md`.
