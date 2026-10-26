# Design notes

## Separation from ROS control

All public algorithms accept Eigen values and STL containers. There are no ROS
messages in the public API. `rm75_motion_benchmark` is a plain offline executable
and the Python Ipopt benchmark imports the already-tested phase-one symbolic
solver. This keeps planning deterministic and testable without ROS Master.

## Rotation convention

`Eigen::Isometry3d` stores `base_T_flange`. Rotation distance is the norm of
`Log(R0^T R1)` in `[0, pi]`. Interpolation uses normalized quaternion SLERP to
avoid Euler discontinuities and chooses the quaternion shortest arc.

## Continuous IK failure contract

The first pose is solved from the supplied initial joint vector; each later
target is solved from the previous accepted vector. Success requires the
phase-one IK tolerances and a norm jump no larger than `max_joint_jump`. Failure
causes midpoint subdivision before invoking an optional fallback callback.
Intermediate points introduced by subdivision are returned together with their
poses so FK validation remains one-to-one. Failure returns the solved prefix and
an explicit reason; it never returns a path with a missing interior target.

## Time law

For segment displacement `d`, the chosen duration is

```text
T >= max_i(1.875 |d_i| / v_i,
           sqrt((10/sqrt(3)) |d_i| / a_i),
           cbrt(60 |d_i| / j_i)).
```

Rounding `T` upward to a control tick cannot increase any derivative. The
generated point contains analytic position, velocity, acceleration, and jerk;
the validator does not estimate them with noisy finite differences.

## Collision contract

`MoveitCollisionChecker` constructs a MoveIt `RobotModel` from the supplied
URDF/SRDF, selects the seven-variable `arm` group, and uses the FCL collision
detector in a local `PlanningScene`. The validator queries it for every final
timed sample, not only the sparse IK waypoints. A collision records the first
contact pair and invalidates the trajectory. When collision checking is a
safety requirement, callers pass `require_collision_check=true`; omitting a
checker then fails closed instead of silently returning a valid report.

## Ipopt fallback process

`PersistentIpoptFallback` launches one isolated Python worker through pipes.
The worker imports the phase-one optimizer, builds and primes the CasADi/Ipopt
NLP before emitting `READY`, then retains the solver and warm-start variables
for subsequent requests. The C++ side uses an unbuffered response stream so
`poll()` and line reads cannot disagree, bounds every wait, serializes calls,
and terminates/reaps the process on all teardown paths.

## Known limitations

- Stopping at every sampled waypoint is conservative and can make long Cartesian
  paths slow. A future path-velocity decomposition or verified Ruckig integration
  should retain geometric error bounds while blending waypoints.
- CasADi remains isolated from the ROS Python environment in a persistent worker
  process; this is an intentional dependency boundary, not an in-process solver.
- The included PlanningScene starts with vendor robot geometry only. A production
  caller must add its calibrated tool, workcell, and live environment objects.
- Straight SE(3) interpolation between two reachable endpoint poses is not
  guaranteed to remain reachable. This is intentionally counted as a failed
  whole path in the benchmark.
