# RM75-6FB SE(3) motion-planning benchmark — 2026-10-13

## Scope and method

All measurements are offline inside the existing development environment. No
ROS Master, driver, simulation controller, or real robot was started, and no
message was published to `/rm_driver/JointPos`.

The C++ benchmark used seed 7506 and ran 1,000 complete paths in each of five
scenes for each of KDL, fixed DLS, adaptive DLS, and adaptive DLS with joint
centering: 20,000 pipeline executions in total. A success means every SE(3)
sample was solved without skipping, every joint jump was at most 0.35 rad, the
path was time-parameterized at 50 Hz, FK error was at most 2 mm / 2 degrees,
joint limits were respected, and velocity/acceleration/jerk stayed within
1.0 rad/s, 0.5 rad/s^2, and 2.0 rad/s^3.

The CasADi 3.7.2/Ipopt supplement reused the first 100 start/goal configurations
from every C++ scene and ran 100 complete paths per scene (500 total) in an
isolated Python 3.12 venv. The smaller count is reported explicitly and is
appropriate for a process-wide offline nonlinear fallback comparison; it is not
mixed into the 1,000-path C++ timing population.

## Whole-path success rate

| Solver | Short line | Large Cartesian | Random 6D | Near singular | Near limit |
|---|---:|---:|---:|---:|---:|
| KDL | 99.9% | 71.8% | 64.0% | 37.6% | 99.8% |
| Fixed DLS | 100.0% | 78.5% | 67.9% | 100.0% | 100.0% |
| Adaptive DLS | 100.0% | 67.8% | 58.6% | 100.0% | 100.0% |
| Adaptive DLS + joint centering | 97.1% | 38.3% | 37.8% | 100.0% | 75.0% |
| Ipopt (shared first 100/scene) | 100.0% | 90.0% | 70.0% | 100.0% | 99.0% |

For a strict same-case comparison on those shared first 100 paths, KDL achieved
100/74/58/40/100%, fixed DLS 100/77/61/100/100%, adaptive DLS
100/71/50/100/100%, and adaptive DLS with joint centering
96/40/30/100/77% in the same scene order. Ipopt therefore had the best
large-range and random-6D success on the exact shared subset, while DLS remained
orders of magnitude faster.

## Timing and constraint observations

- Adaptive DLS mean full-pipeline time was 0.026 ms for short lines, 2.791 ms
  for large Cartesian paths, 2.754 ms for random 6D, 0.022 ms near singular,
  and 0.031 ms near limits. Its P99 was 15.489 ms in random 6D.
- Ipopt mean whole-path time was 4.273, 252.848, 223.852, 2.778, and 7.936 ms
  in the same scene order. Random-6D P99 was 602.438 ms. This directly supports
  using it offline or after a DLS failure, not in an unbounded 50 Hz servo loop.
- Across all successful methods/scenes, the largest emitted velocity was
  0.266 rad/s, acceleration 0.394 rad/s^2, and jerk 2.000 rad/s^3. The jerk
  value reaches, but does not exceed, its analytic limit (display rounding).
- C++ successful-path maximum FK errors were at most 1.000 mm and 0.571 degrees
  for the DLS variants. Ipopt maxima were 1.836 mm and 0.203 degrees.
- Failures are whole-path failures. The dominant DLS causes were line-search
  failure, iteration exhaustion, and discontinuity after a nominal solve. KDL
  was weakest near singularity. Ipopt failures were unsuccessful NLP solutions
  or a solution branch jump above 0.35 rad.

## Interpretation

The first-stage conclusion needs a trajectory-specific qualification. Adaptive
damping remains excellent near singularity and is fast enough for continuous
tracking, but with the current weights/tolerances it is not the highest-success
method for long arbitrary straight SE(3) paths; fixed DLS did better in the two
wide-range scenes. The fixed DLS result is therefore retained as a baseline and
the adaptive parameters require trajectory-level tuning.

Joint centering must stay optional. Its secondary motion made many long paths
and near-limit paths fail even though it can improve static limit margin. The
Ipopt whole-path results improve large/random success but exhibit hundreds of
milliseconds of latency and occasional alternate-branch jumps, so its output
still needs the same continuity validator.

Raw and rendered artifacts:

- `benchmark/results/rm75_6fb_motion_1000x5.csv`
- `benchmark/results/rm75_6fb_motion_1000x5.json`
- `benchmark/results/rm75_6fb_motion_1000x5.md`
- `benchmark/results/rm75_6fb_ipopt_motion_100x5.csv`
- `benchmark/results/rm75_6fb_ipopt_motion_100x5.json`
- `benchmark/results/rm75_6fb_ipopt_motion_100x5.md`

## Post-benchmark defect remediation

The numerical populations above are preserved unchanged and must not be read as
collision-qualified runs. After the benchmark, the validator gained an actual
FCL-backed MoveIt PlanningScene check for every 50 Hz timed sample, together
with fail-closed `require_collision_check` semantics. Tests load the vendor
RM75-6FB URDF and SRDF, verify a collision-free trajectory, insert a world box
that blocks the arm and verify rejection, then remove it and verify recovery.

The callback-only Ipopt boundary was also replaced by a persistent C++ adapter.
An end-to-end smoke test intentionally limits DLS to one iteration and confirms
one real CasADi/Ipopt fallback call with 0.333 mm final position error. The
package now has 19 actual GTests, all passing; no controller, simulator, or real
robot was started for these checks.
