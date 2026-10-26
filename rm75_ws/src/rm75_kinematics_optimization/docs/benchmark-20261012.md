# Offline benchmark — 2026-10-12

Model: vendor RM75-6FB xacro expanded with `link7_type:=Link7_6fb`; base `base_link`, tip
`Link7`. Release build in `rm75_noetic`, Pinocchio 2.7.0, fixed seed 7506. The reported
10,000 targets caused 40,000 solver calls. Targets use the central 82% of each legal joint
range; independent seeds use the central 55%, with deterministic near-limit and zero-pose
cases. There is no collision environment in this numerical benchmark.

| Solver | Success | Mean ms | P95/P99 ms | Success mean position | Success mean rotation | Mean limit margin |
|---|---:|---:|---:|---:|---:|---:|
| KDL NR/JL | 67.49% | 0.471 | 1.142 / 1.195 | 0.000096 mm | 0.000020° | 0.264 rad |
| legacy-style fixed DLS | 65.20% | 0.137 | 0.227 / 0.275 | 1.042 mm | 0.173° | 0.509 rad |
| adaptive DLS | 67.85% | 0.176 | 0.440 / 1.039 | 1.106 mm | 0.192° | 0.557 rad |
| adaptive DLS + joint-centering nullspace | 68.57% | 0.208 | 0.692 / 1.155 | 1.032 mm | 0.216° | 0.592 rad |

All four solvers produced zero NaN/Inf anomalies. Means over *all final states*, including
failures, are retained in JSON/CSV and should not be confused with successful-solve errors.
The fixed-DLS row generalizes the existing teleop fallback's fixed-lambda bounded DLS step
into the new iterative harness; the production fallback itself remains a local one-step
method and was not replaced.

Adaptive damping improved success by 2.65 percentage points over fixed DLS and reduced mean
joint change from 2.505 to 2.411 rad, at +0.040 ms mean latency. Adding joint centering
improved mean minimum limit margin from 0.557 to 0.592 rad (+6.3%) and success by another
0.72 points. This comparison row isolates joint centering; the reference-pose and
manipulability gradients are available but not mixed into this row. Successful pose error remained within the shared 2 mm / 2° contract. The
damped projector is not mathematically exact; its primary-task leakage is unit-tested and
every combined step must pass the same task-error-decreasing line search.

Pinocchio/KDL FK consistency and finite-difference Jacobian measurements:

- zero / near singular: position 0 m, rotation 0 rad, max Jacobian column error 3.33e-8;
- random: position 4.58e-16 m, rotation 4.36e-16 rad, Jacobian 2.98e-8;
- near limit: position 2.91e-16 m, rotation 1.50e-15 rad, Jacobian 1.69e-8.

The 100,000-target command is supported but was not run. CasADi/Ipopt was not available in
the ROS image during this run; it was subsequently implemented in an isolated environment.
See `benchmark-scenarios-ipopt-20261013.md`, which supersedes this report for conclusions.
