# RM75-6FB Stage 4 supervised pilot characterization — 2026-10-16

## Scope and safety state

- Robot: RM75-6FB; ROS1 Noetic; endpoint `base_link -> Link7`; no TCP.
- A human continuously supervised the workcell, the swept volume was clear,
  and the physical E-stop was immediately reachable.
- Touch, force control, tool voltage, contact and soil insertion remained off.
- Every motion was a separately reviewed invocation with explicit
  `execute:=true`; no unattended repetition loop was used.
- The driver was stopped after post-flight checks. No `/rm_driver/JointPos`
  publisher remained.

## Executed pilot set

Fourteen recorded trajectories were executed at 50 Hz: X/Y/Z outward and
return, Rx/Ry/Rz positive and negative, and a combined 6D outward and exact
pose return. Translation used 0.01 m/s. Single-axis rotations were 5 degrees;
the combined 6D motion used 5 mm per translation axis and 2 degrees per
rotation axis.

All 14 recorded executions returned PASS:

- Guard WARN: 0; Guard STOP: 0; collision samples: 0.
- E-stop requests: 0; controller failures: 0; driver error remained `[0]`.
- Worst observed joint tracking error: 0.018656 rad.
- Peak force norm: 2.592 N; peak torque norm: 0.1349 N m.
- Command rate remained approximately 50 Hz; no estimated feedback frames
  were dropped by the executor.
- Recorder-rate finite differences observed at most 0.05397 rad/s and
  1.50 rad/s2. The acceleration value is encoder-quantization sensitive and is
  not a controller-internal acceleration measurement.

The strict fixed-orientation Z -10 mm return failed continuous-IK preflight at
pose index 1. It created no command publisher and moved nothing. A separately
reviewed recovery target returned to the recorded pre-Z Link7 pose while also
correcting 0.00112 rad of accumulated orientation error. This recovery passed
preflight and execution. It is retained as `z-minus-recovery-20261016-002` and
must not be described as a pure fixed-orientation Z sample.

## Preliminary accuracy and response timing

These values are pilot statistics from two runs per scenario, not the required
100-run characterization:

| Scenario | Runs | Position trajectory RMS mean | Orientation trajectory RMS mean | Response delay mean |
|---|---:|---:|---:|---:|
| X | 2 | 0.973 mm | 0.0527 deg | 235.0 ms |
| Y | 2 | 0.712 mm | 0.0619 deg | 272.5 ms |
| Z | 2 | 1.447 mm | 0.0619 deg | 340.0 ms |
| Rx | 2 | 0.253 mm | 0.3125 deg | 292.5 ms |
| Ry | 2 | 0.265 mm | 0.2536 deg | 325.0 ms |
| Rz | 2 | 0.211 mm | 0.3224 deg | 262.5 ms |
| 6D | 2 | 0.737 mm | 0.2425 deg | 275.0 ms |

Delay is estimated from joint-speed cross-correlation and checked with peak
alignment. It is not the raw command/feedback timestamp difference. The ROS
source-stamp to local-receipt offset was 0.361 ms mean and 0.551 ms P95 and is
reported only as a clock/transport diagnostic.

The Z result includes the recovery trajectory described above. The original
raw CSV is immutable. During post-flight review, the recorder's combined-path
ideal-progress estimator was found to over-weight very small rotations; the
implementation was corrected for future acquisitions, but this pilot CSV was
not rewritten. Consequently the pilot error-decomposition values, especially
Z recovery, must not be used as final model parameters.

## Guard evidence

The pilot-only analyzer produced:

- tracking warning/stop: 0.01906 / 0.03106 rad;
- feedback timeout: 0.04212 s;
- force warning/stop: 2.544 / 3.887 N;
- torque warning/stop: 0.1349 / 0.2012 N m.

These values are marked `provisional_do_not_deploy`. They were not copied into
`config/runtime_guard.yaml`. Eligibility requires 100 normal PASS runs for
every scenario, both signs for each single-axis case, stop-performance tests,
and human safety review.

## Pre/post-flight evidence

Pre-flight static audit: 400 samples at 199.853 Hz, maximum stationary span
0.000122 rad, minimum joint-limit margin 0.237851 rad, zero collisions, and FK
agreement of 0.191 mm / 0.001159 rad.

Post-flight static audit: 400 samples at 199.993 Hz, maximum stationary span
0.000110 rad, minimum joint-limit margin 0.247693 rad, zero collisions, and FK
agreement of 0.166 mm / 0.001062 rad.

## Completion boundary

XYZ, RPY and combined 6D supervised pilot execution is complete. The Stage 4
benchmark remains INCOMPLETE because only two attempts per scenario exist;
0.03 m/s trials, 100-run cohorts, tracking/timeout/collision STOP experiments,
stopping latency/distance, and final Guard calibration were not performed.

Raw evidence and generated artifacts are under
`benchmark/real_execution/results/` and `benchmark/real_execution/plots/`.
