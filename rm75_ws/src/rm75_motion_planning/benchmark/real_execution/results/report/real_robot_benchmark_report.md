# RM75-6FB real robot benchmark report

Verdict: **INCOMPLETE**.

This report contains only supervised `real_robot` records. Missing experiments are shown as N/A; no physical result is inferred from simulation or synthetic tests.

## Robot configuration

- Robot: RM75-6FB
- ROS: ROS1 Noetic
- Endpoint: `base_link -> Link7` (no TCP)
- Nominal command period: 20 ms / 50 Hz

## Accuracy and timing

| Scenario | Runs | PASS | STOP | Directions | Position RMS mean (mm) | Position RMS P95 (mm) | Orientation RMS mean (deg) | Delay mean/P95/P99 (ms) |
|---|---:|---:|---:|---|---:|---:|---:|---:|
| x | 2 | 2 | 0 | +- | 0.9725 | 1.08 | 0.05271 | 235/235/235 |
| y | 2 | 2 | 0 | +- | 0.7122 | 0.7841 | 0.06186 | 272.5/274.8/275 |
| z | 2 | 2 | 0 | +- | 1.447 | 2.419 | 0.06191 | 340/344.5/344.9 |
| rx | 2 | 2 | 0 | +- | 0.253 | 0.2942 | 0.3125 | 292.5/294.7/294.9 |
| ry | 2 | 2 | 0 | +- | 0.265 | 0.3445 | 0.2536 | 325/325/325 |
| rz | 2 | 2 | 0 | +- | 0.2115 | 0.2605 | 0.3224 | 262.5/269.2/269.8 |
| 6d | 2 | 2 | 0 | combined | 0.7365 | 0.7699 | 0.2425 | 275/275/275 |

Delay is identified using joint-speed cross-correlation and independently checked by peak alignment. Sample receipt age is retained separately and is not called control delay.
ROS source-stamp to local-receipt offset mean/P95 (ms): 0.3615 / 0.5507. This is a clock/transport diagnostic, not controller delay.
Command frequency mean/P95 (Hz): 50 / 50.
Sampled maximum joint velocity/acceleration (rad/s, rad/s^2): 0.05397 / 1.5.
Joint velocity/acceleration use finite differences at the recorder sample rate and are sensitive to encoder quantization; acceleration is not a controller-internal measurement.

## Error model

Each run stores three distinct terms: continuous ideal pose → FK(command joint) (IK residual), ideal pose → previous held command (trajectory discretization), and FK(command joint) → FK(feedback joint) (driver tracking). They are not algebraically merged into one cause.

Velocity-stratified 0.01/0.03 m/s results are stored under `velocity_profile` in the JSON; a speed is absent unless supervised runs at that requested speed exist.

## Safety

- Guard calibration: **provisional_do_not_deploy**
- Normal PASS coverage complete: False (100 PASS runs per scenario; +/- represented for each single-axis scenario)
- Tracking warning/stop recommendation (rad): 0.01906 / 0.03106
- Feedback timeout recommendation (s): 0.04212
- Force warning/stop recommendation (N): 2.544 / 3.887
- Torque warning/stop recommendation (N m): 0.1349 / 0.2012
- STOP runs: 0; latency mean/P95 (s): N/A / N/A
- STOP distance mean/max (rad): N/A / N/A
- STOP/failure reasons: N/A
- Collision and STOP details remain in each run entry in the JSON.

## Limitations

- Link7 is the current endpoint; there is no calibrated TCP in this benchmark.
- Only low-speed, manually supervised motion is permitted.
- Touch teleoperation and force control are disabled.
- No contact, soil insertion, or unattended repetition is permitted.
- A recommendation marked provisional must not replace the deployed Guard configuration.

## Sources

- `results/6d-plus-20261016-001.csv`
- `results/6d-return-20261016-002.csv`
- `results/rx-minus-20261016-002.csv`
- `results/rx-plus-20261016-001.csv`
- `results/ry-minus-20261016-002.csv`
- `results/ry-plus-20261016-001.csv`
- `results/rz-minus-20261016-002.csv`
- `results/rz-plus-20261016-001.csv`
- `results/x-minus-20261016-002.csv`
- `results/x-plus-20261016-001.csv`
- `results/y-minus-20261016-002.csv`
- `results/y-plus-20261016-001.csv`
- `results/z-minus-recovery-20261016-002.csv`
- `results/z-plus-20261016-001.csv`
