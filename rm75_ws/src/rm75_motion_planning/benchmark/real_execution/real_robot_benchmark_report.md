# RM75-6FB real robot benchmark report

Verdict: **INCOMPLETE — SUPERVISED PILOT COMPLETE, 100-RUN COHORTS PENDING**.

## Robot configuration

- Robot: RM75-6FB
- ROS: ROS1 Noetic
- Endpoint: `base_link -> Link7` (no calibrated TCP)
- Command rate: 50 Hz
- Pilot set: 14 recorded PASS runs; two per X/Y/Z/Rx/Ry/Rz/6D scenario

## Pilot accuracy and response timing

| Scenario | Runs | Position trajectory RMS mean (mm) | Orientation trajectory RMS mean (deg) | Response delay mean (ms) |
|---|---:|---:|---:|---:|
| X | 2 | 0.973 | 0.0527 | 235.0 |
| Y | 2 | 0.712 | 0.0619 | 272.5 |
| Z | 2 | 1.447 | 0.0619 | 340.0 |
| Rx | 2 | 0.253 | 0.3125 | 292.5 |
| Ry | 2 | 0.265 | 0.2536 | 325.0 |
| Rz | 2 | 0.211 | 0.3224 | 262.5 |
| 6D | 2 | 0.737 | 0.2425 | 275.0 |

Delay uses joint-speed cross-correlation with peak-alignment checking. ROS
source-stamp to local-receipt offset was 0.361 ms mean / 0.551 ms P95 and is a
clock/transport diagnostic, not controller delay.

Mean command frequency was 50.0 Hz. Recorder-rate finite differences observed
at most 0.05397 rad/s and 1.50 rad/s2; the acceleration estimate is sensitive
to encoder quantization and is not a controller-internal measurement.

## Safety

- Guard WARN/STOP: 0 / 0
- Collision samples, E-stop requests and controller failures: 0
- Worst joint tracking error: 0.018656 rad
- Peak force/torque: 2.592 N / 0.1349 N m
- Post-flight audit: PASS; 0 collisions; driver error `[0]`
- Driver stopped after testing; no `/rm_driver/JointPos` publisher remains

Pilot Guard recommendations are `provisional_do_not_deploy` and were not
copied into the deployed configuration. STOP latency/distance experiments were
not executed.

## Limitations

- Every scenario has only two runs, not the required 100.
- The Z return contains a documented 0.00112 rad orientation correction after
  strict fixed-orientation Z return failed IK preflight without moving.
- 0.03 m/s, STOP behavior, collision injection and final calibration remain
  pending.
- Current endpoint is Link7; Touch, force control, contact/soil insertion and
  tool voltage were disabled.

The generated detailed report, aggregate CSV/JSON and provisional Guard YAML
are in `results/report/`. Raw CSV/JSON/console logs are in `results/`, plots are
in `plots/`, and the supervised test narrative is in
`../../docs/phase4-real-characterization-20261016.md`.
