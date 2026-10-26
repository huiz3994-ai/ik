# RM75-6FB guarded low-speed commissioning — 2026-10-13

## Authorization and scope

The user explicitly authorized a low-speed, small-displacement physical test.
The test moved only joint 7 by a commanded `+0.02 rad` and returned to the
measured start. Touch, force control, gripper/tool voltage, MoveIt execution,
and all other joints' commanded positions were unchanged.

`rm75_low_speed_commissioning` was the only `/rm_driver/JointPos` publisher.
Before advertising, the topic had no publisher; after the executable exited it
again had no publisher. The observation driver and ROS master were shut down
after post-flight checks.

## Guard configuration

- Quintic path-preserving time law at 50 Hz.
- Velocity/acceleration/jerk limits: 0.05 rad/s, 0.10 rad/s², 0.50 rad/s³.
- Maximum tracking error: 0.03 rad.
- Force/torque limits: 8 N and 0.5 N·m.
- Feedback timeout: 0.10 s.
- Every planned and measured sample checked against the bare RM75-6FB FCL
  collision model.
- Any stale feedback, tracking/force limit, collision, or controller planning
  failure stops streaming and requests `/rm_driver/Emergency_Stop`.

## Preflight

The stationary audit passed on 400 samples at 199.979 Hz. Minimum joint-limit
margin was 0.940137 rad, collisions were 0/400, and local FK differed from the
controller by 0.200 mm and 0.000791 rad. The complete round trip contained 135
samples over 2.68 s and passed offline dynamics, limit, FK, and FCL validation.

## Physical result

| Metric | Result |
|---|---:|
| Commanded joint-7 excursion | 0.020000 rad |
| Measured maximum joint-7 excursion | 0.017450 rad |
| Maximum seven-joint tracking error | 0.010097 rad |
| Return joint-vector error | 0.004733 rad |
| Peak compensated force norm | 1.381063 N |
| Peak compensated torque norm | 0.071986 N·m |
| Guard/stop activation | None |
| Execution verdict | PASS |

## Post-flight

The final 400-sample audit passed at 199.928 Hz. The arm was stationary within
0.000113 rad, minimum limit margin was 0.940416 rad, collisions were 0/400, and
local FK differed from the controller by 0.230 mm and 0.000655 rad. No new
`rm_driver` error was logged during the commissioned motion.

This result validates only the stated joint-7 micro-motion at the sampled
configuration. The PlanningScene still lacks measured tool and workcell
geometry, so it does not authorize larger, Cartesian, autonomous, or payload
motion.
