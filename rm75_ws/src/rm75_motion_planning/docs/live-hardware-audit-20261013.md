# RM75-6FB read-only hardware audit — 2026-10-13

## Safety boundary

The controller was reachable at `192.168.1.18:8080` from host
`192.168.1.100`. Only `rm_driver` and the read-only
`rm75_live_state_audit` executable were used. No process published to
`/rm_driver/JointPos`, `MoveJ`, `MoveJ_P`, `MoveL`, `MoveC`, or teach command
topics. No power, enable, stop, tool-voltage, force-control, or motion command
was sent by this audit.

The controller identified the arm as `RM75-6FB`. UDP feedback used coordinate
mode 1 and supplied seven joint positions, controller flange pose, and six-axis
force data at the configured 5 ms period.

## Final stationary sample

The final audit collected 400 consecutive joint samples:

| Check | Result |
|---|---:|
| Observed joint-state rate | 200.005 Hz |
| Start-to-end joint-vector delta | 0.000025 rad |
| Maximum stationary-window span | 0.000123 rad |
| Minimum URDF joint-limit margin | 0.822401 rad |
| FCL collision samples | 0 / 400 |
| Local FK vs controller position | 0.168 mm |
| Local FK vs controller orientation | 0.001245 rad (0.0713 deg) |
| Audit verdict | PASS |

The latest sampled joint vector was:

```text
[2.087386, 0.256044, 0.021935, 1.532546, -0.120265, 1.035239, 2.219221] rad
```

The locally computed `base_T_Link7` translation was
`[-0.149630, 0.283877, 0.290551] m`.

## Interpretation and remaining boundary

This is evidence that the connected controller model, seven-joint ordering,
feedback rate, local RM75-6FB FK, vendor joint limits, and bare-robot collision
model agree at the sampled pose. It is not a motion test and does not validate
the user's physical tool, payload, workcell obstacles, stopping distance, or
execution bridge.

The arm configuration changed between separate audit runs while all checked ROS
motion topics had no publishers. That change was not commanded by this package;
it may have come from the controller pendant or manual operation. The final
400-sample window itself satisfied the stationary threshold.

At timestamp `1791890134.786`, `rm_driver` received a controller `PLAN_STATE`
with `state=false` and logged `Real Arm Trajectory Planning Error!`. The driver
source emits this message for any failed controller planning result. There was
no ROS motion-topic publisher at that time, so this audit cannot attribute the
event to a ROS command; it should be checked on the controller/pendant before a
commanded-motion test. The observation driver and its ROS master were shut down
after the audit.
