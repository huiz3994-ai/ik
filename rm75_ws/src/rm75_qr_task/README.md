# rm75_qr_task

Four-station marker-based PICK and original-pose RETURN orchestration for the
RM75-6FB. It reuses the existing MoveIt `/execute_trajectory` path and the
existing `rm_driver` tool-voltage topics. It does not implement another robot
or voltage driver.

The checked-in configuration is intentionally **not executable**: all four
stations are disabled and uncalibrated, station/tool/object offsets remain
unset, and server authorization defaults false. Hand-eye, voltage-interface,
and single-command-source facts are user-confirmed; the other readiness gates
remain false. `pick` and `return` are planning previews unless both CLI
execution flags, server authorization, and every readiness gate are present.

Printable markers are delivered as a four-page A4 PDF at repository path
`output/pdf/rm75_apriltag36h11_slots_0_3_A4_80mm.pdf`. Each page contains one
AprilTag 36h11 ID (0–3), an exact nominal outer black-square side of 80.0 mm,
and a 100 mm verification scale. Print at Actual size / 100%, never Fit. The
configured `size_m` is 0.080; keep `calibrated: false` until the mounted print
is measured. If it differs, enter the measured outer black-square side in
metres before enabling the slot.

Safe logic-only server:

```bash
roslaunch rm75_qr_task qr_task_sim.launch
rosrun rm75_qr_task task.py status
```

Read-only real integration (requires an already running camera and MoveIt
stack, but sends no motion or voltage command by default):

```bash
roslaunch rm75_qr_task qr_task_real.launch execution_authorized:=false
rosrun rm75_qr_task task.py pick 0
rosrun rm75_qr_task task.py return 0
rosrun rm75_qr_task task.py status
rosrun rm75_qr_task task.py stop
```

See `docs/QR_TASK.md` for interfaces, calibration rules, recovery, and the
commissioning checklist.
