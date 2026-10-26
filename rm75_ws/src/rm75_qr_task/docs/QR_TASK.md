# Four-marker PICK/RETURN design and commissioning

## Audit result

The legacy `rm75_vision_grasp` AprilTag detector returns only a center pixel.
`DepthToPose` projects a 5×5 depth median and explicitly writes
`orientation.w = 1`; it is 3D position, not marker 6D pose. The legacy grasp
state machine plans pre-grasp/grasp/lift and attaches an object, but has no
release/original-pose return or durable cross-command record.

The vendor voltage contract was kept unchanged:

- command: `/rm_driver/SetToolVoltage`, `std_msgs/Byte`;
- protocol values: `0=0V`, `2=12V`, `3=24V`;
- result: `/rm_driver/Set_Tool_Voltage_Result`, `std_msgs/Bool`.

The adapter snapshots the result sequence before publishing and accepts only a
new callback received after the command. This avoids accepting an already
cached success, but the protocol has no transaction ID. A single voltage
command source remains a commissioning requirement. A true voltage ACK never
counts as object-held or object-released feedback.

## Perception and transforms

`marker_node.py` supports `apriltag16h5`, `apriltag36h11`, and QR texts.
The current five-tool configuration uses AprilTag 16h5 IDs 0–4. QR mode uses
configured texts such as `RM75_SLOT_0`. Both paths produce four ordered corners and use a
square physical model with `solvePnP`/IPPE. Camera intrinsics and distortion are
applied by OpenCV. Reprojection error, freshness, translation jump, rotation
jump, and a multi-frame window are checked before publishing.

The project also retains a legacy four-page AprilTag 36h11 vector print asset.
Each outer
black square is nominally 80.0 mm and each page includes a 100 mm verification
scale. Printing must use Actual size / 100% with Fit/Shrink disabled. PnP uses
the measured outer black-square side, not the A4 page or white quiet zone. The
checked-in `size_m` is `0.080`, while `calibrated` stays false until the mounted
sheet is measured.

The transform chain is:

```text
T_base_tcp = T_base_camera_optical × T_camera_optical_marker × T_marker_tcp
```

Outputs:

- `/qr_task/markers` (`geometry_msgs/PoseArray`): all stable marker poses;
- `/qr_task/slot_N/pose` (`geometry_msgs/PoseStamped`): one stable station;
- `/qr_task/target_pose` (`geometry_msgs/PoseStamped`): selected station;
- `/qr_task/vision_status` (`std_msgs/String`, JSON): quality/failure detail;
- `/qr_task/marker_observation` (`std_msgs/String`, JSON): validated input for
  the task server;
- TF children `qr_slot_N_marker` under `base_link`.

The supplied easy_handeye result is recorded in
`config/handeye_calibration.yaml` as `Link7 -> camera_link`. Optional
`publish_handeye_tf.launch` publishes exactly that transform. It must never be
started while URDF, RealSense, or easy_handeye already publishes the same
`camera_link` child. The hand-eye result and unique-owner condition were
user-confirmed on 2026-10-11; this does not enable the remaining task gates.

## State, persistence, and recovery

`/qr_task/command` uses `rm75_qr_task/TaskCommand`. The server serializes task
commands, persists every physical boundary with fsync + atomic rename, and
archives completed/abandoned records. It stores the seven joints, TCP,
`T_base_marker`, `T_marker_object`, `T_marker_tcp`, `T_tcp_object`, timestamps,
power command, last successful action, and pending action.

PICK follows `DETECT_MARKER -> PLAN_PICK -> MOVE_PRE_PICK -> MOVE_PICK ->
POWER_ON -> VERIFY_HOLD -> LIFT_OBJECT -> RETURN_PICK_START -> HOLDING`.
RETURN follows `PLAN_RETURN -> MOVE_PRE_PLACE -> MOVE_PLACE -> POWER_OFF ->
VERIFY_RELEASE -> RETREAT -> RETURN_RETURN_START -> IDLE`. Return TCP is
computed as `T_base_object_original × inverse(T_tcp_object)`, not xyz addition.

On restart from an active, HOLDING, or SAFE_STOP record, the node enters
`RECOVERY_REQUIRED`; it never guesses the power/object state. `stop` cancels
MoveIt and changes no tool voltage. Recovery requires a separately enabled
supervised path plus fresh independent hold/release feedback.

## Safety and commands

The three gates are separate:

1. request authorization: both CLI danger flags;
2. server authorization: `execution_authorized:=true`;
3. hardware readiness: every named verification in `safety.yaml`, fresh seven
   joint feedback/map heartbeat, and no known competing command source.

Normal commands default to preview:

```bash
rosrun rm75_qr_task task.py preview 0
rosrun rm75_qr_task task.py pick 0
rosrun rm75_qr_task task.py return 0
rosrun rm75_qr_task task.py status
rosrun rm75_qr_task task.py stop
```

The syntactic real-execution request is shown only for commissioning; it still
fails unless server and all readiness gates are deliberately enabled:

```bash
rosrun rm75_qr_task task.py pick 0 --execute --i-understand-this-moves-hardware
```

Recovery commands are `recover-holding <id> --supervised-confirm` and
`recover-idle --supervised-confirm`. They are rejected while
`allow_supervised_recovery: false`.

## Required physical commissioning

- print at 100%, measure the 100 mm scale and outer 80 mm black square, then
  set `calibrated: true` only if the configured/measured size agrees;
- measure every marker-to-object/TCP SE(3) transform;
- replace placeholder TCP/tool/camera/object collision geometry and payload;
- provide an independent, fresh object-present sensor for hold/release;
- validate OctoMap, camera side-mount envelope, target-only contact ACM, paths,
  execution-time collision monitor, stop ACK, braking distance, hard E-stop,
  and controller ownership;
- only then enable each slot and sign off safety flags individually.

## Validation performed without hardware

On 2026-10-09, the package built in the `rm75_noetic` ROS Noetic container.
Four nose suites reported 16 tests, 0 errors, 0 failures. Covered behavior:
6D PnP/rotation, SE(3), IDs 0–3, stale/jump/reprojection rejection, atomic
storage, preview non-mutation, PICK/RETURN, wrong-slot rejection, planning
failure, missing hold/release feedback after voltage ACK, fixed-station motion
lockout, restart recovery, and independent authorization. No Gazebo workflow,
camera stream, real voltage, or real motion was run.

On 2026-10-11, the A4 PDF was rendered to four PNG pages and OpenCV detected
exactly IDs 0, 1, 2 and 3. At 150 dpi the detected sides were 471–472 px versus
472.44 px expected for 80 mm. The PDF page boxes are exact A4. No physical
printer scaling has been measured yet.
