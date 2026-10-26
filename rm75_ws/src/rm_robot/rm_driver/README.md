# RM tool connector voltage

With `rm_driver` connected to the robot, set the RM75 tool connector to 24 V:

```bash
source /opt/ros/noetic/setup.bash
source /root/rm75_ws/devel/setup.bash
rosrun rm_driver set_tool_voltage.py 24
```

The helper accepts `0`, `12`, or `24`, waits for the controller response, and
returns a non-zero exit code if the driver is unavailable, the response times
out, or the controller rejects the request.

The equivalent raw ROS command is:

```bash
rostopic pub -1 /rm_driver/SetToolVoltage std_msgs/Byte "data: 3"
```

The protocol values are `0 = 0 V`, `2 = 12 V`, and `3 = 24 V`. The driver
rejects other values before sending a command to the robot.

## Move to the work pose, switch 24 V on, and return

`tool_voltage_roundtrip.py` records the current seven joint angles, moves to
`[-164.412, 157.617, 92.36, -3.112, -0.016, -0.568]` (XYZ in millimetres,
RPY in radians), sets the tool connector to 24 V, waits two seconds, retreats
150 mm along tool-frame -Z, and returns to the recorded joint angles:

```bash
rosrun rm_driver tool_voltage_roundtrip.py --execute
```

## Approach the work pose, switch tool voltage off, and return

The separate `tool_voltage_off_roundtrip.py` helper records the current seven
joint angles and moves to the
approach pose `[-168.822, 155.166, 242.275, -3.112, -0.016, -0.568]` (XYZ in
millimetres, RPY in radians). It then moves in a straight line to the work pose
`[-164.412, 157.617, 92.36, -3.112, -0.016, -0.568]`, sets the tool connector
to 0 V, retreats 150 mm along tool-frame -Z to the approach pose, and returns
to the recorded joint angles:

```bash
source /opt/ros/noetic/setup.bash
source /root/rm75_ws/devel/setup.bash
rosrun rm_driver tool_voltage_off_roundtrip.py --execute
```

The default speed is 10%. It can be changed, for example, with `--speed 0.05`.
The `--execute` flag is mandatory because this command moves the real arm.
Before running it, stop every other command source, clear the whole swept
volume, and make sure the physical emergency stop is reachable. The helper
also refuses to start if another ROS node is publishing on a known arm-motion
command topic.

Motion completion is verified from two consecutive
`/rm_driver/Arm_Current_State` replies instead of relying only on
`/rm_driver/Plan_State`. Default arrival tolerances are 3 mm Cartesian,
2 degrees tool orientation, and 1.5 degrees per joint. They can be changed with
`--position-tolerance-mm`, `--orientation-tolerance-deg`, and
`--joint-tolerance-deg`.

If target motion succeeds but setting 0 V fails, the helper still attempts to
retreat along the same path and return to the recorded start joints, then exits
with an error. If a motion times out, it requests `/rm_driver/Emergency_Stop`
and sends no further motion.

## Approach a second work pose from 250 mm overhead

The separate `pick_object_a.py` helper stores both the
starting joints and starting tool orientation. It first moves to
`[134.424, 183.920, 469.935]` mm while preserving the starting orientation,
then moves straight down to `[134.424, 183.920, 219.935]` mm while gradually
rotating to RPY `[3.128, 0.030, 1.560]` rad. It enables 24 V, rises 250 mm
while holding that target orientation, restores the starting orientation at
the overhead safety point, and then returns to the stored starting joints:

```bash
rosrun rm_driver pick_object_a.py --execute
```

The matching `place_object_a.py` entry approaches from 200 mm above while
preserving the starting orientation, changes to the target orientation at that
safe height, and then descends vertically while holding the target orientation.
It sets the tool connector to 0 V to release object A, rises 200 mm while still
holding the target orientation, restores the starting orientation at the
overhead point, and returns to the stored starting joints:

```bash
rosrun rm_driver place_object_a.py --execute
```

For 24 V output, verify approximately 24 V DC with a multimeter before wiring
the load: pin 6 / blue is positive and pin 5 / green is ground. Confirm the
connector pinout against the manual for the installed end-effector revision.
