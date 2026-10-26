# RM75 vertical press controller

This package is an independent alternative to `rm75_touch_teleop`.  It does not
replace or modify the existing sampling node.

Button behavior:

- Grey only: control the end-effector orientation from the Touch orientation.
- White only: control the end-effector XYZ position from the Touch position.
- Grey + white: require the tool to be vertical, lock the current X/Y and
  orientation, descend only along `base_link -Z`, and switch to Z force
  control after contact.
- Release both buttons: stop commanding and publish the RM75 emergency stop.

The direct press defaults to 50 mm/s, has a 50 mm free-space travel limit, and
retains the force, torque, feedback timeout, tracking, IK, and joint-rate checks.

Build with `catkin_make`, then start in read-only/disarmed mode first:

```bash
/root/rm75_ws/start_rm75_vertical_press_real.sh
```

After validating topics, force coordinate, button mapping, and physical E-stop:

```bash
/root/rm75_ws/start_rm75_vertical_press_real.sh enable_motion:=true
```
