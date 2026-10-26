#!/usr/bin/env bash
set -u

failures=0
pass() { printf 'PASS: %s\n' "$*"; }
fail() { printf 'FAIL: %s\n' "$*"; failures=$((failures + 1)); }

rostopic list 2>/dev/null | grep -qx /Geomagic/pose_raw \
  && pass 'Touch raw pose is available' || fail 'missing /Geomagic/pose_raw'
rostopic list 2>/dev/null | grep -qx /Geomagic/joy \
  && pass 'Touch buttons are available' || fail 'missing /Geomagic/joy'
rostopic list 2>/dev/null | grep -qx /Touch_Pose_increment \
  && pass 'Touch_my_ws increment mapping is available' || fail 'missing /Touch_Pose_increment'
rostopic list 2>/dev/null | grep -qx /Touch_Anchor_updated \
  && pass 'Touch_my_ws anchor signal is available' || fail 'missing /Touch_Anchor_updated'
rostopic list 2>/dev/null | grep -qx /end_effector_pose \
  && pass 'RM75 end-effector pose is available' || fail 'missing /end_effector_pose'
rostopic list 2>/dev/null | grep -qx /joint_states \
  && pass 'RM75 joint states are available' || fail 'missing /joint_states'
rosservice list 2>/dev/null | grep -qx /arm/controller_manager/list_controllers \
  && pass 'RM75 controller manager is available' || fail 'missing RM75 controller manager'
rosnode list 2>/dev/null | grep -qx /rm75_touch_teleop \
  && pass 'RM75 Touch teleop node is running' || fail 'teleop node is not running'

if [ "$failures" -eq 0 ]; then
  printf 'PASS: Touch-to-RM75 chain is ready\n'
  exit 0
fi
printf 'FAIL: Touch-to-RM75 chain has %d failure(s)\n' "$failures"
exit 1
