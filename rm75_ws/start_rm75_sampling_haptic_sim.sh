#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /root/rm75_ws/devel/setup.bash
set -u
if rosnode list >/dev/null 2>&1; then
    echo 'FAIL: another ROS graph is running. Stop it before starting haptic sampling simulation.' >&2
    exit 1
fi
exec roslaunch rm75_touch_teleop sampling_sim.launch haptic_feedback:=true "$@"
