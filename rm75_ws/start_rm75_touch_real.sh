#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash

if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build the workspace first." >&2
    exit 1
fi
source /root/rm75_ws/devel/setup.bash
set -u

if rosnode list >/dev/null 2>&1; then
    echo "FAIL: a ROS graph is already running. Stop the simulation/old RM75 launch first." >&2
    exit 1
fi

echo "Starting RM75 hardware teleoperation (no Gazebo, no RViz)."
echo "Motion is disarmed unless enable_motion:=true is supplied."
exec roslaunch rm75_touch_teleop real_touch.launch "$@"
