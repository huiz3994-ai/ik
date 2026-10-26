#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash

if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build the workspace first." >&2
    exit 1
fi
source /root/rm75_ws/devel/setup.bash
set -u

if rosnode list >/dev/null 2>&1 && rosnode ping -c 1 /gazebo >/dev/null 2>&1; then
    echo "FAIL: Gazebo is already running. Stop the old RM75 launch before using the combined launcher." >&2
    exit 1
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "FAIL: DISPLAY is empty; Gazebo cannot open its graphical window." >&2
    exit 1
fi

# Keep the known stable direct-position fallback quiet while preserving every
# other Gazebo, MoveIt, Touch, and controller warning.
set -o pipefail
roslaunch rm75_touch_teleop sim_touch.launch "$@" 2>&1 \
    | sed -u '/No p gain specified for pid/d'
