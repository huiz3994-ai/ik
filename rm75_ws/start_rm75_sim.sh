#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash

if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build the workspace first." >&2
    exit 1
fi

source /root/rm75_ws/devel/setup.bash

if [[ "${ROS_PACKAGE_PATH:-}" == *"/root/z1_ws"* ]] || [[ "${CMAKE_PREFIX_PATH:-}" == *"/root/z1_ws"* ]]; then
    echo "FAIL: Z1 workspace contamination detected." >&2
    exit 1
fi

# gazebo_ros_control logs one known false-positive per PositionJointInterface:
# it then intentionally falls back to stable direct-position control. Filter
# only that exact line while preserving all other output and pipeline failures.
set -o pipefail
roslaunch rm_gazebo arm_75_bringup_moveit.launch "$@" 2>&1 \
    | sed -u '/No p gain specified for pid/d'
