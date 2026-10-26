#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build first." >&2
    exit 1
fi
source /root/rm75_ws/devel/setup.bash
set -u

# Do not call rosparam here: on a clean startup there is no ROS master yet and
# `set -e` would abort before roslaunch can create one. real_touch_6f.launch
# sets /use_sim_time=false as part of the launch after the master is available.

# Perception nodes may already own the shared ROS master. Only a live arm or
# Touch-control node is a conflict. A stale registration does not block startup.
node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}
for node in /rm75_touch_teleop /rm_driver /Geomagic/Geomagic /touch_pubforce \
            /rm75_get_end_effector_pose /robot_state_publisher /gazebo; do
    if node_is_live "$node"; then
        echo "FAIL: conflicting live node exists: $node" >&2
        echo "Stop the old RM75/Touch launch, then retry." >&2
        echo "Run: /root/rm75_ws/stop_rm75_touch.sh" >&2
        exit 1
    fi
done

echo "Starting RM75-6FB six-axis-force Touch teleoperation (no Gazebo/RViz)."
echo "Robot: 192.168.1.18, UDP host: 192.168.1.100."
echo "Motion remains disarmed unless enable_motion:=true is supplied."
exec roslaunch rm75_touch_teleop real_touch_6f.launch \
    udp_force_coordinate:=1 "$@"
