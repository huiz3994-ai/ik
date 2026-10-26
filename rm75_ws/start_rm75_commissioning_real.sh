#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build first." >&2
    exit 1
fi
source /root/rm75_ws/devel/setup.bash
set -u

# Do not call rosparam before roslaunch. On a clean startup no ROS master exists
# yet; real_touch_6f.launch sets /use_sim_time=false once roslaunch has created
# or connected to the master.

node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}
for node in /rm75_touch_teleop /rm_driver /Geomagic/Geomagic /touch_pubforce \
            /rm75_get_end_effector_pose /robot_state_publisher /gazebo; do
    if node_is_live "$node"; then
        echo "FAIL: conflicting live node exists: $node" >&2
        echo "The shared ROS master and camera nodes may remain running." >&2
        echo "Run: /root/rm75_ws/stop_rm75_touch.sh" >&2
        exit 1
    fi
done

echo "Starting RM75-6FB low-force commissioning."
echo "Motion remains disarmed unless enable_motion:=true is supplied."
exec roslaunch rm75_touch_teleop sampling_real.launch \
  config:=/root/rm75_ws/src/rm75_touch_teleop/config/rm75_touch_commissioning_real.yaml \
  "$@"
