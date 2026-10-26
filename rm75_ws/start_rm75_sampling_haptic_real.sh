#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /root/rm75_ws/devel/setup.bash
set -u

node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}
for node in /rm75_touch_teleop /rm_driver /Geomagic/Geomagic /touch_pubforce \
            /rm75_get_end_effector_pose /robot_state_publisher /gazebo; do
    if node_is_live "$node"; then
        echo "FAIL: conflicting live node exists: $node" >&2
        exit 1
    fi
done

echo "Starting RM75 Touch teleoperation with low-force haptic feedback."
echo "RM75 motion is ARMED after launch; hold the grey Touch button to command motion."
echo "Haptic output is limited to 0.5 N; force reflection and 60 N/m bounded position coupling are enabled."
echo "The existing RM75 force/torque, tracking-error, timeout, and emergency-stop gates remain active."
exec roslaunch rm75_touch_teleop sampling_real.launch \
  config:=/root/rm75_ws/src/rm75_touch_teleop/config/rm75_touch_haptic_real.yaml \
  enable_motion:=true haptic_feedback:=true "$@"
