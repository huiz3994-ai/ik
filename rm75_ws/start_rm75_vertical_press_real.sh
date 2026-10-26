#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [ ! -f /root/rm75_ws/devel/setup.bash ]; then
    echo "FAIL: /root/rm75_ws/devel/setup.bash is missing; build first." >&2
    exit 1
fi
source /root/rm75_ws/devel/setup.bash
set -u

node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}

for node in /rm75_vertical_press /rm75_touch_teleop /rm75_real_rl_runtime \
            /rm_control /move_group /rm_driver \
            /Geomagic/Geomagic /touch_pubforce /rm75_get_end_effector_pose \
            /robot_state_publisher /gazebo; do
    if node_is_live "$node"; then
        echo "FAIL: conflicting live node exists: $node" >&2
        echo "Run: /root/rm75_ws/stop_rm75_vertical_press.sh" >&2
        exit 1
    fi
done

echo "Starting independent RM75 vertical/direct-press controller."
echo "Grey=orientation; white=position; grey+white=lock XY/orientation and press along base -Z."
echo "Motion remains disarmed unless enable_motion:=true is supplied."
exec roslaunch rm75_vertical_press vertical_press_real.launch "$@"
