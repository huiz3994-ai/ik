#!/usr/bin/env bash
set -o pipefail

source /opt/ros/noetic/setup.bash
if [ -f /root/rm75_ws/devel/setup.bash ]; then
    source /root/rm75_ws/devel/setup.bash
fi
set -u

node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}

if ! rosnode list >/dev/null 2>&1; then
    echo "RM75 vertical-press stack is already stopped (no ROS master)."
    exit 0
fi

active="$(timeout 2 rostopic echo -n 1 /rm75_vertical_press/active 2>/dev/null \
    | awk '/data:/ {print $2; exit}' || true)"
if [ "${active}" = "True" ] && node_is_live /rm_driver; then
    echo "Vertical press is active; requesting /rm_driver/Emergency_Stop first."
    timeout 3 rostopic pub -1 /rm_driver/Emergency_Stop std_msgs/Empty '{}' \
        >/dev/null 2>&1 || true
fi

mapfile -t launch_pids < <(
    pgrep -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_vertical_press ' \
        2>/dev/null || true
)
for pid in "${launch_pids[@]}"; do
    kill -CONT "${pid}" 2>/dev/null || true
    kill -INT "${pid}" 2>/dev/null || true
done

deadline=$((SECONDS + 10))
for pid in "${launch_pids[@]}"; do
    while kill -0 "${pid}" 2>/dev/null && [ "${SECONDS}" -lt "${deadline}" ]; do
        sleep 0.2
    done
    if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
    fi
done

control_nodes=(/rm75_vertical_press /rm_driver /Geomagic/Geomagic
               /touch_pubforce /rm75_get_end_effector_pose
               /robot_state_publisher)
remaining=0
for node in "${control_nodes[@]}"; do
    if node_is_live "${node}"; then
        timeout 3 rosnode kill "${node}" >/dev/null 2>&1 || true
    fi
done
for node in "${control_nodes[@]}"; do
    if node_is_live "${node}"; then
        echo "FAIL: control node is still live: ${node}" >&2
        remaining=$((remaining + 1))
    fi
done
if [ "${remaining}" -ne 0 ]; then
    echo "Use the physical E-stop, then inspect the remaining process." >&2
    exit 1
fi
echo "PASS: RM75 vertical-press stack stopped."
