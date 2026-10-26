#!/usr/bin/env bash
set -o pipefail

source /opt/ros/noetic/setup.bash
if [ -f /root/rm75_ws/devel/setup.bash ]; then
    source /root/rm75_ws/devel/setup.bash
fi
set -u

control_nodes=(
    /rm75_vertical_press
    /rm75_touch_teleop
    /rm75_real_rl_runtime
    /rm_control
    /move_group
    /rm_driver
    /Geomagic/Geomagic
    /touch_pubforce
    /rm75_get_end_effector_pose
    /robot_state_publisher
    /gazebo
)

node_is_live() {
    local ping_output
    ping_output="$(timeout 2 rosnode ping -c 1 "$1" 2>&1 || true)"
    [[ "${ping_output}" == *"xmlrpc reply"* ]]
}

if ! rosnode list >/dev/null 2>&1; then
    echo "RM75/Touch control stack is already stopped (no ROS master)."
    exit 0
fi

# RL and rm_control do not expose the teleop active diagnostic. Always request
# a hardware software-stop before removing any command publisher.
if node_is_live /rm_driver; then
    echo "Requesting /rm_driver/Emergency_Stop before shutdown."
    timeout 3 rostopic pub -1 /rm_driver/Emergency_Stop std_msgs/Empty '{}' \
        >/dev/null 2>&1 || true
fi

# Stop launch parents that may respawn their children. These exact patterns do
# not match RealSense, hand-eye calibration, or the launch process that owns the
# shared ROS master.
launch_patterns=(
    '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_vertical_press '
    '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_touch_teleop '
    '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm_control rm_75_control.launch'
    '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm_75_moveit_config demo_6fb_realrobot.launch'
)
launch_pids=()
for pattern in "${launch_patterns[@]}"; do
    while IFS= read -r pid; do
        [ -n "${pid}" ] && launch_pids+=("${pid}")
    done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done

for pid in "${launch_pids[@]}"; do
    echo "Stopping conflicting roslaunch process ${pid}..."
    kill -CONT "${pid}" 2>/dev/null || true
    kill -INT "${pid}" 2>/dev/null || true
done

deadline=$((SECONDS + 10))
for pid in "${launch_pids[@]}"; do
    while kill -0 "${pid}" 2>/dev/null && [ "${SECONDS}" -lt "${deadline}" ]; do
        sleep 0.2
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "roslaunch ${pid} did not exit after 10 s; sending SIGTERM." >&2
        kill -TERM "${pid}" 2>/dev/null || true
    fi
done

# Fallback for standalone nodes, including the real-RL runtime.
for node in "${control_nodes[@]}"; do
    if node_is_live "${node}"; then
        echo "Stopping remaining control node ${node}..."
        timeout 3 rosnode kill "${node}" >/dev/null 2>&1 || true
    fi
done

# Give ROS shutdown callbacks time to release the robot and network resources.
node_deadline=$((SECONDS + 10))
while [ "${SECONDS}" -lt "${node_deadline}" ]; do
    live_count=0
    for node in "${control_nodes[@]}"; do
        if node_is_live "${node}"; then
            live_count=$((live_count + 1))
        fi
    done
    [ "${live_count}" -eq 0 ] && break
    sleep 0.2
done

# Last graceful fallback: terminate only the exact PID reported by a remaining
# ROS node. Never use broad process-name matching here.
for node in "${control_nodes[@]}"; do
    if node_is_live "${node}"; then
        pid="$(rosnode info "${node}" 2>/dev/null \
            | awk '/Pid:/ {print $2; exit}' || true)"
        if [[ "${pid}" =~ ^[0-9]+$ ]]; then
            echo "Node ${node} did not exit; sending SIGTERM to PID ${pid}." >&2
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    fi
done
sleep 1

remaining=0
for node in "${control_nodes[@]}"; do
    if node_is_live "${node}"; then
        echo "FAIL: control node is still live: ${node}" >&2
        remaining=$((remaining + 1))
    fi
done

if [ "${remaining}" -ne 0 ]; then
    echo "Use the physical E-stop if motion is possible, then inspect the remaining process." >&2
    exit 1
fi

echo "PASS: all conflicting RM75 control nodes stopped."
echo "Shared ROS master and camera/calibration nodes were preserved."
