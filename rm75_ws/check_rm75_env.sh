#!/usr/bin/env bash

pass() { printf 'PASS: %s\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*"; }
fail() { printf 'FAIL: %s\n' "$*"; failures=$((failures + 1)); }
failures=0

source /opt/ros/noetic/setup.bash
if [ -f /root/rm75_ws/devel/setup.bash ]; then
    source /root/rm75_ws/devel/setup.bash
else
    fail "workspace setup is missing"
fi
set -u

[ "$(rosversion -d 2>/dev/null)" = noetic ] && pass "ROS distro is noetic" || fail "ROS distro is not noetic"
[[ "${ROS_PACKAGE_PATH:-}" != *"/root/z1_ws"* ]] && pass "ROS_PACKAGE_PATH is isolated from Z1" || fail "ROS_PACKAGE_PATH contains /root/z1_ws"
[[ "${CMAKE_PREFIX_PATH:-}" != *"/root/z1_ws"* ]] && pass "CMAKE_PREFIX_PATH is isolated from Z1" || fail "CMAKE_PREFIX_PATH contains /root/z1_ws"

for cmd in gazebo rviz catkin ip; do
    command -v "$cmd" >/dev/null 2>&1 && pass "$cmd is installed" || fail "$cmd is missing"
done

for pkg in moveit_ros_move_group gazebo_ros_control controller_manager rm_msgs rm_gazebo rm_description rm_control rm_75_moveit_config geomagic_control rm75_touch_teleop; do
    rospack find "$pkg" >/dev/null 2>&1 && pass "ROS package $pkg is available" || fail "ROS package $pkg is missing"
done

[ -f /usr/lib/libHD.so ] && pass "OpenHaptics HD runtime is installed" || fail "OpenHaptics HD runtime is missing"
[ -f /usr/lib/libHDU.a ] && pass "OpenHaptics HDU runtime is installed" || fail "OpenHaptics HDU runtime is missing"

[ -e /dev/null ] && pass "/dev is accessible" || fail "/dev is inaccessible"
[ -r /run/udev/data ] && pass "/run/udev is accessible" || fail "/run/udev is inaccessible"
ip -brief address >/dev/null 2>&1 && pass "network interfaces are visible" || fail "network interfaces are unavailable"

if rosnode list >/dev/null 2>&1; then
    pass "ROS master is reachable"
    if rostopic list 2>/dev/null | grep -qx /joint_states; then
        pass "/joint_states exists"
        joint_state_output=$(rostopic echo -n 1 /joint_states 2>/dev/null || true)
        joint_count=$(sed -n '/^name:/,/^position:/p' <<<"$joint_state_output" | grep -c '^  - ' || true)
        [ "$joint_count" -eq 7 ] && pass "/joint_states contains 7 joints" || fail "/joint_states contains $joint_count joints"
        if grep -Eiq '(^|[^[:alpha:]])(nan|[-+]?inf)([^[:alpha:]]|$)' <<<"$joint_state_output"; then
            fail "/joint_states contains NaN or Inf"
        else
            pass "/joint_states values are finite"
        fi
    else
        fail "/joint_states is absent"
    fi

    if rosservice list 2>/dev/null | grep -qx /arm/controller_manager/list_controllers; then
        controller_output=$(rosservice call /arm/controller_manager/list_controllers 2>/dev/null || true)
        grep -q 'name: "arm_joint_controller"' <<<"$controller_output" \
            && grep -q 'state: "running"' <<<"$controller_output" \
            && grep -q 'type: "position_controllers/JointTrajectoryController"' <<<"$controller_output" \
            && pass "Gazebo position trajectory controller is running" || fail "Gazebo position trajectory controller is not running"
    else
        fail "/arm/controller_manager/list_controllers is absent"
    fi

    rosnode list 2>/dev/null | grep -qx /move_group && pass "MoveIt move_group is running" || fail "MoveIt move_group is not running"
    rosnode list 2>/dev/null | grep -qx /gazebo && pass "Gazebo ROS node is running" || fail "Gazebo ROS node is not running"

    if rosservice list 2>/dev/null | grep -qx /gazebo/get_model_state; then
        model_state_output=$(rosservice call /gazebo/get_model_state "model_name: rm_75" 2>/dev/null || true)
        if grep -q 'success: True' <<<"$model_state_output"; then
            pass "Gazebo RM75 model state is available"
            if grep -Eiq '(^|[^[:alpha:]])(nan|[-+]?inf)([^[:alpha:]]|$)' <<<"$model_state_output"; then
                fail "Gazebo RM75 model state contains NaN or Inf"
            else
                pass "Gazebo RM75 model state values are finite"
            fi
        else
            fail "Gazebo RM75 model state is unavailable"
        fi
    else
        fail "/gazebo/get_model_state is absent"
    fi
else
    warn "ROS master is not running; runtime checks skipped"
fi

if [ "$failures" -eq 0 ]; then
    printf 'PASS: RM75 environment check completed with no failures\n'
    exit 0
fi
printf 'FAIL: RM75 environment check found %d failure(s)\n' "$failures"
exit 1
