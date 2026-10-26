#!/usr/bin/env bash
set -u

failures=0
check_topic() {
    if timeout 3 rostopic echo -n 1 "$1" >/dev/null 2>&1; then
        printf 'PASS: %s\n' "$2"
    else
        printf 'FAIL: %s (%s)\n' "$2" "$1" >&2
        failures=$((failures + 1))
    fi
}

check_topic /joint_states 'RM75 seven-axis feedback'
check_topic /rm_driver/UdpSixZeroForce 'RM75 compensated six-axis force feedback'
check_topic /Geomagic/pose_raw 'Geomagic Touch pose'
check_topic /Geomagic/joy 'Geomagic Touch buttons'

joint_count="$(timeout 3 rostopic echo -n 1 /joint_states/position 2>/dev/null \
    | grep -c '^-' || true)"
if [ "$joint_count" -eq 7 ]; then
    printf 'PASS: joint feedback contains 7 positions\n'
else
    printf 'FAIL: expected 7 joint positions, got %s\n' "$joint_count" >&2
    failures=$((failures + 1))
fi

output_mode="$(rosparam get /rm75_touch_teleop/output_mode 2>/dev/null || true)"
if [ "$output_mode" = 'rm_joint_pos' ]; then
    printf 'PASS: hardware output mode is rm_joint_pos\n'
else
    printf 'FAIL: unexpected output mode: %s\n' "$output_mode" >&2
    failures=$((failures + 1))
fi

motion_enabled="$(rosparam get /rm75_touch_teleop/motion_enabled 2>/dev/null || true)"
printf 'INFO: motion_enabled=%s\n' "${motion_enabled:-unknown}"

if [ "$failures" -eq 0 ]; then
    printf 'PASS: RM75-6FB real Touch chain is ready\n'
    exit 0
fi
printf 'FAIL: RM75-6FB real Touch chain has %d failure(s)\n' "$failures" >&2
exit 1
