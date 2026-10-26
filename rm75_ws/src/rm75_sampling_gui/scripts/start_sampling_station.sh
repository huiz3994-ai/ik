#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${RM75_CONTAINER:-rm75_noetic}"
DISPLAY_VALUE="${DISPLAY:-:1}"

if ! command -v docker >/dev/null 2>&1; then
    echo "FAIL: docker command not found." >&2
    exit 1
fi
if ! command -v xhost >/dev/null 2>&1; then
    echo "FAIL: xhost command not found." >&2
    exit 1
fi

docker start "${CONTAINER_NAME}" >/dev/null
xhost +SI:localuser:root >/dev/null
cleanup() {
    xhost -SI:localuser:root >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo "Starting RM75 field station on ${DISPLAY_VALUE}."
RM75_CONTROL_PATH="${RM75_CONTROL_PATH:-joint_control}"
case "${RM75_CONTROL_PATH}" in
    joint_control)
        DEFAULT_EXECUTION=true
        DEFAULT_QR=false
        DEFAULT_JOINT=true
        DEFAULT_TRAJECTORY=false
        ;;
    qr_pickup)
        DEFAULT_EXECUTION=true
        DEFAULT_QR=true
        DEFAULT_JOINT=false
        DEFAULT_TRAJECTORY=true
        ;;
    pose_pickup)
        DEFAULT_EXECUTION=true
        DEFAULT_QR=false
        DEFAULT_JOINT=false
        DEFAULT_TRAJECTORY=true
        ;;
    observe)
        DEFAULT_EXECUTION=false
        DEFAULT_QR=false
        DEFAULT_JOINT=false
        DEFAULT_TRAJECTORY=false
        ;;
    *)
        echo "FAIL: RM75_CONTROL_PATH must be joint_control, pose_pickup, qr_pickup, or observe." >&2
        exit 2
        ;;
esac
EXECUTION_ENABLED="${EXECUTION_ENABLED:-${DEFAULT_EXECUTION}}"
QR_EXECUTION_AUTHORIZED="${QR_EXECUTION_AUTHORIZED:-${DEFAULT_QR}}"
JOINT_CONTROL_EXECUTION_AUTHORIZED="${JOINT_CONTROL_EXECUTION_AUTHORIZED:-${DEFAULT_JOINT}}"
TRAJECTORY_EXECUTION_AUTHORIZED="${TRAJECTORY_EXECUTION_AUTHORIZED:-${DEFAULT_TRAJECTORY}}"
SUPERVISED_COMMISSIONING_AUTHORIZED="${SUPERVISED_COMMISSIONING_AUTHORIZED:-true}"
if [[ "${JOINT_CONTROL_EXECUTION_AUTHORIZED}" == "true" && "${TRAJECTORY_EXECUTION_AUTHORIZED}" == "true" ]]; then
    echo "FAIL: joint_control and trajectory_bridge cannot both be authorized." >&2
    exit 2
fi
echo "Control path: ${RM75_CONTROL_PATH}; execution=${EXECUTION_ENABLED}; QR=${QR_EXECUTION_AUTHORIZED}; joint_control=${JOINT_CONTROL_EXECUTION_AUTHORIZED}; trajectory_bridge=${TRAJECTORY_EXECUTION_AUTHORIZED}; supervised=${SUPERVISED_COMMISSIONING_AUTHORIZED}."

docker exec -i \
    -e DISPLAY="${DISPLAY_VALUE}" \
    "${CONTAINER_NAME}" \
    bash -lc "source /opt/ros/noetic/setup.bash && \
              source /root/rm75_ws/devel/setup.bash && \
              exec roslaunch rm75_sampling_gui gui.launch \
              execution_enabled:=${EXECUTION_ENABLED} \
              qr_execution_authorized:=${QR_EXECUTION_AUTHORIZED} \
              joint_control_execution_authorized:=${JOINT_CONTROL_EXECUTION_AUTHORIZED} \
              trajectory_execution_authorized:=${TRAJECTORY_EXECUTION_AUTHORIZED} \
              supervised_commissioning_authorized:=${SUPERVISED_COMMISSIONING_AUTHORIZED}"
