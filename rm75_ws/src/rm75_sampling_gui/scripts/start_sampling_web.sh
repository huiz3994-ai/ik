#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CONTAINER_NAME="${RM75_CONTAINER:-rm75_noetic}"
WEB_PORT="${RM75_WEB_PORT:-8088}"
WEB_VENV="/root/rm75_ws/.venv-rm75-web"
PACKAGE="/root/rm75_ws/src/rm75_sampling_gui"
ARM_IP="${RM75_ARM_IP:-192.168.1.18}"
ARM_PORT="${RM75_ARM_PORT:-8080}"
HARDWARE_STAGE="${WORKSPACE_ROOT}/.rm75_hardware"
RS485_SOURCE="${RM75_RS485_SOURCE:-${HARDWARE_STAGE}/modbus.py}"
MOTOR_SOURCE="${RM75_MOTOR_SOURCE:-${HARDWARE_STAGE}/motor_control.py}"
MOTOR_SDK_SOURCE="${RM75_MOTOR_SDK_SOURCE:-${WORKSPACE_ROOT}/work/dm-motor-sdk/Python例程/u2can/DM_CAN.py}"
PYSERIAL_SOURCE="${RM75_PYSERIAL_SOURCE:-/usr/lib/python3/dist-packages/serial}"

MODE="unified"
DRY_RUN=false
DOCKER_EXEC_PID=""

usage() {
    cat <<EOF
Usage: $0 [--dry-run]

With no arguments, starts the fully authorized unified Web console directly.
There is no terminal menu or confirmation input. Control paths are switched
automatically for each browser function; the initial path is joint control.
The optional --mode argument is retained only for static/simulation validation.
EOF
}

fail() { echo "FAIL: $*" >&2; exit 2; }

while (( "$#" )); do
    case "$1" in
        --mode) [[ "$#" -ge 2 ]] || fail "--mode requires a value."; MODE="$2"; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) usage >&2; fail "unknown argument: $1" ;;
    esac
done

case "${MODE}" in
    unified|observe|simulation|z_force|joint_control|pose_pickup|qr_pickup|tool5_drill) ;;
    *) fail "mode must be unified, observe, simulation, z_force, joint_control, pose_pickup, qr_pickup, or tool5_drill." ;;
esac
SIMULATION=false
EXECUTION_ENABLED=false
QR_EXECUTION_AUTHORIZED=false
JOINT_CONTROL_EXECUTION_AUTHORIZED=false
TRAJECTORY_EXECUTION_AUTHORIZED=false
TOUCH_Z_FORCE_AUTHORIZED=false
TOOL5_DRILL_AUTHORIZED=false
SAMPLING_HARDWARE_AUTHORIZED=false
UNIFIED_CONTROL_AUTHORIZED=false
SUPERVISED_COMMISSIONING_AUTHORIZED=false
case "${MODE}" in
    unified)
        # Start the single-command unified console in the most commonly used
        # control path.  This selects exactly one command source; it does not
        # send a joint target or bypass the joint safety checks.
        EXECUTION_ENABLED=true
        JOINT_CONTROL_EXECUTION_AUTHORIZED=true
        SAMPLING_HARDWARE_AUTHORIZED=true
        UNIFIED_CONTROL_AUTHORIZED=true
        SUPERVISED_COMMISSIONING_AUTHORIZED=true
        ;;
    simulation) SIMULATION=true ;;
    z_force) EXECUTION_ENABLED=true; TOUCH_Z_FORCE_AUTHORIZED=true ;;
    joint_control) EXECUTION_ENABLED=true; JOINT_CONTROL_EXECUTION_AUTHORIZED=true; SUPERVISED_COMMISSIONING_AUTHORIZED=true ;;
    pose_pickup) EXECUTION_ENABLED=true; TRAJECTORY_EXECUTION_AUTHORIZED=true ;;
    qr_pickup) EXECUTION_ENABLED=true; QR_EXECUTION_AUTHORIZED=true; TRAJECTORY_EXECUTION_AUTHORIZED=true ;;
    tool5_drill) EXECUTION_ENABLED=true; TOOL5_DRILL_AUTHORIZED=true; SAMPLING_HARDWARE_AUTHORIZED=true ;;
esac

COMMAND_PATHS=0
for enabled in "${JOINT_CONTROL_EXECUTION_AUTHORIZED}" "${TRAJECTORY_EXECUTION_AUTHORIZED}" \
    "${TOUCH_Z_FORCE_AUTHORIZED}" "${TOOL5_DRILL_AUTHORIZED}"; do
    [[ "${enabled}" == true ]] && COMMAND_PATHS=$((COMMAND_PATHS + 1))
done
(( COMMAND_PATHS <= 1 )) || fail "joint_control, trajectory_bridge, z_force, and tool5_drill are mutually exclusive."

echo "Mode=${MODE}; unified=${UNIFIED_CONTROL_AUTHORIZED}; simulation=${SIMULATION}; execution=${EXECUTION_ENABLED}; joint=${JOINT_CONTROL_EXECUTION_AUTHORIZED}; trajectory=${TRAJECTORY_EXECUTION_AUTHORIZED}; z_force=${TOUCH_Z_FORCE_AUTHORIZED}; tool5_drill=${TOOL5_DRILL_AUTHORIZED}; QR=${QR_EXECUTION_AUTHORIZED}; supervised=${SUPERVISED_COMMISSIONING_AUTHORIZED}; sampling_hardware=${SAMPLING_HARDWARE_AUTHORIZED}"
if [[ "${DRY_RUN}" == true ]]; then
    echo "DRY-RUN: validation passed; no container, network, ROS, or hardware action was performed."
    exit 0
fi

preflight_robot() {
    command -v ping >/dev/null 2>&1 || { echo "FAIL: ping command not found; cannot verify robot connectivity." >&2; exit 1; }
    ping -c 1 -W 1 "${ARM_IP}" >/dev/null 2>&1 || { echo "FAIL: RM75 controller ${ARM_IP} is unreachable." >&2; exit 3; }
    if command -v nc >/dev/null 2>&1; then
        nc -z -w 2 "${ARM_IP}" "${ARM_PORT}" || { echo "FAIL: RM75 controller ${ARM_IP}:${ARM_PORT} is not accepting connections." >&2; exit 3; }
    else
        echo "WARN: nc is unavailable; only ICMP connectivity was verified." >&2
    fi
    echo "RM75 real control preflight passed: ${ARM_IP}:${ARM_PORT}"
}

stage_sampling_hardware() {
    local source
    for source in "${RS485_SOURCE}" "${MOTOR_SOURCE}" "${MOTOR_SDK_SOURCE}"; do
        [[ -f "${source}" ]] || { echo "FAIL: sampling hardware source missing: ${source}" >&2; exit 3; }
    done
    [[ -d "${PYSERIAL_SOURCE}" ]] || { echo "FAIL: pyserial source missing: ${PYSERIAL_SOURCE}" >&2; exit 3; }
    install -d -m 0755 "${HARDWARE_STAGE}" "${HARDWARE_STAGE}/python" "${WORKSPACE_ROOT}/work/dm-motor-sdk/Python例程/u2can"
    if [[ "$(readlink -f "${RS485_SOURCE}")" != "$(readlink -f "${HARDWARE_STAGE}/modbus.py")" ]]; then
        install -m 0644 "${RS485_SOURCE}" "${HARDWARE_STAGE}/modbus.py"
    fi
    if [[ "$(readlink -f "${MOTOR_SOURCE}")" != "$(readlink -f "${HARDWARE_STAGE}/motor_control.py")" ]]; then
        install -m 0755 "${MOTOR_SOURCE}" "${HARDWARE_STAGE}/motor_control.py"
    fi
    if [[ "$(readlink -f "${MOTOR_SDK_SOURCE}")" != "$(readlink -f "${WORKSPACE_ROOT}/work/dm-motor-sdk/Python例程/u2can/DM_CAN.py")" ]]; then
        install -m 0644 "${MOTOR_SDK_SOURCE}" "${WORKSPACE_ROOT}/work/dm-motor-sdk/Python例程/u2can/DM_CAN.py"
    fi
    install -d -m 0755 "${HARDWARE_STAGE}/python/serial"
    cp -a "${PYSERIAL_SOURCE}/." "${HARDWARE_STAGE}/python/serial/"
}

if [[ "${MODE}" != observe && "${MODE}" != simulation ]]; then preflight_robot; fi
if [[ "${SAMPLING_HARDWARE_AUTHORIZED}" == true ]]; then stage_sampling_hardware; fi

command -v docker >/dev/null 2>&1 || { echo "FAIL: docker command not found." >&2; exit 1; }
command -v flock >/dev/null 2>&1 || { echo "FAIL: flock command not found." >&2; exit 1; }
[[ "${WEB_PORT}" =~ ^[0-9]+$ ]] || fail "RM75_WEB_PORT must be numeric."
LOCK_NAME="${CONTAINER_NAME//[^A-Za-z0-9_.-]/_}_${WEB_PORT}"
exec 9>"/tmp/rm75_sampling_web_${LOCK_NAME}.lock"
flock -n 9 || fail "another RM75 Web startup owns container ${CONTAINER_NAME} port ${WEB_PORT}."

docker start "${CONTAINER_NAME}" >/dev/null
if docker exec "${CONTAINER_NAME}" bash -lc "ss -ltn 2>/dev/null | grep -qE '127.0.0.1:${WEB_PORT}[[:space:]]'"; then fail "127.0.0.1:${WEB_PORT} is already in use."; fi

# The HTTP socket can disappear before the previous roslaunch has finished
# stopping its owned base stack and ROS master.  Starting during that window
# attaches the new Web node to a master which the old launch then removes.
# Wait for both old Web processes to be completely gone before proceeding.
PREVIOUS_STOPPED=false
for _attempt in {1..75}; do
    if ! docker exec "${CONTAINER_NAME}" bash -lc \
        "pgrep -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_sampling_gui web.launch([[:space:]]|$)' >/dev/null || pgrep -f '[/]rm75_sampling_gui/sampling_web.py([[:space:]]|$)' >/dev/null"; then
        PREVIOUS_STOPPED=true
        break
    fi
    sleep 0.2
done
[[ "${PREVIOUS_STOPPED}" == true ]] || fail "previous RM75 Web shutdown did not finish within 15 seconds."

docker exec "${CONTAINER_NAME}" test -x "${WEB_VENV}/bin/python" || {
    echo "FAIL: Web venv missing. Run:" >&2
    echo "  docker exec ${CONTAINER_NAME} bash -lc 'python3 -m venv --system-site-packages ${WEB_VENV} && ${WEB_VENV}/bin/pip install -r ${PACKAGE}/web/backend/requirements.txt'" >&2
    exit 3
}
docker exec "${CONTAINER_NAME}" test -f "${PACKAGE}/web/frontend/dist/index.html" || { echo "FAIL: frontend build missing. Run npm install && npm run build in ${PACKAGE}/web/frontend" >&2; exit 3; }
if [[ "${SAMPLING_HARDWARE_AUTHORIZED}" == true ]]; then
    docker exec "${CONTAINER_NAME}" env PYTHONPATH=/root/rm75_ws/.rm75_hardware/python "${WEB_VENV}/bin/python" -c 'import serial' >/dev/null 2>&1 || { echo "FAIL: staged pyserial is unavailable for sampling hardware." >&2; exit 3; }
fi

cleanup() {
    trap - INT TERM EXIT
    docker exec "${CONTAINER_NAME}" bash -lc "pkill -INT -f '^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch rm75_sampling_gui web.launch port:=${WEB_PORT}' 2>/dev/null || true" >/dev/null 2>&1 || true
    if [[ -n "${DOCKER_EXEC_PID}" ]]; then wait "${DOCKER_EXEC_PID}" 2>/dev/null || true; fi
}
handle_signal() { local status="$1"; cleanup; exit "${status}"; }
trap cleanup EXIT
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM

echo "RM75 Web HMI: http://127.0.0.1:${WEB_PORT}"
echo "Press Ctrl+C for graceful shutdown of the Web backend and HMI-owned ROS launches."
docker exec -i "${CONTAINER_NAME}" bash -lc \
  'source /opt/ros/noetic/setup.bash; source /root/rm75_ws/devel/setup.bash; export PYTHONPATH="/root/rm75_ws/.rm75_hardware/python:${PYTHONPATH:-}"; exec "$@"' \
  bash roslaunch rm75_sampling_gui web.launch port:="${WEB_PORT}" simulation:="${SIMULATION}" \
  execution_enabled:="${EXECUTION_ENABLED}" qr_execution_authorized:="${QR_EXECUTION_AUTHORIZED}" \
  joint_control_execution_authorized:="${JOINT_CONTROL_EXECUTION_AUTHORIZED}" touch_z_force_authorized:="${TOUCH_Z_FORCE_AUTHORIZED}" \
  tool5_drill_authorized:="${TOOL5_DRILL_AUTHORIZED}" trajectory_execution_authorized:="${TRAJECTORY_EXECUTION_AUTHORIZED}" \
  supervised_commissioning_authorized:="${SUPERVISED_COMMISSIONING_AUTHORIZED}" sampling_hardware_authorized:="${SAMPLING_HARDWARE_AUTHORIZED}" \
  unified_control_authorized:="${UNIFIED_CONTROL_AUTHORIZED}" &
DOCKER_EXEC_PID=$!
set +e
wait "${DOCKER_EXEC_PID}"
STATUS=$?
set -e
DOCKER_EXEC_PID=""
exit "${STATUS}"
