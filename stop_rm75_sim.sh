#!/usr/bin/env bash
set -euo pipefail

container="rm75_noetic"

if ! docker info >/dev/null 2>&1; then
  echo "无法访问 Docker。请先执行: newgrp docker"
  exit 1
fi

if ! docker container inspect "${container}" >/dev/null 2>&1; then
  echo "容器 ${container} 不存在，无需清理。"
  exit 0
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "${container}")" != "true" ]]; then
  echo "容器 ${container} 未运行，无需清理。"
  exit 0
fi

echo "========================================"
echo "正在清理 ${container} 中的 ROS 进程..."
echo "========================================"

if ! docker exec -i "${container}" bash -s <<'ROS_CLEANUP'
set +e

if [[ -r /opt/ros/noetic/setup.bash ]]; then
  source /opt/ros/noetic/setup.bash
fi

if [[ -r /root/rm75_ws/devel/setup.bash ]]; then
  source /root/rm75_ws/devel/setup.bash
fi


# ------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------

signal_pids()
{
  local sig="$1"
  shift

  for pid in "$@"; do

    [[ "${pid}" =~ ^[0-9]+$ ]] || continue

    # 不允许杀 PID 1
    if [[ "${pid}" -le 1 ]]; then
      continue
    fi

    # 不杀当前清理脚本
    if [[ "${pid}" == "$$" ]]; then
      continue
    fi

    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill "-${sig}" "${pid}" >/dev/null 2>&1 || true
    fi

  done
}


wait_pids()
{
  local loops="$1"
  shift

  for ((i=0; i<loops; i++)); do

    local alive=0

    for pid in "$@"; do
      if [[ "${pid}" =~ ^[0-9]+$ ]] &&
         kill -0 "${pid}" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done

    if [[ "${alive}" -eq 0 ]]; then
      return 0
    fi

    sleep 0.1
  done

  return 1
}


collect_ros_processes()
{
  {
    # apt 安装的 ROS 节点
    pgrep -f '/opt/ros/noetic/lib/' 2>/dev/null || true

    # catkin workspace 编译出来的 ROS 节点
    pgrep -f '/root/rm75_ws/devel/lib/' 2>/dev/null || true
    pgrep -f '/root/rm75_ws/install/lib/' 2>/dev/null || true

    # ROS launch / master / core
    pgrep -f '/opt/ros/noetic/bin/roslaunch([[:space:]]|$)' 2>/dev/null || true
    pgrep -f '/opt/ros/noetic/bin/rosmaster([[:space:]]|$)' 2>/dev/null || true
    pgrep -f '/opt/ros/noetic/bin/roscore([[:space:]]|$)' 2>/dev/null || true

    # roslaunch 启动的 Python/C++ 节点通常带 __name:=
    pgrep -f '__name:=' 2>/dev/null || true

    # Gazebo
    pgrep -x gzserver 2>/dev/null || true
    pgrep -x gzclient 2>/dev/null || true

  } | sort -nu
}


echo
echo "[1/6] 获取当前 ROS 节点 PID..."

declare -a ros_node_pids=()

if timeout 2s rosnode list >/dev/null 2>&1; then

  while IFS= read -r node; do

    [[ -z "${node}" ]] && continue

    pid="$(
      timeout 1s rosnode info "${node}" 2>/dev/null \
        | awk '/^Pid: / {print $2; exit}'
    )"

    if [[ "${pid}" =~ ^[0-9]+$ ]]; then
      ros_node_pids+=("${pid}")
    fi

  done < <(timeout 2s rosnode list 2>/dev/null)

fi


if ((${#ros_node_pids[@]})); then
  mapfile -t ros_node_pids < <(
    printf '%s\n' "${ros_node_pids[@]}" | sort -nu
  )

  echo "检测到 ROS 节点 PID:"
  printf '  %s\n' "${ros_node_pids[@]}"
else
  echo "未通过 ROS Master 获取到节点 PID。"
fi


# ------------------------------------------------------------
# 先停止 roslaunch
#
# 很重要：
# 如果节点设置了 respawn=true，
# 先杀节点的话 roslaunch 会把它重新启动。
# ------------------------------------------------------------

echo
echo "[2/6] 停止 roslaunch..."

mapfile -t launch_pids < <(
  pgrep -f '/opt/ros/noetic/bin/roslaunch([[:space:]]|$)' 2>/dev/null \
    | sort -nu
)

if ((${#launch_pids[@]})); then

  echo "发送 SIGINT 给 roslaunch:"
  printf '  PID %s\n' "${launch_pids[@]}"

  signal_pids INT "${launch_pids[@]}"

  wait_pids 30 "${launch_pids[@]}" || true

else
  echo "没有发现 roslaunch。"
fi


# ------------------------------------------------------------
# 优雅关闭已经记录的 ROS node
# ------------------------------------------------------------

echo
echo "[3/6] 请求 ROS 节点退出..."

if ((${#ros_node_pids[@]})); then

  signal_pids INT "${ros_node_pids[@]}"

  # 最多等待约 2 秒
  wait_pids 20 "${ros_node_pids[@]}" || true

fi


# ------------------------------------------------------------
# 扫描仍然存活的 ROS 进程
# ------------------------------------------------------------

echo
echo "[4/6] 清理残留 ROS 进程..."

mapfile -t remaining_pids < <(
  collect_ros_processes
)

if ((${#remaining_pids[@]})); then

  echo "以下 ROS 进程仍然存活，发送 SIGTERM:"
  printf '  PID %s\n' "${remaining_pids[@]}"

  signal_pids TERM "${remaining_pids[@]}"

  wait_pids 20 "${remaining_pids[@]}" || true

fi


# ------------------------------------------------------------
# 最后一层：SIGKILL
#
# robot_state_publisher 卡住时这里会处理掉它
# ------------------------------------------------------------

mapfile -t stubborn_pids < <(
  collect_ros_processes
)

if ((${#stubborn_pids[@]})); then

  echo
  echo "发现无法正常退出的进程，执行 SIGKILL:"
  printf '  PID %s\n' "${stubborn_pids[@]}"

  signal_pids KILL "${stubborn_pids[@]}"

  sleep 0.5

fi


# ------------------------------------------------------------
# 清理 PID 文件
# ------------------------------------------------------------

echo
echo "[5/6] 清理旧 PID 文件..."

rm -f \
  /tmp/rm75_hot*.pid \
  /tmp/rm75_teleop_direct.pid \
  /tmp/rm75_physical_probe.pid \
  >/dev/null 2>&1


# ------------------------------------------------------------
# 最终验证
# ------------------------------------------------------------

echo
echo "[6/6] 最终检查..."

mapfile -t final_pids < <(
  collect_ros_processes
)

if ((${#final_pids[@]})); then

  echo
  echo "FAIL: 仍然存在 ROS/Gazebo 进程："

  for pid in "${final_pids[@]}"; do
    ps -p "${pid}" -o pid=,ppid=,stat=,args=
  done

  exit 1
fi


echo
echo "ROS/Gazebo 进程已经全部清理。"

ROS_CLEANUP
then
  echo
  echo "========================================"
  echo "FAIL: ROS 清理未完全成功"
  echo "========================================"
  echo
  echo "请检查剩余进程。"
  echo "如果机械臂可能运动，请保持物理急停。"
  exit 1
fi


echo
echo "========================================"
echo "清理完成"
echo "========================================"
echo "之前的 ROS 节点、roslaunch、rosmaster、"
echo "roscore 和 Gazebo 已全部关闭。"
echo
echo "Docker 容器 ${container} 保持运行。"