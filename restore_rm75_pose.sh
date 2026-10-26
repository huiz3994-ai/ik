#!/usr/bin/env bash
set -euo pipefail

container="${RM75_CONTAINER:-rm75_noetic}"

if ! docker inspect "$container" >/dev/null 2>&1; then
    echo "错误：找不到容器 $container" >&2
    exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$container")" != "true" ]]; then
    echo "错误：容器 $container 未运行；请先执行 docker start $container" >&2
    exit 1
fi

docker exec -i "$container" bash -lc '
    restore_args=("$@")
    set --
    source /opt/ros/noetic/setup.bash
    source /root/rm75_ws/devel/setup.bash
    rosrun rm_driver restore_fixed_pose.py "${restore_args[@]}"
' bash "$@"
