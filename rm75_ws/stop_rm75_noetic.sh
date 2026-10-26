#!/usr/bin/env bash

set -u

readonly CONTAINER_NAME="rm75_noetic"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    echo "用法：$0"
    echo "停止 ${CONTAINER_NAME} 容器及其中的全部 ROS 节点；不会删除容器。"
    exit 0
fi

if [[ $# -ne 0 ]]; then
    echo "错误：不支持参数。使用 --help 查看用法。" >&2
    exit 2
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "错误：未找到 docker 命令。" >&2
    exit 1
fi

if ! docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
    echo "错误：Docker 容器 ${CONTAINER_NAME} 不存在。" >&2
    exit 1
fi

running="$(docker inspect --format '{{.State.Running}}' "${CONTAINER_NAME}")"
if [[ "${running}" != "true" ]]; then
    echo "${CONTAINER_NAME} 已经停止，无需操作。"
    exit 0
fi

echo "正在停止 ${CONTAINER_NAME} 及其中的 ROS 节点……"
if docker stop --time 10 "${CONTAINER_NAME}" >/dev/null; then
    echo "${CONTAINER_NAME} 已停止。"
else
    echo "错误：停止 ${CONTAINER_NAME} 失败。" >&2
    exit 1
fi
