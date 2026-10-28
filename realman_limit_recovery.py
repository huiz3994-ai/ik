#!/usr/bin/env python3
"""Safely diagnose/recover RealMan error 0x1016 (joint near soft limit).

The script is read-only by default. Use --recover --yes to clear error 4118 and
move only joints that are too close to a configured soft limit.
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from dataclasses import dataclass
from typing import Any


CURRENT_LOOP_LIMIT_ERROR = 0x1016


class RobotError(RuntimeError):
    pass


@dataclass
class RobotSnapshot:
    joints: list[int]
    system_errors: list[int]
    min_limits: list[int]
    max_limits: list[int]
    joint_errors: list[int]
    enabled: list[int]


class RealManClient:
    def __init__(self, host: str, port: int, timeout: float) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout

    def request(
        self,
        payload: dict[str, Any],
        *,
        wait_for_trajectory: bool = False,
    ) -> list[dict[str, Any]]:
        deadline = time.monotonic() + (max(self.timeout, 30.0) if wait_for_trajectory else self.timeout)
        messages: list[dict[str, Any]] = []
        raw = b""

        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
                sock.sendall(json.dumps(payload, separators=(",", ":")).encode() + b"\r\n")
                sock.settimeout(0.5)

                while time.monotonic() < deadline:
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        if messages and not wait_for_trajectory:
                            break
                        continue
                    if not chunk:
                        break
                    raw += chunk
                    while b"\n" in raw:
                        line, raw = raw.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        messages.append(json.loads(line))
                        if wait_for_trajectory and messages[-1].get("state") == "current_trajectory_state":
                            return messages
        except (OSError, json.JSONDecodeError) as exc:
            raise RobotError(f"与 {self.host}:{self.port} 通信失败：{exc}") from exc

        if raw.strip():
            try:
                messages.append(json.loads(raw))
            except json.JSONDecodeError as exc:
                raise RobotError(f"控制器返回了无法解析的数据：{raw!r}") from exc
        if not messages:
            raise RobotError(f"控制器未响应命令 {payload.get('command')!r}")
        if wait_for_trajectory:
            raise RobotError("运动指令已发送，但等待运动完成超时；请立即检查机械臂状态")
        return messages

    def one(self, command: str) -> dict[str, Any]:
        return self.request({"command": command})[-1]


def as_int_list(value: Any, name: str) -> list[int]:
    if not isinstance(value, list) or not value or not all(isinstance(v, int) for v in value):
        raise RobotError(f"控制器返回的 {name} 格式异常：{value!r}")
    return value


def snapshot(client: RealManClient) -> RobotSnapshot:
    arm = client.one("get_current_arm_state").get("arm_state", {})
    joint_state = client.one("get_joint_err_flag")
    enabled_state = client.one("get_joint_en_state")
    min_state = client.one("get_joint_min_pos")
    max_state = client.one("get_joint_max_pos")

    result = RobotSnapshot(
        joints=as_int_list(arm.get("joint"), "joint"),
        system_errors=as_int_list(arm.get("err"), "err"),
        min_limits=as_int_list(min_state.get("min_pos"), "min_pos"),
        max_limits=as_int_list(max_state.get("max_pos"), "max_pos"),
        joint_errors=as_int_list(joint_state.get("err_flag"), "err_flag"),
        enabled=as_int_list(enabled_state.get("en_state"), "en_state"),
    )
    lengths = {
        len(result.joints), len(result.min_limits), len(result.max_limits),
        len(result.joint_errors), len(result.enabled),
    }
    if len(lengths) != 1:
        raise RobotError("控制器返回的关节数组长度不一致，拒绝继续")
    return result


def recovery_targets(state: RobotSnapshot, trigger_mmdeg: int, safe_mmdeg: int) -> tuple[list[int], list[int]]:
    targets = state.joints.copy()
    affected: list[int] = []
    for i, (position, lower, upper) in enumerate(
        zip(state.joints, state.min_limits, state.max_limits), start=1
    ):
        lower_gap = position - lower
        upper_gap = upper - position
        if lower_gap <= trigger_mmdeg:
            targets[i - 1] = lower + safe_mmdeg
            affected.append(i)
        elif upper_gap <= trigger_mmdeg:
            targets[i - 1] = upper - safe_mmdeg
            affected.append(i)
    return targets, affected


def deg(value: int) -> str:
    return f"{value / 1000:.3f}°"


def print_state(state: RobotSnapshot, trigger_mmdeg: int) -> None:
    print(f"系统错误：{state.system_errors}")
    print("关节  当前位置       软下限       软上限       最近裕量")
    for i, (position, lower, upper) in enumerate(
        zip(state.joints, state.min_limits, state.max_limits), start=1
    ):
        gap = min(position - lower, upper - position)
        marker = "  <-- 接近限位" if gap <= trigger_mmdeg else ""
        print(f"J{i:<2}  {deg(position):>10}  {deg(lower):>10}  {deg(upper):>10}  {deg(gap):>10}{marker}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="诊断并安全恢复睿尔曼机械臂 0x1016（关节处于软限位附近）错误。默认只读。"
    )
    default_host = os.environ.get("RM75_ARM_IP", "192.168.1.18")
    default_port = int(os.environ.get("RM75_ARM_PORT", "8080"))
    parser.add_argument("--host", default=default_host,
                        help=f"机械臂 IP（默认：{default_host}，可由 RM75_ARM_IP 配置）")
    parser.add_argument("--port", type=int, default=default_port,
                        help=f"JSON/TCP 端口（默认：{default_port}，可由 RM75_ARM_PORT 配置）")
    parser.add_argument("--timeout", type=float, default=3.0, help="查询超时秒数（默认：3）")
    parser.add_argument("--trigger-margin", type=float, default=5.0, help="距软限位多少度内视为需恢复（默认：5）")
    parser.add_argument("--safe-margin", type=float, default=10.0, help="恢复后距软限位的目标裕量，必须大于触发裕量（默认：10）")
    parser.add_argument("--speed", type=int, default=5, help="MoveJ 速度百分比 1~10（默认：5）")
    parser.add_argument("--recover", action="store_true", help="执行清错和低速退限位；不加时仅诊断")
    parser.add_argument("--yes", action="store_true", help="确认现场净空、急停可用，并允许机械臂运动")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not (0 < args.trigger_margin < args.safe_margin):
        raise RobotError("必须满足 0 < --trigger-margin < --safe-margin")
    if args.safe_margin > 30:
        raise RobotError("--safe-margin 不得超过 30°")
    if not 1 <= args.speed <= 10:
        raise RobotError("--speed 必须在 1~10 之间")
    if args.recover and not args.yes:
        raise RobotError("恢复会让机械臂运动；确认现场净空和急停可用后，同时添加 --yes")

    client = RealManClient(args.host, args.port, args.timeout)
    trigger = round(args.trigger_margin * 1000)
    safe = round(args.safe_margin * 1000)
    before = snapshot(client)
    print_state(before, trigger)

    targets, affected = recovery_targets(before, trigger, safe)
    if not affected:
        if before.system_errors == [CURRENT_LOOP_LIMIT_ERROR]:
            raise RobotError("存在 4118 错误，但没有关节落入配置的限位阈值；拒绝盲目运动")
        print("结论：没有关节接近软限位，无需恢复。")
        return 0

    print("需要恢复：" + ", ".join(f"J{i}" for i in affected))
    print("目标角度：" + ", ".join(f"J{i}={deg(targets[i-1])}" for i in affected))
    if not args.recover:
        print("当前为只读诊断；确认安全后使用 --recover --yes 执行恢复。")
        return 2

    unexpected_errors = [e for e in before.system_errors if e not in (0, CURRENT_LOOP_LIMIT_ERROR)]
    if unexpected_errors:
        raise RobotError(f"存在非 4118 系统错误 {unexpected_errors}，拒绝运动")
    if any(before.joint_errors):
        raise RobotError(f"存在关节错误 {before.joint_errors}，拒绝运动")
    if any(flag != 1 for flag in before.enabled):
        raise RobotError(f"并非所有关节均已使能 {before.enabled}，拒绝运动")

    if CURRENT_LOOP_LIMIT_ERROR in before.system_errors:
        cleared = client.one("clear_system_err")
        if cleared.get("clear_state") is not True:
            raise RobotError(f"清除 4118 错误失败：{cleared}")

    print(f"开始低速恢复（速度 {args.speed}%）……")
    replies = client.request(
        {
            "command": "movej",
            "joint": targets,
            "v": args.speed,
            "r": 0,
            "trajectory_connect": 0,
        },
        wait_for_trajectory=True,
    )
    accepted = any(r.get("command") == "movej" and r.get("receive_state") is True for r in replies)
    completed = any(
        r.get("state") == "current_trajectory_state" and r.get("trajectory_state") is True
        for r in replies
    )
    if not accepted or not completed:
        raise RobotError(f"恢复运动未成功完成：{replies}")

    after = snapshot(client)
    print_state(after, trigger)
    _, still_affected = recovery_targets(after, trigger, safe)
    if still_affected or after.system_errors != [0] or any(after.joint_errors):
        raise RobotError(
            f"恢复后复检未通过：near_limit={still_affected}, "
            f"system_errors={after.system_errors}, joint_errors={after.joint_errors}"
        )
    print("恢复成功：机械臂已退出软限位附近，系统与关节错误均为 0。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RobotError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        raise SystemExit(1)
