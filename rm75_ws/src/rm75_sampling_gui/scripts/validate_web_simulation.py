#!/usr/bin/env python3
"""Black-box acceptance check for an already running RM75 Web simulation."""
import argparse
import asyncio
import contextlib
import json
import sys

import httpx
import websockets


async def validate(base_url: str) -> None:
    base = base_url.rstrip("/")
    ws_url = ("wss://" if base.startswith("https://") else "ws://") + base.split("://", 1)[1]
    async with httpx.AsyncClient(base_url=base, timeout=4.0) as client:
        health = (await client.get("/api/v1/health")).json()
        if not health.get("simulation") or health.get("offline"):
            raise RuntimeError("目标服务不是安全的SIMULATION模式")

        lease_response = await client.post(
            "/api/v1/control/session", json={"client": "simulation-validator"})
        lease_response.raise_for_status()
        session_id = lease_response.json()["session_id"]
        headers = {"X-RM75-Control-Session": session_id}
        websocket = await websockets.connect(ws_url + "/api/v1/ws")
        heartbeat_task = None
        drain_task = None
        try:
            await websocket.send(json.dumps(
                {"type": "heartbeat", "session_id": session_id}))
            for _ in range(6):
                message = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                if message.get("type") == "heartbeat_ack":
                    break
            else:
                raise RuntimeError("WebSocket控制租约未收到heartbeat_ack")

            async def heartbeat() -> None:
                while True:
                    await websocket.send(json.dumps(
                        {"type": "heartbeat", "session_id": session_id}))
                    await asyncio.sleep(0.15)

            async def drain() -> None:
                async for _message in websocket:
                    pass

            heartbeat_task = asyncio.create_task(heartbeat())
            drain_task = asyncio.create_task(drain())

            target = [0.25, -0.18, 0.12, -0.10, 0.08, 0.06, -0.04]
            response = await client.post("/api/v1/joints/execute",
                                         json={"target": target}, headers=headers)
            response.raise_for_status()
            if not response.json().get("simulation"):
                raise RuntimeError("七轴目标没有进入模拟器")

            tracked = False
            for _ in range(30):
                await asyncio.sleep(0.1)
                state = (await client.get("/api/v1/state")).json()
                if abs(state["joints"]["joint1"] - target[0]) < 0.015:
                    tracked = True
                    break
            if not tracked:
                raise RuntimeError("仿真关节未在3秒内跟踪目标")

            response = await client.post("/api/v1/commands/start_z_force_teleop",
                json={"payload": {}}, headers=headers)
            response.raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if state.get("control_launch_mode") != "z_force_teleop":
                raise RuntimeError("Z力位仿真模式未同步到网页状态")
            if state.get("control_mode") != "POSITION_MODE":
                raise RuntimeError("Z力位仿真未从位置模式启动")
            if not state.get("haptic_enabled") or not state.get("haptic_force_fresh"):
                raise RuntimeError("仿真力反馈状态未就绪")
            haptic = state.get("haptic_command", {})
            norm = sum(float(haptic.get(axis, 0.0)) ** 2
                       for axis in ("x", "y", "z")) ** 0.5
            if haptic.get("x") != 0.0 or haptic.get("z") != 0.0 or not 0.0 < norm <= 0.5:
                raise RuntimeError("仿真力反馈未限制在Touch Y轴或0.5N内")
            response = await client.post("/api/v1/commands/stop_z_force_teleop",
                json={"payload": {}}, headers=headers)
            response.raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if state.get("control_launch_mode") is not None or state.get("haptic_enabled"):
                raise RuntimeError("停止Z力位后仿真力反馈未清零")

            response = await client.post("/api/v1/commands/set_voltage",
                json={"payload": {"voltage": 24}}, headers=headers)
            response.raise_for_status()
            if (await client.get("/api/v1/state")).json()["tool_voltage"] != 24:
                raise RuntimeError("仿真工具电压状态未更新")

            (await client.post("/api/v1/commands/pump_on",
                               json={"payload": {}}, headers=headers)).raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if not state["sampling_hardware"]["sensors"]["pump_on"]:
                raise RuntimeError("仿真水泵状态未更新")
            motor = await client.post("/api/v1/commands/motor", json={"payload": {
                "action": "forward", "speed": 0.5, "duration": 0.2}}, headers=headers)
            motor.raise_for_status()
            if not motor.json().get("simulation"):
                raise RuntimeError("电机命令没有进入模拟器")

            (await client.post("/api/v1/commands/emergency_stop",
                               json={"payload": {}}, headers=headers)).raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if state["operator_state"] != "SAFE_STOP":
                raise RuntimeError("仿真软件急停没有锁存SAFE_STOP")
            if state["sampling_hardware"]["sensors"]["pump_on"]:
                raise RuntimeError("仿真软件急停没有关闭水泵")
            blocked = await client.post("/api/v1/commands/enter_force_control",
                                        json={"payload": {}}, headers=headers)
            if blocked.status_code != 409:
                raise RuntimeError("SAFE_STOP下的功能命令未被拒绝")
            blocked_joint = await client.post("/api/v1/joints/execute",
                                               json={"target": [0.0] * 7}, headers=headers)
            if blocked_joint.status_code != 409:
                raise RuntimeError("SAFE_STOP下的七轴目标未被拒绝")
            blocked_voltage = await client.post("/api/v1/commands/set_voltage",
                json={"payload": {"voltage": 24}}, headers=headers)
            if blocked_voltage.status_code != 409:
                raise RuntimeError("SAFE_STOP下的工具电压未被拒绝")
            (await client.post("/api/v1/commands/recover",
                               json={"payload": {}}, headers=headers)).raise_for_status()
            if (await client.get("/api/v1/state")).json()["operator_state"] != "IDLE":
                raise RuntimeError("仿真恢复后未回到IDLE")

            # QR and manually configured pickup are independent HMI workflows.
            # Exercise the complete QR state machine, including fail-closed order,
            # honest grasp status, cancellation while holding, and return cleanup.
            select = await client.post("/api/v1/commands/select_tool", json={
                "payload": {"tool": "Tool2"}}, headers=headers)
            select.raise_for_status()
            out_of_order = await client.post("/api/v1/commands/plan_tool",
                json={"payload": {}}, headers=headers)
            if out_of_order.status_code != 409:
                raise RuntimeError("二维码目标未计算时，Dry Run未被拒绝")
            for command in ("detect_marker", "compute_targets", "preview_tool",
                            "plan_tool", "execute_tool"):
                response = await client.post("/api/v1/commands/{}".format(command),
                    json={"payload": {}}, headers=headers)
                response.raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            qr = state.get("five_target_status", {})
            targets = state.get("five_targets", {}).get("targets", [])
            if (not qr.get("targets_valid") or qr.get("holding_slot") != 2 or
                    qr.get("grasp_confirmed") is not False or len(targets) != 5):
                raise RuntimeError("二维码仿真取物状态或五目标数据不完整")
            if state.get("operator_state") != "TOOL_HOLD" or state.get(
                    "tool_voltage") != 24:
                raise RuntimeError("二维码仿真取物未进入持有/24V状态")
            (await client.post("/api/v1/commands/cancel_tool",
                json={"payload": {}}, headers=headers)).raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if (state.get("operator_state") != "TOOL_HOLD" or
                    state.get("five_target_status", {}).get("holding_slot") != 2):
                raise RuntimeError("取消运动时错误地清除了实际持有状态")
            (await client.post("/api/v1/commands/return_tool",
                json={"payload": {}}, headers=headers)).raise_for_status()
            state = (await client.get("/api/v1/state")).json()
            if (state.get("operator_state") != "IDLE" or
                    state.get("five_target_status", {}).get("holding_slot") is not None or
                    state.get("tool_voltage") != 0):
                raise RuntimeError("二维码归还后状态/末端电压未安全复位")
            invalid_return = await client.post("/api/v1/commands/return_tool",
                json={"payload": {}}, headers=headers)
            if invalid_return.status_code != 409:
                raise RuntimeError("未持有工具时的归还命令未被拒绝")

            tools = (await client.get("/api/v1/tools")).json().get("tools", [])
            if len(tools) != 5:
                raise RuntimeError("五工具持久化配置没有完整返回")
            dry_run = await client.post("/api/v1/tools/1/dry-run", headers=headers)
            dry_run.raise_for_status()
            if not dry_run.json().get("dry_run"):
                raise RuntimeError("Tool1没有完成安全Dry Run")
            pickup = await client.post("/api/v1/tools/1/execute", headers=headers)
            pickup.raise_for_status()
            if pickup.json().get("grasp_confirmed") is not False:
                raise RuntimeError("无物体传感器时错误报告了抓取成功")
            pick_state = (await client.get(
                "/api/v1/state")).json().get("configured_pick_status", {})
            if (pick_state.get("status") != "COMPLETED_UNVERIFIED" or
                    pick_state.get("step") != 6):
                raise RuntimeError("五工具取物步骤状态没有完整结束")

            frame = await client.get("/api/v1/vision/frame.jpg")
            frame.raise_for_status()
            if not frame.content.startswith(b"\xff\xd8") or len(frame.content) < 1000:
                raise RuntimeError("仿真相机没有返回有效JPEG")
            print("PASS: simulation health, lease heartbeat, joint tracking, Z-only "
                  "force/position mode, bounded Touch-Y haptics, voltage, sampling "
                  "hardware, SAFE_STOP rejection/recovery, full QR detect/compute/"
                  "preview/plan/execute/cancel/return flow with order guards, "
                  "five-tool manual Dry Run/execute with honest unverified grasp "
                  "status, and camera JPEG")
        finally:
            for task in (heartbeat_task, drain_task):
                if task is not None:
                    task.cancel()
            await websocket.close()
            with contextlib.suppress(Exception):
                await client.delete("/api/v1/control/session", headers=headers)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8088")
    args = parser.parse_args()
    try:
        asyncio.run(validate(args.url))
    except Exception as error:
        print("FAIL: {}".format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
