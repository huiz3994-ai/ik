#!/usr/bin/env python3
"""Web transport safety tests.  No ROS publisher or real command is created."""
import time
from pathlib import Path
import threading
import shutil

import pytest
from fastapi.testclient import TestClient

from rm75_sampling_gui.web.backend.app import create_app
from rm75_sampling_gui.web.backend.services import WebRuntime, load_web_configuration
from rm75_sampling_gui.web.backend.session import ControlSessionManager, SessionError
from rm75_sampling_gui.web.backend.hardware import SamplingHardware


def test_control_lease_requires_bound_websocket_and_expires():
    sessions = ControlSessionManager(0.4)
    lease = sessions.acquire("test browser")
    with pytest.raises(SessionError, match="WebSocket"):
        sessions.require(lease["session_id"])
    sessions.bind_websocket(lease["session_id"], "socket-a")
    assert sessions.require(lease["session_id"]).client == "test browser"
    time.sleep(0.42)
    with pytest.raises(SessionError, match="超时"):
        sessions.require(lease["session_id"])


def test_control_lease_is_exclusive_and_socket_bound():
    sessions = ControlSessionManager(0.5)
    lease = sessions.acquire("first")
    with pytest.raises(SessionError, match="占用"):
        sessions.acquire("second")
    sessions.bind_websocket(lease["session_id"], "socket-a")
    with pytest.raises(SessionError, match="不匹配"):
        sessions.heartbeat(lease["session_id"], "socket-b")
    assert not sessions.release(lease["session_id"], "socket-b")
    assert sessions.release(lease["session_id"], "socket-a")


def test_session_ids_cannot_be_forged():
    sessions = ControlSessionManager(0.5)
    sessions.acquire("owner")
    with pytest.raises(SessionError):
        sessions.require("not-the-session")


class _FakeRuntime:
    def __init__(self):
        self.offline = False
        self.simulation = False
        self.sessions = ControlSessionManager(0.5)
        self.closed = False
        self.commands = []

    def connect(self): return False
    def close(self): self.closed = True
    def publish_guarded_heartbeat(self): return False
    def sample_experiment(self): pass
    def snapshot(self):
        return {"sequence": 1, "timestamp": time.time(), "health": {"master": False}}
    def model_descriptor(self): return {"model": "RM75-6FB", "joints": [], "visuals": []}
    def parameters(self): return {"gui": {}, "sources": {}, "tools": {}}
    def experiments(self): return []
    def preview(self, target): return {"target": list(target), "violations": []}
    def command(self, name, session_id, payload):
        self.sessions.require(session_id); self.commands.append(name); return {"ok": True}
    def execute_joints(self, session_id, values):
        self.sessions.require(session_id); return {"target": list(values)}
    def lifecycle(self, name, action, session_id):
        self.sessions.require(session_id); return {"name": name, "action": action}


def test_rest_preview_never_requires_or_creates_control_session():
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime)) as client:
        response = client.post("/api/v1/joints/preview", json={"target": [0] * 7})
        assert response.status_code == 200
        assert response.json()["target"] == [0] * 7
        assert not runtime.sessions.snapshot()["active"]


def test_commands_require_websocket_heartbeat_bound_lease():
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime)) as client:
        lease = client.post("/api/v1/control/session", json={"client": "test"}).json()
        headers = {"X-RM75-Control-Session": lease["session_id"]}
        assert client.post("/api/v1/commands/recover", json={"payload": {}},
                           headers=headers).status_code == 409
        with client.websocket_connect("/api/v1/ws") as socket:
            socket.send_json({"type": "heartbeat", "session_id": lease["session_id"]})
            messages = [socket.receive_json(), socket.receive_json()]
            assert any(item["type"] == "heartbeat_ack" for item in messages)
            response = client.post("/api/v1/commands/recover",
                                   json={"payload": {}}, headers=headers)
            assert response.status_code == 200
            assert runtime.commands == ["recover"]


def test_motor_stop_interrupts_an_active_bounded_command(tmp_path):
    script = tmp_path / "fake_motor.py"
    script.write_text(
        "import sys, time\n"
        "command = sys.argv[1]\n"
        "time.sleep(5 if command == 'forward' else 0.02)\n"
        "print('completed; motor stopped and disabled')\n",
        encoding="utf-8")
    hardware = SamplingHardware({}, authorized=False)
    hardware.authorized = True
    hardware.motor_script = str(script)
    errors = []

    def run_motion():
        try:
            hardware.motor_command("forward", speed=0.5, duration=5.0)
        except RuntimeError as error:
            errors.append(str(error))

    worker = threading.Thread(target=run_motion)
    worker.start()
    deadline = time.monotonic() + 2.0
    while hardware._motor_process is None and time.monotonic() < deadline:
        time.sleep(0.01)
    started = time.monotonic()
    result = hardware.motor_command("stop")
    worker.join(timeout=2.0)
    hardware.authorized = False
    assert result["ok"] is True
    assert not worker.is_alive()
    assert time.monotonic() - started < 2.0
    assert errors


def test_simulation_plant_exercises_control_without_ros_output():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    runtime.ros.publish_joint_control_heartbeat = lambda: pytest.fail(
        "simulation must never publish the ROS control heartbeat")
    assert runtime.connect() is False
    initial = runtime.snapshot()
    assert initial["simulation"] is True
    assert initial["fresh"]["joints"] is True
    assert all(key in initial["tcp"] for key in ("x", "y", "z", "rx", "ry", "rz"))
    lease = runtime.sessions.acquire("simulation test")
    runtime.sessions.bind_websocket(lease["session_id"], "socket-sim")
    result = runtime.execute_joints(lease["session_id"], [0.2, 0, 0, 0, 0, 0, 0])
    assert result["simulation"] is True
    time.sleep(0.05)
    moved = runtime.snapshot()
    assert moved["joints"]["joint1"] > 0
    assert runtime.publish_guarded_heartbeat() is False
    z_start = runtime.command("start_z_force_teleop", lease["session_id"])
    assert z_start["simulation"] is True
    z_state = runtime.snapshot()
    assert z_state["operator_state"] == "FORCE_CONTROL"
    assert z_state["control_launch_mode"] == "z_force_teleop"
    assert z_state["control_mode"] == "POSITION_MODE"
    assert z_state["haptic_enabled"] is True
    assert z_state["haptic_force_fresh"] is True
    haptic = z_state["haptic_command"]
    assert haptic["x"] == 0.0 and haptic["z"] == 0.0
    assert 0.0 < abs(haptic["y"]) <= 0.5
    runtime.command("stop_z_force_teleop", lease["session_id"])
    stopped_z = runtime.snapshot()
    assert stopped_z["control_launch_mode"] is None
    assert stopped_z["haptic_enabled"] is False
    assert stopped_z["haptic_command"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    runtime.command("start_z_force_teleop", lease["session_id"])
    runtime.command("emergency_stop", lease["session_id"])
    emergency = runtime.snapshot()
    assert emergency["operator_state"] == "SAFE_STOP"
    assert emergency["control_launch_mode"] is None
    assert emergency["haptic_enabled"] is False
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.command("pump_on", lease["session_id"])
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.command("motor", lease["session_id"], {"action": "forward"})
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.execute_joints(lease["session_id"], [0] * 7)
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.command("set_voltage", lease["session_id"], {"voltage": 24})
    runtime.command("recover", lease["session_id"])
    selected = runtime.command("select_tool", lease["session_id"], {"tool": "Tool2"})
    assert selected["command"] == "select_tool"
    for qr_command in ("detect_marker", "compute_targets", "preview_tool",
                       "plan_tool", "execute_tool"):
        result = runtime.command(qr_command, lease["session_id"])
        assert result["simulation"] is True
    assert runtime.snapshot()["operator_state"] == "TOOL_HOLD"
    runtime.command("cancel_tool", lease["session_id"])
    pump = runtime.command("pump_on", lease["session_id"])
    assert pump["simulation"] is True
    assert runtime.snapshot()["sampling_hardware"]["sensors"]["pump_on"] is True
    motor = runtime.command("motor", lease["session_id"], {
        "action": "forward", "speed": 0.5, "duration": 0.2})
    assert motor["simulation"] is True
    runtime.command("pump_off", lease["session_id"])
    assert runtime.snapshot()["operator_state"] == "IDLE"
    assert runtime.simulation_jpeg().startswith(b"\xff\xd8")
    runtime.close()


def test_five_tool_configuration_persists_and_simulation_executes_unverified(tmp_path):
    package = Path(__file__).resolve().parents[1]
    tools_path = tmp_path / "tools.yaml"
    shutil.copyfile(str(package / "config/tools.yaml"), str(tools_path))
    config = load_web_configuration(str(package / "config/gui.yaml"), str(tools_path))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    lease = runtime.sessions.acquire("five-tool-test")
    runtime.sessions.bind_websocket(lease["session_id"], "socket-tools")
    saved = runtime.save_tool_configuration(3, {
        "x": 0.31, "y": -0.12, "z": 0.42,
        "rx": 3.1, "ry": 0.02, "rz": -1.4,
        "lift_height": 0.075,
    }, lease["session_id"])
    assert saved["slot_id"] == 3
    assert saved["x"] == pytest.approx(0.31)
    reloaded = load_web_configuration(str(package / "config/gui.yaml"), str(tools_path))
    persisted = reloaded["tools"][2]["configured_pick"]
    assert persisted["position_m"] == pytest.approx([0.31, -0.12, 0.42])
    assert persisted["lift_height_m"] == pytest.approx(0.075)
    dry_run = runtime.dry_run_configured_tool(3, lease["session_id"])
    assert dry_run["dry_run"] is True
    result = runtime.execute_configured_tool(3, lease["session_id"])
    assert result["executed"] is True
    assert result["grasp_confirmed"] is False
    status = runtime.snapshot()["configured_pick_status"]
    assert status["status"] == "COMPLETED_UNVERIFIED"
    assert status["step"] == 6
    runtime.close()


def test_five_tool_configuration_rejects_non_finite_or_nonpositive_lift(tmp_path):
    package = Path(__file__).resolve().parents[1]
    tools_path = tmp_path / "tools.yaml"
    shutil.copyfile(str(package / "config/tools.yaml"), str(tools_path))
    config = load_web_configuration(str(package / "config/gui.yaml"), str(tools_path))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    lease = runtime.sessions.acquire("five-tool-validation")
    runtime.sessions.bind_websocket(lease["session_id"], "socket-validation")
    payload = {"x": 0.1, "y": 0.2, "z": 0.3,
               "rx": 0.0, "ry": 0.0, "rz": 0.0, "lift_height": 0.0}
    with pytest.raises(ValueError, match="Lift Height"):
        runtime.save_tool_configuration(1, payload, lease["session_id"])
    payload["lift_height"] = 0.1
    payload["x"] = float("nan")
    with pytest.raises(ValueError, match="NaN"):
        runtime.save_tool_configuration(1, payload, lease["session_id"])
    runtime.close()
