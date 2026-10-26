#!/usr/bin/env python3
"""Web transport safety tests.  No ROS publisher or real command is created."""
import os
import subprocess
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
from rm75_sampling_gui.web.backend.tool5_drill import Tool5DrillWorkflow


class _AutomaticPathTasks:
    def __init__(self, statuses, fail_start=None):
        self.statuses = {name: dict(value) for name, value in statuses.items()}
        self.fail_start = fail_start
        self.calls = []

    def subsystem_status(self):
        return {name: dict(value) for name, value in self.statuses.items()}

    def stop_subsystem(self, name):
        self.calls.append(("stop", name))
        self.statuses[name] = {"running": False, "owned": False}
        return {"stopped": True}

    def set_control_path(self, path):
        self.calls.append(("path", path))
        return {"control_path": path, "execution": {}}

    def start_subsystem(self, name):
        self.calls.append(("start", name))
        if name == self.fail_start:
            raise RuntimeError("injected start failure")
        self.statuses[name] = {"running": True, "owned": True}
        return {"running": True}


def _automatic_path_runtime(tasks):
    runtime = WebRuntime.__new__(WebRuntime)
    runtime.unified_control_authorized = True
    runtime.simulation = False
    runtime.active_control_path = "joint_control"
    runtime.tasks = tasks
    runtime.tool5_drill = type("Drill", (), {
        "snapshot": lambda self: {"state": "IDLE"}
    })()
    runtime._control_path_lock = threading.RLock()
    runtime._subsystem_lock = threading.Lock()
    runtime._subsystem_cache = {}
    runtime._subsystem_checked = 0.0
    return runtime


def test_automatic_path_switch_stops_and_restarts_in_dependency_order():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": True},
        "camera": {"running": False, "owned": False},
        "qr": {"running": False, "owned": False},
    })
    runtime = _automatic_path_runtime(tasks)

    result = runtime._activate_control_path(
        "qr_pickup", ("base", "camera", "qr")
    )

    assert result["changed"] is True
    assert runtime.active_control_path == "qr_pickup"
    assert tasks.calls == [
        ("stop", "base"), ("path", "qr_pickup"),
        ("start", "base"), ("start", "camera"), ("start", "qr"),
    ]


def test_pose_pickup_starts_no_camera_or_marker_detector():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": True},
        "camera": {"running": True, "owned": True},
        "qr": {"running": True, "owned": True},
        "pose": {"running": False, "owned": False},
    })
    runtime = _automatic_path_runtime(tasks)

    result = runtime._activate_control_path(
        "pose_pickup", ("base", "pose")
    )

    assert result["changed"] is True
    assert runtime.active_control_path == "pose_pickup"
    assert tasks.calls == [
        ("stop", "qr"), ("stop", "camera"), ("stop", "base"),
        ("path", "pose_pickup"), ("start", "base"), ("start", "pose"),
    ]


def test_configured_pickup_declares_no_camera_or_qr_dependency():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(
        str(package / "config/gui.yaml"), str(package / "config/tools.yaml")
    )
    pose = config["system_launches"]["pose"]
    launch = (package.parent / "rm75_qr_task/launch/configured_pickup.launch").read_text()

    assert pose["requires"] == ["/rm_driver", "/move_group"]
    assert pose["expected_nodes"] == ["/rm75_five_target_server"]
    assert 'type="five_target_server.py"' in launch
    assert "marker_node.py" not in launch
    assert "publish_handeye_tf" not in launch


def test_automatic_path_switch_rejects_external_subsystems_without_stopping():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": False},
        "camera": {"running": False, "owned": False},
        "qr": {"running": False, "owned": False},
    })
    runtime = _automatic_path_runtime(tasks)

    with pytest.raises(RuntimeError, match="非网页管理"):
        runtime._activate_control_path("z_force", ("base",))

    assert tasks.calls == []
    assert runtime.active_control_path == "joint_control"


def test_automatic_path_switch_failure_cleans_up_and_falls_back_to_observe():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": True},
        "camera": {"running": False, "owned": False},
        "qr": {"running": False, "owned": False},
    }, fail_start="camera")
    runtime = _automatic_path_runtime(tasks)

    with pytest.raises(RuntimeError, match="已停止新控制链路"):
        runtime._activate_control_path(
            "qr_pickup", ("base", "camera", "qr")
        )

    assert runtime.active_control_path == "observe"
    assert tasks.calls == [
        ("stop", "base"), ("path", "qr_pickup"),
        ("start", "base"), ("start", "camera"),
        ("stop", "base"), ("path", "observe"),
    ]


def test_fixed_start_command_ignores_inherited_mode_and_authorization_flags():
    script = Path(__file__).resolve().parents[1] / "scripts/start_sampling_web.sh"
    environment = os.environ.copy()
    environment.update({
        "RM75_CONTROL_PATH": "observe",
        "EXECUTION_ENABLED": "false",
        "QR_EXECUTION_AUTHORIZED": "true",
        "JOINT_CONTROL_EXECUTION_AUTHORIZED": "false",
        "TRAJECTORY_EXECUTION_AUTHORIZED": "true",
        "TOUCH_Z_FORCE_AUTHORIZED": "true",
        "TOOL5_DRILL_AUTHORIZED": "true",
        "SAMPLING_HARDWARE_AUTHORIZED": "false",
        "SUPERVISED_COMMISSIONING_AUTHORIZED": "false",
    })
    result = subprocess.run(
        [str(script), "--dry-run"], check=True, capture_output=True,
        text=True, env=environment,
    )
    assert "Mode=unified" in result.stdout
    assert "execution=true" in result.stdout
    assert "joint=true" in result.stdout
    assert "trajectory=false" in result.stdout
    assert "z_force=false" in result.stdout
    assert "tool5_drill=false" in result.stdout
    assert "supervised=true" in result.stdout
    assert "sampling_hardware=true" in result.stdout


def test_removed_confirmation_arguments_are_rejected():
    script = Path(__file__).resolve().parents[1] / "scripts/start_sampling_web.sh"
    for old_argument in ("--confirm-real-robot", "--confirm-tool5-drill"):
        result = subprocess.run(
            [str(script), old_argument, "--dry-run"], capture_output=True,
            text=True,
        )
        assert result.returncode == 2
        assert "unknown argument" in result.stderr


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
    def set_control_path(self, session_id, path):
        self.sessions.require(session_id); self.commands.append("path:" + path)
        return {"control_path": path}


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


def test_control_path_switch_requires_live_exclusive_lease():
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime)) as client:
        lease = client.post("/api/v1/control/session", json={"client": "test"}).json()
        headers = {"X-RM75-Control-Session": lease["session_id"]}
        assert client.put("/api/v1/control/path", json={"path": "z_force"},
                          headers=headers).status_code == 409
        with client.websocket_connect("/api/v1/ws") as socket:
            socket.send_json({"type": "heartbeat", "session_id": lease["session_id"]})
            messages = [socket.receive_json(), socket.receive_json()]
            assert any(item["type"] == "heartbeat_ack" for item in messages)
            response = client.put("/api/v1/control/path",
                                  json={"path": "z_force"}, headers=headers)
            assert response.status_code == 200
            assert response.json()["control_path"] == "z_force"
            assert runtime.commands == ["path:z_force"]


def test_spa_entry_is_not_cached_across_frontend_rebuilds(tmp_path):
    frontend = tmp_path / "dist"
    frontend.mkdir()
    (frontend / "index.html").write_text(
        "<!doctype html><title>RM75 test</title>", encoding="utf-8")
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime, str(frontend))) as client:
        for path in ("/", "/robot", "/index.html"):
            response = client.get(path)
            assert response.status_code == 200
            assert "no-store" in response.headers["cache-control"]


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


def test_motor_command_enforces_validated_speed_mode_limits():
    hardware = SamplingHardware({}, simulation=True)
    snapshot = hardware.snapshot()["motor"]
    assert snapshot["control_mode"] == "VEL"
    assert snapshot["limits_verified"] is True
    assert snapshot["max_speed"] == pytest.approx(1.0)
    assert snapshot["max_current"] == pytest.approx(4.0)
    assert snapshot["max_torque"] == pytest.approx(15.0)
    with pytest.raises(ValueError, match="1.0 rad/s"):
        hardware.motor_command("forward", speed=1.01, duration=1.0)
    assert hardware.motor_command("dig", speed=0.5, duration=60.0)["ok"] is True
    with pytest.raises(ValueError, match="60秒"):
        hardware.motor_command("dig", speed=0.5, duration=60.1)
    with pytest.raises(ValueError, match="不允许"):
        hardware.motor_command("dig-hard", speed=1.0, duration=1.0)


def _drill_telemetry():
    return {
        "health": {"driver": True},
        "fresh": {"joints": True, "wrench": True, "tcp": True},
        "wrench": {"Fz": -10.0, "Tx": 0.1, "Ty": 0.2, "Tz": 0.1},
        "tcp": {"z": 0.5}, "stop_reason": "", "tool5_drill_state": "DRILLING",
    }


def test_tool5_depth_reached_keeps_motor_running_then_stops_without_retract():
    hardware = SamplingHardware({}, simulation=True)
    raw = _drill_telemetry()
    calls = []
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05}},
        hardware, lambda: raw, lambda: calls.append("prepare"),
        lambda depth: calls.append(("engage", depth)),
        lambda: calls.append("hold"), lambda: calls.append("safe_stop"),
        lambda: True, simulation=True)
    workflow.start(0.005)
    time.sleep(0.12)
    reached = workflow.snapshot()
    assert reached["state"] == "DEPTH_REACHED"
    assert reached["feed_state"] == "HOLDING"
    assert reached["motor_running"] is True
    completed = workflow.stop_rotation()
    assert completed["state"] == "COMPLETED"
    assert completed["motor_running"] is False
    assert calls == ["prepare", ("engage", 0.005), "hold"]


@pytest.mark.parametrize("mutation,reason", [
    (lambda raw: raw["wrench"].update(Fz=61.0), "轴向力"),
    (lambda raw: raw["wrench"].update(Tx=15.1), "力矩"),
    (lambda raw: raw["fresh"].update(wrench=False), "反馈超时"),
    (lambda raw: raw.update(stop_reason="joint tracking error exceeded"), "tracking"),
])
def test_tool5_runtime_faults_safe_stop_motor_and_feed(mutation, reason):
    hardware = SamplingHardware({}, simulation=True)
    raw = _drill_telemetry()
    calls = []
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05}},
        hardware, lambda: raw, lambda: None, lambda depth: None,
        lambda: calls.append("hold"), lambda: calls.append("safe_stop"),
        lambda: True, simulation=True)
    workflow.start(0.02)
    mutation(raw)
    workflow.poll()
    status = workflow.snapshot()
    assert status["state"] == "SAFE_STOP"
    assert reason in status["fault_reason"]
    assert calls == ["safe_stop"]


def test_tool5_motor_stop_failure_is_safe_stop():
    class BrokenStopHardware(SamplingHardware):
        def motor_command(self, name, speed=0.5, duration=3.0):
            if name == "stop":
                raise RuntimeError("stop ACK missing")
            return super().motor_command(name, speed, duration)
    hardware = BrokenStopHardware({}, simulation=True)
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05}},
        hardware, _drill_telemetry, lambda: None, lambda depth: None,
        lambda: None, lambda: None, lambda: True, simulation=True)
    workflow.start(0.02)
    with pytest.raises(RuntimeError, match="停止失败"):
        workflow.stop_rotation()
    status = workflow.snapshot()
    assert status["state"] == "SAFE_STOP"
    assert "stop ACK missing" in status["fault_reason"]


def test_tool5_continuous_motor_has_no_fixed_time_limit_and_remains_stoppable():
    hardware = SamplingHardware({}, simulation=True)
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05}},
        hardware, _drill_telemetry, lambda: None, lambda depth: None,
        lambda: None, lambda: None, lambda: True, simulation=True)
    workflow.start(0.002)
    time.sleep(0.08)
    status = workflow.snapshot()
    assert status["state"] == "DEPTH_REACHED"
    assert status["motor_running"] is True
    assert workflow.stop_rotation()["state"] == "COMPLETED"


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
    with pytest.raises(ValueError, match="Tool5取土状态机"):
        runtime.command("motor", lease["session_id"], {"action": "dig_continuous"})
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.execute_joints(lease["session_id"], [0] * 7)
    with pytest.raises(RuntimeError, match="SAFE_STOP"):
        runtime.command("set_voltage", lease["session_id"], {"voltage": 24})
    runtime.command("recover", lease["session_id"])
    selected = runtime.command("select_tool", lease["session_id"], {"tool": "Tool2"})
    assert selected["command"] == "select_tool"
    with pytest.raises(RuntimeError, match="先计算"):
        runtime.command("plan_tool", lease["session_id"])
    for qr_command in ("detect_marker", "compute_targets", "preview_tool",
                       "plan_tool", "execute_tool"):
        result = runtime.command(qr_command, lease["session_id"])
        assert result["simulation"] is True
    qr_state = runtime.snapshot()
    assert qr_state["operator_state"] == "TOOL_HOLD"
    assert qr_state["five_target_status"]["targets_valid"] is True
    assert qr_state["five_target_status"]["holding_slot"] == 2
    assert qr_state["five_target_status"]["grasp_confirmed"] is False
    assert len(qr_state["five_targets"]["targets"]) == 5
    assert qr_state["tool_voltage"] == 24
    runtime.command("cancel_tool", lease["session_id"])
    cancelled = runtime.snapshot()
    assert cancelled["operator_state"] == "TOOL_HOLD"
    assert cancelled["five_target_status"]["holding_slot"] == 2
    runtime.command("return_tool", lease["session_id"])
    returned = runtime.snapshot()
    assert returned["operator_state"] == "IDLE"
    assert returned["five_target_status"]["holding_slot"] is None
    assert returned["tool_voltage"] == 0
    with pytest.raises(RuntimeError, match="当前持有"):
        runtime.command("return_tool", lease["session_id"])
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


def test_tool_configuration_rejects_millimetres_submitted_as_metres():
    with pytest.raises(ValueError, match="按毫米输入"):
        WebRuntime._validated_tool_payload({
            "x": -105.208, "y": 263.142, "z": 152.255,
            "rx": -3.088, "ry": -0.044, "rz": 1.828,
            "lift_height": 0.1,
        })


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
