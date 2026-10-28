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
import rm75_sampling_gui.web.backend.app as web_app
from rm75_sampling_gui.web.backend.limit_recovery import run_limit_recovery
from rm75_sampling_gui.web.backend.services import WebRuntime, load_web_configuration
from rm75_sampling_gui.web.backend.session import ControlSessionManager, SessionError
from rm75_sampling_gui.web.backend.hardware import SamplingHardware
from rm75_sampling_gui.web.backend.tool5_drill import Tool5DrillWorkflow
from rm75_sampling_gui.state.state_machine import RobotState, StateMachine


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


def test_motor_only_tool5_allows_switch_to_independent_z_force_path():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": True},
    })
    runtime = _automatic_path_runtime(tasks)
    runtime.tool5_drill = type("MotorOnlyDrill", (), {
        "motor_only": True,
        "snapshot": lambda self: {"state": "ROTATING"},
    })()

    result = runtime._activate_control_path("z_force", ("base",))

    assert result["changed"] is True
    assert runtime.active_control_path == "z_force"
    assert tasks.calls == [
        ("stop", "base"), ("path", "z_force"), ("start", "base"),
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


def test_automatic_path_switch_failure_restores_previous_path_and_systems():
    tasks = _AutomaticPathTasks({
        "base": {"running": True, "owned": True},
        "camera": {"running": False, "owned": False},
        "qr": {"running": False, "owned": False},
    }, fail_start="camera")
    runtime = _automatic_path_runtime(tasks)

    with pytest.raises(RuntimeError, match="已恢复原控制链路joint_control"):
        runtime._activate_control_path(
            "qr_pickup", ("base", "camera", "qr")
        )

    assert runtime.active_control_path == "joint_control"
    assert tasks.calls == [
        ("stop", "base"), ("path", "qr_pickup"),
        ("start", "base"), ("start", "camera"),
        ("stop", "base"), ("path", "joint_control"),
        ("start", "base"),
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
    def require_session(self, session_id): return self.sessions.require(session_id)
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
    def cartesian_jog_status(self): return {"state": "IDLE"}
    def start_cartesian_jog(self, session_id, direction):
        self.sessions.require(session_id); self.commands.append("jog:" + direction)
        return {"ok": True, "direction": direction}
    def stop_cartesian_jog(self, session_id):
        self.sessions.require(session_id); self.commands.append("jog:stop")
        return {"ok": True, "stopped": True}


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


def test_cartesian_jog_api_is_lease_bound_and_direction_allowlisted():
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime)) as client:
        assert client.get("/api/v1/cartesian/jog/status").status_code == 200
        lease = client.post("/api/v1/control/session", json={"client": "jog"}).json()
        headers = {"X-RM75-Control-Session": lease["session_id"]}
        assert client.post("/api/v1/cartesian/jog/start",
                           json={"direction": "x_plus"}, headers=headers).status_code == 409
        with client.websocket_connect("/api/v1/ws") as socket:
            socket.send_json({"type": "heartbeat", "session_id": lease["session_id"]})
            messages = [socket.receive_json(), socket.receive_json()]
            assert any(item["type"] == "heartbeat_ack" for item in messages)
            assert client.post("/api/v1/cartesian/jog/start",
                               json={"direction": "/rm_driver/JointPos"},
                               headers=headers).status_code == 422
            assert client.post("/api/v1/cartesian/jog/start",
                               json={"direction": "rz_minus"},
                               headers=headers).status_code == 200
            assert client.post("/api/v1/cartesian/jog/stop",
                               headers=headers).status_code == 200
            assert runtime.commands == ["jog:rz_minus", "jog:stop"]


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


def test_trajectory_api_crud_twenty_points_generation_id_execution_and_stop(tmp_path):
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    config["trajectory_library"]["directory"] = str(tmp_path / "trajectories")
    runtime = WebRuntime(config, config["tools"], simulation=True)
    cancel = threading.Event()
    started = threading.Event()

    def executor(points, callback):
        callback({"current_point": 1, "total_points": len(points),
                  "progress": 0.25, "max_error_rad": 0.002})
        started.set()
        cancel.wait(2.0)
        return {"cancelled": True, "status": "PREEMPTED"}

    runtime.trajectories.executor = executor
    runtime.trajectories.stop_executor = (
        lambda: cancel.set() or {"cancel_requested": True, "simulation": True}
    )
    with TestClient(create_app(runtime)) as client:
        assert client.get("/api/v1/trajectories").status_code == 200
        assert client.post("/api/v1/trajectories", json={"name": "无租约"}).status_code == 401

        lease = runtime.sessions.acquire("trajectory api")
        runtime.sessions.bind_websocket(lease["session_id"], "trajectory-test-socket")
        headers = {"X-RM75-Control-Session": lease["session_id"]}
        created = client.post("/api/v1/trajectories", headers=headers, json={
            "name": "采样撤离路径01", "trajectory_id": "sampling_retreat_01",
        })
        assert created.status_code == 200
        assert created.json()["trajectory_id"] == "sampling_retreat_01"

        for _ in range(20):
            response = client.post(
                "/api/v1/trajectories/sampling_retreat_01/waypoints",
                headers=headers,
            )
            assert response.status_code == 200
        document = client.get(
            "/api/v1/trajectories/sampling_retreat_01"
        ).json()
        assert len(document["waypoints"]) == 20
        assert document["trajectory_status"]["state"] == "STALE"

        generated = client.post(
            "/api/v1/trajectories/sampling_retreat_01/generate", headers=headers
        )
        assert generated.status_code == 200
        assert generated.json()["trajectory_status"]["state"] == "READY"
        assert len(generated.json()["trajectory"]) > 20

        updated = client.put(
            "/api/v1/trajectories/sampling_retreat_01", headers=headers, json={
                "name": "采样撤离路径01（更新）",
                "trajectory_config": {"speed_scale": 0.3, "acceleration_scale": 0.25},
            },
        )
        assert updated.status_code == 200
        assert updated.json()["name"].endswith("（更新）")
        assert updated.json()["trajectory_status"]["state"] == "STALE"
        assert client.post(
            "/api/v1/trajectories/sampling_retreat_01/generate", headers=headers
        ).status_code == 200

        waypoint_id = document["waypoints"][1]["waypoint_id"]
        moved = client.post(
            "/api/v1/trajectories/sampling_retreat_01/waypoints/{}/move".format(
                waypoint_id),
            headers=headers, json={"direction": "up"},
        )
        assert moved.status_code == 200
        assert moved.json()["trajectory_status"]["state"] == "STALE"
        regenerated = client.post(
            "/api/v1/trajectories/sampling_retreat_01/generate", headers=headers
        )
        assert regenerated.status_code == 200

        execute = client.post(
            "/api/v1/trajectories/sampling_retreat_01/execute", headers=headers
        )
        assert execute.status_code == 200
        assert execute.json()["trajectory_id"] == "sampling_retreat_01"
        assert started.wait(1.0)
        status = client.get("/api/v1/trajectories/execution/status").json()
        assert status["state"] == "EXECUTING"
        assert status["progress"] == pytest.approx(0.25)
        stopped = client.post(
            "/api/v1/trajectories/execution/stop", headers=headers
        )
        assert stopped.status_code == 200
        assert stopped.json()["cancel_requested"] is True

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            status = client.get("/api/v1/trajectories/execution/status").json()
            if status["state"] != "EXECUTING":
                break
            time.sleep(0.01)
        assert status["state"] == "CANCELLED"

        listing = client.get("/api/v1/trajectories").json()["trajectories"]
        assert listing[0]["trajectory_id"] == "sampling_retreat_01"
        assert client.delete(
            "/api/v1/trajectories/sampling_retreat_01", headers=headers
        ).status_code == 200


def test_simulation_saved_trajectory_executor_tracks_feedback_to_completion(tmp_path):
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    config["trajectory_library"]["directory"] = str(tmp_path / "trajectories")
    runtime = WebRuntime(config, config["tools"], simulation=True)
    try:
        lease = runtime.sessions.acquire("simulation trajectory")
        runtime.sessions.bind_websocket(lease["session_id"], "simulation-trajectory-socket")
        document = runtime.trajectories.create(
            "simulation execution", trajectory_id="simulation_execution"
        )
        document = runtime.trajectories.record_current_waypoint(document["trajectory_id"])
        document = runtime.trajectories.record_current_waypoint(document["trajectory_id"])
        start = document["waypoints"][0]["joint_positions"]
        target = list(start)
        target[0] += 0.08
        document["waypoints"][1]["joint_positions"] = target
        document["trajectory"] = [
            {"positions": start, "velocities": [0.0] * 7,
             "accelerations": [0.0] * 7, "time_from_start": 0.0},
            {"positions": [(a + b) / 2.0 for a, b in zip(start, target)],
             "velocities": [0.2] + [0.0] * 6,
             "accelerations": [0.0] * 7, "time_from_start": 0.3},
            {"positions": target, "velocities": [0.0] * 7,
             "accelerations": [0.0] * 7, "time_from_start": 0.6},
        ]
        document["trajectory_status"].update({
            "state": "READY", "message": "ready", "waypoint_indices": [0, 2],
            "trajectory_point_count": 3, "duration_s": 0.6,
        })
        runtime.trajectories.save(document)

        runtime.trajectories.execute("simulation_execution")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            status = runtime.trajectories.execution_status()
            if status["state"] != "EXECUTING":
                break
            time.sleep(0.02)

        assert status["state"] == "COMPLETED"
        assert status["progress"] == pytest.approx(1.0)
        actual = runtime.snapshot()["joints"]
        assert actual["joint1"] == pytest.approx(target[0], abs=0.015)
        assert runtime.snapshot()["operator_state"] == "IDLE"
    finally:
        runtime.close()


def test_simulation_saved_trajectory_stops_when_control_lease_is_lost(tmp_path):
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    config["trajectory_library"]["directory"] = str(tmp_path / "trajectories")
    runtime = WebRuntime(config, config["tools"], simulation=True)
    try:
        lease = runtime.sessions.acquire("lease loss trajectory")
        runtime.sessions.bind_websocket(lease["session_id"], "lease-loss-socket")
        document = runtime.trajectories.create("lease loss", trajectory_id="lease_loss")
        document = runtime.trajectories.record_current_waypoint("lease_loss")
        document = runtime.trajectories.record_current_waypoint("lease_loss")
        start = document["waypoints"][0]["joint_positions"]
        target = list(start)
        target[0] += 0.15
        document["waypoints"][1]["joint_positions"] = target
        document["trajectory"] = [
            {"positions": start, "velocities": [0.0] * 7,
             "accelerations": [0.0] * 7, "time_from_start": 0.0},
            {"positions": target, "velocities": [0.0] * 7,
             "accelerations": [0.0] * 7, "time_from_start": 2.0},
        ]
        document["trajectory_status"].update({
            "state": "READY", "message": "ready", "waypoint_indices": [0, 1],
            "trajectory_point_count": 2, "duration_s": 2.0,
        })
        runtime.trajectories.save(document)
        runtime.trajectories.execute("lease_loss")
        time.sleep(0.05)

        runtime.sessions.release(lease["session_id"], "lease-loss-socket")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            status = runtime.trajectories.execution_status()
            if status["state"] != "EXECUTING":
                break
            time.sleep(0.02)

        assert status["state"] == "CANCELLED"
        assert runtime.snapshot()["operator_state"] == "IDLE"
    finally:
        runtime.close()


def test_limit_recovery_runner_accepts_diagnostic_exit_code_two(tmp_path, monkeypatch):
    script = tmp_path / "limit_tool.py"
    script.write_text(
        "import sys\n"
        "print('需要恢复：J2')\n"
        "raise SystemExit(0 if '--recover' in sys.argv else 2)\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("RM75_LIMIT_RECOVERY_SCRIPT", str(script))
    monkeypatch.setenv("RM75_ARM_IP", "10.20.30.40")
    monkeypatch.setenv("RM75_ARM_PORT", "9000")

    diagnosis = run_limit_recovery(False)
    recovery = run_limit_recovery(True)

    assert diagnosis["needs_recovery"] is True
    assert diagnosis["exit_code"] == 2
    assert diagnosis["output"] == "需要恢复：J2"
    assert diagnosis["target"] == "10.20.30.40:9000"
    assert recovery["recovered"] is True
    assert recovery["command"].endswith("--recover --yes")


def test_limit_recovery_api_is_read_only_without_lease_and_protects_motion(monkeypatch):
    calls = []

    def fake_runner(recover=False):
        calls.append(recover)
        return {"ok": True, "needs_recovery": False, "recovered": recover,
                "exit_code": 0, "command": "tool", "output": "ok"}

    monkeypatch.setattr(web_app, "run_limit_recovery", fake_runner)
    runtime = _FakeRuntime()
    with TestClient(create_app(runtime)) as client:
        config = client.get("/api/v1/limit-recovery/config")
        assert config.status_code == 200
        assert config.json()["target"]
        assert client.get("/api/v1/limit-recovery").status_code == 200
        assert client.post("/api/v1/limit-recovery").status_code == 401

        lease = runtime.sessions.acquire("limit test")
        runtime.sessions.bind_websocket(lease["session_id"], "test-socket")
        headers = {"X-RM75-Control-Session": lease["session_id"]}
        response = client.post("/api/v1/limit-recovery", headers=headers)
        assert response.status_code == 200
        assert response.json()["recovered"] is True

    assert calls == [False, True]


def test_limit_recovery_motion_is_disabled_in_simulation(monkeypatch):
    monkeypatch.setattr(web_app, "run_limit_recovery", lambda recover=False: {})
    runtime = _FakeRuntime()
    runtime.simulation = True
    with TestClient(create_app(runtime)) as client:
        lease = runtime.sessions.acquire("limit test")
        runtime.sessions.bind_websocket(lease["session_id"], "test-socket")
        response = client.post(
            "/api/v1/limit-recovery",
            headers={"X-RM75-Control-Session": lease["session_id"]},
        )
        assert response.status_code == 403


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


class _Tool5HardwareDouble:
    def __init__(self):
        self.commands = []
        self.keepalive_calls = 0
        self.motor = {"online": True, "velocity": 0.0}

    def snapshot(self):
        return {"motor": dict(self.motor)}

    def motor_command(self, name, speed=0.5, duration=3.0):
        del speed, duration
        self.commands.append(name)
        if name == "status":
            self.motor["online"] = True
        if name == "stop":
            self.motor["velocity"] = 0.0
        return {"ok": True, "command": name}

    def motor_keepalive(self):
        self.keepalive_calls += 1


class _RecordingSimulationHardware(SamplingHardware):
    def __init__(self):
        super().__init__({}, simulation=True)
        self.commands = []

    def motor_command(self, name, speed=0.5, duration=3.0, on_started=None):
        self.commands.append(name)
        return super().motor_command(name, speed, duration, on_started)


class _BlockingTool5Hardware(_Tool5HardwareDouble):
    def __init__(self):
        super().__init__()
        self.before_motor_launch = threading.Event()
        self.allow_motor_launch = threading.Event()
        self.release_motor = threading.Event()

    def motor_command(self, name, speed=0.5, duration=3.0, on_started=None):
        del speed, duration
        if name != "dig_continuous":
            return super().motor_command(name)
        self.before_motor_launch.set()
        assert self.allow_motor_launch.wait(timeout=1.0)
        self.commands.append(name)
        self.motor["velocity"] = 0.5
        if on_started is not None:
            on_started()
        self.release_motor.wait(timeout=1.0)
        self.motor["velocity"] = 0.0
        return {"ok": True, "command": name}


def _starting_tool5(raw, hardware, lease_active=lambda: True):
    arm_stops = []
    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.05}, hardware, lambda: raw,
        lambda: None, lambda depth: None, lambda: None,
        lambda: arm_stops.append("safe_stop"), lease_active,
        simulation=False)
    with workflow._lock:
        workflow._status.update({"state": "STARTING", "feed_state": "STARTING"})
    return workflow, arm_stops


def test_tool5_starting_snapshot_does_not_stop_before_outputs_begin():
    raw = _drill_telemetry()
    raw["fresh"]["joints"] = False
    hardware = _Tool5HardwareDouble()
    workflow, arm_stops = _starting_tool5(raw, hardware)

    status = workflow.snapshot()

    assert status["state"] == "STARTING"
    assert hardware.commands == []
    assert arm_stops == []


def test_tool5_start_waits_for_fresh_robot_feedback():
    stale = _drill_telemetry()
    stale["fresh"] = {"joints": False, "wrench": False, "tcp": False}
    fresh = _drill_telemetry()
    snapshots = [stale, stale, fresh]
    telemetry_calls = []

    def telemetry():
        telemetry_calls.append(True)
        return snapshots.pop(0) if snapshots else fresh

    hardware = _RecordingSimulationHardware()
    actions = []
    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.2}, hardware, telemetry,
        lambda: actions.append("prepare"),
        lambda depth: actions.append(("engage", depth)),
        lambda: actions.append("hold"),
        lambda: actions.append("safe_stop"), lambda: True,
        simulation=False)

    status = workflow.start(0.001)

    assert status["state"] == "DRILLING"
    assert len(telemetry_calls) >= 3
    assert actions == ["prepare", ("engage", 0.001)]
    workflow.emergency_stop(stop_arm=False)


def test_tool5_motor_only_rotates_without_starting_or_holding_arm():
    hardware = SamplingHardware({}, simulation=True)
    arm_actions = []
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"motor_only": True}}, hardware,
        lambda: pytest.fail("motor-only rotation must not require arm telemetry"),
        lambda: arm_actions.append("prepare"),
        lambda depth: arm_actions.append(("engage", depth)),
        lambda: arm_actions.append("hold"),
        lambda: arm_actions.append("safe_stop"), lambda: True,
        simulation=True)

    rotating = workflow.start()

    assert rotating["state"] == "ROTATING"
    assert rotating["motor_running"] is True
    assert rotating["feed_state"] == "INDEPENDENT"
    assert arm_actions == []
    stopped = workflow.stop_rotation()
    assert stopped["state"] == "COMPLETED"
    assert stopped["motor_running"] is False
    assert arm_actions == []


def test_motor_only_rotation_and_independent_force_control_run_together():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    try:
        lease = runtime.sessions.acquire("motor plus force")
        runtime.sessions.bind_websocket(lease["session_id"], "motor-force-socket")

        rotation = runtime.command(
            "start_tool5_rotation", lease["session_id"])
        force = runtime.command("enter_force_control", lease["session_id"])
        simultaneous = runtime.snapshot()

        assert rotation["state"] == "ROTATING"
        assert force["simulation"] is True
        assert simultaneous["tool5_drill"]["state"] == "ROTATING"
        assert simultaneous["tool5_drill"]["motor_running"] is True
        assert simultaneous["control_launch_mode"] == "force_control"
        runtime.command("stop_sampling", lease["session_id"])
        assert runtime.command(
            "stop_tool5_rotation", lease["session_id"])["state"] == "COMPLETED"
    finally:
        runtime.close()


def test_tool5_engages_z_feed_before_motor_and_keeps_both_active():
    events = []

    class OrderedHardware(SamplingHardware):
        def motor_command(self, name, speed=0.5, duration=3.0, on_started=None):
            if name == "dig_continuous":
                events.append("motor")
            return super().motor_command(name, speed, duration, on_started)

    hardware = OrderedHardware({}, simulation=True)
    workflow = Tool5DrillWorkflow(
        {}, hardware, _drill_telemetry, lambda: events.append("prepare"),
        lambda depth: events.append(("z_feed", depth)), lambda: None,
        lambda: events.append("safe_stop"), lambda: True, simulation=True)

    status = workflow.start(0.01)

    assert events[:3] == ["prepare", ("z_feed", 0.01), "motor"]
    assert status["state"] == "DRILLING"
    assert status["feed_state"] == "FEEDING"
    assert status["motor_running"] is True
    workflow.emergency_stop(stop_arm=False)


def test_tool5_start_feedback_timeout_safe_stops():
    stale = _drill_telemetry()
    stale["health"]["driver"] = False
    stale["fresh"] = {"joints": False, "wrench": False, "tcp": False}
    hardware = _RecordingSimulationHardware()
    arm_stops = []
    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.02}, hardware, lambda: stale,
        lambda: None, lambda depth: None, lambda: None,
        lambda: arm_stops.append("safe_stop"), lambda: True,
        simulation=False)

    with pytest.raises(RuntimeError, match="启动反馈等待超时"):
        workflow.start(0.001)

    status = workflow.snapshot()
    assert status["state"] == "SAFE_STOP"
    assert "driver, joints, tcp" in status["fault_reason"]
    assert hardware.commands == ["stop"]
    assert arm_stops == ["safe_stop"]


def test_tool5_starting_still_stops_when_lease_is_lost():
    hardware = _Tool5HardwareDouble()
    workflow, arm_stops = _starting_tool5(
        _drill_telemetry(), hardware, lease_active=lambda: False)

    status = workflow.snapshot()

    assert status["state"] == "SAFE_STOP"
    assert "租约" in status["fault_reason"]
    assert hardware.commands == ["stop"]
    assert arm_stops == ["safe_stop"]


def test_tool5_starting_with_motor_output_applies_runtime_safety():
    raw = _drill_telemetry()
    raw["fresh"]["tcp"] = False
    hardware = _Tool5HardwareDouble()
    workflow, arm_stops = _starting_tool5(raw, hardware)
    with workflow._lock:
        workflow._motor_running = True
        workflow._started_monotonic = time.monotonic()

    status = workflow.snapshot()

    assert status["state"] == "SAFE_STOP"
    assert "tcp反馈超时" in status["fault_reason"]
    assert hardware.keepalive_calls == 1
    assert hardware.commands == ["stop"]
    assert arm_stops == ["safe_stop"]


def test_tool5_live_start_aborts_when_lease_is_lost_during_feedback_wait():
    stale = _drill_telemetry()
    stale["fresh"] = {"joints": False, "wrench": False, "tcp": False}
    fresh = _drill_telemetry()
    feedback_fresh = threading.Event()
    telemetry_seen = threading.Event()
    lease = {"active": True}
    hardware = _BlockingTool5Hardware()
    actions = []
    errors = []

    def telemetry():
        telemetry_seen.set()
        return fresh if feedback_fresh.is_set() else stale

    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.5}, hardware, telemetry,
        lambda: actions.append("prepare"),
        lambda depth: actions.append(("engage", depth)), lambda: None,
        lambda: actions.append("safe_stop"), lambda: lease["active"],
        simulation=False)
    worker = threading.Thread(
        target=lambda: _capture_error(errors, workflow.start, 0.001))
    worker.start()
    assert telemetry_seen.wait(timeout=1.0)

    lease["active"] = False
    assert workflow.snapshot()["state"] == "SAFE_STOP"
    feedback_fresh.set()
    hardware.allow_motor_launch.set()
    hardware.release_motor.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert errors and "中止" in errors[0]
    assert workflow.snapshot()["state"] == "SAFE_STOP"
    assert hardware.commands == ["stop"]
    assert actions == ["safe_stop"]


def test_tool5_live_start_aborts_when_lease_is_lost_after_prepare():
    lease = {"active": True}
    prepared = threading.Event()
    resume_prepare = threading.Event()
    hardware = _BlockingTool5Hardware()
    actions = []
    errors = []

    def prepare():
        actions.append("prepare")
        prepared.set()
        assert resume_prepare.wait(timeout=1.0)

    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.5}, hardware, _drill_telemetry,
        prepare, lambda depth: actions.append(("engage", depth)), lambda: None,
        lambda: actions.append("safe_stop"), lambda: lease["active"],
        simulation=False)
    worker = threading.Thread(
        target=lambda: _capture_error(errors, workflow.start, 0.001))
    worker.start()
    assert prepared.wait(timeout=1.0)

    lease["active"] = False
    assert workflow.snapshot()["state"] == "SAFE_STOP"
    resume_prepare.set()
    hardware.allow_motor_launch.set()
    hardware.release_motor.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert errors and "中止" in errors[0]
    assert workflow.snapshot()["state"] == "SAFE_STOP"
    assert hardware.commands == ["stop"]
    assert actions == ["prepare", "safe_stop"]


def test_tool5_motor_launch_and_safe_stop_are_serialized():
    lease = {"active": True}
    hardware = _BlockingTool5Hardware()
    actions = []
    errors = []
    workflow = Tool5DrillWorkflow(
        {"feedback_timeout_s": 0.5}, hardware, _drill_telemetry,
        lambda: actions.append("prepare"),
        lambda depth: actions.append(("engage", depth)), lambda: None,
        lambda: actions.append("safe_stop"), lambda: lease["active"],
        simulation=False)
    starter = threading.Thread(
        target=lambda: _capture_error(errors, workflow.start, 0.001))
    starter.start()
    assert hardware.before_motor_launch.wait(timeout=1.0)

    lease["active"] = False
    stopper = threading.Thread(target=workflow.snapshot)
    stopper.start()
    time.sleep(0.02)
    hardware.allow_motor_launch.set()
    stopper.join(timeout=1.0)
    hardware.release_motor.set()
    starter.join(timeout=1.0)

    assert not stopper.is_alive()
    assert not starter.is_alive()
    assert workflow.snapshot()["state"] == "SAFE_STOP"
    assert hardware.commands[-1] == "stop"
    assert "dig_continuous" not in hardware.commands[hardware.commands.index("stop") + 1:]
    assert actions == ["prepare", ("engage", 0.001), "safe_stop"]


def _capture_error(errors, callback, *args):
    try:
        callback(*args)
    except Exception as error:
        errors.append(str(error))


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


def test_tool5_missing_force_feedback_does_not_block_direct_z_start():
    hardware = SamplingHardware({}, simulation=True)
    raw = _drill_telemetry()
    raw["fresh"]["wrench"] = False
    raw["wrench"] = {}
    actions = []
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05,
                         "require_force_feedback": False}},
        hardware, lambda: raw, lambda: actions.append("prepare"),
        lambda depth: actions.append(("engage", depth)), lambda: None,
        lambda: actions.append("safe_stop"), lambda: True, simulation=True)

    status = workflow.start(0.002)

    assert status["state"] == "DRILLING"
    assert status["feed_control"] == "BOUNDED_POSITION"
    assert actions == ["prepare", ("engage", 0.002)]
    workflow.emergency_stop(stop_arm=False)


def test_tool5_motor_stop_failure_is_safe_stop():
    class BrokenStopHardware(SamplingHardware):
        def motor_command(self, name, speed=0.5, duration=3.0, on_started=None):
            if name == "stop":
                raise RuntimeError("stop ACK missing")
            return super().motor_command(name, speed, duration, on_started)
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


def test_tool5_completed_task_restarts_directly_without_manual_reset():
    hardware = SamplingHardware({}, simulation=True)
    actions = []
    workflow = Tool5DrillWorkflow(
        {"tool5_drill": {"max_z_velocity_m_s": 0.05}},
        hardware, _drill_telemetry, lambda: actions.append("prepare"),
        lambda depth: actions.append(("engage", depth)), lambda: None,
        lambda: None, lambda: True, simulation=True)

    workflow.start(0.002)
    assert workflow.stop_rotation()["state"] == "COMPLETED"
    restarted = workflow.start(0.003)

    assert restarted["state"] == "DRILLING"
    assert actions == ["prepare", ("engage", 0.002),
                       "prepare", ("engage", 0.003)]
    workflow.emergency_stop(stop_arm=False)


def test_every_cartesian_preset_has_a_bounded_ik_solution():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    try:
        poses = runtime.poses()
        assert poses
        for pose in poses:
            result = runtime.inverse_kinematics(pose)
            assert result["ok"] is True, pose["name"]
            assert result["position_error_m"] <= 0.006
            assert result["angle_error_rad"] <= 0.08
    finally:
        runtime.close()


def test_ik_accepts_accurate_bounded_solution_after_iteration_limit():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    current = [-0.11796200399398804, 0.6353196165084839,
               2.8649933265686034, -0.2441429530620575,
               0.22445934853553773, -1.3730357669830322,
               4.502780575561523]
    runtime.snapshot = lambda: {
        "joints": {"joint{}".format(i + 1): value
                   for i, value in enumerate(current)}}
    try:
        pose = next(item for item in runtime.poses() if item["name"] == "收拢")
        result = runtime.inverse_kinematics(pose)
        assert result["ok"] is True
        assert result["position_error_m"] <= 0.006
        assert result["angle_error_rad"] <= 0.08
    finally:
        runtime.close()


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
    assert moved["joints"]["joint1"] > initial["joints"]["joint1"]
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
    assert qr_state["operator_state"] == "IDLE"
    assert qr_state["five_target_status"]["targets_valid"] is True
    assert qr_state["five_target_status"]["holding_slot"] == 2
    assert qr_state["five_target_status"]["grasp_confirmed"] is False
    assert len(qr_state["five_targets"]["targets"]) == 5
    assert qr_state["tool_voltage"] == 24
    runtime.command("start_teleop", lease["session_id"])
    teleop = runtime.snapshot()
    assert teleop["operator_state"] == "TELEOP"
    assert teleop["five_target_status"]["holding_slot"] == 2
    assert teleop["tool_voltage"] == 24
    runtime.command("stop_teleop", lease["session_id"])
    runtime.command("cancel_tool", lease["session_id"])
    cancelled = runtime.snapshot()
    assert cancelled["operator_state"] == "IDLE"
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


def test_cartesian_jog_simulation_maps_twelve_directions_and_enforces_release():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    lease = runtime.sessions.acquire("Cartesian jog mapping")
    token = lease["session_id"]
    runtime.sessions.bind_websocket(token, "jog-map-socket")
    runtime.sessions.timeout_s = 2.0
    directions = (
        "x_plus", "x_minus", "y_plus", "y_minus", "z_plus", "z_minus",
        "rx_plus", "rx_minus", "ry_plus", "ry_minus", "rz_plus", "rz_minus",
    )
    try:
        for direction in directions:
            runtime.sessions.heartbeat(token, "jog-map-socket")
            result = runtime.start_cartesian_jog(token, direction)
            assert result["simulation"] is True
            assert runtime.cartesian_jog_status()["state"] == "JOGGING"
            start = runtime._simulation_jog_start_matrix.copy()
            target = runtime._simulation_jog_target_matrix.copy()
            rotating = direction.startswith("r")
            axis = {"x": 0, "y": 1, "z": 2}[
                direction[1] if rotating else direction[0]]
            sign = 1.0 if direction.endswith("plus") else -1.0
            if rotating:
                relative = start[:3, :3].T.dot(target[:3, :3])
                expected = runtime._rotation(
                    axis, sign * runtime._jog_limits["rotation_rad"])
                assert relative == pytest.approx(expected, abs=1e-10)
                assert target[:3, 3] == pytest.approx(start[:3, 3], abs=1e-12)
            else:
                delta = target[:3, 3] - start[:3, 3]
                expected = [0.0, 0.0, 0.0]
                expected[axis] = sign * runtime._jog_limits["translation_m"]
                assert delta == pytest.approx(expected, abs=1e-12)
                assert target[:3, :3] == pytest.approx(start[:3, :3], abs=1e-12)
            with pytest.raises(RuntimeError, match="必须先松开"):
                runtime.start_cartesian_jog(token, direction)
            runtime.sessions.heartbeat(token, "jog-map-socket")
            stopped = runtime.stop_cartesian_jog(token)
            assert stopped["stopped"] is True
    finally:
        runtime.close()


def test_cartesian_jog_simulation_limit_and_lease_loss_stop_without_ros_output():
    package = Path(__file__).resolve().parents[1]
    config = load_web_configuration(str(package / "config/gui.yaml"),
                                    str(package / "config/tools.yaml"))
    runtime = WebRuntime(config, config["tools"], simulation=True)
    runtime.ros.publish_cartesian_jog_heartbeat = lambda: pytest.fail(
        "simulation must never publish Cartesian ROS heartbeat")
    lease = runtime.sessions.acquire("Cartesian jog watchdog")
    token = lease["session_id"]
    runtime.sessions.bind_websocket(token, "jog-watchdog-socket")
    runtime.sessions.timeout_s = 2.0
    runtime.sessions.heartbeat(token, "jog-watchdog-socket")
    try:
        runtime.start_cartesian_jog(token, "x_plus")
        runtime._simulation_jog_started -= 2.0
        limited = runtime.snapshot()["cartesian_jog"]
        assert limited["state"] == "LIMIT_REACHED"
        assert limited["progress"] == 1.0
        assert "maximum jog travel" in limited["stop_reason"]
        runtime.sessions.heartbeat(token, "jog-watchdog-socket")
        runtime.stop_cartesian_jog(token)
        runtime.sessions.heartbeat(token, "jog-watchdog-socket")
        runtime.start_cartesian_jog(token, "rz_plus")
        assert runtime.sessions.release(token, "jog-watchdog-socket")
        assert runtime.publish_guarded_heartbeat() is False
        stopped = runtime.snapshot()["cartesian_jog"]
        assert stopped["state"] == "STOPPED"
        assert "lease expired" in stopped["stop_reason"]
        assert runtime.snapshot()["operator_state"] == "IDLE"
    finally:
        runtime.close()


def test_cartesian_executor_exit_latches_safe_stop_and_attempts_driver_stop_once():
    class StopRecorder:
        def __init__(self):
            self.calls = 0

        def emergency_stop(self):
            self.calls += 1
            return {"acknowledged": True}

    runtime = object.__new__(WebRuntime)
    runtime.state_machine = StateMachine()
    runtime.ros = StopRecorder()
    runtime._cartesian_exit_stop_attempted = False
    runtime._handle_cartesian_executor_exit()
    runtime._handle_cartesian_executor_exit()
    assert runtime.state_machine.state == RobotState.SAFE_STOP
    assert runtime.ros.calls == 1


def test_cartesian_executor_exit_stop_failure_remains_safe_stop():
    class FailedStop:
        @staticmethod
        def emergency_stop():
            raise RuntimeError("driver ACK timeout")

    runtime = object.__new__(WebRuntime)
    runtime.state_machine = StateMachine()
    runtime.ros = FailedStop()
    runtime._cartesian_exit_stop_attempted = False
    runtime._handle_cartesian_executor_exit()
    assert runtime.state_machine.state == RobotState.SAFE_STOP
    assert runtime._cartesian_exit_stop_attempted is True


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
