"""Framework-neutral service layer shared by REST and WebSocket handlers."""
from collections import deque
import json
import logging
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Dict, Iterable, List, Optional

import numpy as np
import yaml

from rm75_sampling_gui.control.joint_control_adapter import JointControlAdapter
from rm75_sampling_gui.logger.experiment_logger import ExperimentLogger
from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel
from rm75_sampling_gui.ros.ros_interface import ROSInterface
from rm75_sampling_gui.state.state_machine import RobotState, StateMachine
from rm75_sampling_gui.task.task_manager import TaskManager
from .session import ControlSessionManager
from .hardware import SamplingHardware


LOG = logging.getLogger("rm75_sampling_gui.web")


def load_parameter_sources(config: Dict[str, Any]) -> Dict[str, Any]:
    """Read the same YAML sources as the legacy read-only parameter page."""
    import rospkg
    result = {"gui": {"path": config.get("config_path"), "values": dict(config)}}
    for name, key in (("teleop", "teleop_config"),
                      ("z_force_teleop", "z_force_teleop_config"),
                      ("force", "force_control_config")):
        try:
            path = Path(str(config[key])).expanduser()
            if not path.is_absolute():
                package = ("rm75_touch_teleop" if name in (
                               "teleop", "z_force_teleop") else
                           str(config.get("force_control_package",
                                          "rm75_vertical_press")))
                path = Path(rospkg.RosPack().get_path(package)) / path
            with path.open(encoding="utf-8") as stream:
                values = yaml.safe_load(stream)
            if not isinstance(values, dict):
                raise ValueError("YAML root must be a mapping")
            result[name] = {"path": str(path), "values": values}
        except Exception as error:
            result[name] = {"path": config.get(key), "values": {},
                            "error": str(error)}
    return result


def _plain(value: Any) -> Any:
    """Convert telemetry to bounded JSON data without ROS message objects."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, np.generic):
        return _plain(value.item())
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()
                if k != "image_message"}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return str(value)


class EventBuffer(logging.Handler):
    """Bounded application log used by the dashboard and diagnostics page."""

    def __init__(self, capacity: int = 300) -> None:
        super().__init__(logging.INFO)
        self.items = deque(maxlen=capacity)
        self.lock = threading.RLock()

    def emit(self, record: logging.LogRecord) -> None:
        with self.lock:
            self.items.append({
                "timestamp": record.created,
                "level": record.levelname,
                "source": record.name,
                "message": record.getMessage(),
            })

    def snapshot(self) -> List[Dict[str, Any]]:
        with self.lock:
            return list(self.items)


class WebRuntime:
    """Own existing non-Qt services and expose serializable domain methods."""

    def __init__(self, config: Dict[str, Any], tools: List[Dict[str, Any]],
                 offline: bool = False, simulation: bool = False) -> None:
        if offline and simulation:
            raise ValueError("OFFLINE与SIMULATION模式不能同时启用")
        self.config = config
        self.tools = tools
        self.offline = bool(offline)
        self.simulation = bool(simulation)
        self.state_machine = StateMachine()
        self.robot = RM75RobotModel(config.get("robot_model", {}).get("xacro_path"))
        self.ros = ROSInterface(config)
        self.joint_adapter = JointControlAdapter(
            self.robot, config, ros_interface=self.ros,
            state_machine=self.state_machine)
        self.tasks = TaskManager(
            self.state_machine, self.ros, tools, config,
            execution_enabled=bool(config.get("execution_enabled", False)),
            joint_adapter=self.joint_adapter)
        self.sessions = ControlSessionManager(float(
            config.get("web", {}).get("control_session_timeout_s", 0.65)))
        self.events = EventBuffer()
        logging.getLogger("rm75_sampling_gui").addHandler(self.events)
        self.experiment = ExperimentLogger(
            config.get("experiment", {}).get("directory", "experiments"), config)
        self.state_machine.add_listener(self.experiment.transition)
        self._sequence = 0
        self._sequence_lock = threading.Lock()
        self._last_snapshot: Dict[str, Any] = {}
        self._offline_started = time.monotonic()
        self._simulation_lock = threading.RLock()
        self._simulation_joints = np.zeros(7, dtype=float)
        self._simulation_target = np.zeros(7, dtype=float)
        self._simulation_velocity = np.zeros(7, dtype=float)
        self._simulation_updated = time.monotonic()
        self._simulation_operator_state = RobotState.IDLE.value
        self._simulation_control_launch_mode: Optional[str] = None
        self._simulation_stop_reason = ""
        self._simulation_voltage = 0
        self._simulation_force_setpoint = 8.0
        self._simulation_subsystems = {name: True for name in ("base", "camera", "qr")}
        self._subsystem_cache: Dict[str, Any] = {}
        self._subsystem_checked = 0.0
        self._subsystem_lock = threading.Lock()
        self._tool_config_lock = threading.RLock()
        self._simulation_pick_status: Dict[str, Any] = {
            "status": "IDLE", "step": 0, "step_name": "等待执行"}
        self.hardware = SamplingHardware(
            config,
            authorized=bool(config.get("sampling_hardware_authorized", False)),
            simulation=self.simulation,
        )

    def connect(self) -> bool:
        if self.offline:
            LOG.warning("Web平台以OFFLINE模式启动；所有真实命令均被禁止")
            return False
        if self.simulation:
            LOG.warning("Web平台以SIMULATION模式启动；命令仅进入内存模拟器")
            return False
        return self.ros.connect()

    def next_sequence(self) -> int:
        with self._sequence_lock:
            self._sequence += 1
            return self._sequence

    def snapshot(self) -> Dict[str, Any]:
        if self.simulation:
            with self._simulation_lock:
                raw = self._simulation_snapshot()
                raw["control_launch_mode"] = self._simulation_control_launch_mode
        elif self.offline:
            raw = self._offline_snapshot()
        else:
            raw = self.ros.telemetry_snapshot()
        raw["operator_state"] = (self._simulation_operator_state if self.simulation
                                 else self.state_machine.state.value)
        raw["subsystems"] = self._subsystems()
        raw["control_session"] = self.sessions.snapshot()
        raw["selected_tool"] = self.tasks.selected_tool
        if not self.simulation:
            raw["control_launch_mode"] = self.tasks.control_mode
        raw["offline"] = self.offline
        raw["simulation"] = self.simulation
        raw["execution"] = {
            "general": bool(self.config.get("execution_enabled", False)),
            "joint": bool(self.config.get("joint_control_execution_authorized", False)),
            "touch_z_force": bool(self.config.get("touch_z_force_authorized", False)),
            "trajectory": bool(self.config.get("trajectory_execution_authorized", False)),
            "qr": bool(self.config.get("qr_execution_authorized", False)),
            "supervised": bool(self.config.get("supervised_commissioning_authorized", False)),
            "sampling_hardware": bool(self.config.get("sampling_hardware_authorized", False)),
        }
        raw["sampling_hardware"] = self.hardware.snapshot()
        if self.simulation:
            raw["configured_pick_status"] = dict(self._simulation_pick_status)
        else:
            raw["configured_pick_status"] = dict(
                (raw.get("five_target_status") or {}).get("configured_pick", {}))
        raw["events"] = self.events.snapshot()[-80:]
        raw["diagnostics"] = self._diagnostics(raw)
        raw["timestamp"] = time.time()
        raw["sequence"] = self.next_sequence()
        result = _plain(raw)
        self._last_snapshot = result
        return result

    def _subsystems(self) -> Dict[str, Any]:
        if self.simulation:
            with self._simulation_lock:
                return {name: {"running": running, "owned": True,
                               "expected_nodes": ["/simulation/{}".format(name)]}
                        for name, running in self._simulation_subsystems.items()}
        if self.offline:
            return {name: {"running": False, "owned": False,
                           "expected_nodes": []}
                    for name in ("base", "camera", "qr")}
        now = time.monotonic()
        with self._subsystem_lock:
            if now - self._subsystem_checked >= 1.0 or not self._subsystem_cache:
                self._subsystem_cache = self.tasks.subsystem_status()
                self._subsystem_checked = now
            return self._subsystem_cache

    def _offline_snapshot(self) -> Dict[str, Any]:
        return {
            "connected": False,
            "health": {"master": False, "driver": False, "touch": False,
                       "teleop_control": False, "force_control": False,
                       "joint_control": False, "qr": False, "marker": False},
            "joints": {}, "joint_velocities": {},
            "joint_velocity_source": "unavailable",
            "joint_limits": {name: list(self.robot.limits[name]) for name in JOINT_NAMES},
            "wrench": {}, "tcp": {}, "fresh": {}, "ages": {},
            "vision_status": {}, "task_status": {}, "five_target_status": {},
            "force_z": None, "filtered_force_z": None,
            "force_setpoint": None, "control_mode": "OFFLINE",
            "stop_reason": "", "tool_voltage": None,
            "image_topic": "", "source_stamps": {}, "sequences": {},
        }

    def _simulation_snapshot(self) -> Dict[str, Any]:
        """Advance a deterministic in-memory plant and return realistic telemetry."""
        with self._simulation_lock:
            now_mono = time.monotonic()
            dt = min(max(now_mono - self._simulation_updated, 0.0), 0.25)
            error = self._simulation_target - self._simulation_joints
            max_step = np.asarray([
                self.robot.velocity_limits[name] for name in JOINT_NAMES
            ], dtype=float) * dt * 0.35
            step = np.clip(error, -max_step, max_step)
            self._simulation_joints += step
            self._simulation_velocity = step / dt if dt > 1e-6 else np.zeros(7)
            self._simulation_updated = now_mono
            phase = now_mono - self._offline_started
            force_z = self._simulation_force_setpoint + 0.28 * math.sin(phase * 1.7)
            z_force_active = self._simulation_control_launch_mode == "z_force_teleop"
            # The commissioned basis maps robot Z force onto Touch Y.  Keep the
            # simulation renderer deliberately bounded by the real 0.5 N cap.
            haptic_y = (math.copysign(
                min(0.5, max(0.0, abs(force_z) - 1.0) * 0.03), force_z)
                if z_force_active else 0.0)
            transforms = self.robot.forward_kinematics(self._simulation_joints)
            tcp_matrix = transforms["Link7"]
            pitch = math.asin(max(-1.0, min(1.0, -float(tcp_matrix[2, 0]))))
            if abs(math.cos(pitch)) > 1e-8:
                roll = math.atan2(float(tcp_matrix[2, 1]), float(tcp_matrix[2, 2]))
                yaw = math.atan2(float(tcp_matrix[1, 0]), float(tcp_matrix[0, 0]))
            else:
                roll = math.atan2(-float(tcp_matrix[1, 2]), float(tcp_matrix[1, 1]))
                yaw = 0.0
            now = time.time()
            return {
                "connected": True,
                "health": {"master": True, "driver": True, "touch": True,
                           "teleop_control": self._simulation_operator_state == "TELEOP",
                           "force_control": self._simulation_operator_state == "FORCE_CONTROL",
                           "joint_control": self._simulation_operator_state == "JOINT_CONTROL",
                           "qr": self._simulation_subsystems["qr"],
                           "marker": self._simulation_subsystems["qr"],
                           "camera": self._simulation_subsystems["camera"]},
                "joints": {name: float(self._simulation_joints[index])
                           for index, name in enumerate(JOINT_NAMES)},
                "joint_velocities": {name: float(self._simulation_velocity[index])
                                      for index, name in enumerate(JOINT_NAMES)},
                "joint_velocity_source": "simulation_plant",
                "joint_limits": {name: list(self.robot.limits[name]) for name in JOINT_NAMES},
                "wrench": {"Fx": 0.12 * math.sin(phase), "Fy": 0.08 * math.cos(phase),
                           "Fz": force_z, "Tx": 0.01 * math.sin(phase * 0.7),
                           "Ty": 0.01 * math.cos(phase * 0.8), "Tz": 0.0},
                "tcp": {"x": float(tcp_matrix[0, 3]), "y": float(tcp_matrix[1, 3]),
                        "z": float(tcp_matrix[2, 3]), "rx": roll,
                        "ry": pitch, "rz": yaw},
                "fresh": {"joints": True, "wrench": True, "tcp": True,
                          "image": self._simulation_subsystems["camera"]},
                "ages": {"joints": 0.0, "wrench": 0.0, "tcp": 0.0, "image": 0.0},
                "vision_status": {"ok": True, "stable": True, "marker_id": 0},
                "task_status": {"mode": "simulation"},
                "five_target_status": {"ok": True, "stable": True,
                                       "source_marker_id": 0},
                "marker_0": {"x": 0.42, "y": -0.06, "z": 0.31,
                             "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                "force_z": force_z, "filtered_force_z": force_z,
                "force_setpoint": self._simulation_force_setpoint,
                "control_mode": ("POSITION_MODE" if z_force_active else
                                 "CONTACT_MODE" if
                                 self._simulation_control_launch_mode == "force_control"
                                 else "SIMULATION"),
                "haptic_enabled": z_force_active,
                "haptic_force_fresh": z_force_active,
                "haptic_command": {"x": 0.0, "y": haptic_y, "z": 0.0},
                "stop_reason": self._simulation_stop_reason,
                "tool_voltage": self._simulation_voltage,
                "image_topic": "/simulation/camera/color/image_raw",
                "source_stamps": {"joints": now, "wrench": now, "image": now},
                "sequences": {"image": int(phase * 6.0)},
            }

    @staticmethod
    def _diagnostics(data: Dict[str, Any]) -> List[Dict[str, Any]]:
        now = time.time()
        faults: List[Dict[str, Any]] = []
        def add(code: str, level: str, reason: str, active: bool = True) -> None:
            if active:
                faults.append({"code": code, "level": level,
                               "timestamp": now, "reason": reason,
                               "status": "ACTIVE"})
        add("ROS_MASTER", "ERROR", "ROS Master不可达",
            not data.get("health", {}).get("master", False))
        add("FEEDBACK_TIMEOUT", "ERROR", "七轴反馈超时",
            bool(data.get("health", {}).get("driver")) and
            not data.get("fresh", {}).get("joints", False))
        add("SAFE_STOP", "CRITICAL", "安全停止已锁存",
            data.get("operator_state") == "SAFE_STOP")
        add("DRIVER_ARM", "ERROR", "驱动ArmError={}".format(
            data.get("driver_arm_error")), bool(data.get("driver_arm_error")))
        add("DRIVER_SYSTEM", "ERROR", "驱动SysError={}".format(
            data.get("driver_system_error")), bool(data.get("driver_system_error")))
        add("PLANNING", "ERROR", "控制器规划状态失败",
            data.get("driver_plan_ok") is False)
        add("CAMERA", "WARN", "相机或RGB帧不可用",
            bool(data.get("subsystems", {}).get("camera", {}).get("running")) and
            not data.get("fresh", {}).get("image", False))
        add("TF", "ERROR", "base_link到Link7的TF不可用或过期",
            bool(data.get("subsystems", {}).get("base", {}).get("running")) and
            not data.get("fresh", {}).get("tcp", False))
        sources = [key for key in ("teleop_control", "force_control", "joint_control")
                   if data.get("health", {}).get(key, False)]
        add("CONTROL_SOURCE", "CRITICAL", "检测到多个控制源：{}".format(
            ", ".join(sources)), len(sources) > 1)
        for channel, ack in (data.get("ack", {}) or {}).items():
            add("ACK_{}".format(str(channel).upper()), "ERROR",
                "{}命令收到失败ACK".format(channel),
                isinstance(ack, dict) and ack.get("value") is False)
        stop_reason = str(data.get("stop_reason") or "")
        add("JOINT_TRACKING", "ERROR", stop_reason,
            "tracking" in stop_reason.lower() or "跟踪" in stop_reason)
        recent_text = "\n".join(str(item.get("message", ""))
                                  for item in data.get("events", [])[-20:])
        add("COLLISION_CHECK", "ERROR", "最近的MoveIt/FCL碰撞检查失败",
            "碰撞" in recent_text and ("失败" in recent_text or "拒绝" in recent_text))
        add("CONTROL_SESSION", "WARN", "当前无浏览器控制租约",
            not data.get("control_session", {}).get("active", False))
        return faults

    def model_descriptor(self) -> Dict[str, Any]:
        visuals = []
        for index, visual in enumerate(self.robot.visuals):
            matrix = visual.origin
            visuals.append({
                "id": index, "link": visual.link,
                "mesh": "/api/v1/model/mesh/{}".format(index),
                "scale": list(visual.scale), "matrix": matrix.reshape(-1).tolist(),
                "source": visual.filename,
            })
        joints = []
        for joint in self.robot.chain:
            from rm75_sampling_gui.model.robot_model import _origin_matrix
            joints.append({
                "name": joint.name, "parent": joint.parent, "child": joint.child,
                "axis": [float(x) for x in joint.axis],
                "matrix": _origin_matrix(joint.origin).reshape(-1).tolist(),
                "lower": self.robot.limits[joint.name][0],
                "upper": self.robot.limits[joint.name][1],
                "velocity": self.robot.velocity_limits[joint.name],
            })
        return {
            "model": "RM75-6FB", "base": "base_link", "tip": "Link7",
            "xacro": self.robot.xacro_path, "link7_variant": "Link7_6fb",
            "joints": joints, "visuals": visuals,
        }

    def mesh_path(self, index: int) -> str:
        if index < 0 or index >= len(self.robot.visuals):
            raise IndexError("unknown mesh")
        return self.robot.visuals[index].filename

    def preview(self, values: Iterable[float]) -> Dict[str, Any]:
        vector = self.robot.validate_vector(values)
        transforms = self.robot.forward_kinematics(vector)
        return {"target": vector.tolist(), "links": {
            name: matrix.reshape(-1).tolist() for name, matrix in transforms.items()},
            "violations": self.robot.limit_violations(vector)}

    def require_session(self, session_id: str) -> None:
        if self.offline:
            raise RuntimeError("OFFLINE模式禁止控制命令")
        self.sessions.require(session_id)

    def execute_joints(self, session_id: str, values: Iterable[float]) -> Dict[str, Any]:
        self.require_session(session_id)
        if self.simulation:
            vector = self.robot.validate_vector(values)
            violations = self.robot.limit_violations(vector)
            if violations:
                raise ValueError("仿真目标超出关节限位：{}".format(", ".join(violations)))
            with self._simulation_lock:
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                self._simulation_target = vector.copy()
                self._simulation_operator_state = RobotState.JOINT_CONTROL.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = ""
            LOG.info("SIMULATION七轴目标已接收：%s", vector.tolist())
            return {"accepted": True, "simulation": True, "target": vector.tolist()}
        self.experiment.begin_task()
        LOG.info("Web受控七轴目标请求已进入既有自动安全检查流程")
        result = self.tasks.execute_joint_target(values)
        LOG.info("Web受控七轴目标请求完成：%s", result)
        return result

    @staticmethod
    def _validated_tool_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
        keys = ("x", "y", "z", "rx", "ry", "rz", "lift_height")
        try:
            values = {key: float(payload[key]) for key in keys}
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("工具配置必须包含有限的X/Y/Z/Rx/Ry/Rz/Lift Height") from error
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("工具配置不能包含NaN或无穷值")
        if values["lift_height"] <= 0.0:
            raise ValueError("Lift Height必须大于0；单位为米")
        return values

    def _read_tools_document(self) -> Dict[str, Any]:
        path = Path(str(self.config["tools_config_path"]))
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        tools = document.get("tools")
        if not isinstance(tools, list) or len(tools) != 5:
            raise RuntimeError("tools.yaml必须且只能包含五个工具")
        return document

    def tool_configurations(self) -> List[Dict[str, Any]]:
        with self._tool_config_lock:
            document = self._read_tools_document()
        result = []
        for tool in document["tools"]:
            configured = dict(tool.get("configured_pick") or {})
            position = list(configured.get("position_m") or [])
            rpy = list(configured.get("rpy_rad") or [])
            if len(position) != 3 or len(rpy) != 3:
                raise RuntimeError("{}缺少完整configured_pick配置".format(tool.get("name")))
            result.append({
                "slot_id": int(tool["slot_id"]), "name": str(tool["name"]),
                "x": float(position[0]), "y": float(position[1]),
                "z": float(position[2]), "rx": float(rpy[0]),
                "ry": float(rpy[1]), "rz": float(rpy[2]),
                "lift_height": float(configured["lift_height_m"]),
                "updated_at": configured.get("updated_at"),
                "position_unit": "m", "angle_unit": "rad",
                "pose_reference": "Link7", "frame": "base_link",
            })
        return result

    def save_tool_configuration(self, slot_id: int, payload: Dict[str, Any],
                                session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if int(slot_id) not in range(1, 6):
            raise ValueError("工具编号必须是1..5")
        values = self._validated_tool_payload(payload)
        with self._tool_config_lock:
            document = self._read_tools_document()
            matches = [tool for tool in document["tools"]
                       if int(tool.get("slot_id", 0)) == int(slot_id)]
            if len(matches) != 1:
                raise RuntimeError("工具{}配置不存在或重复".format(slot_id))
            matches[0]["configured_pick"] = {
                "position_m": [values["x"], values["y"], values["z"]],
                "rpy_rad": [values["rx"], values["ry"], values["rz"]],
                "lift_height_m": values["lift_height"],
                "updated_at": time.time(),
            }
            path = Path(str(self.config["tools_config_path"]))
            temporary = path.with_name(path.name + ".tmp")
            with temporary.open("w", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, allow_unicode=True,
                               sort_keys=False, default_flow_style=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(str(temporary), str(path))
            self.tools = [dict(tool) for tool in document["tools"]]
            self.tasks.tools = {str(tool["name"]): dict(tool)
                                for tool in document["tools"]}
            self.config["tools_document"] = document
        LOG.info("Tool%d绝对取物位姿已原子保存到%s", slot_id, path)
        return next(item for item in self.tool_configurations()
                    if item["slot_id"] == int(slot_id))

    def dry_run_configured_tool(self, slot_id: int, session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if self.simulation:
            config = next((item for item in self.tool_configurations()
                           if item["slot_id"] == int(slot_id)), None)
            if config is None:
                raise ValueError("工具编号必须是1..5")
            with self._simulation_lock:
                self._simulation_pick_status = {
                    "status": "DRY_RUN_OK", "step": 0,
                    "step_name": "Dry Run规划通过", "slot_id": int(slot_id)}
            return {"ok": True, "simulation": True, "dry_run": True,
                    "configuration": config,
                    "sequence": ["预抓取", "基坐标Z下降", "24V取物ACK",
                                 "取物结果未独立确认", "基坐标Z抬升", "保持"]}
        return self.tasks.plan_configured_tool(int(slot_id))

    def execute_configured_tool(self, slot_id: int, session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if self.simulation:
            self.dry_run_configured_tool(slot_id, session_id)
            with self._simulation_lock:
                self.tasks.selected_tool = "Tool{}".format(int(slot_id))
                self._simulation_voltage = 24
                self._simulation_operator_state = RobotState.TOOL_HOLD.value
                self._simulation_pick_status = {
                    "status": "COMPLETED_UNVERIFIED", "step": 6,
                    "step_name": "运动完成；取物未独立确认", "slot_id": int(slot_id),
                    "grasp_confirmed": False}
            return {"executed": True, "simulation": True,
                    "status": "COMPLETED_UNVERIFIED", "grasp_confirmed": False}
        self.experiment.begin_task()
        return self.tasks.execute_configured_tool(int(slot_id))

    def command(self, name: str, session_id: str,
                payload: Optional[Dict[str, Any]] = None) -> Any:
        self.require_session(session_id)
        payload = payload or {}
        if name == "pump_on":
            state = (self._simulation_operator_state if self.simulation
                     else self.state_machine.state.value)
            if state == RobotState.SAFE_STOP.value:
                raise RuntimeError("SAFE_STOP已锁存，仅允许关闭采样硬件")
            return self.hardware.set_pump(True)
        if name == "pump_off":
            return self.hardware.set_pump(False)
        if name == "motor":
            action = str(payload.get("action", "status"))
            state = (self._simulation_operator_state if self.simulation
                     else self.state_machine.state.value)
            if (state == RobotState.SAFE_STOP.value and
                    action not in {"stop", "status"}):
                raise RuntimeError("SAFE_STOP已锁存，仅允许停止或读取电机")
            return self.hardware.motor_command(
                action,
                float(payload.get("speed", 0.5)),
                float(payload.get("duration", 3.0)),
            )
        if self.simulation:
            result = self._simulation_command(name, payload)
            if name == "emergency_stop":
                self.hardware.emergency_stop()
            return result
        if name == "emergency_stop":
            try:
                return self.tasks.emergency_stop()
            finally:
                self.hardware.emergency_stop()
        commands = {
            "recover": self.tasks.recover,
            "cancel_joints": self.tasks.cancel_joint_control,
            "start_teleop": self.tasks.start_teleop,
            "stop_teleop": self.tasks.stop_teleop,
            "start_z_force_teleop": self.tasks.start_z_force_teleop,
            "stop_z_force_teleop": self.tasks.stop_z_force_teleop,
            "enter_force_control": self.tasks.enter_force_control,
            "stop_sampling": self.tasks.stop_sampling,
            "detect_tool": self.tasks.detect_tool,
            "detect_marker": self.tasks.detect_common_marker,
            "compute_targets": self.tasks.compute_five_targets,
            "preview_tool": self.tasks.preview_selected_target,
            "plan_tool": self.tasks.plan_selected_target,
            "execute_tool": self.tasks.execute_selected_target,
            "return_tool": self.tasks.return_selected_target,
            "cancel_tool": self.tasks.cancel_target_execution,
        }
        if name == "select_tool":
            self.tasks.select_tool(str(payload.get("tool", "")))
            LOG.info("Web工具选择：%s", self.tasks.selected_tool)
            return {"selected_tool": self.tasks.selected_tool}
        if name == "set_voltage":
            result = self.tasks.set_voltage(int(payload.get("voltage", -1)))
            LOG.info("Web工具电压命令完成：%s", result)
            return result
        callback = commands.get(name)
        if callback is None:
            raise ValueError("不允许的命令：{}".format(name))
        LOG.info("Web白名单命令开始：%s", name)
        result = callback()
        LOG.info("Web白名单命令完成：%s result=%s", name, result)
        return result

    def _simulation_command(self, name: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        states = {
            "start_teleop": RobotState.TELEOP.value,
            "stop_teleop": RobotState.IDLE.value,
            "start_z_force_teleop": RobotState.FORCE_CONTROL.value,
            "stop_z_force_teleop": RobotState.IDLE.value,
            "enter_force_control": RobotState.FORCE_CONTROL.value,
            "stop_sampling": RobotState.IDLE.value,
            "detect_tool": RobotState.TOOL_DETECT.value,
            "detect_marker": RobotState.TOOL_DETECT.value,
            "compute_targets": RobotState.TOOL_DETECT.value,
            "preview_tool": RobotState.TOOL_PICK.value,
            "plan_tool": RobotState.TOOL_PICK.value,
            "execute_tool": RobotState.TOOL_HOLD.value,
            "return_tool": RobotState.TOOL_RETURN.value,
            "cancel_tool": RobotState.IDLE.value,
            "cancel_joints": RobotState.IDLE.value,
        }
        allowed = set(states).union({"emergency_stop", "recover", "select_tool", "set_voltage"})
        if name not in allowed:
            raise ValueError("不允许的命令：{}".format(name))
        with self._simulation_lock:
            if name == "emergency_stop":
                self._simulation_target = self._simulation_joints.copy()
                self._simulation_operator_state = RobotState.SAFE_STOP.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = "SIMULATION software emergency stop"
            elif name == "recover":
                self._simulation_operator_state = RobotState.IDLE.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = ""
            elif name == "select_tool":
                tool = str(payload.get("tool", ""))
                if tool not in {"Tool{}".format(index) for index in range(1, 6)}:
                    raise ValueError("仿真工具必须是Tool1..Tool5")
                self.tasks.selected_tool = tool
            elif name == "set_voltage":
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                voltage = int(payload.get("voltage", -1))
                if voltage not in (0, 24):
                    raise ValueError("仿真工具电压仅允许0V或24V")
                self._simulation_voltage = voltage
            else:
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                self._simulation_operator_state = states[name]
                launch_modes = {
                    "start_teleop": "teleop",
                    "start_z_force_teleop": "z_force_teleop",
                    "enter_force_control": "force_control",
                }
                if name in launch_modes:
                    self._simulation_control_launch_mode = launch_modes[name]
                else:
                    self._simulation_control_launch_mode = None
        LOG.info("SIMULATION白名单命令完成：%s", name)
        return {"ok": True, "simulation": True, "command": name,
                "operator_state": self._simulation_operator_state}

    def lifecycle(self, name: str, action: str, session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if name not in ("base", "camera", "qr"):
            raise ValueError("不允许的子系统")
        if self.simulation:
            if action not in ("start", "stop"):
                raise ValueError("不允许的生命周期操作")
            with self._simulation_lock:
                self._simulation_subsystems[name] = action == "start"
            LOG.info("SIMULATION生命周期%s：%s", action, name)
            return {"ok": True, "simulation": True, "name": name,
                    "running": action == "start"}
        if action == "start":
            result = self.tasks.start_subsystem(name)
            LOG.info("Web生命周期启动完成：%s result=%s", name, result)
            return result
        if action == "stop":
            result = self.tasks.stop_subsystem(name)
            LOG.info("Web生命周期停止完成：%s result=%s", name, result)
            return result
        raise ValueError("不允许的生命周期操作")

    def parameters(self) -> Dict[str, Any]:
        return {
            "gui": _plain(self.config),
            "sources": _plain(self.config.get("parameter_sources", {})),
            "tools": _plain(self.config.get("tools_document", {})),
        }

    def experiments(self) -> List[Dict[str, Any]]:
        root = Path(self.config.get("experiment", {}).get("directory", "experiments"))
        if not root.exists():
            return []
        return [{"name": item.name, "path": str(item),
                 "modified": item.stat().st_mtime}
                for item in sorted(root.iterdir(), reverse=True) if item.is_dir()][:50]

    def sample_experiment(self) -> None:
        if self._last_snapshot:
            self.experiment.sample(self._last_snapshot)

    def publish_guarded_heartbeat(self) -> bool:
        if self.offline or self.simulation or not self.sessions.snapshot().get("active"):
            return False
        self.ros.publish_joint_control_heartbeat()
        return True

    def close(self) -> None:
        try:
            self.hardware.close()
        finally:
            try:
                self.tasks.shutdown()
            finally:
                self.ros.shutdown()
                self.experiment.close()

    def simulation_jpeg(self) -> bytes:
        if not self.simulation:
            raise RuntimeError("当前不是SIMULATION模式")
        import cv2
        height, width = 240, 424
        phase = time.monotonic() - self._offline_started
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :, 0] = np.linspace(24, 70, width, dtype=np.uint8)
        image[:, :, 1] = np.linspace(38, 95, height, dtype=np.uint8)[:, None]
        center = (int(width / 2 + 55 * math.sin(phase * 0.6)), int(height / 2))
        cv2.rectangle(image, (center[0] - 28, center[1] - 28),
                      (center[0] + 28, center[1] + 28), (30, 220, 210), 2)
        cv2.putText(image, "SIMULATION  APRILTAG 0", (18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 245, 245), 1, cv2.LINE_AA)
        cv2.putText(image, "NO PHYSICAL CAMERA", (18, height - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 170, 255), 1, cv2.LINE_AA)
        ok, encoded = cv2.imencode(".jpg", image,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            raise RuntimeError("仿真图像JPEG编码失败")
        return encoded.tobytes()


def load_web_configuration(config_path: str, tools_path: str,
                           overrides: Optional[Dict[str, bool]] = None) -> Dict[str, Any]:
    with open(config_path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    with open(tools_path, encoding="utf-8") as stream:
        tools_document = yaml.safe_load(stream) or {}
    tools = tools_document.get("tools")
    if not isinstance(tools, list) or len(tools) != 5:
        raise ValueError("tools.yaml必须定义五个工具")
    config["config_path"] = str(Path(config_path).resolve())
    config["tools_config_path"] = str(Path(tools_path).resolve())
    config["tools_document"] = tools_document
    config["tools"] = tools
    for key, value in (overrides or {}).items():
        config[key] = bool(value)
    experiment = config.setdefault("experiment", {})
    directory = Path(str(experiment.get("directory", "experiments"))).expanduser()
    if not directory.is_absolute():
        directory = Path(config_path).resolve().parent.parent / directory
    experiment["directory"] = str(directory)
    config["parameter_sources"] = load_parameter_sources(config)
    return config
