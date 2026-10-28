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
from rm75_sampling_gui.trajectory.repository import TrajectoryRepository
from rm75_sampling_gui.trajectory.service import TrajectoryService
from rm75_sampling_gui.trajectory.interpolation import trajectory_coefficients, evaluate_quintic
from .session import ControlSessionManager
from .hardware import SamplingHardware
from .tool5_drill import ACTIVE_STATES, Tool5DrillWorkflow


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
        self.unified_control_authorized = bool(
            config.get("unified_control_authorized", False)
        )
        self.active_control_path = self._configured_control_path(config)
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
        self._control_path_lock = threading.RLock()
        self._cartesian_exit_stop_attempted = False
        # A collision-free, non-singular posture captured by the existing
        # Cartesian commissioning evidence. Starting the in-memory plant at
        # all-zero joints makes some 10 mm/5 deg directions kinematically
        # unreachable and would not exercise all twelve controls.
        self._simulation_joints = np.asarray([
            -2.0259972946, -0.2410891997, -2.7937972946, 0.4415548149,
            2.8620793129, -1.5531722160, -0.3058461575,
        ], dtype=float)
        self._simulation_target = self._simulation_joints.copy()
        self._simulation_velocity = np.zeros(7, dtype=float)
        self._simulation_updated = time.monotonic()
        self._simulation_operator_state = RobotState.IDLE.value
        self._simulation_control_launch_mode: Optional[str] = None
        self._simulation_stop_reason = ""
        self._simulation_voltage = 0
        self._simulation_force_setpoint = 8.0
        jog_options = dict(config.get("cartesian_jog", {}))
        self._jog_limits = {
            "translation_m": min(0.01, float(jog_options.get("max_translation_m", 0.01))),
            "rotation_rad": min(math.radians(5.0), float(jog_options.get("max_rotation_rad", math.radians(5.0)))),
            "linear_speed_mps": min(0.03, max(0.01, float(jog_options.get("link7_speed_mps", 0.01)))),
            "angular_speed_rad_s": min(math.radians(10.0), max(1e-3, float(jog_options.get("angular_speed_rad_s", math.radians(5.0))))),
        }
        self._simulation_jog: Dict[str, Any] = self._empty_jog_status()
        self._simulation_jog_started = 0.0
        self._simulation_jog_start_matrix: Optional[np.ndarray] = None
        self._simulation_jog_target_matrix: Optional[np.ndarray] = None
        self._simulation_jog_start_joints: Optional[np.ndarray] = None
        self._simulation_jog_target_joints: Optional[np.ndarray] = None
        self._simulation_subsystems = {name: True for name in ("base", "camera", "qr")}
        self._subsystem_cache: Dict[str, Any] = {}
        self._subsystem_checked = 0.0
        self._subsystem_lock = threading.Lock()
        self._tool_config_lock = threading.RLock()
        self._ui_config_lock = threading.RLock()
        self._simulation_pick_status: Dict[str, Any] = {
            "status": "IDLE", "step": 0, "step_name": "等待执行"}
        self._simulation_qr_plan_slot: Optional[int] = None
        self._simulation_five_targets: Dict[str, Any] = {}
        self._simulation_qr_status: Dict[str, Any] = {
            "mode": "single_marker_five_targets",
            "selected_slot": 1,
            "vision": {"ok": True, "marker_id": 0, "stable": True,
                       "source": "simulation_camera"},
            "targets_valid": False,
            "execution_ready": True,
            "execution_rejections": [],
            "holding_slot": None,
            "staged_pickup_slot": None,
            "simulation_only": True,
            "step": "等待二维码目标计算",
        }
        self.hardware = SamplingHardware(
            config,
            authorized=bool(config.get("sampling_hardware_authorized", False)),
            simulation=self.simulation,
        )
        self.tool5_drill = Tool5DrillWorkflow(
            config, self.hardware, self._tool5_telemetry,
            self._prepare_tool5_drill, self._engage_tool5_drill,
            self._hold_tool5_drill, self._safe_stop_tool5_drill,
            lambda: bool(self.sessions.snapshot().get("active")),
            simulation=self.simulation,
        )
        trajectory_options = dict(config.get("trajectory_library", {}))
        trajectory_directory = str(trajectory_options.get(
            "directory", "~/.ros/rm75_sampling_gui/trajectories"
        ))
        collision_required = bool(trajectory_options.get(
            "collision_check_required", True
        ))
        collision_checker = (
            (lambda values: False) if self.simulation
            else None if self.offline
            else self.joint_adapter._collision_at
        )
        self.trajectories = TrajectoryService(
            TrajectoryRepository(trajectory_directory),
            self.robot,
            self._trajectory_feedback_snapshot,
            trajectory_options.get("acceleration_limits_rad_s2", [1.5] * 7),
            collision_checker=collision_checker,
            collision_required=(collision_required and not self.simulation),
            joint_limit_margin_rad=float(
                config.get("joint_control", {}).get("joint_limit_margin_rad", 0.02)
            ),
            executor=self._execute_trajectory_points,
            stop_executor=self._stop_trajectory_execution,
        )

    @staticmethod
    def _configured_control_path(config: Dict[str, Any]) -> str:
        paths = []
        if config.get("joint_control_execution_authorized", False):
            paths.append("joint_control")
        if config.get("cartesian_jog_execution_authorized", False):
            paths.append("cartesian_jog")
        if config.get("touch_z_force_authorized", False):
            paths.append("z_force")
        if config.get("trajectory_execution_authorized", False):
            paths.append(
                "qr_pickup"
                if config.get("qr_execution_authorized", False)
                else "pose_pickup"
            )
        if config.get("tool5_drill_authorized", False):
            paths.append("tool5_drill")
        return paths[0] if len(paths) == 1 else "observe"

    def _empty_jog_status(self) -> Dict[str, Any]:
        return {
            "state": "IDLE", "direction": "", "stop_reason": "",
            "reference_frame": "base_link", "rotation_frame": "Link7",
            "end_effector": "Link7", "tcp_calibrated": False,
            "progress": 0.0, "actual_pose": {}, "target_pose": {},
            "limits": dict(getattr(self, "_jog_limits", {
                "translation_m": 0.01, "rotation_rad": math.radians(5.0),
                "linear_speed_mps": 0.01,
                "angular_speed_rad_s": math.radians(5.0),
            })),
        }

    @staticmethod
    def _rotation(axis: int, angle: float) -> np.ndarray:
        vector = np.zeros(3, dtype=float)
        vector[axis] = 1.0
        x, y, z = vector
        c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
        return np.asarray([
            [c + x*x*one, x*y*one-z*s, x*z*one+y*s],
            [y*x*one+z*s, c+y*y*one, y*z*one-x*s],
            [z*x*one-y*s, z*y*one+x*s, c+z*z*one],
        ], dtype=float)

    @staticmethod
    def _matrix_pose(matrix: np.ndarray) -> Dict[str, float]:
        pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
        if abs(math.cos(pitch)) > 1e-8:
            roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
            yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
        else:
            roll = math.atan2(-float(matrix[1, 2]), float(matrix[1, 1]))
            yaw = 0.0
        rotation = matrix[:3, :3]
        trace = float(np.trace(rotation))
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = float(rotation[2, 1] - rotation[1, 2]) / scale
            qy = float(rotation[0, 2] - rotation[2, 0]) / scale
            qz = float(rotation[1, 0] - rotation[0, 1]) / scale
        else:
            axis = int(np.argmax(np.diag(rotation)))
            if axis == 0:
                scale = math.sqrt(1.0 + float(rotation[0, 0]) - float(rotation[1, 1]) - float(rotation[2, 2])) * 2.0
                qw = float(rotation[2, 1] - rotation[1, 2]) / scale
                qx = 0.25 * scale
                qy = float(rotation[0, 1] + rotation[1, 0]) / scale
                qz = float(rotation[0, 2] + rotation[2, 0]) / scale
            elif axis == 1:
                scale = math.sqrt(1.0 + float(rotation[1, 1]) - float(rotation[0, 0]) - float(rotation[2, 2])) * 2.0
                qw = float(rotation[0, 2] - rotation[2, 0]) / scale
                qx = float(rotation[0, 1] + rotation[1, 0]) / scale
                qy = 0.25 * scale
                qz = float(rotation[1, 2] + rotation[2, 1]) / scale
            else:
                scale = math.sqrt(1.0 + float(rotation[2, 2]) - float(rotation[0, 0]) - float(rotation[1, 1])) * 2.0
                qw = float(rotation[1, 0] - rotation[0, 1]) / scale
                qx = float(rotation[0, 2] + rotation[2, 0]) / scale
                qy = float(rotation[1, 2] + rotation[2, 1]) / scale
                qz = 0.25 * scale
        return {"x": float(matrix[0, 3]), "y": float(matrix[1, 3]),
                "z": float(matrix[2, 3]), "rx": roll, "ry": pitch, "rz": yaw,
                "qx": qx, "qy": qy, "qz": qz, "qw": qw}

    def _advance_simulation_jog(self, now_mono: float) -> None:
        if self._simulation_jog.get("state") != "JOGGING":
            return
        if not self.sessions.snapshot().get("active"):
            self._simulation_target = self._simulation_joints.copy()
            self._simulation_jog.update({
                "state": "STOPPED", "direction": "",
                "stop_reason": "web control lease expired; simulated stop confirmed",
            })
            self._simulation_operator_state = RobotState.IDLE.value
            return
        start = self._simulation_jog_start_matrix
        target = self._simulation_jog_target_matrix
        start_joints = self._simulation_jog_start_joints
        target_joints = self._simulation_jog_target_joints
        if (start is None or target is None or start_joints is None or
                target_joints is None):
            self._simulation_jog.update({"state": "SAFE_STOP",
                                         "stop_reason": "simulation jog pose unavailable"})
            self._simulation_operator_state = RobotState.SAFE_STOP.value
            return
        direction = str(self._simulation_jog["direction"])
        rotating = direction.startswith("r")
        maximum = (self._jog_limits["rotation_rad"] if rotating
                   else self._jog_limits["translation_m"])
        speed = (self._jog_limits["angular_speed_rad_s"] if rotating
                 else self._jog_limits["linear_speed_mps"])
        progress = min(1.0, max(0.0, (now_mono - self._simulation_jog_started) * speed / maximum))
        desired = start.copy()
        if rotating:
            axis = {"x": 0, "y": 1, "z": 2}[direction[1]]
            sign = 1.0 if direction.endswith("plus") else -1.0
            desired[:3, :3] = start[:3, :3].dot(
                self._rotation(axis, sign * maximum * progress))
        else:
            axis = {"x": 0, "y": 1, "z": 2}[direction[0]]
            sign = 1.0 if direction.endswith("plus") else -1.0
            desired[axis, 3] = start[axis, 3] + sign * maximum * progress
        # The full-distance target passed IK before STARTED. Interpolate that
        # fixed solution here so telemetry refresh never performs expensive IK
        # on the ASGI event loop and therefore cannot starve lease heartbeats.
        self._simulation_target = (
            start_joints + (target_joints - start_joints) * progress)
        self._simulation_jog.update({
            "progress": progress, "actual_pose": self.robot.tcp(self._simulation_joints),
            "target_pose": self._matrix_pose(desired),
        })
        if progress >= 1.0:
            self._simulation_jog.update({
                "state": "LIMIT_REACHED",
                "stop_reason": "maximum jog travel reached; release before restarting",
            })

    def connect(self) -> bool:
        if self.offline:
            LOG.warning("Web平台以OFFLINE模式启动；所有真实命令均被禁止")
            return False
        if self.simulation:
            LOG.warning("Web平台以SIMULATION模式启动；命令仅进入内存模拟器")
            return False
        return self.ros.connect()

    def _trajectory_feedback_snapshot(self) -> Dict[str, Any]:
        if self.simulation:
            with self._simulation_lock:
                return self._simulation_snapshot()
        if self.offline:
            return self._offline_snapshot()
        return self.ros.telemetry_snapshot()

    def _execute_trajectory_points(
        self, points: List[Dict[str, Any]], progress_callback: Any,
        cancel_event: threading.Event,
    ) -> Dict[str, Any]:
        if self.simulation:
            positions = [np.asarray(point["positions"], dtype=float) for point in points]
            times = [float(point["time_from_start"]) for point in points]
            coefficients = trajectory_coefficients(points)
            with self._simulation_lock:
                if cancel_event.is_set():
                    return {"cancelled": True, "status": "CANCELLED_BEFORE_SEND"}
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                start_error = float(np.max(np.abs(
                    self._simulation_joints - positions[0]
                )))
                if start_error > float(self.config.get("joint_control", {}).get(
                        "trajectory_start_tolerance_rad", 0.05)):
                    raise RuntimeError(
                        "当前关节与轨迹起点误差{:.4f}rad过大".format(start_error)
                    )
                self._simulation_operator_state = RobotState.JOINT_CONTROL.value
                self._simulation_stop_reason = ""
            started = time.monotonic()
            progress_index = 0
            try:
                while True:
                    if cancel_event.is_set():
                        with self._simulation_lock:
                            self._simulation_target = self._simulation_joints.copy()
                            if self._simulation_operator_state != RobotState.SAFE_STOP.value:
                                self._simulation_operator_state = RobotState.IDLE.value
                        return {"cancelled": True, "status": "PREEMPTED"}
                    if not self.sessions.snapshot().get("active"):
                        with self._simulation_lock:
                            self._simulation_target = self._simulation_joints.copy()
                            if self._simulation_operator_state != RobotState.SAFE_STOP.value:
                                self._simulation_operator_state = RobotState.IDLE.value
                            self._simulation_stop_reason = "web control lease expired"
                        return {"cancelled": True, "status": "LEASE_EXPIRED"}
                    elapsed = time.monotonic() - started
                    segment = min(
                        max(0, int(np.searchsorted(times, elapsed, side="right") - 1)),
                        len(points) - 2,
                    )
                    interval = max(1e-9, times[segment + 1] - times[segment])
                    local_time = min(interval, max(0.0, elapsed - times[segment]))
                    desired = evaluate_quintic(coefficients[segment], local_time)["positions"]
                    with self._simulation_lock:
                        if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                            self._simulation_target = self._simulation_joints.copy()
                            return {"cancelled": True, "status": "SAFE_STOP"}
                        if cancel_event.is_set():
                            continue
                        self._simulation_target = desired
                        feedback = self._simulation_snapshot()
                        actual = np.asarray(
                            [feedback["joints"][name] for name in JOINT_NAMES], dtype=float
                        )
                    progress = self.joint_adapter.feedback_progress(
                        actual, positions, progress_index,
                        int(np.searchsorted(times, elapsed, side="right") - 1),
                    )
                    progress_index = int(progress["current_point"])
                    progress_callback(progress)
                    final_error = float(np.max(np.abs(actual - positions[-1])))
                    if elapsed >= times[-1] and final_error <= 0.01:
                        with self._simulation_lock:
                            if cancel_event.is_set() or self._simulation_operator_state == RobotState.SAFE_STOP.value:
                                continue
                            self._simulation_target = positions[-1].copy()
                            self._simulation_operator_state = RobotState.IDLE.value
                        return {"cancelled": False, "status": "SUCCEEDED",
                                "max_error_rad": final_error}
                    if elapsed > times[-1] + 5.0:
                        raise RuntimeError("SIMULATION轨迹跟踪超时")
                    time.sleep(0.01)
            except Exception as error:
                with self._simulation_lock:
                    self._simulation_target = self._simulation_joints.copy()
                    self._simulation_operator_state = RobotState.SAFE_STOP.value
                    self._simulation_stop_reason = str(error)
                raise
        if self.offline:
            raise RuntimeError("OFFLINE模式禁止执行轨迹")
        return self._run_on_control_path(
            "joint_control", ("base",),
            lambda: self.tasks.execute_trajectory(points, progress_callback, cancel_event),
        )

    def _stop_trajectory_execution(self) -> Dict[str, Any]:
        if self.simulation:
            with self._simulation_lock:
                self._simulation_target = self._simulation_joints.copy()
            return {"cancel_requested": True, "simulation": True}
        return self.tasks.cancel_joint_control()

    def next_sequence(self) -> int:
        with self._sequence_lock:
            self._sequence += 1
            return self._sequence

    def _handle_cartesian_executor_exit(self) -> None:
        """Latch SAFE_STOP and make one ACK-checked driver stop attempt."""

        self.state_machine.fail(
            RuntimeError("Cartesian executor exited"), "笛卡尔点动执行器")
        if self._cartesian_exit_stop_attempted:
            return
        self._cartesian_exit_stop_attempted = True
        try:
            self.ros.emergency_stop()
        except Exception as error:
            self.state_machine.fail(
                RuntimeError(
                    "Cartesian executor exited; STOP_UNCONFIRMED: {}".format(error)),
                "笛卡尔点动执行器",
            )

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
        if not self.simulation and not self.offline:
            jog = raw.get("cartesian_jog") or {}
            jog_state = str(jog.get("state", "IDLE"))
            if (jog_state == "SAFE_STOP" and
                    self.state_machine.state != RobotState.SAFE_STOP):
                self.state_machine.fail(
                    RuntimeError(str(jog.get("stop_reason") or "Cartesian executor SAFE_STOP")),
                    "笛卡尔点动执行器",
                )
            elif (jog_state == "STOPPED" and
                  self.state_machine.state == RobotState.JOINT_CONTROL and
                  self.active_control_path == "cartesian_jog"):
                self.state_machine.transition(RobotState.IDLE, "笛卡尔点动停稳")
            elif (self.active_control_path == "cartesian_jog" and
                  self.state_machine.state == RobotState.JOINT_CONTROL and
                  not raw["subsystems"].get("base", {}).get("running", False)):
                self._handle_cartesian_executor_exit()
            raw["operator_state"] = self.state_machine.state.value
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
            "tool5_drill": bool(self.config.get("tool5_drill_authorized", False)),
            "trajectory": bool(self.config.get("trajectory_execution_authorized", False)),
            "qr": bool(self.config.get("qr_execution_authorized", False)),
            "supervised": bool(self.config.get("supervised_commissioning_authorized", False)),
            "sampling_hardware": bool(self.config.get("sampling_hardware_authorized", False)),
            "unified_control": self.unified_control_authorized,
        }
        raw["active_control_path"] = self.active_control_path
        raw["available_control_paths"] = (
            ["observe", "joint_control", "cartesian_jog", "z_force", "pose_pickup", "qr_pickup",
             "tool5_drill"]
            if self.unified_control_authorized else [self.active_control_path]
        )
        raw["sampling_hardware"] = self.hardware.snapshot()
        raw["tool5_drill"] = self.tool5_drill.snapshot()
        raw["trajectory_execution"] = self.trajectories.execution_status()
        if self.simulation:
            raw["cartesian_jog"] = dict(self._simulation_jog)
        else:
            raw.setdefault("cartesian_jog", self._empty_jog_status())
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
            self._advance_simulation_jog(now_mono)
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
            if self._simulation_jog.get("state") != "IDLE":
                self._simulation_jog["actual_pose"] = self._matrix_pose(tcp_matrix)
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
                "tcp": self._matrix_pose(tcp_matrix),
                "fresh": {"joints": True, "wrench": True, "tcp": True,
                          "image": self._simulation_subsystems["camera"]},
                "ages": {"joints": 0.0, "wrench": 0.0, "tcp": 0.0, "image": 0.0},
                "vision_status": dict(self._simulation_qr_status["vision"]),
                "task_status": {"mode": "simulation"},
                "five_target_status": dict(self._simulation_qr_status),
                "five_targets": dict(self._simulation_five_targets),
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
                "tool5_drill_state": "IDLE",
                "tool5_drill_actual_depth_m": 0.0,
                "stop_reason": self._simulation_stop_reason,
                "tool_voltage": self._simulation_voltage,
                "image_topic": "/simulation/camera/color/image_raw",
                "source_stamps": {"joints": now, "tcp": now, "wrench": now, "image": now},
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

    def forward_kinematics(self, values: Iterable[float]) -> Dict[str, Any]:
        vector = self.robot.validate_vector(values)
        return {"joints": vector.tolist(), "tcp": self.robot.tcp(vector)}

    def inverse_kinematics(self, pose: Dict[str, Any]) -> Dict[str, Any]:
        """Bounded numerical IK used for preview; execution still uses the guarded joint path."""
        snap = self.snapshot()
        seed = np.asarray([snap.get("joints", {}).get(name, 0.0)
                           for name in JOINT_NAMES], dtype=float)
        return self._solve_inverse_kinematics(pose, seed)

    def _solve_inverse_kinematics(
            self, pose: Dict[str, Any], seed: np.ndarray) -> Dict[str, Any]:
        from scipy.optimize import least_squares
        keys = ("x", "y", "z", "rx", "ry", "rz")
        try:
            wanted = np.asarray([float(pose[key]) for key in keys], dtype=float)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("需要有限的x/y/z/rx/ry/rz（米/弧度）") from error
        if not np.isfinite(wanted).all():
            raise ValueError("笛卡尔目标不能包含NaN或无穷值")
        lower = np.asarray([self.robot.limits[name][0] for name in JOINT_NAMES])
        upper = np.asarray([self.robot.limits[name][1] for name in JOINT_NAMES])
        seed = np.clip(np.asarray(seed, dtype=float), lower, upper)

        wanted_rotation = (self._rotation(2, wanted[5]).dot(
            self._rotation(1, wanted[4])).dot(self._rotation(0, wanted[3])))

        def rotation_vector(matrix: np.ndarray) -> np.ndarray:
            cosine = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) / 2.0))
            angle = math.acos(cosine)
            vee = np.asarray([matrix[2, 1] - matrix[1, 2],
                              matrix[0, 2] - matrix[2, 0],
                              matrix[1, 0] - matrix[0, 1]], dtype=float)
            if angle < 1e-8:
                return 0.5 * vee
            return angle * vee / (2.0 * math.sin(angle))

        def residual(joints: np.ndarray) -> np.ndarray:
            matrix = self.robot.forward_kinematics(joints)["Link7"]
            rotation_error = rotation_vector(
                wanted_rotation.T.dot(matrix[:3, :3]))
            return np.concatenate(((matrix[:3, 3] - wanted[:3]) * 4.0,
                                   rotation_error))

        result = least_squares(residual, seed, bounds=(lower, upper),
                               max_nfev=240, xtol=1e-8, ftol=1e-8, gtol=1e-8)
        error = residual(result.x)
        position_error = float(np.linalg.norm(error[:3]) / 4.0)
        angle_error = float(np.linalg.norm(error[3:]))
        # SciPy may report max-function-evaluations even after the bounded
        # solution has reached the requested pose.  The safety contract is the
        # final finite residual plus the explicit joint bounds above, not the
        # optimizer's termination wording.  Reject only an inaccurate result.
        if (not math.isfinite(position_error) or not math.isfinite(angle_error) or
                position_error > 0.006 or angle_error > 0.08):
            raise ValueError("IK未收敛：位置误差{:.1f}mm，姿态误差{:.1f}°".format(
                position_error * 1000.0, angle_error * 180.0 / math.pi))
        if not result.success:
            LOG.warning(
                "IK优化器状态=%s，但最终误差已达标：位置=%.3fmm 姿态=%.3fdeg",
                result.status, position_error * 1000.0,
                angle_error * 180.0 / math.pi)
        return {"ok": True, "joints": result.x.tolist(),
                "tcp": self.robot.tcp(result.x),
                "position_error_m": position_error, "angle_error_rad": angle_error,
                "optimizer_success": bool(result.success),
                "optimizer_status": int(result.status)}

    def execute_cartesian(self, session_id: str, pose: Dict[str, Any]) -> Dict[str, Any]:
        self.require_session(session_id)
        solution = self.inverse_kinematics(pose)
        result = self.execute_joints(session_id, solution["joints"])
        return {"ok": True, "ik": solution, "execution": result}

    def start_cartesian_jog(self, session_id: str, direction: str) -> Dict[str, Any]:
        self.require_session(session_id)
        allowed = {
            "x_plus", "x_minus", "y_plus", "y_minus", "z_plus", "z_minus",
            "rx_plus", "rx_minus", "ry_plus", "ry_minus", "rz_plus", "rz_minus",
        }
        if direction not in allowed:
            raise ValueError("仅允许固定的十二个笛卡尔点动方向")

        def begin() -> Dict[str, Any]:
            if self.simulation:
                with self._simulation_lock:
                    if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                        raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                    if self._simulation_jog.get("state") in {
                            "JOGGING", "STOPPING", "LIMIT_REACHED"}:
                        raise RuntimeError("已有点动方向激活；必须先松开")
                    if self._simulation_control_launch_mode not in (None, "cartesian_jog"):
                        raise RuntimeError("其他运动控制源正在运行")
                    start = self.robot.forward_kinematics(
                        self._simulation_joints)["Link7"].copy()
                    target = start.copy()
                    rotating = direction.startswith("r")
                    axis = {"x": 0, "y": 1, "z": 2}[
                        direction[1] if rotating else direction[0]]
                    sign = 1.0 if direction.endswith("plus") else -1.0
                    if rotating:
                        target[:3, :3] = start[:3, :3].dot(self._rotation(
                            axis, sign * self._jog_limits["rotation_rad"]))
                    else:
                        target[axis, 3] += sign * self._jog_limits["translation_m"]
                    # Reject unreachable directions before reporting STARTED.
                    solution = self._solve_inverse_kinematics(
                        self._matrix_pose(target), self._simulation_joints.copy())
                    self._simulation_jog_start_matrix = start
                    self._simulation_jog_target_matrix = target
                    self._simulation_jog_start_joints = self._simulation_joints.copy()
                    self._simulation_jog_target_joints = np.asarray(
                        solution["joints"], dtype=float)
                    self._simulation_jog_started = time.monotonic()
                    self._simulation_jog = self._empty_jog_status()
                    self._simulation_jog.update({
                        "state": "JOGGING", "direction": direction,
                        "actual_pose": self._matrix_pose(start),
                        "target_pose": self._matrix_pose(target),
                    })
                    self._simulation_operator_state = RobotState.JOINT_CONTROL.value
                    self._simulation_control_launch_mode = "cartesian_jog"
                return {"ok": True, "simulation": True,
                        "direction": direction, "status": dict(self._simulation_jog)}
            result = self.ros.start_cartesian_jog(direction)
            self._cartesian_exit_stop_attempted = False
            self.state_machine.transition(
                RobotState.JOINT_CONTROL, "开始受控笛卡尔点动 {}".format(direction))
            return result

        if self.simulation:
            self.active_control_path = "cartesian_jog"
            return begin()
        if not self.unified_control_authorized:
            if (self.active_control_path != "cartesian_jog" or
                    not self.config.get("cartesian_jog_execution_authorized", False)):
                raise RuntimeError("当前启动未授权笛卡尔点动控制路径")
            return begin()
        return self._run_on_control_path("cartesian_jog", ("base",), begin)

    def stop_cartesian_jog(self, session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if self.simulation:
            with self._simulation_lock:
                self._simulation_target = self._simulation_joints.copy()
                self._simulation_jog.update({
                    "state": "STOPPED", "direction": "",
                    "stop_reason": "operator release; simulated stationary feedback confirmed",
                })
                self._simulation_operator_state = RobotState.IDLE.value
                self._simulation_control_launch_mode = None
                return {"ok": True, "simulation": True, "stopped": True,
                        "status": dict(self._simulation_jog)}
        result = self.ros.stop_cartesian_jog()
        if self.state_machine.state == RobotState.JOINT_CONTROL:
            self.state_machine.transition(RobotState.IDLE, "笛卡尔点动停止已确认")
        return result

    def cartesian_jog_status(self) -> Dict[str, Any]:
        if self.simulation:
            with self._simulation_lock:
                return dict(self._simulation_jog)
        return dict(self.ros.telemetry_snapshot().get(
            "cartesian_jog", self._empty_jog_status()))

    def _ui_path(self, name: str) -> Path:
        return Path(str(self.config["config_path"])).resolve().parent / name

    def poses(self) -> List[Dict[str, Any]]:
        path = self._ui_path("poses.yaml")
        if not path.exists():
            return []
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        return list(data.get("poses") or [])

    def save_pose(self, index: int, payload: Dict[str, Any]) -> Dict[str, Any]:
        with self._ui_config_lock:
            poses = self.poses()
            if index < 0 or index >= len(poses):
                raise ValueError("预设姿态索引不存在")
            values = {key: float(payload[key]) for key in ("x", "y", "z", "rx", "ry", "rz")}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError("预设姿态必须是有限数字")
            values["name"] = str(payload.get("name") or poses[index].get("name", "姿态"))[:40]
            poses[index] = values
            self._atomic_yaml(self._ui_path("poses.yaml"), {"poses": poses})
            return values

    def settings(self) -> Dict[str, Any]:
        defaults = {"planner": "ompl", "speed": 5}
        path = self._ui_path("ui_settings.yaml")
        try:
            with path.open(encoding="utf-8") as stream:
                control = (yaml.safe_load(stream) or {}).get("control", {})
        except OSError:
            control = {}
        planner = str(control.get("planner", defaults["planner"]))
        speed = int(control.get("speed", defaults["speed"]))
        return {"planner": planner if planner in {"ompl", "lin", "ptp", "direct"} else "ompl",
                "speed": speed if speed in {5, 20, 30, 50, 100} else 5}

    def save_settings(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        current = self.settings()
        planner = str(payload.get("planner", current["planner"]))
        speed = int(payload.get("speed", current["speed"]))
        if planner not in {"ompl", "lin", "ptp", "direct"}:
            raise ValueError("不支持的规划器")
        if speed not in {5, 20, 30, 50, 100}:
            raise ValueError("速度档必须是5/20/30/50/100")
        with self._ui_config_lock:
            self._atomic_yaml(self._ui_path("ui_settings.yaml"),
                              {"control": {"planner": planner, "speed": speed}})
        return {"planner": planner, "speed": speed}

    @staticmethod
    def _atomic_yaml(path: Path, payload: Dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(payload, stream, allow_unicode=True, sort_keys=False)
        os.replace(str(temporary), str(path))

    def require_session(self, session_id: str) -> None:
        if self.offline:
            raise RuntimeError("OFFLINE模式禁止控制命令")
        self.sessions.require(session_id)

    def set_control_path(self, session_id: str, path: str) -> Dict[str, Any]:
        """Compatibility API: safely activate one path and restore owned systems."""

        self.require_session(session_id)
        return self._activate_control_path(path)

    def _activate_control_path(
            self, path: str, required_systems: Iterable[str] = ()) -> Dict[str, Any]:
        """Select the exclusive motion chain needed by a user action.

        Unified control authorizes every web feature, but the ROS motion command
        sources remain mutually exclusive.  A path change therefore stops only
        HMI-owned launch groups, changes the base composition while fully stopped,
        and restarts the previous/required groups in dependency order.
        """

        if not self.unified_control_authorized:
            raise RuntimeError("当前启动未授予网页全部控制功能")
        if path not in {"observe", "joint_control", "cartesian_jog", "z_force", "pose_pickup",
                        "qr_pickup", "tool5_drill"}:
            raise ValueError("不支持的控制路径：{}".format(path))
        required = set(required_systems)
        invalid = required.difference({"base", "camera", "qr", "pose"})
        if invalid:
            raise ValueError("不支持的子系统：{}".format(", ".join(sorted(invalid))))

        if self.simulation:
            self.active_control_path = path
            return {"control_path": path, "simulation": True}

        with self._control_path_lock:
            statuses = self.tasks.subsystem_status()
            if path == self.active_control_path:
                started = []
                for name in ("base", "camera", "qr", "pose"):
                    if name in required and not statuses.get(name, {}).get("running"):
                        self.tasks.start_subsystem(name)
                        started.append(name)
                return {"control_path": path, "changed": False,
                        "started_systems": started}

            machine = getattr(self, "state_machine", None)
            machine_state = getattr(machine, "state", None)
            if machine_state is not None and machine_state != RobotState.IDLE:
                raise RuntimeError(
                    "当前任务状态为{}，结束任务后才能自动切换控制链路".format(
                        getattr(machine_state, "value", machine_state)
                    )
                )
            control_mode = getattr(self.tasks, "control_mode", None)
            if control_mode is not None:
                raise RuntimeError(
                    "控制源{}正在运行，请先停止后再使用其他功能".format(control_mode)
                )
            drill_state = str(self.tool5_drill.snapshot().get("state", "IDLE"))
            motor_only_force_switch = (
                bool(getattr(self.tool5_drill, "motor_only", False)) and
                path == "z_force" and drill_state in ACTIVE_STATES
            )
            if (drill_state not in {"IDLE", "COMPLETED"} and
                    not motor_only_force_switch):
                raise RuntimeError(
                    "Tool5任务{}未安全结束，不能自动切换控制链路".format(drill_state)
                )

            external = [name for name, status in statuses.items()
                        if status.get("running") and not status.get("owned")]
            if external:
                raise RuntimeError(
                    "检测到非网页管理的运行子系统，不能自动切换：{}".format(
                        ", ".join(sorted(external))
                    )
                )
            restore = {name for name, status in statuses.items()
                       if status.get("owned")}
            previous_path = self.active_control_path
            stopped = []
            started = []
            path_changed = False
            try:
                for name in ("pose", "qr", "camera", "base"):
                    if name in restore:
                        self.tasks.stop_subsystem(name)
                        stopped.append(name)
                result = self.tasks.set_control_path(path)
                path_changed = True
                self.active_control_path = path
                desired = restore.union(required)
                if path == "pose_pickup":
                    desired = set(required)
                else:
                    # QR and saved-pose launches own the same motion-service
                    # node and must never be restarted together.
                    desired.discard("pose")
                for name in ("base", "camera", "qr", "pose"):
                    # The saved-pose workflow is intentionally isolated from
                    # camera/QR processes, even if they were running earlier.
                    if name in desired:
                        self.tasks.start_subsystem(name)
                        started.append(name)
            except Exception as error:
                if path_changed:
                    for name in reversed(started):
                        try:
                            self.tasks.stop_subsystem(name)
                        except Exception:
                            LOG.exception("自动切换失败后的子系统清理失败：%s", name)
                    recovery_started = []
                    recovered = False
                    try:
                        self.tasks.set_control_path(previous_path)
                        self.active_control_path = previous_path
                        recovery_desired = set(restore)
                        if previous_path != "pose_pickup":
                            recovery_desired.discard("pose")
                        for name in ("base", "camera", "qr", "pose"):
                            if name in recovery_desired:
                                self.tasks.start_subsystem(name)
                                recovery_started.append(name)
                        recovered = True
                    except Exception:
                        LOG.exception("自动切换失败后无法恢复原控制链路%s", previous_path)
                        for name in reversed(recovery_started):
                            try:
                                self.tasks.stop_subsystem(name)
                            except Exception:
                                LOG.exception("原控制链路恢复失败后的清理失败：%s", name)
                        try:
                            self.tasks.set_control_path("observe")
                            self.active_control_path = "observe"
                        except Exception:
                            LOG.exception("自动切换失败后无法回退到观察模式")
                else:
                    for name in ("base", "camera", "qr", "pose"):
                        if name in stopped:
                            try:
                                self.tasks.start_subsystem(name)
                            except Exception:
                                LOG.exception("切换前停止失败后无法恢复子系统：%s", name)
                if path_changed and recovered:
                    raise RuntimeError(
                        "自动切换到{}失败，已恢复原控制链路{}：{}".format(
                            path, previous_path, error
                        )
                    ) from error
                raise RuntimeError(
                    "自动切换到{}失败，已停止新控制链路：{}".format(path, error)
                ) from error

            with self._subsystem_lock:
                self._subsystem_cache = {}
                self._subsystem_checked = 0.0
            LOG.warning("网页按功能自动切换控制链路：%s；重启=%s",
                        path, ",".join(started) or "无")
            result.update({"changed": True, "stopped_systems": stopped,
                           "started_systems": started})
            return result

    def _run_on_control_path(self, path: str, required_systems: Iterable[str],
                             callback: Any) -> Any:
        """Keep path activation and its immediate command one atomic operation."""

        with self._control_path_lock:
            self._activate_control_path(path, required_systems)
            return callback()

    def _tool5_telemetry(self) -> Dict[str, Any]:
        if self.simulation:
            return self._simulation_snapshot()
        if self.offline:
            return self._offline_snapshot()
        return self.ros.telemetry_snapshot()

    def _prepare_tool5_drill(self) -> Any:
        if self.simulation:
            with self._simulation_lock:
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                if self._simulation_control_launch_mode is not None:
                    raise RuntimeError("已有控制源占用机械臂")
                self._simulation_operator_state = RobotState.FORCE_CONTROL.value
                self._simulation_control_launch_mode = "tool5_drill"
                self._simulation_stop_reason = ""
            return {"simulation": True, "prepared": True}
        return self.tasks.prepare_tool5_drill()

    def _engage_tool5_drill(self, depth_m: float) -> Any:
        if self.simulation:
            return {"simulation": True, "depth_m": float(depth_m)}
        return self.tasks.engage_tool5_drill(depth_m)

    def _hold_tool5_drill(self) -> Any:
        if self.simulation:
            with self._simulation_lock:
                self._simulation_target = self._simulation_joints.copy()
            return {"simulation": True, "holding": True}
        return self.tasks.hold_tool5_drill()

    def _safe_stop_tool5_drill(self) -> Any:
        if self.simulation:
            with self._simulation_lock:
                self._simulation_target = self._simulation_joints.copy()
                self._simulation_operator_state = RobotState.SAFE_STOP.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = "Tool5 drill SAFE_STOP"
            return {"simulation": True, "safe_stop": True}
        return self.tasks.emergency_stop()

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
        def execute() -> Any:
            self.experiment.begin_task()
            LOG.info("Web受控七轴目标请求已进入既有自动安全检查流程")
            return self.tasks.execute_joint_target(values)

        result = self._run_on_control_path("joint_control", ("base",), execute)
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
        if any(abs(values[key]) > 2.0 for key in ("x", "y", "z")):
            raise ValueError(
                "X/Y/Z超出2米安全配置范围；网页坐标请按毫米输入，禁止将毫米直接作为米提交"
            )
        if any(abs(values[key]) > 2.0 * math.pi for key in ("rx", "ry", "rz")):
            raise ValueError("Rx/Ry/Rz超出±2π；姿态单位为弧度")
        if not 0.0 < values["lift_height"] <= 0.5:
            raise ValueError("Lift Height必须在(0, 0.5]米范围内")
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
        return self._run_on_control_path(
            "pose_pickup", ("base", "pose"),
            lambda: self.tasks.plan_configured_tool(int(slot_id)),
        )

    def execute_configured_tool(self, slot_id: int, session_id: str) -> Dict[str, Any]:
        self.require_session(session_id)
        if self.simulation:
            self.dry_run_configured_tool(slot_id, session_id)
            with self._simulation_lock:
                self.tasks.selected_tool = "Tool{}".format(int(slot_id))
                self._simulation_voltage = 24
                self._simulation_operator_state = RobotState.IDLE.value
                self._simulation_pick_status = {
                    "status": "COMPLETED_UNVERIFIED", "step": 6,
                    "step_name": "运动完成；取物未独立确认", "slot_id": int(slot_id),
                    "grasp_confirmed": False}
            return {"executed": True, "simulation": True,
                    "status": "COMPLETED_UNVERIFIED", "grasp_confirmed": False}
        def execute() -> Any:
            self.experiment.begin_task()
            return self.tasks.execute_configured_tool(int(slot_id))

        return self._run_on_control_path(
            "pose_pickup", ("base", "pose"), execute
        )

    def command(self, name: str, session_id: str,
                payload: Optional[Dict[str, Any]] = None) -> Any:
        self.require_session(session_id)
        payload = payload or {}
        if name in {"start_tool5_drill", "start_tool5_rotation"}:
            if self.tool5_drill.motor_only:
                return self.tool5_drill.start()
            if not self.simulation:
                return self._run_on_control_path(
                    "tool5_drill", ("base",),
                    lambda: self.tool5_drill.start(
                        float(payload.get("depth_cm", 0.0)) / 100.0
                    ),
                )
            return self.tool5_drill.start(
                float(payload.get("depth_cm", 0.0)) / 100.0
            )
        if name == "stop_tool5_rotation":
            return self.tool5_drill.stop_rotation()
        if name == "reset_tool5_drill":
            self.tool5_drill.reset()
            return self.tool5_drill.snapshot()
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
            if action == "dig_continuous":
                raise ValueError("连续旋转仅允许由Tool5取土状态机启动")
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
                self.tool5_drill.emergency_stop()
                self.hardware.emergency_stop()
            elif name == "recover":
                self.tool5_drill.reset()
            return result
        if name == "emergency_stop":
            try:
                self.tool5_drill.emergency_stop(stop_arm=False)
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
        automatic_paths = {
            "start_teleop": ("z_force", ("base",)),
            "start_z_force_teleop": ("z_force", ("base",)),
            "enter_force_control": ("z_force", ("base",)),
            "detect_tool": ("qr_pickup", ("base", "camera", "qr")),
            "detect_marker": ("qr_pickup", ("base", "camera", "qr")),
            "compute_targets": ("qr_pickup", ("base", "camera", "qr")),
            "preview_tool": ("qr_pickup", ("base", "camera", "qr")),
            "plan_tool": ("qr_pickup", ("base", "camera", "qr")),
            "execute_tool": ("qr_pickup", ("base", "camera", "qr")),
            "return_tool": ("qr_pickup", ("base", "camera", "qr")),
        }
        automatic = automatic_paths.get(name)
        callback = commands.get(name)
        if callback is None:
            raise ValueError("不允许的命令：{}".format(name))
        LOG.info("Web白名单命令开始：%s", name)
        result = (self._run_on_control_path(automatic[0], automatic[1], callback)
                  if automatic is not None else callback())
        if name == "recover":
            self.tool5_drill.reset()
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
            "cancel_joints": RobotState.IDLE.value,
        }
        qr_commands = {"detect_tool", "detect_marker", "compute_targets",
                       "preview_tool", "plan_tool", "execute_tool",
                       "return_tool", "cancel_tool"}
        allowed = set(states).union(qr_commands, {
            "emergency_stop", "recover", "select_tool", "set_voltage"})
        if name not in allowed:
            raise ValueError("不允许的命令：{}".format(name))
        if name in {"emergency_stop", "cancel_joints"}:
            self.trajectories.stop()
        with self._simulation_lock:
            if name == "emergency_stop":
                self._simulation_target = self._simulation_joints.copy()
                self._simulation_operator_state = RobotState.SAFE_STOP.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = "SIMULATION software emergency stop"
                self._simulation_qr_plan_slot = None
                self._simulation_qr_status["staged_pickup_slot"] = None
                self._simulation_qr_status["step"] = "安全停止已锁存"
            elif name == "recover":
                self._simulation_operator_state = RobotState.IDLE.value
                self._simulation_control_launch_mode = None
                self._simulation_stop_reason = ""
                self._simulation_qr_status["step"] = (
                    "安全恢复；仍持有工具" if
                    self._simulation_qr_status["holding_slot"] is not None
                    else "安全恢复完成")
            elif name == "select_tool":
                tool = str(payload.get("tool", ""))
                if tool not in {"Tool{}".format(index) for index in range(1, 6)}:
                    raise ValueError("仿真工具必须是Tool1..Tool5")
                self.tasks.selected_tool = tool
                slot = int(tool.replace("Tool", ""))
                self._simulation_qr_status["selected_slot"] = slot
                if self._simulation_qr_plan_slot != slot:
                    self._simulation_qr_plan_slot = None
                    self._simulation_qr_status["staged_pickup_slot"] = None
            elif name == "set_voltage":
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                voltage = int(payload.get("voltage", -1))
                if voltage not in (0, 24):
                    raise ValueError("仿真工具电压仅允许0V或24V")
                self._simulation_voltage = voltage
            elif name in qr_commands:
                if self._simulation_operator_state == RobotState.SAFE_STOP.value:
                    raise RuntimeError("SIMULATION SAFE_STOP已锁存，必须先恢复")
                self._simulation_qr_command(name)
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
                "operator_state": self._simulation_operator_state,
                "five_target_status": dict(self._simulation_qr_status)}

    @staticmethod
    def _simulation_pose(position: Iterable[float], rpy: Iterable[float]) -> Dict[str, Any]:
        """Serialize an RPY pose exactly like the real five-target ROS topic."""
        x, y, z = [float(value) for value in position]
        roll, pitch, yaw = [float(value) for value in rpy]
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return {
            "translation": [x, y, z],
            "quaternion_xyzw": [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ],
        }

    def _simulation_targets(self) -> Dict[str, Any]:
        targets = []
        for tool in self.tools:
            configured = tool.get("configured_pick", {})
            pose = self._simulation_pose(
                configured.get("position_m", [0.0, 0.0, 0.0]),
                configured.get("rpy_rad", [0.0, 0.0, 0.0]))
            targets.append({
                "slot_id": int(tool["slot_id"]),
                "name": str(tool.get("name", "Tool{}".format(tool["slot_id"]))),
                "tcp": dict(pose), "link7": dict(pose),
            })
        return {
            "ok": True, "marker_id": 0, "marker_stamp": time.time(),
            "marker_age_s": 0.0, "calibration_version": "simulation-reference",
            "selected_slot": self._simulation_qr_status["selected_slot"],
            "target_pose_source": "simulation_reference_only",
            "targets": targets,
        }

    def _simulation_qr_command(self, name: str) -> None:
        status = self._simulation_qr_status
        slot = int(status["selected_slot"])
        if name in {"detect_tool", "detect_marker"}:
            status["vision"] = {
                "ok": True, "marker_id": 0, "stable": True,
                "stamp": time.time(), "source": "simulation_camera"}
            status["step"] = "二维码ID 0识别稳定（仿真）"
            self._simulation_operator_state = RobotState.TOOL_DETECT.value
        elif name == "compute_targets":
            if not status["vision"].get("ok"):
                raise RuntimeError("仿真二维码观测无效，不能计算目标")
            self._simulation_five_targets = self._simulation_targets()
            status["targets_valid"] = True
            status["step"] = "五个仿真目标已计算"
            self._simulation_operator_state = RobotState.TOOL_DETECT.value
        elif name in {"preview_tool", "plan_tool"}:
            if not status["targets_valid"]:
                raise RuntimeError("请先计算五个二维码目标")
            if status["holding_slot"] is not None:
                raise RuntimeError("已持有工具，不能规划新的取物动作")
            self._simulation_operator_state = RobotState.TOOL_PICK.value
            if name == "preview_tool":
                status["step"] = "Tool{}目标预览完成".format(slot)
            else:
                self._simulation_qr_plan_slot = slot
                status["staged_pickup_slot"] = slot
                status["step"] = "Tool{} Dry Run规划通过（无物理输出）".format(slot)
        elif name == "execute_tool":
            if self._simulation_qr_plan_slot != slot:
                raise RuntimeError("必须先为当前工具完成Dry Run规划")
            if status["holding_slot"] is not None:
                raise RuntimeError("已持有工具，不能再次取物")
            status["holding_slot"] = slot
            status["staged_pickup_slot"] = None
            status["step"] = (
                "Tool{}仿真运动完成；24V ACK已模拟，抓取未独立确认".format(slot))
            status["grasp_confirmed"] = False
            self._simulation_qr_plan_slot = None
            self._simulation_voltage = 24
            self._simulation_operator_state = RobotState.IDLE.value
        elif name == "return_tool":
            if status["holding_slot"] != slot:
                raise RuntimeError("只能归还当前持有的工具")
            status["holding_slot"] = None
            status["grasp_confirmed"] = False
            status["step"] = "Tool{}仿真归还完成；末端已断电".format(slot)
            self._simulation_voltage = 0
            self._simulation_operator_state = RobotState.IDLE.value
        elif name == "cancel_tool":
            self._simulation_qr_plan_slot = None
            status["staged_pickup_slot"] = None
            if status["holding_slot"] is None:
                self._simulation_operator_state = RobotState.IDLE.value
                status["step"] = "二维码任务已终止"
            else:
                self._simulation_operator_state = RobotState.IDLE.value
                status["step"] = "运动已终止；仍持有Tool{}".format(
                    status["holding_slot"])
        self._simulation_control_launch_mode = None

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
        self.tool5_drill.poll()
        active = bool(self.sessions.snapshot().get("active"))
        if self.simulation:
            with self._simulation_lock:
                if not active and self._simulation_jog.get("state") == "JOGGING":
                    self._simulation_target = self._simulation_joints.copy()
                    self._simulation_jog.update({
                        "state": "STOPPED", "direction": "",
                        "stop_reason": "web control lease expired; simulated stop confirmed",
                    })
                    self._simulation_operator_state = RobotState.IDLE.value
                    self._simulation_control_launch_mode = None
            return False
        if self.offline or not active:
            return False
        self.ros.publish_joint_control_heartbeat()
        self.ros.publish_cartesian_jog_heartbeat()
        return True

    def close(self) -> None:
        try:
            try:
                self.trajectories.stop()
            except Exception:
                LOG.exception("Web关闭时轨迹停止请求失败")
            if not self.offline and not self.simulation:
                try:
                    status = self.cartesian_jog_status()
                    if status.get("state") in {"JOGGING", "STOPPING", "LIMIT_REACHED"}:
                        self.ros.stop_cartesian_jog()
                except Exception:
                    LOG.exception("Web关闭时笛卡尔点动停止未确认")
            self.tool5_drill.emergency_stop("Web平台关闭")
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
