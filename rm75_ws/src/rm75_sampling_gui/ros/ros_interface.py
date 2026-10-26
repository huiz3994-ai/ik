"""Reviewed ROS1 boundary used by the RM75 sampling HMI.

This module only calls interfaces already provided by ``rm_driver``,
``rm75_qr_task`` and ``rm75_touch_teleop``. It never generates poses, joint
commands or trajectories. Command topics use a fresh acknowledgement barrier
so a stale result message cannot be mistaken for success.
"""

from dataclasses import dataclass
import json
import logging
import math
import threading
import time
from typing import Any, Dict, List, Optional

import rosnode
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Byte, Empty, Float64, String, UInt16
from rm_msgs.msg import Plan_State

from rm75_qr_task.srv import TaskCommand, TaskCommandRequest
from .monitoring import Monitor


LOGGER = logging.getLogger(__name__)


class ROSInterfaceError(RuntimeError):
    """Raised when an existing ROS endpoint is missing or rejects a request."""


@dataclass(frozen=True)
class InterfaceResult:
    """Compatibility result for the generic facade methods."""

    operation: str
    dispatched: bool
    message: str


class ROSInterface:
    """Thread-safe facade over the existing, reviewed RM75 ROS endpoints."""

    VOLTAGE_PROTOCOL = {0: 0, 24: 3}

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = dict(config)
        namespace = str(config.get("ros_namespace", "/rm75_sampling_gui"))
        self.namespace = self._normalize_namespace(namespace)
        self.node_name = str(config.get("ros_node_name", "rm75_sampling_gui")).strip()
        self.ack_timeout = float(config.get("ack_timeout_s", 5.0))
        self.service_timeout = float(config.get("service_timeout_s", 5.0))
        self.feedback_max_age = float(config.get("feedback_max_age_s", 1.0))

        self._connected = False
        self._lock = threading.RLock()
        self._ack_condition = threading.Condition(self._lock)
        self._ack: Dict[str, Dict[str, Any]] = {
            "voltage": self._empty_ack(),
            "emergency": self._empty_ack(),
            "clear_error": self._empty_ack(),
        }
        self._telemetry: Dict[str, Any] = {
            "teleop_active": False,
            "control_mode": "UNKNOWN",
            "stop_reason": "",
            "force_z": None,
            "filtered_force_z": None,
            "force_setpoint": None,
            "haptic_enabled": False,
            "haptic_force_fresh": False,
            "haptic_command": {"x": 0.0, "y": 0.0, "z": 0.0},
            "tool5_drill_state": "IDLE",
            "tool5_drill_actual_depth_m": 0.0,
            "tool_voltage": None,
            "voltage_commanded": None,
            "voltage_command_wall_time": 0.0,
            "force_wall_time": 0.0,
            "joint_wall_time": 0.0,
            "vision_status": {},
            "vision_wall_time": 0.0,
            "task_status": {},
            "task_wall_time": 0.0,
            "driver_plan_ok": None,
            "driver_arm_error": None,
            "driver_system_error": None,
        }
        self._publishers: List[Any] = []
        self._subscribers: List[Any] = []
        self._voltage_pub: Optional[Any] = None
        self._emergency_pub: Optional[Any] = None
        self._clear_error_pub: Optional[Any] = None
        self._joint_heartbeat_pub: Optional[Any] = None
        self._tool5_drill_start_pub: Optional[Any] = None
        self._tool5_drill_stop_pub: Optional[Any] = None
        self.monitor = Monitor(config)

    @staticmethod
    def _empty_ack() -> Dict[str, Any]:
        return {"sequence": 0, "value": None, "wall_time": 0.0}

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        value = namespace.strip() or "/rm75_sampling_gui"
        if not value.startswith("/"):
            value = "/" + value
        return value.rstrip("/") or "/"

    @property
    def connected(self) -> bool:
        if self.monitor.thread is not None:
            return bool(self.monitor.snapshot()["health"].get("master"))
        with self._lock:
            return self._connected

    def connect(self) -> bool:
        """Initialize the GUI node, publishers and read-only telemetry feeds."""

        try:
            if not rospy.core.is_initialized():
                rospy.init_node(self.node_name, disable_signals=True)
            rospy.get_master().getPid()
            with self._lock:
                if not self._publishers and not self._subscribers:
                    self._bind_endpoints()
                self._connected = True
            self.monitor.start()
        except Exception as error:
            with self._lock:
                self._connected = False
            LOGGER.warning("ROS Master/接口连接失败：%s", error)
            return False
        LOGGER.info("ROS接口已绑定，namespace=%s", self.namespace)
        return True

    def _bind_endpoints(self) -> None:
        self._voltage_pub = rospy.Publisher(
            "/rm_driver/SetToolVoltage", Byte, queue_size=1
        )
        self._emergency_pub = rospy.Publisher(
            "/rm_driver/Emergency_Stop", Empty, queue_size=1
        )
        self._clear_error_pub = rospy.Publisher(
            "/rm_driver/Clear_System_Err", Empty, queue_size=1
        )
        self._joint_heartbeat_pub = rospy.Publisher(
            "/rm75_sampling_gui/joint_control_heartbeat", Empty, queue_size=1
        )
        self._tool5_drill_start_pub = rospy.Publisher(
            "/rm75_vertical_press/drill_start_depth_m", Float64, queue_size=1
        )
        self._tool5_drill_stop_pub = rospy.Publisher(
            "/rm75_vertical_press/drill_stop_feed", Empty, queue_size=1
        )
        self._publishers = [
            self._voltage_pub,
            self._emergency_pub,
            self._clear_error_pub,
            self._joint_heartbeat_pub,
            self._tool5_drill_start_pub,
            self._tool5_drill_stop_pub,
        ]
        self._subscribers = [
            rospy.Subscriber(
                "/rm_driver/Set_Tool_Voltage_Result",
                Bool,
                lambda message: self._ack_callback("voltage", message),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm_driver/SetToolVoltage",
                Byte,
                self._voltage_command_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm_driver/Set_Arm_Stop_Result",
                Bool,
                lambda message: self._ack_callback("emergency", message),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm_driver/System_En_State_Result",
                Bool,
                lambda message: self._ack_callback("clear_error", message),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/active",
                Bool,
                lambda message: self._set_telemetry("teleop_active", bool(message.data)),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/control_mode",
                String,
                lambda message: self._set_telemetry("control_mode", message.data),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/stop_reason",
                String,
                lambda message: self._set_telemetry("stop_reason", message.data),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/force_z",
                Float64,
                lambda message: self._force_callback("force_z", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/filtered_force_z",
                Float64,
                lambda message: self._force_callback("filtered_force_z", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/force_setpoint",
                Float64,
                lambda message: self._force_callback("force_setpoint", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/haptic_enabled",
                Bool,
                lambda message: self._set_telemetry(
                    "haptic_enabled", bool(message.data)),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/haptic_force_fresh",
                Bool,
                lambda message: self._set_telemetry(
                    "haptic_force_fresh", bool(message.data)),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_touch_teleop/haptic_device_command",
                Vector3Stamped,
                self._haptic_command_callback,
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/active",
                Bool,
                lambda message: self._set_telemetry("teleop_active", bool(message.data)),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/control_mode",
                String,
                lambda message: self._set_telemetry("control_mode", message.data),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/stop_reason",
                String,
                lambda message: self._set_telemetry("stop_reason", message.data),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/drill_state",
                String,
                lambda message: self._set_telemetry(
                    "tool5_drill_state", message.data),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/drill_actual_depth_m",
                Float64,
                lambda message: self._set_telemetry(
                    "tool5_drill_actual_depth_m", float(message.data)),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/force_z",
                Float64,
                lambda message: self._force_callback("force_z", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/filtered_force_z",
                Float64,
                lambda message: self._force_callback("filtered_force_z", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/rm75_vertical_press/force_setpoint",
                Float64,
                lambda message: self._force_callback("force_setpoint", message),
                queue_size=1,
            ),
            rospy.Subscriber(
                "/qr_task/vision_status",
                String,
                lambda message: self._json_callback("vision_status", message),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/qr_task/task_status",
                String,
                lambda message: self._json_callback("task_status", message),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/joint_states", JointState, self._joint_callback, queue_size=1
            ),
            rospy.Subscriber(
                "/rm_driver/Plan_State", Plan_State,
                lambda message: self._driver_status("driver_plan_ok", bool(message.state)),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm_driver/ArmError", UInt16,
                lambda message: self._driver_status("driver_arm_error", int(message.data)),
                queue_size=10,
            ),
            rospy.Subscriber(
                "/rm_driver/SysError", UInt16,
                lambda message: self._driver_status("driver_system_error", int(message.data)),
                queue_size=10,
            ),
        ]

    def _driver_status(self, key: str, value: Any) -> None:
        self.monitor.put(key, value)
        with self._lock:
            self._telemetry[key] = value

    def _ack_callback(self, channel: str, message: Bool) -> None:
        with self._ack_condition:
            now = time.monotonic()
            record = self._ack[channel]
            record["sequence"] += 1
            record["value"] = bool(message.data)
            record["wall_time"] = now
            if (
                channel == "voltage"
                and record["value"]
                and self._telemetry["voltage_command_wall_time"] <= now
                and now - self._telemetry["voltage_command_wall_time"]
                <= self.ack_timeout
            ):
                self._telemetry["tool_voltage"] = self._telemetry[
                    "voltage_commanded"
                ]
            self._ack_condition.notify_all()
        LOGGER.log(logging.INFO if message.data else logging.ERROR,
                   "ACK channel=%s accepted=%s sequence=%s",
                   channel, bool(message.data), record["sequence"])

    def _voltage_command_callback(self, message: Byte) -> None:
        inverse_protocol = {
            value: voltage for voltage, value in {0: 0, 12: 2, 24: 3}.items()
        }
        with self._lock:
            self._telemetry["voltage_commanded"] = inverse_protocol.get(
                int(message.data), "UNKNOWN"
            )
            self._telemetry["voltage_command_wall_time"] = time.monotonic()

    def _set_telemetry(self, key: str, value: Any) -> None:
        with self._lock:
            self._telemetry[key] = value

    def _force_callback(self, key: str, message: Float64) -> None:
        if not math.isfinite(float(message.data)):
            return
        self.monitor.put(key, float(message.data))
        with self._lock:
            self._telemetry[key] = float(message.data)
            if key in ("force_z", "filtered_force_z"):
                self._telemetry["force_wall_time"] = time.monotonic()

    def _haptic_command_callback(self, message: Vector3Stamped) -> None:
        values = (float(message.vector.x), float(message.vector.y),
                  float(message.vector.z))
        if not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            self._telemetry["haptic_command"] = {
                "x": values[0], "y": values[1], "z": values[2]
            }

    def _joint_callback(self, _message: JointState) -> None:
        self.monitor.joints(_message)
        with self._lock:
            self._telemetry["joint_wall_time"] = time.monotonic()

    def _json_callback(self, key: str, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError("JSON root is not an object")
        except Exception as error:
            LOGGER.warning("忽略无效%s JSON：%s", key, error)
            return
        with self._lock:
            self._telemetry[key] = value
            self._telemetry[key.replace("status", "wall_time")] = time.monotonic()
        self.monitor.put(key, value)

    def _publish_and_wait_ack(
        self, channel: str, publisher: Any, message: Any, description: str
    ) -> Dict[str, Any]:
        if not self.connected or publisher is None:
            raise ROSInterfaceError("ROS接口未连接")
        deadline = time.monotonic() + self.ack_timeout
        while publisher.get_num_connections() == 0:
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise ROSInterfaceError("{}命令topic无订阅者".format(description))
            time.sleep(0.05)

        with self._ack_condition:
            baseline = int(self._ack[channel]["sequence"])
            sent_at = time.monotonic()
            publisher.publish(message)
            while True:
                record = self._ack[channel]
                if (
                    int(record["sequence"]) > baseline
                    and float(record["wall_time"]) >= sent_at
                ):
                    if not record["value"]:
                        raise ROSInterfaceError("控制器拒绝{}命令".format(description))
                    return {
                        "ack": True,
                        "sequence": int(record["sequence"]),
                        "description": description,
                        "note": "ACK表示命令被控制器接受，不等同于外部物理量测量",
                    }
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ROSInterfaceError("{}新鲜ACK超时".format(description))
                self._ack_condition.wait(remaining)

    def set_tool_voltage(self, voltage: int) -> Dict[str, Any]:
        voltage = int(voltage)
        if voltage not in self.VOLTAGE_PROTOCOL:
            raise ROSInterfaceError("仅允许0V或24V，收到{}V".format(voltage))
        result = self._publish_and_wait_ack(
            "voltage",
            self._voltage_pub,
            Byte(data=self.VOLTAGE_PROTOCOL[voltage]),
            "工具电压{}V".format(voltage),
        )
        result["voltage"] = voltage
        with self._lock:
            self._telemetry["tool_voltage"] = voltage
        LOGGER.info("工具电压%dV已收到新鲜ACK", voltage)
        return result

    def emergency_stop(self) -> Dict[str, Any]:
        result = self._publish_and_wait_ack(
            "emergency", self._emergency_pub, Empty(), "软件急停"
        )
        LOGGER.warning("软件急停已收到新鲜ACK")
        return result

    def clear_system_error(self) -> Dict[str, Any]:
        result = self._publish_and_wait_ack(
            "clear_error", self._clear_error_pub, Empty(), "清除控制器错误"
        )
        LOGGER.info("控制器错误清除已收到新鲜ACK")
        return result

    def publish_joint_control_heartbeat(self) -> None:
        """Non-motion liveness signal consumed by the guarded joint adapter."""

        publisher = self._joint_heartbeat_pub
        if self.connected and publisher is not None:
            publisher.publish(Empty())

    def start_tool5_drill(self, depth_m: float) -> None:
        value = float(depth_m)
        if not math.isfinite(value) or value <= 0.0:
            raise ROSInterfaceError("Tool5下降距离必须为有限正数")
        publisher = self._tool5_drill_start_pub
        if not self.connected or publisher is None:
            raise ROSInterfaceError("Tool5钻进命令接口尚未连接")
        publisher.publish(Float64(data=value))

    def stop_tool5_drill_feed(self) -> None:
        publisher = self._tool5_drill_stop_pub
        if not self.connected or publisher is None:
            raise ROSInterfaceError("Tool5停止进给接口尚未连接")
        publisher.publish(Empty())

    def call_qr_task(
        self,
        command: str,
        slot_id: int,
        execute: bool,
        supervised_confirm: bool = False,
    ) -> Dict[str, Any]:
        service_name = "/qr_task/command"
        try:
            rospy.wait_for_service(service_name, timeout=self.service_timeout)
            proxy = rospy.ServiceProxy(service_name, TaskCommand, persistent=False)
            request = TaskCommandRequest(
                command=str(command),
                slot_id=int(slot_id),
                execute=bool(execute),
                supervised_confirm=bool(supervised_confirm),
            )
            response = proxy(request)
        except Exception as error:
            raise ROSInterfaceError("QR任务服务调用失败：{}".format(error)) from error

        try:
            details = json.loads(response.details_json) if response.details_json else {}
        except (TypeError, ValueError):
            details = {"raw_details": response.details_json}
        if not isinstance(details, dict):
            details = {"details": details}
        details.update(
            {
                "accepted": bool(response.accepted),
                "state": str(response.state),
                "message": str(response.message),
                "command": str(command),
                "slot_id": int(slot_id),
            }
        )
        if not response.accepted:
            raise ROSInterfaceError(
                "QR任务拒绝{}(slot {})：{}".format(
                    command, slot_id, response.message
                )
            )
        LOGGER.info(
            "QR任务接受：command=%s slot=%d state=%s",
            command,
            slot_id,
            response.state,
        )
        return details

    def call_five_target_task(
        self, command: str, slot_id: int = 0, execute: bool = False,
        supervised_confirm: bool = False,
    ) -> Dict[str, Any]:
        """Call the common-marker five-target positioning/pickup service."""

        service_name = "/qr_task/five_targets/command"
        try:
            rospy.wait_for_service(service_name, timeout=self.service_timeout)
            proxy = rospy.ServiceProxy(service_name, TaskCommand, persistent=False)
            request = TaskCommandRequest(
                command=str(command), slot_id=int(slot_id),
                execute=bool(execute),
                supervised_confirm=bool(supervised_confirm))
            response = proxy(request)
        except Exception as error:
            raise ROSInterfaceError("五目标定位服务调用失败：{}".format(error)) from error
        try:
            details = json.loads(response.details_json) if response.details_json else {}
        except (TypeError, ValueError):
            details = {"raw_details": response.details_json}
        if not response.accepted:
            raise ROSInterfaceError(
                "五目标定位拒绝{}(Tool {})：{}".format(
                    command, slot_id, response.message))
        details.update({"accepted": True, "command": command,
                        "slot_id": int(slot_id), "message": response.message})
        return details

    def detect_tool(
        self,
        tools: List[Dict[str, Any]],
        preferred: Optional[str],
        timeout: float,
    ) -> Dict[str, Any]:
        """Wait for a stable Mark detection and map its slot through tools.yaml."""

        by_slot = {int(tool["slot_id"]): dict(tool) for tool in tools}
        preferred_slot = None
        if preferred:
            matches = [
                slot for slot, tool in by_slot.items() if tool.get("name") == preferred
            ]
            if matches:
                preferred_slot = matches[0]
        start_time = time.monotonic()
        deadline = start_time + float(timeout)
        last_reason = "尚未收到/qr_task/vision_status"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                status = dict(self._telemetry["vision_status"])
                seen_at = float(self._telemetry["vision_wall_time"])
            if seen_at >= start_time:
                last_reason = str(status.get("reason", "未发现稳定Mark"))
                stable_slots = [
                    int(item.get("slot_id", -1))
                    for item in status.get("detections", [])
                    if item.get("stable")
                    and int(item.get("slot_id", -1)) in by_slot
                ]
                if preferred_slot is not None:
                    stable_slots = [slot for slot in stable_slots if slot == preferred_slot]
                    if not stable_slots:
                        last_reason = "未稳定识别当前选择工具{}".format(preferred)
                if stable_slots:
                    if len(set(stable_slots)) > 1 and preferred_slot is None:
                        raise ROSInterfaceError("同时识别多个工具，请先在GUI选择目标工具")
                    slot_id = stable_slots[0]
                    result = dict(by_slot[slot_id])
                    result.update({"slot_id": slot_id, "vision": status})
                    return result
            time.sleep(0.05)
        raise ROSInterfaceError("Mark识别超时：{}".format(last_reason))

    def wait_for_node(self, node_name: str, timeout: float) -> None:
        deadline = time.monotonic() + float(timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                if node_name in rosnode.get_node_names():
                    return
            except Exception:
                pass
            time.sleep(0.1)
        raise ROSInterfaceError("等待节点{}超时".format(node_name))

    def wait_for_force_feedback(self, timeout: float) -> None:
        deadline = time.monotonic() + float(timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.telemetry_snapshot()["force_feedback_fresh"]:
                return
            time.sleep(0.05)
        raise ROSInterfaceError("等待力传感器反馈超时")

    def telemetry_snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        monitoring = self.monitor.snapshot()
        with self._lock:
            result = dict(self._telemetry)
            result["connected"] = self._connected
            result["force_feedback_fresh"] = bool(
                result["force_wall_time"]
                and now - float(result["force_wall_time"]) <= self.feedback_max_age
            )
            result["joint_state_fresh"] = bool(
                result["joint_wall_time"]
                and now - float(result["joint_wall_time"]) <= self.feedback_max_age
            )
            result["vision_age_s"] = (
                now - float(result["vision_wall_time"])
                if result["vision_wall_time"]
                else None
            )
            result["task_age_s"] = (
                now - float(result["task_wall_time"])
                if result["task_wall_time"]
                else None
            )
            result["ack"] = {name: dict(value) for name, value in self._ack.items()}
            result.update(monitoring)
            if self.monitor.thread is not None:
                result["connected"] = bool(monitoring["health"].get("master"))
                result["joint_state_fresh"] = bool(monitoring["fresh"].get("joints"))
            return result

    def publish_command(
        self, interface: str, command: str, payload: Optional[Dict[str, Any]] = None
    ) -> InterfaceResult:
        """Compatibility dispatcher; movement commands are deliberately absent."""

        payload = payload or {}
        if interface == "voltage" and command == "set_voltage":
            result = self.set_tool_voltage(int(payload["voltage"]))
        elif interface == "emergency_stop" and command == "stop":
            result = self.emergency_stop()
        else:
            raise ROSInterfaceError(
                "{}:{}未绑定；运动必须经TaskManager调用现有任务接口".format(
                    interface, command
                )
            )
        return InterfaceResult("{}:{}".format(interface, command), True, str(result))

    def call_service(self, service_name: str, request: Any = None) -> InterfaceResult:
        if service_name != "/qr_task/command" or not isinstance(request, dict):
            raise ROSInterfaceError("仅允许经抽象层调用/qr_task/command")
        result = self.call_qr_task(
            request["command"],
            request.get("slot_id", 0),
            request.get("execute", False),
            request.get("supervised_confirm", False),
        )
        return InterfaceResult("service:{}".format(service_name), True, str(result))

    def send_action(self, action_name: str, goal: Any = None) -> InterfaceResult:
        raise ROSInterfaceError(
            "当前已审查的RM75功能没有HMI可直接使用的Action端点：{}".format(action_name)
        )

    def shutdown(self) -> None:
        """Unregister GUI-owned endpoints without shutting down the ROS graph."""

        self.monitor.close()
        with self._lock:
            endpoints = list(self._publishers) + list(self._subscribers)
            self._publishers = []
            self._subscribers = []
            self._connected = False
        for endpoint in endpoints:
            try:
                endpoint.unregister()
            except Exception:
                pass
        LOGGER.info("ROSInterface已关闭")
