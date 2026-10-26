"""Lifecycle manager for existing ROS launch files used by the field station.

This module owns roslaunch child processes only.  It never publishes robot
commands and it never reimplements functionality from the underlying packages.
"""

import logging
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rosnode
import rospy
from sensor_msgs.msg import Image


LOGGER = logging.getLogger(__name__)


class SystemLaunchError(RuntimeError):
    """Raised when a configured ROS subsystem cannot be started or stopped."""


class SystemLaunchManager:
    """Start and stop configured ROS launch groups from the HMI."""

    ORDER = ("base", "camera", "qr", "pose")
    MOTION_COMMAND_NODES = {
        "/trajectory_bridge",
        "/joint_control_server",
        "/rm75_touch_teleop",
        "/rm75_vertical_press",
        "/rm_control",
        "/rm75_low_speed_commissioning",
    }

    def __init__(self, config: Dict[str, Any], execution_enabled: bool) -> None:
        self._config = config
        self._execution_enabled = bool(execution_enabled)
        self._qr_execution_authorized = bool(
            config.get("qr_execution_authorized", False)
        )
        self._trajectory_execution_authorized = bool(
            config.get("trajectory_execution_authorized", False)
        )
        self._joint_control_execution_authorized = bool(
            config.get("joint_control_execution_authorized", False)
        )
        self._touch_z_force_authorized = bool(
            config.get("touch_z_force_authorized", False)
        )
        self._tool5_drill_authorized = bool(
            config.get("tool5_drill_authorized", False)
        )
        self._supervised_commissioning_authorized = bool(
            config.get("supervised_commissioning_authorized", False)
        )
        self._lock = threading.RLock()
        self._processes: Dict[str, subprocess.Popen] = {}

    def _definition(self, name: str) -> Dict[str, Any]:
        definitions = self._config.get("system_launches", {})
        value = definitions.get(name)
        if not isinstance(value, dict):
            raise SystemLaunchError("未配置子系统：{}".format(name))
        return value

    @staticmethod
    def _live_nodes() -> set:
        try:
            return set(rosnode.get_node_names())
        except Exception as error:
            raise SystemLaunchError("无法读取ROS节点列表：{}".format(error))

    def _expected_nodes(self, definition: Dict[str, Any]) -> List[str]:
        key = "expected_nodes"
        if (self._joint_control_execution_authorized
                and definition.get("expected_nodes_joint_control")):
            key = "expected_nodes_joint_control"
        elif (self._touch_z_force_authorized
              and definition.get("expected_nodes_touch_control")):
            key = "expected_nodes_touch_control"
        elif (self._tool5_drill_authorized
              and definition.get("expected_nodes_tool5_control")):
            key = "expected_nodes_tool5_control"
        return [str(node) for node in definition.get(key, [])]

    def set_control_path(self, path: str) -> None:
        """Select the next mutually-exclusive base composition while stopped."""

        if path not in {"observe", "joint_control", "z_force", "pose_pickup",
                        "qr_pickup", "tool5_drill"}:
            raise SystemLaunchError("不支持的控制路径：{}".format(path))
        with self._lock:
            owned = [name for name, process in self._processes.items()
                     if process is not None and process.poll() is None]
            if owned:
                raise SystemLaunchError(
                    "切换控制路径前必须停止本平台子系统：{}".format(
                        ", ".join(sorted(owned))
                    )
                )
        live = self._live_nodes()
        command_sources = sorted(self.MOTION_COMMAND_NODES.intersection(live))
        if command_sources:
            raise SystemLaunchError(
                "切换控制路径前必须停止外部运动命令源：{}".format(
                    ", ".join(command_sources)
                )
            )
        base_nodes = sorted({"/rm_driver", "/move_group", "/robot_state_publisher"}.intersection(live))
        if base_nodes:
            raise SystemLaunchError(
                "切换控制路径前必须停止基础栈节点：{}".format(
                    ", ".join(base_nodes)
                )
            )
        self._joint_control_execution_authorized = path == "joint_control"
        self._touch_z_force_authorized = path == "z_force"
        self._tool5_drill_authorized = path == "tool5_drill"
        self._trajectory_execution_authorized = path in {"pose_pickup", "qr_pickup"}
        self._qr_execution_authorized = path == "qr_pickup"
        self._execution_enabled = path != "observe"

    def status(self) -> Dict[str, Dict[str, Any]]:
        """Return process ownership and expected-node status for the UI."""

        try:
            live = self._live_nodes()
        except SystemLaunchError:
            live = set()
        result: Dict[str, Dict[str, Any]] = {}
        with self._lock:
            for name in self.ORDER:
                definition = self._definition(name)
                process = self._processes.get(name)
                expected = self._expected_nodes(definition)
                result[name] = {
                    "running": bool(expected) and all(node in live for node in expected),
                    "owned": process is not None and process.poll() is None,
                    "expected_nodes": expected,
                    "exit_code": None if process is None else process.poll(),
                }
        return result

    def start(self, name: str) -> Dict[str, Any]:
        """Start one existing launch and wait for its declared nodes."""

        definition = self._definition(name)
        expected = self._expected_nodes(definition)
        with self._lock:
            current = self._processes.get(name)
            if current is not None and current.poll() is None:
                return {"name": name, "already_running": True, "owned": True}

        live = self._live_nodes()
        if name == "base":
            command_sources = sorted(self.MOTION_COMMAND_NODES.intersection(live))
            if command_sources:
                raise SystemLaunchError(
                    "存在外部运动命令源，拒绝启动基础栈：{}".format(
                        ", ".join(command_sources)
                    )
                )
        duplicates = sorted(node for node in expected if node in live)
        if duplicates:
            raise SystemLaunchError(
                "{}已有外部节点，拒绝重复启动：{}".format(name, ", ".join(duplicates))
            )
        requires = [str(node) for node in definition.get("requires", [])]
        missing = sorted(node for node in requires if node not in live)
        if missing:
            raise SystemLaunchError(
                "{}缺少前置节点：{}".format(name, ", ".join(missing))
            )

        self._prepare_usb(definition)

        package = str(definition.get("package", "")).strip()
        launch = str(definition.get("launch", "")).strip()
        if not package or not launch:
            raise SystemLaunchError("{}的package/launch配置不完整".format(name))
        arguments: List[str] = []
        for key, value in dict(definition.get("args", {})).items():
            rendered = str(value)
            rendered = rendered.replace(
                "${EXECUTION_ENABLED}", "true" if self._execution_enabled else "false"
            )
            rendered = rendered.replace(
                "${QR_EXECUTION_AUTHORIZED}",
                "true" if self._qr_execution_authorized else "false",
            )
            rendered = rendered.replace(
                "${TRAJECTORY_EXECUTION_AUTHORIZED}",
                "true" if self._trajectory_execution_authorized else "false",
            )
            rendered = rendered.replace(
                "${JOINT_CONTROL_EXECUTION_AUTHORIZED}",
                "true" if self._joint_control_execution_authorized else "false",
            )
            rendered = rendered.replace(
                "${TOUCH_Z_FORCE_AUTHORIZED}",
                "true" if self._touch_z_force_authorized else "false",
            )
            rendered = rendered.replace(
                "${TOOL5_DRILL_AUTHORIZED}",
                "true" if self._tool5_drill_authorized else "false",
            )
            rendered = rendered.replace(
                "${SUPERVISED_COMMISSIONING_AUTHORIZED}",
                "true" if self._supervised_commissioning_authorized else "false",
            )
            rendered = rendered.replace(
                "${TOOLS_CONFIG}", str(self._config.get("tools_config_path", ""))
            )
            arguments.append("{}:={}".format(key, rendered))

        process = subprocess.Popen(
            ["roslaunch", package, launch] + arguments,
            start_new_session=True,
        )
        with self._lock:
            self._processes[name] = process
        timeout = float(definition.get(
            "timeout_s", self._config.get("node_start_timeout_s", 12.0)
        ))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                with self._lock:
                    self._processes.pop(name, None)
                raise SystemLaunchError(
                    "{}启动进程退出，code={}".format(name, return_code)
                )
            live = self._live_nodes()
            if expected and all(node in live for node in expected):
                try:
                    self._wait_for_services(definition, deadline)
                    self._wait_for_topics(definition, deadline)
                except Exception:
                    self._stop_owned(name)
                    raise
                LOGGER.info("子系统已启动并收到数据：%s (%s)",
                            name, ", ".join(expected))
                return {"name": name, "running": True, "owned": True}
            time.sleep(0.2)

        self._stop_owned(name)
        missing = sorted(set(expected) - self._live_nodes())
        raise SystemLaunchError(
            "{}启动超时，缺少节点：{}".format(name, ", ".join(missing))
        )

    @staticmethod
    def _prepare_usb(definition: Dict[str, Any]) -> None:
        """Recover one explicitly configured USB device before its launch."""

        recovery = definition.get("usb_recovery")
        if not isinstance(recovery, dict):
            return
        vendor = str(recovery.get("vendor_id", "")).strip().lower()
        product = str(recovery.get("product_id", "")).strip().lower()
        try:
            int(vendor, 16)
            int(product, 16)
        except ValueError as error:
            raise SystemLaunchError(
                "USB恢复配置的VID/PID无效：{}:{}".format(vendor, product)
            ) from error

        # A camera may disappear briefly while udev finishes an earlier
        # disconnect/reconnect.  Treat that as an enumeration delay instead of
        # failing the complete control-path switch immediately.
        timeout = max(0.0, float(recovery.get("enumerate_timeout_s", 5.0)))
        poll_interval = max(0.05, float(recovery.get("enumerate_poll_s", 0.25)))
        deadline = time.monotonic() + timeout
        matches: List[Path] = []
        while True:
            matches = []
            for device in Path("/sys/bus/usb/devices").glob("*"):
                try:
                    if (device / "idVendor").read_text().strip().lower() == vendor and (
                            device / "idProduct").read_text().strip().lower() == product:
                        matches.append(device)
                except (OSError, FileNotFoundError):
                    continue
            if matches or time.monotonic() >= deadline:
                break
            time.sleep(poll_interval)

        device_label = str(recovery.get("device_name", "USB设备"))
        if not matches:
            raise SystemLaunchError(
                "未检测到{} {:04x}:{:04x}；请检查相机供电、USB 3.x线缆与接口，"
                "确认 lsusb 可见该VID:PID后重试".format(
                    device_label, int(vendor, 16), int(product, 16)
                )
            )
        if len(matches) > 1:
            raise SystemLaunchError(
                "检测到{} {:04x}:{:04x} 共{}个，无法确定恢复对象".format(
                    device_label, int(vendor, 16), int(product, 16), len(matches)
                )
            )
        device = matches[0]
        authorized = device / "authorized"
        try:
            authorized.write_text("0")
            time.sleep(float(recovery.get("off_s", 2.0)))
            authorized.write_text("1")
            time.sleep(float(recovery.get("settle_s", 5.0)))
            control = device / "power" / "control"
            if control.exists():
                control.write_text("on")
        except OSError as error:
            raise SystemLaunchError("USB恢复失败 {}：{}".format(device.name, error))
        LOGGER.info("USB设备恢复完成：%s (%s:%s)", device.name, vendor, product)

    @staticmethod
    def _wait_for_services(definition: Dict[str, Any], deadline: float) -> None:
        """Require declared ROS services before reporting a subsystem ready."""

        for service in definition.get("expected_services", []):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SystemLaunchError("等待服务超时：{}".format(service))
            try:
                rospy.wait_for_service(str(service), timeout=remaining)
            except Exception as error:
                raise SystemLaunchError(
                    "等待服务失败 {}：{}".format(service, error)
                )

    @staticmethod
    def _wait_for_topics(definition: Dict[str, Any], deadline: float) -> None:
        for topic in definition.get("expected_image_topics", []):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SystemLaunchError("等待图像首帧超时：{}".format(topic))
            try:
                message = rospy.wait_for_message(str(topic), Image, timeout=remaining)
            except Exception as error:
                raise SystemLaunchError(
                    "等待图像首帧失败 {}：{}".format(topic, error)
                )
            if not message.data or message.width <= 0 or message.height <= 0:
                raise SystemLaunchError("收到无效图像首帧：{}".format(topic))

    def _stop_owned(self, name: str) -> bool:
        with self._lock:
            process = self._processes.get(name)
        if process is None:
            return False
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
        with self._lock:
            if self._processes.get(name) is process:
                self._processes.pop(name, None)
        LOGGER.info("已停止HMI管理的子系统：%s", name)
        return True

    def stop(self, name: str) -> Dict[str, Any]:
        """Stop a subsystem only when its roslaunch is owned by this HMI."""

        stopped = self._stop_owned(name)
        if not stopped:
            raise SystemLaunchError(
                "{}不是由当前HMI启动，未执行停止".format(name)
            )
        return {"name": name, "stopped": True}

    def shutdown(self) -> None:
        """Stop all owned children in dependency-safe reverse order."""

        for name in reversed(self.ORDER):
            try:
                self._stop_owned(name)
            except Exception:
                LOGGER.exception("关闭子系统失败：%s", name)
