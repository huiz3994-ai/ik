#!/usr/bin/env python3
"""Entry point for the RM75 alpine sampling real-interface HMI."""

import argparse
import logging
from logging.handlers import RotatingFileHandler
import os
import signal
import sys
import math
from typing import Any, Dict, List

import rospkg
import rospy
import yaml
from PyQt5.QtCore import QCoreApplication, Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox


PACKAGE_PATH = rospkg.RosPack().get_path("rm75_sampling_gui")
PACKAGE_PARENT = os.path.dirname(PACKAGE_PATH)
if PACKAGE_PARENT not in sys.path:
    sys.path.insert(0, PACKAGE_PARENT)

from rm75_sampling_gui.ros.ros_interface import ROSInterface  # noqa: E402
from rm75_sampling_gui.state.state_machine import StateMachine  # noqa: E402
from rm75_sampling_gui.task.task_manager import TaskManager  # noqa: E402
from rm75_sampling_gui.gui.main_window import MainWindow  # noqa: E402
from rm75_sampling_gui.widgets.parameter_widget import load_parameter_sources  # noqa: E402
from rm75_sampling_gui.model.robot_model import RM75RobotModel  # noqa: E402
from rm75_sampling_gui.control.joint_control_adapter import JointControlAdapter  # noqa: E402


DEFAULT_CONFIG: Dict[str, Any] = {
    "window_title": "高原寒区采样机器人控制系统",
    "robot_name": "RM75",
    "tool_count": 5,
    "ros_namespace": "/rm75_sampling_gui",
    "ros_node_name": "rm75_sampling_gui",
    "log_level": "INFO",
    "log_directory": "~/.ros/rm75_sampling_gui",
    "log_max_bytes": 2 * 1024 * 1024,
    "log_backup_count": 5,
    "ack_timeout_s": 5.0,
    "service_timeout_s": 8.0,
    "marker_timeout_s": 10.0,
    "node_start_timeout_s": 12.0,
    "feedback_timeout_s": 5.0,
    "feedback_max_age_s": 1.0,
    "teleop_config": "config/rm75_touch_real_6f.yaml",
    "force_control_package": "rm75_vertical_press",
    "force_control_config": "config/rm75_vertical_press_real.yaml",
}

LOGGER = logging.getLogger("rm75_sampling_gui.entrypoint")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--offline", action="store_true",
                        help="Start all pages without a ROS master or control endpoints")
    parser.add_argument(
        "--config",
        default=os.path.join(PACKAGE_PATH, "config", "gui.yaml"),
        help="GUI YAML configuration file",
    )
    parser.add_argument(
        "--tools",
        default=os.path.join(PACKAGE_PATH, "config", "tools.yaml"),
        help="Five-tool and Mark station YAML configuration",
    )
    parser.add_argument(
        "--execution-enabled",
        choices=("true", "false"),
        default="false",
        help="Explicitly arm real command dispatch (default: false)",
    )
    parser.add_argument(
        "--qr-execution-authorized",
        choices=("true", "false"),
        default="false",
        help="Independent authorization for the existing QR task server",
    )
    parser.add_argument(
        "--joint-control-execution-authorized",
        choices=("true", "false"),
        default="false",
        help="Independent authorization for seven-axis real execution",
    )
    parser.add_argument(
        "--trajectory-execution-authorized",
        choices=("true", "false"),
        default="false",
        help="Independent authorization passed to the existing trajectory bridge",
    )
    parser.add_argument(
        "--supervised-commissioning-authorized",
        choices=("true", "false"),
        default="false",
        help="Enable restricted watchdog-protected seven-axis execution",
    )
    # roslaunch appends __name/__log remapping arguments; rospy.myargv removes
    # them before argparse validates the GUI-specific command line.
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def load_config(path: str) -> Dict[str, Any]:
    """Load, merge and validate the YAML configuration."""

    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError("GUI configuration root must be a mapping")

    config = dict(DEFAULT_CONFIG)
    config.update(document)
    if not str(config["robot_name"]).strip():
        raise ValueError("robot_name cannot be empty")
    if not isinstance(config["tool_count"], int) or config["tool_count"] <= 0:
        raise ValueError("tool_count must be a positive integer")
    if not str(config["ros_namespace"]).strip():
        raise ValueError("ros_namespace cannot be empty")
    level = str(config["log_level"]).upper()
    if level not in logging._nameToLevel:  # pylint: disable=protected-access
        raise ValueError("unsupported log_level: {}".format(level))
    config["log_level"] = level
    config["config_path"] = os.path.abspath(os.path.expanduser(path))
    for key in (
        "ack_timeout_s",
        "service_timeout_s",
        "marker_timeout_s",
        "node_start_timeout_s",
        "feedback_timeout_s",
        "feedback_max_age_s",
    ):
        config[key] = float(config[key])
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError("{} must be positive".format(key))
    monitoring = config.get("monitoring", {})
    for key in ("refresh_ms", "plot_samples", "graph_poll_s", "graph_max_age_s", "image_max_pixels"):
        if key in monitoring and (not math.isfinite(float(monitoring[key])) or float(monitoring[key]) <= 0):
            raise ValueError("monitoring.{} must be positive".format(key))
    joint_control = config.get("joint_control", {})
    for key in (
        "joint_limit_margin_rad", "max_single_delta_rad", "max_velocity_rad_s",
        "max_acceleration_rad_s2", "max_jerk_rad_s3", "feedback_timeout_s",
        "goal_tolerance_rad", "max_tracking_error_rad", "command_ttl_s",
        "tracking_latency_allowance_s", "settling_tracking_limit_rad",
        "minimum_duration_s", "trajectory_period_s", "service_timeout_s",
        "action_server_timeout_s", "result_timeout_s", "cancel_timeout_s",
    ):
        if key in joint_control:
            value = float(joint_control[key])
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("joint_control.{} must be positive".format(key))
    for key in ("collision_samples", "jog_repeat_ms"):
        if key in joint_control and int(joint_control[key]) <= 0:
            raise ValueError("joint_control.{} must be positive".format(key))
    experiment = config.setdefault("experiment", {})
    directory = os.path.expanduser(experiment.get("directory", "experiments"))
    experiment["directory"] = (directory if os.path.isabs(directory)
                               else os.path.join(PACKAGE_PATH, directory))
    if int(experiment.get("queue_size", 2048)) <= 0:
        raise ValueError("experiment.queue_size must be positive")
    return config


def load_tools(path: str) -> List[Dict[str, Any]]:
    """Load the GUI mapping from the same YAML consumed by qr_task."""

    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    tools = document.get("tools") if isinstance(document, dict) else None
    mode = document.get("mode", "legacy_multi_marker") if isinstance(document, dict) else None
    slots = document.get("slots") if isinstance(document, dict) else None
    if not isinstance(tools, list):
        raise ValueError("tools.yaml must contain a tools list")
    if len(tools) != 5 or {item.get("name") for item in tools} != {
        "Tool1",
        "Tool2",
        "Tool3",
        "Tool4",
        "Tool5",
    }:
        raise ValueError("tools.yaml must define exactly Tool1 through Tool5")
    tool_slots = [int(item["slot_id"]) for item in tools]
    if len(set(tool_slots)) != 5:
        raise ValueError("every tool must use a unique slot_id")
    if mode == "single_marker_five_targets":
        if sorted(tool_slots) != [1, 2, 3, 4, 5]:
            raise ValueError("single-marker mode requires slot_id 1 through 5")
        if any(int(item.get("marker_id", -1)) != 0 for item in tools):
            raise ValueError("all five tools must reference common marker_id 0")
    else:
        if not isinstance(slots, list):
            raise ValueError("legacy tools.yaml must contain a slots list")
        configured_slots = {int(item["id"]) for item in slots}
        if not set(tool_slots).issubset(configured_slots):
            raise ValueError("every tool slot_id must exist in the slots list")
    return [dict(item) for item in tools]


def configure_logging(config: Dict[str, Any]) -> str:
    """Configure console and rotating-file logs and return the log path."""

    log_directory = os.path.abspath(
        os.path.expanduser(str(config["log_directory"]))
    )
    os.makedirs(log_directory, exist_ok=True)
    log_path = os.path.join(log_directory, "sampling_gui.log")
    formatter = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )

    application_logger = logging.getLogger("rm75_sampling_gui")
    application_logger.handlers.clear()
    application_logger.setLevel(logging._nameToLevel[config["log_level"]])
    application_logger.propagate = False

    console = logging.StreamHandler()
    console.setFormatter(formatter)
    application_logger.addHandler(console)

    rotating = RotatingFileHandler(
        log_path,
        maxBytes=int(config["log_max_bytes"]),
        backupCount=int(config["log_backup_count"]),
        encoding="utf-8",
    )
    rotating.setFormatter(formatter)
    application_logger.addHandler(rotating)
    return log_path


def main() -> int:
    QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    application = QApplication(sys.argv[:1])
    application.setApplicationName("rm75_sampling_gui")

    try:
        arguments = parse_args()
        config = load_config(arguments.config)
        config["tools"] = load_tools(arguments.tools)
        with open(os.path.expanduser(arguments.tools), encoding="utf-8") as stream:
            config["tools_document"] = yaml.safe_load(stream)
        config["tools_config_path"] = os.path.abspath(
            os.path.expanduser(arguments.tools)
        )
        log_path = configure_logging(config)
        LOGGER.info("日志文件：%s", log_path)
        execution_enabled = arguments.execution_enabled == "true"
        joint_execution_authorized = (
            arguments.joint_control_execution_authorized == "true"
        )
        trajectory_execution_authorized = (
            arguments.trajectory_execution_authorized == "true"
        )
        supervised_commissioning_authorized = (
            arguments.supervised_commissioning_authorized == "true"
        )
        if joint_execution_authorized and trajectory_execution_authorized:
            raise ValueError(
                "七轴受控服务与trajectory_bridge不能同时授权")
        if arguments.offline and (execution_enabled or joint_execution_authorized
                                  or trajectory_execution_authorized
                                  or supervised_commissioning_authorized):
            raise ValueError("--offline不能与任何真实执行授权同时使用")
        config["offline"] = arguments.offline
        config["execution_enabled"] = execution_enabled
        config["qr_execution_authorized"] = (
            arguments.qr_execution_authorized == "true"
        )
        config["joint_control_execution_authorized"] = joint_execution_authorized
        config["trajectory_execution_authorized"] = trajectory_execution_authorized
        config["supervised_commissioning_authorized"] = (
            supervised_commissioning_authorized
        )
        config["parameter_sources"] = load_parameter_sources(config)
        LOGGER.warning("真实执行授权：%s", execution_enabled)

        state_machine = StateMachine()
        ros_interface = ROSInterface(config)
        robot_model = RM75RobotModel(
            config.get("robot_model", {}).get("xacro_path")
        )
        joint_adapter = JointControlAdapter(
            robot_model, config, ros_interface=ros_interface,
            state_machine=state_machine,
        )
        task_manager = TaskManager(
            state_machine,
            ros_interface,
            config["tools"],
            config,
            execution_enabled=execution_enabled,
            joint_adapter=joint_adapter,
        )
        window = MainWindow(config, state_machine, ros_interface, task_manager)
        window.show()
    except Exception as error:
        LOGGER.exception("上位机启动失败")
        QMessageBox.critical(None, "启动失败", str(error))
        return 1

    # Route normal process signals through the same active-task close guard.
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)
    return application.exec_()


if __name__ == "__main__":
    sys.exit(main())
