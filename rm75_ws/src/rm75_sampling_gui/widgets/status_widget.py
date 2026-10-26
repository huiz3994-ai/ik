"""RM75 digital-twin status page and seven-axis preview controls."""

import math
from typing import Any, Dict, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QLabel, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel
from rm75_sampling_gui.model.joint_models import JointStateModel
from rm75_sampling_gui.widgets.joint_control_widget import JointControlWidget
from rm75_sampling_gui.widgets.robot_3d_widget import Robot3DWidget


def number(value: Any, digits: int = 3) -> str:
    return "—" if value is None else ("{:.%df}" % digits).format(value)


class StatusWidget(QWidget):
    """65/35 digital-twin and joint-control workspace."""

    def __init__(
        self, config: Dict[str, Any], parent: QWidget = None,
        robot: Optional[RM75RobotModel] = None,
    ) -> None:
        super().__init__(parent)
        self.config = config
        self.robot = robot or RM75RobotModel(
            config.get("robot_model", {}).get("xacro_path")
        )
        self.joint_state_model = JointStateModel()
        root = QVBoxLayout(self)
        self.connections = QLabel("等待ROS连接")
        self.connections.setWordWrap(True)
        root.addWidget(self.connections)

        splitter = QSplitter(Qt.Horizontal)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.robot_view = Robot3DWidget(self.robot, config)
        left_layout.addWidget(self.robot_view, 1)
        self.tcp = QLabel("base_link → Link7：等待TF")
        self.tcp.setWordWrap(True)
        left_layout.addWidget(self.tcp)
        self.tool = QLabel("持有工具：未知；电压：未知")
        left_layout.addWidget(self.tool)
        splitter.addWidget(left)

        self.joint_control = JointControlWidget(self.robot, config)
        self.joint_control.target_changed.connect(self.robot_view.update_target)
        splitter.addWidget(self.joint_control)
        splitter.setStretchFactor(0, 65)
        splitter.setStretchFactor(1, 35)
        splitter.setSizes([1250, 670])
        root.addWidget(splitter, 1)

        # Hidden compatibility table for older callers/tests. The visible right
        # panel contains the same values plus target, velocity and error.
        self.joints = QTableWidget(7, 3)
        self.joints.setHorizontalHeaderLabels(["关节", "当前角度 (°)", "最近限位距离 (°)"])
        self.joints.setVisible(False)
        for row, name in enumerate(JOINT_NAMES):
            for col, text in enumerate((name, "—", "—")):
                self.joints.setItem(row, col, QTableWidgetItem(text))

    def update_snapshot(self, data: Dict[str, Any]) -> None:
        health = data.get("health", {})
        labels = (("master", "ROS master"), ("driver", "rm_driver"),
                  ("touch", "Touch设备"), ("qr", "QR任务"), ("marker", "Mark节点"))
        control_source = data.get("control_source") or "NONE"
        self.connections.setText("   |   ".join(
            "{}: {}".format(label, "OK" if health.get(key) else "ERROR")
            for key, label in labels) + "   |   当前控制源: {}".format(control_source))

        fresh = bool(data.get("fresh", {}).get("joints"))
        self.joint_state_model.update_snapshot(data)
        values = data.get("joints", {}) if fresh else {}
        vector = self.joint_state_model.vector() if fresh else None
        limits = data.get("joint_limits", {}) or self.robot.limits
        for row, name in enumerate(JOINT_NAMES):
            angle = values.get(name)
            margin = None
            if angle is not None and name in limits:
                lower, upper = limits[name]
                margin = min(angle - lower, upper - angle)
            self.joints.item(row, 1).setText(
                number(math.degrees(angle) if angle is not None else None)
            )
            self.joints.item(row, 2).setText(
                number(math.degrees(margin) if margin is not None else None)
            )
        if vector is not None:
            self.robot_view.update_actual(
                vector, True,
                data.get("source_stamps", {}).get("joints"),
                data.get("ages", {}).get("joints"),
            )
        else:
            self.robot_view.update_actual(
                self.robot_view._actual, False,
                data.get("source_stamps", {}).get("joints"),
                data.get("ages", {}).get("joints"),
            )
        self.joint_control.update_snapshot(data)

        tcp = data.get("tcp", {}) if data.get("fresh", {}).get("tcp") else {}
        options = self.config.get("monitoring", {})
        self.tcp.setText("{} → {}　XYZ (m): {}　RPY (°): {}".format(
            options.get("base_frame", "base_link"), options.get("tip_frame", "Link7"),
            "  ".join("{}={}".format(key, number(tcp.get(key))) for key in ("x", "y", "z")),
            "  ".join("{}={}".format(
                key, number(math.degrees(tcp[key]) if key in tcp else None)
            ) for key in ("rx", "ry", "rz"))))
        task = data.get("task_status", {}) if data.get("fresh", {}).get("task_status") else {}
        slot = task.get("held_slot_id")
        tools = self.config.get("tools", [])
        common_mode = bool(tools) and all(
            int(tool.get("marker_id", -1)) == 0 for tool in tools)
        if common_mode and isinstance(slot, int) and 0 <= slot < 5:
            # task_status belongs to the retained legacy PICK/RETURN server,
            # whose historical slot ids are 0..4.
            name = "Tool{}".format(slot + 1)
        else:
            name = next((tool["name"] for tool in tools
                         if tool["slot_id"] == slot), "未知/未持有")
        voltage = data.get("tool_voltage")
        self.tool.setText("持有工具：{}　选择：{}　电压：{}（命令ACK，非实测）".format(
            name, data.get("selected_tool") or "未选择",
            "未知" if voltage is None else "{}V".format(voltage)))
