"""Field operation station and existing task controls."""

from concurrent.futures import Future, ThreadPoolExecutor
import logging
from pathlib import Path
from typing import Any, Callable, Dict, Optional

from PyQt5.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QCloseEvent, QFont, QFontDatabase
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QStatusBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from rm75_sampling_gui.ros.ros_interface import ROSInterface
from rm75_sampling_gui.state.state_machine import RobotState, StateMachine, StateTransition
from rm75_sampling_gui.task.task_manager import TaskManager
from rm75_sampling_gui.widgets.status_widget import StatusWidget
from rm75_sampling_gui.widgets.force_widget import ForceMonitorWidget
from rm75_sampling_gui.widgets.vision_widget import VisionWidget
from rm75_sampling_gui.widgets.diagnostic_widget import DiagnosticWidget
from rm75_sampling_gui.widgets.parameter_widget import ParameterWidget
from rm75_sampling_gui.logger.experiment_logger import ExperimentLogger


LOGGER = logging.getLogger(__name__)
APPLICATION_LOGGER = logging.getLogger("rm75_sampling_gui")


class _LogEmitter(QObject):
    message = pyqtSignal(str)


class QtLogHandler(logging.Handler):
    """Thread-safe logging handler forwarding formatted records to Qt."""

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__()
        self.emitter = _LogEmitter(parent)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.emitter.message.emit(self.format(record))
        except Exception:
            self.handleError(record)


class MainWindow(QMainWindow):
    """Operator window; potentially blocking ROS work runs off the Qt thread."""

    operation_finished = pyqtSignal(str, bool, str, object)
    state_changed = pyqtSignal(object)
    connection_changed = pyqtSignal(bool)

    def __init__(
        self,
        config: Dict[str, Any],
        state_machine: StateMachine,
        ros_interface: ROSInterface,
        task_manager: TaskManager,
    ) -> None:
        super().__init__()
        font_file = Path(__file__).resolve().parents[1] / "assets/fonts/DroidSansFallbackFull.ttf"
        if not font_file.exists():
            import rospkg
            font_file = Path(rospkg.RosPack().get_path("rm75_sampling_gui")) / "assets/fonts/DroidSansFallbackFull.ttf"
        font_id = QFontDatabase.addApplicationFont(str(font_file))
        families = QFontDatabase.applicationFontFamilies(font_id)
        if families:
            self.setFont(QFont(families[0], 10))
        self._config = config
        self._state_machine = state_machine
        self._ros = ros_interface
        self._tasks = task_manager
        self._buttons: Dict[str, QPushButton] = {}
        self._qt_log_handler: Optional[QtLogHandler] = None
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="rm75-hmi")
        self._emergency_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="hmi-stop")
        self._pending = 0
        self._emergency_pending = False
        self._busy = False
        self._closing = False
        self._reported_qr_mismatch = False
        self._bottom_safe_stop_seen = False
        self._last_task_state = "UNKNOWN"
        self._last_system_status: Dict[str, Dict[str, Any]] = {}
        self.recorder = ExperimentLogger(
            config.get("experiment", {}).get("directory", str(Path(__file__).resolve().parents[1] / "experiments")),
            config)
        APPLICATION_LOGGER.addHandler(self.recorder)
        self._state_machine.add_listener(self.recorder.transition)

        self.setWindowTitle(str(config["window_title"]))
        self.setMinimumSize(1280, 800)
        self._build_ui()
        self._install_log_handler()
        self.operation_finished.connect(self._on_operation_finished)
        self.state_changed.connect(self._on_state_transition)
        self.connection_changed.connect(self._connected_changed)
        self._state_listener = self.state_changed.emit
        self._state_machine.add_listener(self._state_listener)
        self._refresh_state_controls()

        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.timeout.connect(self._refresh_telemetry)
        self._telemetry_timer.start(int(config.get("monitoring", {}).get("refresh_ms", 100)))
        self._system_timer = QTimer(self)
        self._system_timer.timeout.connect(self._refresh_system_status)
        self._system_timer.start(1000)
        if not config.get("offline", False):
            QTimer.singleShot(0, self._connect_ros)

    def _build_ui(self) -> None:
        central = QWidget(self)
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 14, 18, 14)
        root.setSpacing(12)

        title = QLabel(str(self._config["window_title"]))
        title.setObjectName("title")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont(self.font().family(), 20, QFont.Bold))
        root.addWidget(title)
        root.addWidget(self._build_status_group(), stretch=0)
        self.tabs = QTabWidget()
        control_page = QWidget()
        control_layout = QVBoxLayout(control_page)
        control_layout.addWidget(self._build_system_group())

        controls = QHBoxLayout()
        controls.setSpacing(12)
        left = QVBoxLayout()
        left.addWidget(self._build_teleop_group())
        left.addWidget(self._build_tool_group(), stretch=1)
        right = QVBoxLayout()
        right.addWidget(self._build_sampling_group())
        right.addWidget(self._build_power_group())
        right.addStretch(1)
        controls.addLayout(left, stretch=3)
        controls.addLayout(right, stretch=2)
        control_layout.addLayout(controls)
        self.tabs.addTab(control_page, "操作台")
        robot_model = (
            self._tasks.joint_adapter.robot
            if getattr(self._tasks, "joint_adapter", None) is not None else None
        )
        self.status_widget = StatusWidget(self._config, robot=robot_model)
        joint = self.status_widget.joint_control
        joint.execute_requested.connect(
            lambda target: self._submit(
                "执行七轴目标", self._tasks.execute_joint_target, target
            )
        )
        joint.cancel_requested.connect(self._cancel_joint_execution)
        joint.emergency_requested.connect(
            lambda: self._submit("关节控制急停", self._tasks.emergency_stop, emergency=True)
        )
        self.force_widget = ForceMonitorWidget(self._config)
        self.vision_widget = VisionWidget(self._config)
        self.diagnostic_widget = DiagnosticWidget()
        self.parameter_widget = ParameterWidget(self._config)
        for widget, title in ((self.status_widget, "机器人状态"),
                              (self.force_widget, "实时力"),
                              (self.vision_widget, "视觉"),
                              (self.diagnostic_widget, "故障诊断"),
                              (self.parameter_widget, "参数")):
            self.tabs.addTab(widget, title)
        root.addWidget(self.tabs, stretch=5)
        self.recording_label = QLabel("实验记录：等待任务")
        root.addWidget(self.recording_label)
        root.addWidget(self._build_safety_group())

        self.setCentralWidget(central)
        self.setStatusBar(QStatusBar(self))
        mode = "真实执行已授权" if self._tasks.execution_enabled else "安全预览（真实执行禁用）"
        self.statusBar().showMessage(mode)
        self.setStyleSheet(
            """
            QMainWindow { background: #eef2f5; }
            QLabel#title { color: #17324d; padding: 8px; }
            QGroupBox {
                background: white; border: 1px solid #c7d1da;
                border-radius: 6px; margin-top: 10px; padding-top: 10px;
                font-weight: bold;
            }
            QGroupBox::title { left: 10px; padding: 0 5px; }
            QPushButton { min-height: 34px; padding: 4px 12px; }
            QPushButton#emergencyStop {
                color: white; background: #b71c1c; font-weight: bold;
                border-radius: 4px; min-height: 42px;
            }
            QPushButton#powerOn { background: #fff3cd; }
            QPushButton#powerOff { background: #d9edf7; }
            QLabel#safeState { color: #b71c1c; font-weight: bold; }
            """
        )

    def _build_system_group(self) -> QGroupBox:
        group = QGroupBox("ROS系统启动（由界面管理）")
        layout = QGridLayout(group)
        definitions = (
            ("base", "RM75基础栈"),
            ("camera", "RGB-D相机"),
            ("qr", "Mark/QR任务"),
        )
        self._system_labels: Dict[str, QLabel] = {}
        for row, (name, title) in enumerate(definitions):
            label = QLabel("未运行")
            self._system_labels[name] = label
            self._buttons["system_{}_start".format(name)] = self._button(
                "启动", lambda checked=False, item=name: self._submit(
                    "启动{}".format(item), self._tasks.start_subsystem, item
                )
            )
            self._buttons["system_{}_stop".format(name)] = self._button(
                "停止", lambda checked=False, item=name: self._submit(
                    "停止{}".format(item), self._tasks.stop_subsystem, item
                )
            )
            layout.addWidget(QLabel(title), row, 0)
            layout.addWidget(label, row, 1)
            layout.addWidget(self._buttons["system_{}_start".format(name)], row, 2)
            layout.addWidget(self._buttons["system_{}_stop".format(name)], row, 3)
        note = QLabel("各子系统按需启动；控制按钮依据真实节点和设备状态启用")
        note.setStyleSheet("color: #52616b;")
        layout.addWidget(note, len(definitions), 0, 1, 4)
        return group

    def _build_status_group(self) -> QGroupBox:
        group = QGroupBox("机器人状态")
        layout = QGridLayout(group)
        layout.addWidget(QLabel("机器人："), 0, 0)
        layout.addWidget(QLabel(str(self._config["robot_name"])), 0, 1)
        layout.addWidget(QLabel("当前功能："), 0, 2)
        self.state_label = QLabel(self._state_machine.state.value)
        layout.addWidget(self.state_label, 0, 3)
        layout.addWidget(QLabel("当前工具："), 1, 0)
        self.tool_label = QLabel("未选择")
        layout.addWidget(self.tool_label, 1, 1)
        layout.addWidget(QLabel("ROS/关节反馈："), 1, 2)
        self.ros_label = QLabel("检查中")
        layout.addWidget(self.ros_label, 1, 3)
        layout.addWidget(QLabel("QR任务："), 2, 0)
        self.qr_status_label = QLabel("无状态")
        layout.addWidget(self.qr_status_label, 2, 1)
        layout.addWidget(QLabel("执行授权："), 2, 2)
        self.armed_label = QLabel("已授权" if self._tasks.execution_enabled else "未授权")
        self.armed_label.setStyleSheet(
            "color: #b71c1c; font-weight: bold;"
            if self._tasks.execution_enabled
            else "color: #7f6000; font-weight: bold;"
        )
        layout.addWidget(self.armed_label, 2, 3)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(1000)
        self.log_view.setMaximumHeight(105)
        self.log_view.setPlaceholderText("运行日志")
        layout.addWidget(self.log_view, 3, 0, 1, 4)
        layout.setRowStretch(3, 1)
        return group

    def _build_teleop_group(self) -> QGroupBox:
        group = QGroupBox("Touch遥操作")
        layout = QHBoxLayout(group)
        self._buttons["teleop_start"] = self._button(
            "启动遥操作", lambda: self._submit("启动遥操作", self._tasks.start_teleop)
        )
        self._buttons["teleop_stop"] = self._button(
            "停止遥操作", lambda: self._submit("停止遥操作", self._tasks.stop_teleop)
        )
        layout.addWidget(self._buttons["teleop_start"])
        layout.addWidget(self._buttons["teleop_stop"])
        return group

    def _build_tool_group(self) -> QGroupBox:
        group = QGroupBox("公共AprilTag五目标定位（ID=0）")
        layout = QVBoxLayout(group)
        self.tool_list = QListWidget()
        self.tool_list.setSelectionMode(QAbstractItemView.SingleSelection)
        for tool in self._config["tools"]:
            self.tool_list.addItem(str(tool["name"]))
        self.tool_list.currentTextChanged.connect(self._select_tool)
        layout.addWidget(self.tool_list)

        buttons = QGridLayout()
        self._buttons["tool_detect"] = self._button(
            "识别公共二维码", lambda: self._submit(
                "识别公共二维码", self._tasks.detect_common_marker)
        )
        self._buttons["rack_calibrate"] = self._button(
            "标定工具架", lambda: self._submit(
                "标定工具架", self._tasks.calibrate_tool_rack)
        )
        self._buttons["rack_validate"] = self._button(
            "确认现场验收", self._confirm_five_target_validation
        )
        self._buttons["camera_observation"] = self._button(
            "到达观察位", lambda: self._submit(
                "到达观察位", self._tasks.move_to_camera_observation)
        )
        self._buttons["target_preview"] = self._button(
            "预览目标位姿", lambda: self._submit(
                "预览目标位姿", self._tasks.preview_selected_target)
        )
        self._buttons["target_plan"] = self._button(
            "规划选定目标", lambda: self._submit(
                "规划选定目标", self._tasks.plan_selected_target)
        )
        self._buttons["target_execute"] = self._button(
            "自动取选中工具", lambda: self._submit(
                "自动取选中工具", self._tasks.execute_selected_target)
        )
        self._buttons["target_return"] = self._button(
            "归还所持工具", lambda: self._submit(
                "归还所持工具", self._tasks.return_selected_target)
        )
        self._buttons["confirm_held_tool"] = self._button(
            "确认持有选中工具", lambda: self._submit(
                "确认持有选中工具", self._tasks.confirm_selected_tool_held)
        )
        self._buttons["target_cancel"] = self._button(
            "取消执行", self._cancel_target_execution
        )
        for index, name in enumerate(("tool_detect", "rack_calibrate",
                                      "rack_validate",
                                      "camera_observation",
                                      "target_preview", "target_plan",
                                      "target_execute", "target_return",
                                      "confirm_held_tool", "target_cancel")):
            buttons.addWidget(self._buttons[name], index // 2, index % 2)
        layout.addLayout(buttons)
        return group

    def _build_sampling_group(self) -> QGroupBox:
        group = QGroupBox("垂直下压力控采样")
        layout = QGridLayout(group)
        layout.addWidget(QLabel("当前Fz："), 0, 0)
        self.force_label = QLabel("-- N")
        layout.addWidget(self.force_label, 0, 1)
        layout.addWidget(QLabel("目标力："), 1, 0)
        self.force_target_label = QLabel("-- N")
        layout.addWidget(self.force_target_label, 1, 1)
        layout.addWidget(QLabel("采样状态："), 2, 0)
        self.sampling_status_label = QLabel("UNKNOWN")
        layout.addWidget(self.sampling_status_label, 2, 1)

        self._buttons["force_enter"] = self._button(
            "进入力控", lambda: self._submit("进入力控", self._tasks.enter_force_control)
        )
        self._buttons["sampling_start"] = self._button(
            "开始采样", lambda: self._submit("确认开始采样", self._tasks.start_sampling)
        )
        self._buttons["sampling_stop"] = self._button(
            "停止采样", lambda: self._submit("停止采样", self._tasks.stop_sampling)
        )
        layout.addWidget(self._buttons["force_enter"], 3, 0, 1, 2)
        layout.addWidget(self._buttons["sampling_start"], 4, 0)
        layout.addWidget(self._buttons["sampling_stop"], 4, 1)
        return group

    def _build_safety_group(self) -> QGroupBox:
        group = QGroupBox("安全")
        layout = QHBoxLayout(group)
        self._buttons["recovery"] = self._button(
            "恢复检查（不运动）", lambda: self._submit("恢复检查", self._tasks.recover)
        )
        self._buttons["emergency_stop"] = self._button(
            "急停", lambda: self._submit("急停", self._tasks.emergency_stop, emergency=True)
        )
        self._buttons["emergency_stop"].setObjectName("emergencyStop")
        layout.addWidget(self._buttons["recovery"])
        layout.addWidget(self._buttons["emergency_stop"])
        return group

    def _build_power_group(self) -> QGroupBox:
        group = QGroupBox("工具电压（新鲜ACK）")
        layout = QHBoxLayout(group)
        self.voltage_label = QLabel("状态：未知")
        self._buttons["power_on"] = self._button(
            "24V ON", lambda: self._submit("工具24V", self._tasks.set_voltage, 24)
        )
        self._buttons["power_on"].setObjectName("powerOn")
        self._buttons["power_off"] = self._button(
            "0V OFF", lambda: self._submit("工具0V", self._tasks.set_voltage, 0)
        )
        self._buttons["power_off"].setObjectName("powerOff")
        layout.addWidget(self.voltage_label)
        layout.addWidget(self._buttons["power_on"])
        layout.addWidget(self._buttons["power_off"])
        return group

    @staticmethod
    def _button(text: str, callback: Callable[[], None]) -> QPushButton:
        button = QPushButton(text)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        button.clicked.connect(callback)
        return button

    def _install_log_handler(self) -> None:
        handler = QtLogHandler(self)
        handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
        handler.emitter.message.connect(self.log_view.appendPlainText)
        APPLICATION_LOGGER.addHandler(handler)
        handler.emitter.message.connect(self._diagnostic_log)
        self._qt_log_handler = handler

    def _connect_ros(self) -> None:
        future = self._executor.submit(self._ros.connect)
        future.add_done_callback(lambda result: self.connection_changed.emit(
            False if result.exception() else bool(result.result())))

    def _connected_changed(self, connected: bool) -> None:
        if connected:
            LOGGER.info("GUI已连接ROS，等待RM75反馈")
        else:
            LOGGER.warning("ROS连接失败；控制按钮保持锁定")
        self._refresh_telemetry()
        self._refresh_state_controls()

    def _diagnostic_log(self, text: str) -> None:
        if "[ERROR]" in text or "[WARNING]" in text:
            self.diagnostic_widget.add_error(text)

    def _select_tool(self, tool_name: str) -> None:
        if not tool_name:
            return
        try:
            self._tasks.select_tool(tool_name)
            self.tool_label.setText(tool_name)
        except Exception as error:
            self._operator_warning(str(error))

    def _submit(
        self,
        name: str,
        operation: Callable[..., Any],
        *args: Any,
        emergency: bool = False,
    ) -> None:
        if self._closing:
            return
        if self._config.get("offline", False):
            if emergency:
                self._state_machine.fail(RuntimeError("离线急停锁存"), "SAFE_STOP")
            return
        if emergency and self._emergency_pending:
            return
        if self._busy and not emergency:
            self._operator_warning("已有任务执行中，请等待或使用急停")
            return
        if not self._ros.connected and not emergency:
            self._operator_warning("ROS未连接")
            return
        if not emergency:
            if self._state_machine.state == RobotState.IDLE and operation in (
                    self._tasks.start_teleop, self._tasks.detect_tool):
                self.recorder.begin_task()
            self._pending += 1
            self._busy = True
            if name == "执行七轴目标":
                self.status_widget.joint_control.set_busy(True)
            self._refresh_state_controls()
        else:
            self._emergency_pending = True
            # Latch synchronously before any ROS or worker scheduling latency.
            self._state_machine.fail(RuntimeError("操作员急停"), "SAFE_STOP")
        LOGGER.info("操作开始：%s", name)
        executor = self._emergency_executor if emergency else self._executor
        future = executor.submit(operation, *args)
        future.add_done_callback(
            lambda result, operation_name=name, is_emergency=emergency: self._future_done(
                operation_name, is_emergency, result
            )
        )

    def _future_done(self, name: str, emergency: bool, future: Future) -> None:
        try:
            result = future.result()
            self.operation_finished.emit(name, True, "完成",
                                         {"emergency": emergency, "value": result})
        except Exception as error:
            LOGGER.exception("操作失败：%s", name)
            self.operation_finished.emit(name, False, str(error), {"emergency": emergency})

    def _on_operation_finished(
        self, name: str, success: bool, message: str, result: object
    ) -> None:
        envelope = result if isinstance(result, dict) else {}
        if envelope.get("emergency"):
            self._emergency_pending = False
        else:
            self._pending = max(0, self._pending - 1)
        self._busy = self._pending > 0 or self._emergency_pending
        result = envelope.get("value", {})
        if success:
            if name == "执行七轴目标":
                self.status_widget.joint_control.set_busy(False)
            if isinstance(result, dict) and result.get("name"):
                tool_name = str(result["name"])
                self.tool_label.setText(tool_name)
                matches = self.tool_list.findItems(tool_name, Qt.MatchExactly)
                if matches:
                    self.tool_list.setCurrentItem(matches[0])
            LOGGER.info("操作完成：%s", name)
            self.statusBar().showMessage("{}：完成".format(name), 5000)
        else:
            if name == "执行七轴目标":
                self.status_widget.joint_control.set_busy(False)
            self._operator_warning("{}失败：{}".format(name, message))
        self._refresh_state_controls()

    def _on_state_transition(self, transition: StateTransition) -> None:
        if transition.current == RobotState.SAFE_STOP:
            self.diagnostic_widget.add_error("SAFE_STOP: " + transition.reason)
        self.state_label.setText(transition.current.value)
        self.state_label.setObjectName(
            "safeState" if transition.current == RobotState.SAFE_STOP else ""
        )
        self.state_label.style().unpolish(self.state_label)
        self.state_label.style().polish(self.state_label)
        self.statusBar().showMessage(
            "{} → {}：{}".format(
                transition.previous.value,
                transition.current.value,
                transition.reason,
            ),
            5000,
        )
        self._refresh_state_controls()

    def _refresh_telemetry(self) -> None:
        if not self._config.get("offline", False):
            self._ros.publish_joint_control_heartbeat()
        snapshot = self._ros.telemetry_snapshot()
        snapshot["selected_tool"] = self._tasks.selected_tool
        control_source = self._tasks.control_mode
        if control_source is None:
            health = snapshot.get("health", {})
            task_state = str((snapshot.get("task_status") or {}).get("state", ""))
            if health.get("teleop_control"):
                control_source = "TOUCH_EXTERNAL"
            elif health.get("force_control"):
                control_source = "FORCE_EXTERNAL"
            elif task_state not in ("", "IDLE", "NOT_RUNNING", "UNKNOWN"):
                control_source = "QR_TASK"
        snapshot["control_source"] = control_source or "NONE"
        for widget in (self.status_widget, self.force_widget,
                       self.vision_widget, self.diagnostic_widget):
            widget.update_snapshot(snapshot)
        self.recorder.sample(snapshot)
        self.recording_label.setText(
            self.recorder.error or "实验记录：{}  {}".format(
                "记录中" if self.recorder.active else "已关闭 / 等待任务",
                self.recorder.directory or ""))
        if self.recorder.error:
            self.diagnostic_widget.add_error(self.recorder.error)
        connected = bool(snapshot.get("connected"))
        joint_fresh = bool(snapshot.get("joint_state_fresh"))
        text = "已连接 / 反馈正常" if connected and joint_fresh else (
            "已连接 / 无新鲜关节反馈" if connected else "未连接"
        )
        self.ros_label.setText(text)
        self.ros_label.setStyleSheet(
            "color: #1b5e20; font-weight: bold;"
            if connected and joint_fresh
            else "color: #b71c1c; font-weight: bold;"
        )
        fresh = snapshot.get("fresh", {})
        force = snapshot.get("filtered_force_z") if fresh.get("filtered_force_z") else None
        if force is None:
            force = snapshot.get("force_z") if fresh.get("force_z") else None
        if force is None and fresh.get("wrench"):
            force = snapshot.get("wrench", {}).get("Fz")
        target = snapshot.get("force_setpoint") if fresh.get("force_setpoint") else None
        self.force_label.setText("-- N" if force is None else "{:.2f} N".format(force))
        self.force_target_label.setText(
            "-- N" if target is None else "{:.2f} N".format(target)
        )
        mode = str(snapshot.get("control_mode") or "UNKNOWN")
        active = "ACTIVE" if snapshot.get("teleop_active") else "INACTIVE"
        self.sampling_status_label.setText("{} / {}".format(mode, active))
        stop_reason = str(snapshot.get("stop_reason") or "").strip()
        if (mode == "SAFE_STOP" and not self._bottom_safe_stop_seen and
                self._state_machine.state not in (RobotState.SAFE_STOP, RobotState.RECOVERY)):
            self._bottom_safe_stop_seen = True
            self._state_machine.fail(
                RuntimeError(stop_reason or "底层Touch控制进入SAFE_STOP"),
                "底层控制安全停止",
            )
        elif mode != "SAFE_STOP":
            self._bottom_safe_stop_seen = False
        task_status = snapshot.get("task_status") or {}
        five_status = snapshot.get("five_target_status") or {}
        task_state = str(task_status.get("state") or (
            "五目标就绪" if five_status.get("targets_valid") else
            (five_status.get("vision", {}).get("reason") or "无状态")))
        self._last_task_state = task_state
        self.qr_status_label.setText(task_state)
        voltage = snapshot.get("tool_voltage")
        self.voltage_label.setText(
            "状态：未知" if voltage is None else "状态：{}V ACK".format(voltage)
        )
        if fresh.get("task_status") and not self._reported_qr_mismatch:
            self._reported_qr_mismatch = True
            if task_state != "IDLE":
                message = "QR任务当前状态为{}，请按现场实际状态操作".format(task_state)
                LOGGER.warning(message)
                self.diagnostic_widget.add_error(message)
        self._refresh_state_controls()

    def _cancel_joint_execution(self) -> None:
        """Cancellation is non-blocking and remains available while busy."""

        try:
            result = self._tasks.cancel_joint_control()
            LOGGER.warning("关节控制取消请求：%s", result)
            self.statusBar().showMessage("已请求取消；等待轨迹桥停止ACK", 5000)
        except Exception as error:
            self._operator_warning("取消关节控制失败：{}".format(error))

    def _cancel_target_execution(self) -> None:
        """Request a MoveIt stop even while a normal worker is busy."""

        if self._closing or self._config.get("offline", False):
            return
        future = self._emergency_executor.submit(
            self._tasks.cancel_target_execution)
        future.add_done_callback(
            lambda result: self._future_done("取消目标执行", True, result))
        LOGGER.warning("已提交公共标记目标取消请求")

    def _confirm_five_target_validation(self) -> None:
        answer = QMessageBox.question(
            self,
            "确认现场验收",
            "仅当已在监护人、物理急停和5%速度下完成五个目标及返回路径验收时才能确认。\n"
            "此操作只写入验收记录，不运动、不通电。是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self._submit("确认五目标现场验收",
                         self._tasks.confirm_five_target_validation)

    def _refresh_system_status(self) -> None:
        if self._config.get("offline", False) or not self._ros.connected:
            statuses: Dict[str, Dict[str, Any]] = {}
        else:
            try:
                statuses = self._tasks.subsystem_status()
            except Exception as error:
                LOGGER.debug("子系统状态刷新失败：%s", error)
                statuses = {}
        self._last_system_status = statuses
        for name, label in self._system_labels.items():
            status = statuses.get(name, {})
            running = bool(status.get("running"))
            owned = bool(status.get("owned"))
            if running:
                label.setText("运行中（HMI管理）" if owned else "运行中（外部）")
                label.setStyleSheet("color: #1b5e20; font-weight: bold;")
            elif status.get("exit_code") is not None:
                label.setText("已退出 code={}".format(status["exit_code"]))
                label.setStyleSheet("color: #b71c1c; font-weight: bold;")
            else:
                label.setText("未运行")
                label.setStyleSheet("color: #7f6000;")
        self._refresh_state_controls()

    def _refresh_state_controls(self) -> None:
        if not self._buttons:
            return
        state = self._state_machine.state
        ready = (self._ros.connected and not self._busy and not self._emergency_pending
                 and not self._config.get("offline", False))
        armed = self._tasks.execution_enabled
        system_status = self._last_system_status
        for name in ("base", "camera", "qr"):
            status = system_status.get(name, {})
            self._buttons["system_{}_start".format(name)].setEnabled(
                ready and not status.get("running", False)
            )
            self._buttons["system_{}_stop".format(name)].setEnabled(
                ready and status.get("owned", False)
            )
        base_running = bool(system_status.get("base", {}).get("running", False))
        qr_running = bool(system_status.get("qr", {}).get("running", False))
        control_mode = self._tasks.control_mode
        telemetry = self._ros.telemetry_snapshot()
        joint_server_live = bool(
            telemetry.get("health", {}).get("joint_control")
        )
        holding_slot = (telemetry.get("five_target_status") or {}).get(
            "holding_slot")
        normal_ready = ready and state != RobotState.SAFE_STOP
        self._buttons["teleop_start"].setEnabled(
            normal_ready and base_running and control_mode is None
            and not joint_server_live
        )
        self._buttons["teleop_stop"].setEnabled(
            ready and control_mode == "teleop"
        )
        self._buttons["tool_detect"].setEnabled(
            normal_ready and qr_running
        )
        tool_selected = self._tasks.selected_tool is not None
        for name in ("rack_calibrate", "target_preview", "target_plan"):
            self._buttons[name].setEnabled(
                normal_ready and qr_running and tool_selected and control_mode is None
            )
        self._buttons["rack_validate"].setEnabled(
            normal_ready and qr_running and tool_selected and control_mode is None
            and self._config.get("supervised_commissioning_authorized", False)
        )
        self._buttons["camera_observation"].setEnabled(
            normal_ready and armed and self._config.get(
                "qr_execution_authorized", False) and self._config.get(
                    "trajectory_execution_authorized", False) and qr_running
            and tool_selected and control_mode is None
        )
        self._buttons["target_execute"].setEnabled(
            normal_ready and armed and self._config.get(
                "qr_execution_authorized", False) and self._config.get(
                    "trajectory_execution_authorized", False) and qr_running
            and tool_selected and control_mode is None
        )
        self._buttons["target_return"].setEnabled(
            normal_ready and armed and self._config.get(
                "qr_execution_authorized", False) and self._config.get(
                    "trajectory_execution_authorized", False) and qr_running
            and tool_selected and control_mode is None and holding_slot is not None
        )
        self._buttons["confirm_held_tool"].setEnabled(
            normal_ready and armed and self._config.get(
                "qr_execution_authorized", False) and self._config.get(
                    "supervised_commissioning_authorized", False) and qr_running
            and tool_selected and control_mode is None and holding_slot is None
        )
        self._buttons["target_cancel"].setEnabled(
            self._ros.connected and qr_running and not self._config.get("offline", False)
        )
        self._buttons["force_enter"].setEnabled(
            normal_ready and armed and base_running and control_mode is None
            and not joint_server_live
        )
        self._buttons["sampling_start"].setEnabled(
            normal_ready and control_mode == "force_control"
        )
        self._buttons["sampling_stop"].setEnabled(
            ready and control_mode == "force_control"
        )
        # A bottom-layer emergency stop may outlive the Touch node that
        # reported it.  Keep the ACK-checked, non-motion recovery available in
        # IDLE as well as SAFE_STOP so the operator can clear that latch.
        self._buttons["recovery"].setEnabled(
            ready
        )
        self._buttons["power_on"].setEnabled(
            normal_ready and armed and base_running
        )
        self._buttons["power_off"].setEnabled(
            normal_ready and armed and base_running
        )
        self._buttons["emergency_stop"].setEnabled(not self._emergency_pending)
        self.tool_list.setEnabled(not self._busy and state != RobotState.SAFE_STOP)

    def _operator_warning(self, message: str) -> None:
        LOGGER.warning("操作未接受：%s", message)
        self.statusBar().showMessage(message, 7000)

    def closeEvent(self, event: QCloseEvent) -> None:  # noqa: N802 (Qt API)
        qr_active = self._last_task_state not in (
            "", "IDLE", "NOT_RUNNING", "UNKNOWN", "无状态"
        )
        if (self._busy or self._emergency_pending
                or self._tasks.control_mode is not None or qr_active):
            QMessageBox.warning(
                self,
                "不能退出",
                "仍有控制节点或QR任务活动。请先停止相应功能，或执行急停后再退出。",
            )
            event.ignore()
            return
        answer = QMessageBox.question(
            self,
            "退出",
            "确认退出上位机？\n请先停止遥操作/采样；退出不会替代物理急停。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            event.ignore()
            return

        self._closing = True
        self._telemetry_timer.stop()
        self._system_timer.stop()
        self._state_machine.remove_listener(self._state_listener)
        self._state_machine.remove_listener(self.recorder.transition)
        try:
            self._tasks.shutdown()
        except Exception:
            LOGGER.exception("关闭GUI管理的控制节点失败")
        self._ros.shutdown()
        self._executor.shutdown(wait=False)
        self._emergency_executor.shutdown(wait=False)
        APPLICATION_LOGGER.removeHandler(self.recorder)
        self.recorder.close()
        if self._qt_log_handler is not None:
            APPLICATION_LOGGER.removeHandler(self._qt_log_handler)
            self._qt_log_handler.close()
        event.accept()
