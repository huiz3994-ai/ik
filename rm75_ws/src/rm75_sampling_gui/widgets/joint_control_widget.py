"""Seven-axis actual/target panel. Sliders only edit the preview target."""

import math
from typing import Dict, Iterable, List, Optional

import numpy as np
from PyQt5.QtCore import QEvent, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QPushButton, QSlider, QVBoxLayout, QWidget,
)

from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel
from rm75_sampling_gui.model.joint_models import JointTargetModel


class JointControlWidget(QWidget):
    target_changed = pyqtSignal(object)
    execute_requested = pyqtSignal(object)
    cancel_requested = pyqtSignal()
    emergency_requested = pyqtSignal()

    def __init__(self, robot: RM75RobotModel, config: Dict, parent=None) -> None:
        super().__init__(parent)
        self.robot = robot
        self.config = config
        self._actual: Optional[np.ndarray] = None
        self.target_model = JointTargetModel(robot)
        self._target = self.target_model.vector()
        # Use the first fresh robot sample as the preview baseline. Without
        # this, untouched axes would retain the model's zero vector and a
        # one-click execution could unintentionally command them toward zero.
        # An operator edit before that sample is explicit and is not replaced.
        self._target_initialized = False
        self._editing = False
        self._execution_available = bool(
            config.get("joint_control_execution_authorized", False)
        )
        self._hold_index: Optional[int] = None
        self._hold_direction = 0
        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(int(config.get("joint_control", {}).get("jog_repeat_ms", 180)))
        self._hold_timer.timeout.connect(self._held_step)
        self.installEventFilter(self)

        root = QVBoxLayout(self)
        gate = bool(config.get("joint_control_execution_authorized", False))
        bridge = bool(config.get("trajectory_execution_authorized", False))
        supervised = bool(config.get("supervised_commissioning_authorized", False))
        commissioned = all(bool(config.get("joint_control", {}).get(key, False)) for key in (
            "execution_path_verified", "gui_loss_watchdog_verified", "collision_scene_verified"
        ))
        self.mode = QLabel(
            "{}｜关节控制授权={}｜受控适配器={}｜旧轨迹桥授权={}｜现场验收锁={}".format(
                "现场监护受限控制" if supervised else "离线预览模式",
                "ON" if gate else "OFF", "ON" if supervised else "OFF",
                "ON" if bridge else "OFF",
                "通过" if commissioned else "锁止",
            )
        )
        self.mode.setStyleSheet(
            "color:{};font-weight:bold;".format(
                "#1b5e20" if gate and supervised else
                ("#b71c1c" if not gate or (not bridge and not commissioned) else "#1b5e20")
            )
        )
        self.mode.setWordWrap(True)
        root.addWidget(self.mode)

        header = QGridLayout()
        for column, text in enumerate(("轴", "实际°", "速度°/s", "目标°", "误差°", "余量°", "点动")):
            header.addWidget(QLabel(text), 0, column)
        root.addLayout(header)
        self.actual_labels: List[QLabel] = []
        self.velocity_labels: List[QLabel] = []
        self.target_boxes: List[QDoubleSpinBox] = []
        self.error_labels: List[QLabel] = []
        self.margin_labels: List[QLabel] = []
        self.sliders: List[QSlider] = []
        for index, name in enumerate(JOINT_NAMES):
            line = QGridLayout()
            line.addWidget(QLabel("J{}".format(index + 1)), 0, 0)
            actual, velocity, error, margin = QLabel("—"), QLabel("—"), QLabel("—"), QLabel("—")
            self.actual_labels.append(actual)
            self.velocity_labels.append(velocity)
            self.error_labels.append(error)
            self.margin_labels.append(margin)
            line.addWidget(actual, 0, 1)
            line.addWidget(velocity, 0, 2)
            box = QDoubleSpinBox()
            lower, upper = self.robot.limits[name]
            box.setRange(math.degrees(lower), math.degrees(upper))
            box.setDecimals(3)
            box.setSingleStep(0.1)
            box.valueChanged.connect(lambda value, item=index: self._box_changed(item, value))
            self.target_boxes.append(box)
            line.addWidget(box, 0, 3)
            line.addWidget(error, 0, 4)
            line.addWidget(margin, 0, 5)
            minus = QPushButton("−")
            plus = QPushButton("+")
            for button, direction in ((minus, -1), (plus, 1)):
                button.setAutoRepeat(False)
                button.pressed.connect(lambda item=index, sign=direction: self._start_hold(item, sign))
                button.released.connect(self._stop_hold)
            jog = QHBoxLayout()
            jog.setSpacing(2)
            jog.addWidget(minus)
            jog.addWidget(plus)
            line.addLayout(jog, 0, 6)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(math.ceil(math.degrees(lower) * 10)),
                            int(math.floor(math.degrees(upper) * 10)))
            slider.valueChanged.connect(lambda value, item=index: self._slider_changed(item, value))
            self.sliders.append(slider)
            line.addWidget(slider, 1, 0, 1, 7)
            root.addLayout(line)

        options = QHBoxLayout()
        options.addWidget(QLabel("点动步长"))
        self.step = QComboBox()
        self.step.addItems(("0.1°", "0.5°", "1.0°"))
        options.addWidget(self.step)
        self.sync_button = QPushButton("同步实际关节")
        self.sync_button.clicked.connect(self.sync_actual)
        self.preview_button = QPushButton("预览目标")
        self.preview_button.clicked.connect(lambda: self.target_changed.emit(self._target.copy()))
        options.addWidget(self.sync_button)
        options.addWidget(self.preview_button)
        root.addLayout(options)

        self.parameters = QLabel(
            "轨迹速度使用厂商URDF各关节上限；不设置额外加速度/jerk拒绝阈值"
        )
        self.parameters.setWordWrap(True)
        root.addWidget(self.parameters)
        self.validation = QLabel("点击执行后自动完成安全检查并提交目标")
        self.validation.setWordWrap(True)
        root.addWidget(self.validation)
        actions = QHBoxLayout()
        self.execute_button = QPushButton("执行七轴目标")
        self.execute_button.clicked.connect(self._execute)
        self.cancel_button = QPushButton("取消执行")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_requested)
        self.estop_button = QPushButton("软件急停")
        self.estop_button.setObjectName("emergencyStop")
        self.estop_button.clicked.connect(self.emergency_requested)
        for button in (self.execute_button, self.cancel_button, self.estop_button):
            actions.addWidget(button)
        root.addLayout(actions)
        root.addStretch(1)
        self._update_execute()

    def _invalidate(self) -> None:
        self.validation.setText("点击执行后自动检查当前目标")
        self._update_execute()

    def _set_target(self, vector: np.ndarray, emit: bool = True) -> None:
        try:
            vector = self.target_model.set_vector(vector)
        except Exception:
            return
        self._target_initialized = True
        self._target = vector.copy()
        self._editing = True
        try:
            for index, value in enumerate(vector):
                degrees = math.degrees(float(value))
                self.target_boxes[index].setValue(degrees)
                self.sliders[index].setValue(int(round(degrees * 10)))
        finally:
            self._editing = False
        self._invalidate()
        self._update_errors()
        if emit:
            self.target_changed.emit(self._target.copy())

    def _box_changed(self, index: int, value: float) -> None:
        if self._editing:
            return
        vector = self._target.copy()
        vector[index] = math.radians(value)
        self._set_target(vector)

    def _slider_changed(self, index: int, value: int) -> None:
        if self._editing:
            return
        vector = self._target.copy()
        vector[index] = math.radians(value / 10.0)
        self._set_target(vector)

    def _step_rad(self) -> float:
        return math.radians(float(self.step.currentText().replace("°", "")))

    def _start_hold(self, index: int, direction: int) -> None:
        self._hold_index, self._hold_direction = index, direction
        self._held_step()
        self._hold_timer.start()

    def _held_step(self) -> None:
        if self._hold_index is None:
            return
        vector = self._target.copy()
        vector[self._hold_index] += self._hold_direction * self._step_rad()
        name = JOINT_NAMES[self._hold_index]
        lower, upper = self.robot.limits[name]
        vector[self._hold_index] = min(upper, max(lower, vector[self._hold_index]))
        self._set_target(vector)

    def _stop_hold(self) -> None:
        self._hold_timer.stop()
        self._hold_index = None

    def eventFilter(self, watched, event):  # noqa: N802
        if event.type() in (QEvent.WindowDeactivate, QEvent.FocusOut, QEvent.Hide):
            self._stop_hold()
        return super().eventFilter(watched, event)

    def sync_actual(self) -> None:
        if self._actual is not None:
            self._set_target(self._actual)

    def update_snapshot(self, data: Dict) -> None:
        values = data.get("joints", {})
        fresh = bool(data.get("fresh", {}).get("joints"))
        velocities = data.get("joint_velocities", {})
        source = str(data.get("joint_velocity_source", "unavailable"))
        if not data.get("fresh", {}).get("joint_velocities"):
            source = "unavailable"
        if fresh and all(name in values for name in JOINT_NAMES):
            self._actual = np.asarray([values[name] for name in JOINT_NAMES])
            if not self._target_initialized:
                self._set_target(self._actual)
        else:
            self._actual = None
        for index, name in enumerate(JOINT_NAMES):
            actual = None if self._actual is None else float(self._actual[index])
            self.actual_labels[index].setText("—" if actual is None else "{:.3f}".format(math.degrees(actual)))
            velocity = velocities.get(name) if source == "measured" else None
            self.velocity_labels[index].setText(
                "—" if velocity is None else "{:.3f}实测".format(math.degrees(float(velocity)))
            )
            lower, upper = self.robot.limits[name]
            margin = None if actual is None else min(actual - lower, upper - actual)
            self.margin_labels[index].setText(
                "—" if margin is None else "{:.3f}".format(math.degrees(margin))
            )
        self._update_errors()

    def _update_errors(self) -> None:
        for index in range(7):
            error = None if self._actual is None else self._target[index] - self._actual[index]
            self.error_labels[index].setText(
                "—" if error is None else "{:+.3f}".format(math.degrees(float(error)))
            )

    def set_validation_result(self, result: Dict) -> None:
        lines = ["[{}] {}：{}".format("√" if item["ok"] else "×", item["name"], item["detail"])
                 for item in result.get("checks", [])]
        self.validation.setText("\n".join(lines) or "安全检查：无结果")
        self._update_execute()

    def set_busy(self, active: bool) -> None:
        self.cancel_button.setEnabled(active)
        self._update_execute(active)

    def _update_execute(self, busy: bool = False) -> None:
        self.execute_button.setEnabled(
            not busy and self._execution_available
        )

    def _execute(self) -> None:
        if not self._execution_available:
            return
        self.validation.setText("正在自动检查并执行当前目标…")
        self.execute_requested.emit(self._target.copy())

    @property
    def target(self) -> np.ndarray:
        return self._target.copy()
