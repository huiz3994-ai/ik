"""Embedded VTK view for the actual and preview RM75-6FB models."""

import os
import time
from typing import Dict, Iterable, List, Tuple

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QCheckBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
)

from rm75_sampling_gui.model.robot_model import RM75RobotModel


class Robot3DWidget(QWidget):
    """Mouse-interactive digital twin rendered from vendor STL meshes."""

    def __init__(self, robot: RM75RobotModel, config: Dict, parent=None) -> None:
        super().__init__(parent)
        self.robot = robot
        self._actual = np.zeros(7)
        self._target = np.zeros(7)
        self._actual_actors: List[Tuple[object, object]] = []
        self._target_actors: List[Tuple[object, object]] = []
        self._axes: List[Tuple[str, object]] = []
        self._vtk = None
        self._renderer = None
        self._interactor = None

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        toolbar = QHBoxLayout()
        self.actual_toggle = QCheckBox("实际模型")
        self.actual_toggle.setChecked(True)
        self.target_toggle = QCheckBox("目标预览（青色透明）")
        self.target_toggle.setChecked(True)
        self.axes_toggle = QCheckBox("坐标系")
        self.axes_toggle.setChecked(True)
        toolbar.addWidget(self.actual_toggle)
        toolbar.addWidget(self.target_toggle)
        toolbar.addWidget(self.axes_toggle)
        for text, view in (("正视", "front"), ("侧视", "side"),
                           ("俯视", "top"), ("复位", "reset")):
            button = QPushButton(text)
            button.clicked.connect(lambda _checked=False, name=view: self.set_view(name))
            toolbar.addWidget(button)
        toolbar.addStretch(1)
        root.addLayout(toolbar)

        disabled = not bool(config.get("rendering", {}).get("enabled", True))
        disabled = disabled or os.environ.get("QT_QPA_PLATFORM") == "offscreen"
        try:
            if disabled:
                raise RuntimeError("offscreen test mode")
            import vtk
            from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
            self._vtk = vtk
            self._interactor = QVTKRenderWindowInteractor(self)
            self._renderer = vtk.vtkRenderer()
            self._renderer.SetBackground(0.055, 0.09, 0.13)
            self._renderer.SetBackground2(0.13, 0.20, 0.27)
            self._renderer.GradientBackgroundOn()
            self._interactor.GetRenderWindow().AddRenderer(self._renderer)
            style = vtk.vtkInteractorStyleTrackballCamera()
            self._interactor.SetInteractorStyle(style)
            root.addWidget(self._interactor, 1)
            self._build_scene()
            self._interactor.Initialize()
        except Exception as error:
            self.placeholder = QLabel(
                "3D渲染未启用\n{}\nURDF模型与离线关节计算仍然可用".format(error)
            )
            self.placeholder.setAlignment(Qt.AlignCenter)
            self.placeholder.setStyleSheet(
                "background:#102331;color:#c8d8e4;font-size:15px;padding:30px;"
            )
            root.addWidget(self.placeholder, 1)

        self.status = QLabel("数据源：等待 /joint_states　时间戳：—　延迟：—")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        self.actual_toggle.toggled.connect(self._visibility)
        self.target_toggle.toggled.connect(self._visibility)
        self.axes_toggle.toggled.connect(self._visibility)

    def _matrix(self, value: np.ndarray):
        matrix = self._vtk.vtkMatrix4x4()
        for row in range(4):
            for column in range(4):
                matrix.SetElement(row, column, float(value[row, column]))
        return matrix

    def _build_scene(self) -> None:
        vtk = self._vtk
        plane = vtk.vtkPlaneSource()
        plane.SetOrigin(-0.8, -0.8, 0.0)
        plane.SetPoint1(0.8, -0.8, 0.0)
        plane.SetPoint2(-0.8, 0.8, 0.0)
        plane.SetResolution(16, 16)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(plane.GetOutputPort())
        floor = vtk.vtkActor()
        floor.SetMapper(mapper)
        floor.GetProperty().SetColor(0.2, 0.25, 0.28)
        floor.GetProperty().SetRepresentationToWireframe()
        self._renderer.AddActor(floor)

        for visual in self.robot.visuals:
            if not visual.filename.lower().endswith(".stl"):
                continue
            reader = vtk.vtkSTLReader()
            reader.SetFileName(visual.filename)
            mesh_mapper = vtk.vtkPolyDataMapper()
            mesh_mapper.SetInputConnection(reader.GetOutputPort())
            actual = vtk.vtkActor()
            actual.SetMapper(mesh_mapper)
            actual.SetScale(*visual.scale)
            actual.GetProperty().SetColor(0.82, 0.86, 0.91)
            actual.GetProperty().SetSpecular(0.28)
            target = vtk.vtkActor()
            target.SetMapper(mesh_mapper)
            target.SetScale(*visual.scale)
            target.GetProperty().SetColor(0.0, 0.88, 0.95)
            target.GetProperty().SetOpacity(0.32)
            self._renderer.AddActor(actual)
            self._renderer.AddActor(target)
            self._actual_actors.append((visual, actual))
            self._target_actors.append((visual, target))

        for frame in ["base_link"] + list(self.robot.chain[index].name
                                            for index in range(7)) + ["Link7"]:
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(0.045, 0.045, 0.045)
            axes.SetShaftTypeToCylinder()
            axes.AxisLabelsOff()
            self._renderer.AddActor(axes)
            self._axes.append((frame, axes))
        self.update_actual(self._actual, False, None, None)
        self.update_target(self._target)
        self.set_view("reset")

    def _set_actors(self, actors: List[Tuple[object, object]], values: Iterable[float]) -> None:
        transforms = {visual.link + "|" + visual.filename: transform
                      for visual, transform in self.robot.visual_transforms(values)}
        for visual, actor in actors:
            actor.SetUserMatrix(self._matrix(
                transforms[visual.link + "|" + visual.filename]
            ))

    def update_actual(
        self, values: Iterable[float], realtime: bool,
        source_stamp: float, age: float
    ) -> None:
        self._actual = self.robot.validate_vector(values)
        if self._renderer is not None:
            self._set_actors(self._actual_actors, self._actual)
            frames = self.robot.forward_kinematics(self._actual)
            for frame, axes in self._axes:
                axes.SetUserMatrix(self._matrix(frames[frame]))
            for _visual, actor in self._actual_actors:
                actor.GetProperty().SetColor(
                    0.82 if realtime else 0.85,
                    0.86 if realtime else 0.22,
                    0.91 if realtime else 0.18,
                )
                actor.GetProperty().SetOpacity(1.0 if realtime else 0.55)
            self._render()
        stamp_text = "—" if source_stamp is None else "{:.3f}".format(source_stamp)
        age_text = "—" if age is None or not np.isfinite(age) else "{:.0f} ms".format(age * 1000)
        state = "实时" if realtime else "非实时/反馈过期"
        self.status.setText(
            "实际数据源：/joint_states　状态：{}　时间戳：{}　延迟：{}　目标数据源：离线预览".format(
                state, stamp_text, age_text
            )
        )

    def update_target(self, values: Iterable[float]) -> None:
        self._target = self.robot.validate_vector(values)
        if self._renderer is not None:
            self._set_actors(self._target_actors, self._target)
            self._render()

    def _visibility(self) -> None:
        for _visual, actor in self._actual_actors:
            actor.SetVisibility(self.actual_toggle.isChecked())
        for _visual, actor in self._target_actors:
            actor.SetVisibility(self.target_toggle.isChecked())
        for _frame, actor in self._axes:
            actor.SetVisibility(self.axes_toggle.isChecked())
        self._render()

    def set_view(self, name: str) -> None:
        if self._renderer is None:
            return
        camera = self._renderer.GetActiveCamera()
        focal = (0.0, 0.0, 0.42)
        if name == "front":
            camera.SetPosition(1.4, 0.0, 0.42)
            camera.SetViewUp(0.0, 0.0, 1.0)
        elif name == "side":
            camera.SetPosition(0.0, 1.4, 0.42)
            camera.SetViewUp(0.0, 0.0, 1.0)
        elif name == "top":
            camera.SetPosition(0.0, 0.0, 1.8)
            camera.SetViewUp(0.0, 1.0, 0.0)
        else:
            camera.SetPosition(1.15, 1.15, 0.95)
            camera.SetViewUp(0.0, 0.0, 1.0)
        camera.SetFocalPoint(*focal)
        self._renderer.ResetCameraClippingRange()
        self._render()

    def _render(self) -> None:
        if self._interactor is not None:
            self._interactor.GetRenderWindow().Render()

