"""Bounded force plots drawn by Qt; no control or numerical force filtering."""
from collections import deque
from typing import Any, Dict
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from .status_widget import number


class ForcePlot(QWidget):
    def __init__(self, capacity: int, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.samples = deque(maxlen=capacity)
        self.setMinimumHeight(260)

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#142536"))
        painter.setRenderHint(QPainter.Antialiasing)
        left, top, width, height = 65, 35, max(1, self.width()-90), max(1, self.height()-65)
        values = [v for _, row in self.samples for v in row if v is not None]
        lo, hi = (min(values), max(values)) if values else (-1.0, 1.0)
        pad = max(1.0, (hi-lo)*0.15)
        lo, hi = lo-pad, hi+pad
        painter.setPen(QColor("#8faabd"))
        painter.drawText(8, 20, "N")
        painter.drawText(5, top+10, "{:.1f}".format(hi))
        painter.drawText(5, top+height, "{:.1f}".format(lo))
        painter.drawRect(left, top, width, height)
        if not self.samples:
            painter.drawText(self.rect(), Qt.AlignCenter, "等待实时力数据")
            return
        t0, t1 = self.samples[0][0], self.samples[-1][0]
        span = max(0.1, t1-t0)
        colors = ("#53c9f5", "#f7b267", "#75efb0")
        for axis, color in enumerate(colors):
            painter.setPen(QPen(QColor(color), 3 if axis == 2 else 1.5))
            segment = []
            for timestamp, row in self.samples:
                if row[axis] is None:
                    if len(segment) > 1:
                        painter.drawPolyline(QPolygonF(segment))
                    segment = []
                    continue
                segment.append(QPointF(left+(timestamp-t0)/span*width,
                                       top+(hi-row[axis])/(hi-lo)*height))
            if len(segment) > 1:
                painter.drawPolyline(QPolygonF(segment))
            painter.drawText(left+axis*145, 22, ("Fx 原始", "Fy 原始", "Fz 实时")[axis])
        painter.setPen(QColor("#8faabd"))
        painter.drawText(left, self.height()-8, "最近 {:.1f} s（断流处留空）".format(span))


class ForceMonitorWidget(QWidget):
    def __init__(self, config: Dict[str, Any], parent: QWidget = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.values = QLabel("当前力：—   目标力：—   误差：—")
        self.values.setStyleSheet("font-size: 22px; font-weight: bold;")
        layout.addWidget(self.values)
        self.plot = ForcePlot(int(config.get("monitoring", {}).get("plot_samples", 600)))
        layout.addWidget(self.plot)
        self.source = QLabel(
            "Fx/Fy: Six_Force 原始补偿值；Fz: 优先使用力控 filtered_force_z，"
            "其次 force_z，力控未运行时显示 Six_Force 原始 Fz。\n"
            "不同来源/符号约定分别保留；误差 = force_setpoint − 当前力控Fz。")
        self.source.setWordWrap(True)
        layout.addWidget(self.source)

    def update_snapshot(self, data: Dict[str, Any]) -> None:
        fresh = data.get("fresh", {})
        wrench = data.get("wrench", {}) if fresh.get("wrench") else {}
        fz = data.get("filtered_force_z") if fresh.get("filtered_force_z") else (
            data.get("force_z") if fresh.get("force_z") else wrench.get("Fz"))
        target = data.get("force_setpoint") if fresh.get("force_setpoint") else None
        error = target-fz if fz is not None and target is not None else None
        self.values.setText("当前Fz：{} N    目标：{} N    误差：{} N".format(
            number(fz, 2), number(target, 2), number(error, 2)))
        self.plot.samples.append((data.get("timestamp", 0.0), (wrench.get("Fx"), wrench.get("Fy"), fz)))
        self.plot.update()
