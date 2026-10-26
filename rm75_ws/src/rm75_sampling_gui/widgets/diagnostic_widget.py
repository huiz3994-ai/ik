"""Health diagnostics with bounded, deduplicated fault history."""
from collections import deque
from typing import Any, Dict
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPlainTextEdit


class DiagnosticWidget(QWidget):
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.errors = QPlainTextEdit()
        self.errors.setReadOnly(True)
        self.errors.setMaximumBlockCount(200)
        layout.addWidget(self.errors)
        self.previous = {}
        self.last_log = ""
        self.history = deque(maxlen=200)

    def add_error(self, message: str) -> None:
        if message == self.last_log:
            return
        self.last_log = message
        kind = ("ACK失败" if "ACK" in message else
                "SAFE_STOP" if "SAFE_STOP" in message else
                "timeout" if "timeout" in message.lower() or "超时" in message else
                "sensor lost" if "sensor" in message.lower() or "反馈" in message else "ERROR")
        text = "[{}] {}".format(kind, message)
        self.history.append(text)
        self.errors.appendPlainText(text)

    def update_snapshot(self, data: Dict[str, Any]) -> None:
        fresh, health = data.get("fresh", {}), data.get("health", {})
        task = data.get("task_status", {})
        checks = {
            "通信": bool(health.get("master") and health.get("driver") and fresh.get("joints")),
            "力反馈": bool(fresh.get("force_z") or fresh.get("filtered_force_z")),
            "工具": bool(fresh.get("task_status") and task.get("state") in ("IDLE", "HOLDING")
                         and data.get("tool_voltage") in (0, 24)),
            "QR": bool(health.get("qr") and health.get("marker") and fresh.get("vision_status")),
        }
        self.summary.setText("\n".join("{}: {}".format(name, "OK" if ok else "ERROR")
                                       for name, ok in checks.items()))
        for name, ok in checks.items():
            if not ok and self.previous.get(name) is not False:
                self.add_error("sensor lost / timeout：{}不可用或数据过期".format(name))
        self.previous = checks
        if data.get("control_mode") == "SAFE_STOP":
            self.add_error("SAFE_STOP：" + str(data.get("stop_reason", "")))
        if task.get("last_error"):
            self.add_error(str(task["last_error"]))

