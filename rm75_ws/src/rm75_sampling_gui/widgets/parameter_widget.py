"""YAML-only read-only parameter view; no rosparam set or controller writes."""
from pathlib import Path
from typing import Any, Dict
import yaml
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem


def load_parameter_sources(config: Dict[str, Any]) -> Dict[str, Any]:
    import rospkg
    result = {"gui": {"path": config.get("config_path"), "values": dict(config)}}
    for name, key in (("teleop", "teleop_config"), ("force", "force_control_config")):
        try:
            path = Path(str(config[key])).expanduser()
            if not path.is_absolute():
                package = ("rm75_touch_teleop" if name == "teleop"
                           else str(config.get("force_control_package",
                                               "rm75_vertical_press")))
                path = Path(rospkg.RosPack().get_path(package)) / path
            with path.open(encoding="utf-8") as stream:
                values = yaml.safe_load(stream)
            if not isinstance(values, dict):
                raise ValueError("YAML root must be a mapping")
            result[name] = {"path": str(path), "values": values}
        except Exception as error:
            result[name] = {"path": config.get(key), "values": {}, "error": str(error)}
    return result


class ParameterWidget(QWidget):
    def __init__(self, config: Dict[str, Any], parent: QWidget = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("YAML参数快照（只读）；不是运行中ROS参数覆盖值。未配置的时间不推断。"))
        bindings = config.get("parameter_fields", [])
        sources = config.get("parameter_sources", {})
        self.table = QTableWidget(len(bindings), 4)
        self.table.setHorizontalHeaderLabels(["参数", "值", "单位", "来源文件 / 键"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        for row, binding in enumerate(bindings):
            source = sources.get(binding["source"], {})
            value = source.get("values", {})
            for part in binding["key"].split("."):
                value = value.get(part) if isinstance(value, dict) else None
            cells = (binding["label"], "未配置" if value is None else str(value),
                     binding.get("unit", ""), "{} / {}".format(source.get("path", "不可用"), binding["key"]))
            for col, cell in enumerate(cells):
                self.table.setItem(row, col, QTableWidgetItem(cell))
        layout.addWidget(self.table)
