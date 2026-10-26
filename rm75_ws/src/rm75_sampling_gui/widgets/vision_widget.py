"""Read-only RGB and QR observations. ROS callbacks never access Qt objects."""
from typing import Any, Dict
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem


class VisionWidget(QWidget):
    def __init__(self, config: Dict[str, Any], parent: QWidget = None) -> None:
        super().__init__(parent)
        self.tools = config.get("tools", [])
        layout = QVBoxLayout(self)
        self.image = QLabel("未发现RGB图像话题；等待自动订阅")
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(320, 240)
        self.image.setStyleSheet("background: #142536; color: white;")
        layout.addWidget(self.image, 1)
        self.topic = QLabel("RGB接口待连接")
        layout.addWidget(self.topic)
        self.table = QTableWidget(len(self.tools), 6)
        self.table.setHorizontalHeaderLabels([
            "工具", "公共Tag", "检测/质量", "控制器示教 (mm/rad)",
            "视觉目标末端 (m/q)", "MoveIt Link7目标 (m/q)"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table)
        self.sequence = -1

    def update_snapshot(self, data: Dict[str, Any]) -> None:
        fresh = data.get("fresh", {})
        self.topic.setText("RGB: " + (data.get("image_topic") or "未发现（保留接口）"))
        sequence = data.get("sequences", {}).get("image", 0)
        if not fresh.get("image"):
            self.image.setPixmap(QPixmap())
            self.image.setText("RGB未连接或图像已过期")
        elif sequence != self.sequence:
            self.sequence = sequence
            msg = data.get("image_message")
            try:
                formats = {"rgb8": (QImage.Format_RGB888, 3), "bgr8": (QImage.Format_RGB888, 3),
                           "rgba8": (QImage.Format_RGBA8888, 4), "mono8": (QImage.Format_Grayscale8, 1)}
                fmt, channels = formats[msg.encoding]
                if msg.step < msg.width*channels or len(msg.data) < msg.step*msg.height:
                    raise ValueError("incomplete image buffer")
                image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, fmt).copy()
                if msg.encoding == "bgr8":
                    image = image.rgbSwapped()
                self.image.setPixmap(QPixmap.fromImage(image).scaled(
                    self.image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            except (KeyError, ValueError, AttributeError):
                self.image.setText("图像编码不支持或数据不完整")
        status = data.get("vision_status", {})
        detection_items = status.get("detections", [])
        common_mode = any("marker_id" in item for item in detection_items) or bool(
            data.get("five_target_status") or data.get("five_targets"))
        detections = {item.get("marker_id", item.get("slot_id")): item
                      for item in detection_items}
        common = detections.get(0, {})
        target_message = data.get("five_targets", {}) if fresh.get("five_targets") else {}
        targets = {int(item.get("slot_id", -1)): item
                   for item in target_message.get("targets", [])}
        for row, tool in enumerate(self.tools):
            slot = tool["slot_id"]
            legacy_slot = row
            detection = common if common_mode else detections.get(legacy_slot, {})
            state = ("稳定" if detection.get("stable") else detection.get("reason", "未检测"))
            if ("pixel_size" in detection or
                    "reprojection_error_px" in detection):
                state += " / {:.1f}px / {:.3f}px".format(
                    float(detection.get("pixel_size", 0.0)),
                    float(detection.get("reprojection_error_px", 0.0)))
            if not fresh.get("vision_status"):
                state = "观测已过期 / 未连接"
            target = targets.get(int(slot), {})
            tcp = target.get("tcp", {})
            link7 = target.get("link7", {})
            tcp_text = "—" if not tcp else "{} / {}".format(
                tcp.get("translation"), tcp.get("quaternion_xyzw"))
            link7_text = "—" if not link7 else "{} / {}".format(
                link7.get("translation"), link7.get("quaternion_xyzw"))
            reference = tool.get("reference_controller_pose", "—")
            if not common_mode:
                key = "marker_{}".format(legacy_slot)
                pose = data.get(key, {}) if fresh.get(key) else {}
                if pose:
                    reference = "{} / {} ({})".format(
                        pose.get("translation"), pose.get("quaternion_xyzw"),
                        pose.get("frame_id"))
            values = (tool["name"], "ID=0" if common_mode else legacy_slot, state,
                      reference, tcp_text, link7_text)
            for col, value in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(str(value)))
