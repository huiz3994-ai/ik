"""Thread-safe atomic JSON persistence for RM75 trajectory resources."""

import json
import os
from pathlib import Path
import tempfile
import threading
from typing import Any, Dict, List

from .models import TRAJECTORY_ID, utc_now, validate_document


class TrajectoryRepository:
    def __init__(self, directory: str) -> None:
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.scan_errors: Dict[str, str] = {}

    @staticmethod
    def _validate_id(trajectory_id: str) -> str:
        value = str(trajectory_id)
        if not TRAJECTORY_ID.fullmatch(value):
            raise ValueError("trajectory_id无效")
        return value

    def _path(self, trajectory_id: str) -> Path:
        return self.directory / (self._validate_id(trajectory_id) + ".json")

    def get(self, trajectory_id: str) -> Dict[str, Any]:
        path = self._path(trajectory_id)
        with self._lock:
            if not path.is_file():
                raise KeyError("轨迹不存在：{}".format(trajectory_id))
            try:
                with path.open(encoding="utf-8") as stream:
                    payload = json.load(stream)
            except json.JSONDecodeError as error:
                raise ValueError("轨迹JSON解析失败：{}".format(error)) from error
            document = validate_document(payload)
            if document["trajectory_id"] != trajectory_id:
                raise ValueError("轨迹文件ID与文件名不一致")
            return document

    def list(self) -> List[Dict[str, Any]]:
        with self._lock:
            self.scan_errors = {}
            result = []
            for path in sorted(self.directory.glob("*.json")):
                try:
                    document = self.get(path.stem)
                except Exception as error:
                    self.scan_errors[path.name] = "JSON或轨迹结构无效：{}".format(error)
                    continue
                status = document["trajectory_status"]
                result.append({
                    "trajectory_id": document["trajectory_id"],
                    "name": document["name"],
                    "robot_model": document["robot_model"],
                    "created_at": document["created_at"],
                    "updated_at": document["updated_at"],
                    "waypoint_count": len(document["waypoints"]),
                    "trajectory_point_count": len(document["trajectory"]),
                    "duration_s": float(status.get("duration_s", 0.0)),
                    "state": status["state"],
                    "message": str(status.get("message", "")),
                })
            result.sort(key=lambda item: (item["updated_at"], item["trajectory_id"]), reverse=True)
            return result

    def save(self, document: Dict[str, Any]) -> Dict[str, Any]:
        normalized = validate_document(document)
        normalized["updated_at"] = utc_now()
        path = self._path(normalized["trajectory_id"])
        temporary_name = None
        with self._lock:
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=str(self.directory),
                    prefix=".{}-".format(normalized["trajectory_id"]),
                    suffix=".tmp", delete=False,
                ) as stream:
                    temporary_name = stream.name
                    json.dump(normalized, stream, ensure_ascii=False, indent=2, sort_keys=True)
                    stream.write("\n")
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary_name, str(path))
                temporary_name = None
                try:
                    directory_fd = os.open(str(self.directory), os.O_RDONLY)
                    try:
                        os.fsync(directory_fd)
                    finally:
                        os.close(directory_fd)
                except OSError:
                    # Some filesystems do not support directory fsync; the file
                    # itself was already flushed before the atomic replace.
                    pass
            finally:
                if temporary_name:
                    try:
                        os.unlink(temporary_name)
                    except FileNotFoundError:
                        pass
        return normalized

    def delete(self, trajectory_id: str) -> bool:
        path = self._path(trajectory_id)
        with self._lock:
            try:
                path.unlink()
            except FileNotFoundError:
                return False
            return True
