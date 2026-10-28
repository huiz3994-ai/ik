"""Trajectory application service: feedback capture, editing and generation."""

import math
import threading
import uuid
import copy
from typing import Any, Callable, Dict, Iterable, List, Optional

from rm75_sampling_gui.model.robot_model import JOINT_NAMES

from .interpolation import generate_quintic
from .models import new_trajectory, utc_now
from .repository import TrajectoryRepository


class TrajectoryService:
    def __init__(
        self,
        repository: TrajectoryRepository,
        robot: Any,
        snapshot_provider: Callable[[], Dict[str, Any]],
        acceleration_limits: Iterable[float],
        collision_checker: Optional[Callable[[Iterable[float]], bool]] = None,
        collision_required: bool = True,
        joint_limit_margin_rad: float = 0.0,
        executor: Optional[Callable[[List[Dict[str, Any]], Callable[[Dict[str, Any]], None], threading.Event], Dict[str, Any]]] = None,
        stop_executor: Optional[Callable[[], Dict[str, Any]]] = None,
    ) -> None:
        self.repository = repository
        self.robot = robot
        self.snapshot_provider = snapshot_provider
        self.acceleration_limits = [float(value) for value in acceleration_limits]
        if len(self.acceleration_limits) != 7 or not all(
                math.isfinite(value) and value > 0.0 for value in self.acceleration_limits):
            raise ValueError("关节加速度限制必须是七个正有限数")
        self.collision_checker = collision_checker
        self.collision_required = bool(collision_required)
        self.joint_limit_margin_rad = float(joint_limit_margin_rad)
        self.executor = executor
        self.stop_executor = stop_executor
        self._lock = threading.RLock()
        self._execution_thread: Optional[threading.Thread] = None
        self._cancel_event = threading.Event()
        self._active_waypoint_indices: List[int] = []
        self._execution: Dict[str, Any] = {
            "state": "IDLE", "trajectory_id": None, "message": "待机",
            "current_point": 0, "total_points": 0,
            "current_waypoint": 0, "total_waypoints": 0,
            "progress": 0.0, "max_error_rad": None, "error": "",
        }

    def list(self) -> List[Dict[str, Any]]:
        return self.repository.list()

    def get(self, trajectory_id: str) -> Dict[str, Any]:
        return self.repository.get(trajectory_id)

    def create(
        self, name: str, trajectory_id: Optional[str] = None,
        tool_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        with self._lock:
            document = new_trajectory(name, trajectory_id=trajectory_id, tool_id=tool_id)
            return self.repository.create(document)

    def save(self, document: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            self._require_mutable(document["trajectory_id"])
            return self.repository.save(document)

    def delete(self, trajectory_id: str) -> bool:
        with self._lock:
            self._require_mutable(trajectory_id)
            return self.repository.delete(trajectory_id)

    def _require_mutable(self, trajectory_id: str) -> None:
        if (self._execution.get("state") == "EXECUTING" and
                self._execution.get("trajectory_id") == trajectory_id):
            raise RuntimeError("轨迹执行中，禁止修改或删除当前资源")

    @staticmethod
    def _finite(value: Any, label: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as error:
            raise RuntimeError("{}反馈无效".format(label)) from error
        if not math.isfinite(result):
            raise RuntimeError("{}反馈必须是有限数".format(label))
        return result

    def _capture(self) -> Dict[str, Any]:
        snapshot = self.snapshot_provider()
        fresh = snapshot.get("fresh") or {}
        if not fresh.get("joints"):
            raise RuntimeError("/joint_states反馈不新鲜，拒绝记录示教点")
        if not fresh.get("tcp"):
            raise RuntimeError("base_link到Link7的TF反馈不新鲜，拒绝记录示教点")
        joints = snapshot.get("joints") or {}
        if not all(name in joints for name in JOINT_NAMES):
            raise RuntimeError("关节反馈缺少joint1-joint7")
        positions = [self._finite(joints[name], name) for name in JOINT_NAMES]
        tcp = snapshot.get("tcp") or {}
        position = {
            key: self._finite(tcp.get(key), "Link7 {}".format(key))
            for key in ("x", "y", "z")
        }
        orientation = {
            key: self._finite(tcp.get(key), "Link7 TF {}".format(key))
            for key in ("qx", "qy", "qz", "qw")
        }
        norm = math.sqrt(sum(value * value for value in orientation.values()))
        if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
            raise RuntimeError("Link7 TF四元数无效或未归一化")
        stamps = snapshot.get("source_stamps") or {}
        joint_stamp = self._finite(stamps.get("joints"), "关节ROS时间戳")
        tf_stamp = self._finite(stamps.get("tcp"), "TF ROS时间戳")
        if joint_stamp <= 0.0 or tf_stamp <= 0.0:
            raise RuntimeError("反馈ROS时间戳必须为正数")
        return {
            "waypoint_id": str(uuid.uuid4()),
            "joint_names": list(JOINT_NAMES),
            "joint_positions": positions,
            "link7_pose": {"position": position, "orientation": orientation},
            "ros_timestamp": joint_stamp,
            "tf_ros_timestamp": tf_stamp,
        }

    @staticmethod
    def _invalidate(document: Dict[str, Any], message: str) -> None:
        document["trajectory"] = []
        count = len(document["waypoints"])
        document["trajectory_status"] = {
            "state": "STALE" if count else "EMPTY",
            "message": message if count else "尚未记录示教点",
            "generated_at": None,
            "waypoint_count": count,
            "trajectory_point_count": 0,
            "duration_s": 0.0,
            "validation": {},
        }

    @staticmethod
    def _renumber(document: Dict[str, Any]) -> None:
        for sequence, waypoint in enumerate(document["waypoints"], 1):
            waypoint["sequence"] = sequence
            waypoint["joint_names"] = list(JOINT_NAMES)

    def record_current_waypoint(self, trajectory_id: str) -> Dict[str, Any]:
        with self._lock:
            self._require_mutable(trajectory_id)
        captured = self._capture()
        with self._lock:
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            captured["sequence"] = len(document["waypoints"]) + 1
            document["waypoints"].append(captured)
            self._invalidate(document, "示教点已修改，需要重新生成轨迹")
            return self.repository.save(document)

    def delete_waypoint(self, trajectory_id: str, waypoint_id: str) -> Dict[str, Any]:
        with self._lock:
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            kept = [item for item in document["waypoints"] if item["waypoint_id"] != waypoint_id]
            if len(kept) == len(document["waypoints"]):
                raise KeyError("示教点不存在：{}".format(waypoint_id))
            document["waypoints"] = kept
            self._renumber(document)
            self._invalidate(document, "示教点已删除，需要重新生成轨迹")
            return self.repository.save(document)

    def move_waypoint(self, trajectory_id: str, waypoint_id: str, offset: int) -> Dict[str, Any]:
        if offset not in (-1, 1):
            raise ValueError("示教点移动offset只能为-1或1")
        with self._lock:
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            indices = [
                index for index, item in enumerate(document["waypoints"])
                if item["waypoint_id"] == waypoint_id
            ]
            if not indices:
                raise KeyError("示教点不存在：{}".format(waypoint_id))
            current = indices[0]
            target = current + offset
            if target < 0 or target >= len(document["waypoints"]):
                return document
            document["waypoints"][current], document["waypoints"][target] = (
                document["waypoints"][target], document["waypoints"][current]
            )
            self._renumber(document)
            self._invalidate(document, "示教点顺序已修改，需要重新生成轨迹")
            return self.repository.save(document)

    def clear_waypoints(self, trajectory_id: str) -> Dict[str, Any]:
        with self._lock:
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            document["waypoints"] = []
            self._invalidate(document, "尚未记录示教点")
            return self.repository.save(document)

    def update_metadata(
        self, trajectory_id: str, name: Optional[str] = None,
        trajectory_config: Optional[Dict[str, Any]] = None,
        tool_id: Any = Ellipsis,
    ) -> Dict[str, Any]:
        with self._lock:
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            changed_generation_input = False
            if name is not None:
                document["name"] = str(name).strip()
            if trajectory_config is not None:
                changed_generation_input = any(
                    document["trajectory_config"].get(key) != value
                    for key, value in trajectory_config.items()
                )
                document["trajectory_config"].update(dict(trajectory_config))
            if tool_id is not Ellipsis:
                document["tool_id"] = None if tool_id is None else str(tool_id)
            if changed_generation_input:
                self._invalidate(document, "轨迹参数已修改，需要重新生成轨迹")
            return self.repository.save(document)

    def generate(self, trajectory_id: str) -> Dict[str, Any]:
        with self._lock:
            if self._execution.get("state") == "EXECUTING":
                raise RuntimeError("轨迹执行中，禁止生成轨迹以免阻塞停止请求")
            self._require_mutable(trajectory_id)
            document = self.repository.get(trajectory_id)
            if len(document["waypoints"]) < 2:
                raise ValueError("生成轨迹至少需要两个示教点")
            velocity_limits = [self.robot.velocity_limits[name] for name in JOINT_NAMES]
            try:
                generated = generate_quintic(
                    document["waypoints"], document["trajectory_config"],
                    velocity_limits, self.acceleration_limits,
                )
                collision_checks = 0
                for index, point in enumerate(generated["points"]):
                    violations = self.robot.limit_violations(
                        point["positions"], self.joint_limit_margin_rad
                    )
                    if violations:
                        raise RuntimeError(
                            "密集轨迹点{}侵入关节限位：{}".format(index, ", ".join(violations))
                        )
                    if self.collision_checker is None:
                        if self.collision_required:
                            raise RuntimeError("MoveIt碰撞检查不可用")
                    else:
                        collision_checks += 1
                        if self.collision_checker(point["positions"]):
                            raise RuntimeError("MoveIt在密集轨迹点{}检测到碰撞".format(index))
            except Exception as error:
                document["trajectory"] = []
                document["trajectory_status"] = {
                    "state": "ERROR", "message": str(error), "generated_at": None,
                    "waypoint_count": len(document["waypoints"]),
                    "trajectory_point_count": 0, "duration_s": 0.0,
                    "validation": {},
                }
                self.repository.save(document)
                raise
            document["trajectory"] = generated["points"]
            document["trajectory_status"] = {
                "state": "READY",
                "message": "轨迹已生成并通过关节限位与碰撞检查",
                "generated_at": utc_now(),
                "waypoint_count": len(document["waypoints"]),
                "trajectory_point_count": len(generated["points"]),
                "duration_s": generated["duration"],
                "waypoint_indices": generated["waypoint_indices"],
                "validation": {
                    "joint_limits_checked": len(generated["points"]),
                    "collision_checked": collision_checks,
                    "limit_summary": generated["limit_summary"],
                },
            }
            return self.repository.save(document)

    def execution_status(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._execution)

    def _progress(self, update: Dict[str, Any]) -> None:
        with self._lock:
            if self._execution["state"] != "EXECUTING":
                return
            current = max(0, int(update.get("current_point", 0)))
            current_waypoint = sum(
                1 for point_index in self._active_waypoint_indices
                if point_index <= current
            )
            self._execution.update({
                "current_point": current,
                "total_points": int(update.get("total_points", self._execution["total_points"])),
                "current_waypoint": current_waypoint,
                "progress": max(0.0, min(1.0, float(update.get("progress", 0.0)))),
                "max_error_rad": update.get("max_error_rad"),
            })

    def execute(self, trajectory_id: str) -> Dict[str, Any]:
        if self.executor is None:
            raise RuntimeError("轨迹执行器未配置")
        with self._lock:
            if self._execution.get("state") == "EXECUTING":
                raise RuntimeError("已有轨迹正在执行")
            document = self.repository.get(trajectory_id)
            if not document["trajectory"]:
                raise RuntimeError("轨迹尚未生成或已失效")
            if document["trajectory_status"]["state"] not in {
                    "READY", "COMPLETED", "CANCELLED"}:
                raise RuntimeError("轨迹状态{}不可执行".format(
                    document["trajectory_status"]["state"]
                ))
            waypoint_indices = list(document["trajectory_status"].get(
                "waypoint_indices", []
            ))
            if len(waypoint_indices) != len(document["waypoints"]):
                raise RuntimeError("轨迹缺少示教点索引，需要重新生成")
            # One event per accepted run: stopping before the worker starts or
            # during validation must survive every downstream execution entry.
            self._cancel_event = threading.Event()
            self._active_waypoint_indices = waypoint_indices
            self._execution = {
                "state": "EXECUTING", "trajectory_id": trajectory_id,
                "message": "轨迹执行中", "current_point": 0,
                "total_points": len(document["trajectory"]),
                "current_waypoint": 0, "total_waypoints": len(document["waypoints"]),
                "progress": 0.0, "max_error_rad": None, "error": "",
            }
            document["trajectory_status"]["state"] = "EXECUTING"
            document["trajectory_status"]["message"] = "轨迹执行中"
            self.repository.save(document)
            points = copy.deepcopy(document["trajectory"])
            self._execution_thread = threading.Thread(
                target=self._execution_worker,
                args=(trajectory_id, points, self._cancel_event),
                name="rm75-trajectory-execution",
                daemon=True,
            )
            self._execution_thread.start()
            return {"accepted": True, **copy.deepcopy(self._execution)}

    def _execution_worker(
        self, trajectory_id: str, points: List[Dict[str, Any]], cancel_event: threading.Event,
    ) -> None:
        state = "ERROR"
        message = "轨迹执行失败"
        error_text = ""
        try:
            result = (self.executor(points, self._progress, cancel_event)
                      if self.executor and not cancel_event.is_set() else {"cancelled": True})
            cancelled = bool(result.get("cancelled")) or cancel_event.is_set()
            state = "CANCELLED" if cancelled else "COMPLETED"
            message = "轨迹执行已停止" if cancelled else "轨迹执行完成"
        except Exception as error:
            error_text = str(error)
            message = "轨迹执行失败：{}".format(error)
        with self._lock:
            document = self.repository.get(trajectory_id)
            document["trajectory_status"]["state"] = state
            document["trajectory_status"]["message"] = message
            document["trajectory_status"]["last_execution_at"] = utc_now()
            self.repository.save(document)
            self._execution.update({"state": state, "message": message, "error": error_text})
            if state == "COMPLETED":
                self._execution.update({
                    "current_point": max(0, len(points) - 1),
                    "current_waypoint": self._execution["total_waypoints"],
                    "progress": 1.0,
                })

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self._execution.get("state") != "EXECUTING":
                return {"cancel_requested": False, "state": self._execution.get("state")}
            self._cancel_event.set()
            # Keep the run reserved until stop has been dispatched, otherwise a
            # delayed stop could cancel a newly accepted run.
            result = self.stop_executor() if self.stop_executor is not None else {"cancel_requested": True}
            return {"cancel_requested": True, **(result or {})}
