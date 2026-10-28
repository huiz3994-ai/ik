"""Versioned, JSON-safe RM75 trajectory document model."""

import copy
from datetime import datetime, timezone
import math
import re
import uuid
from typing import Any, Dict, Iterable, List, Optional

from rm75_sampling_gui.model.robot_model import JOINT_NAMES


SCHEMA_VERSION = "1.0"
TRAJECTORY_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
STATES = {"EMPTY", "STALE", "READY", "VALIDATING", "EXECUTING", "COMPLETED", "CANCELLED", "ERROR"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _finite_vector(value: Any, length: int, label: str) -> List[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("{}必须是{}维数组".format(label, length))
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError("{}必须全部为有限数".format(label))
    return result


def _finite_number(value: Any, label: str, positive: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0.0):
        qualifier = "正有限数" if positive else "有限数"
        raise ValueError("{}必须是{}".format(label, qualifier))
    return result


def default_config() -> Dict[str, Any]:
    return {
        "interpolation_algorithm": "piecewise_quintic_c2",
        "speed_scale": 0.2,
        "acceleration_scale": 0.2,
        "sample_period_s": 0.02,
    }


def new_trajectory(
    name: str,
    trajectory_id: Optional[str] = None,
    robot_model: str = "RM75-6FB",
    joint_names: Iterable[str] = JOINT_NAMES,
    frame_id: str = "base_link",
    tool_id: Optional[str] = None,
) -> Dict[str, Any]:
    display_name = str(name).strip()
    if not display_name:
        raise ValueError("轨迹名称不能为空")
    resource_id = trajectory_id or str(uuid.uuid4())
    if not TRAJECTORY_ID.fullmatch(resource_id):
        raise ValueError("trajectory_id只能包含字母、数字、下划线或连字符，最长64字符")
    names = list(joint_names)
    if names != list(JOINT_NAMES):
        raise ValueError("RM75关节名称及顺序必须为joint1-joint7")
    now = utc_now()
    return {
        "schema_version": SCHEMA_VERSION,
        "trajectory_id": resource_id,
        "name": display_name,
        "robot_model": str(robot_model),
        "joint_names": names,
        "frame_id": str(frame_id),
        "tool_id": None if tool_id is None else str(tool_id),
        "created_at": now,
        "updated_at": now,
        "waypoints": [],
        "trajectory_config": default_config(),
        "trajectory": [],
        "trajectory_status": {
            "state": "EMPTY",
            "message": "尚未记录示教点",
            "generated_at": None,
            "waypoint_count": 0,
            "trajectory_point_count": 0,
            "duration_s": 0.0,
            "validation": {},
        },
    }


def _validate_pose(pose: Any, index: int) -> Dict[str, Any]:
    if not isinstance(pose, dict):
        raise ValueError("waypoint {}的Link7位姿无效".format(index))
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError("waypoint {}的Link7位姿缺少位置或四元数".format(index))
    result_position = {
        key: _finite_number(position.get(key), "waypoint {} position.{}".format(index, key))
        for key in ("x", "y", "z")
    }
    result_orientation = {
        key: _finite_number(orientation.get(key), "waypoint {} orientation.{}".format(index, key))
        for key in ("qx", "qy", "qz", "qw")
    }
    norm = math.sqrt(sum(value * value for value in result_orientation.values()))
    if norm < 1e-9 or abs(norm - 1.0) > 1e-3:
        raise ValueError("waypoint {}的Link7四元数必须归一化".format(index))
    return {"position": result_position, "orientation": result_orientation}


def validate_document(payload: Any) -> Dict[str, Any]:
    """Return a normalized deep copy or raise for an invalid persisted document."""

    if not isinstance(payload, dict):
        raise ValueError("轨迹JSON根节点必须是对象")
    result = copy.deepcopy(payload)
    required = (
        "schema_version", "trajectory_id", "name", "robot_model", "joint_names",
        "frame_id", "created_at", "updated_at", "waypoints", "trajectory_config",
        "trajectory", "trajectory_status",
    )
    missing = [key for key in required if key not in result]
    if missing:
        raise ValueError("轨迹JSON缺少字段：{}".format(", ".join(missing)))
    if result["schema_version"] != SCHEMA_VERSION:
        raise ValueError("不支持的schema_version：{}".format(result["schema_version"]))
    resource_id = str(result["trajectory_id"])
    if not TRAJECTORY_ID.fullmatch(resource_id):
        raise ValueError("trajectory_id无效")
    result["trajectory_id"] = resource_id
    result["name"] = str(result["name"]).strip()
    if not result["name"]:
        raise ValueError("轨迹名称不能为空")
    if list(result["joint_names"]) != list(JOINT_NAMES):
        raise ValueError("关节名称及顺序必须为joint1-joint7")
    if not isinstance(result["waypoints"], list):
        raise ValueError("waypoints必须是数组")
    seen = set()
    for sequence, waypoint in enumerate(result["waypoints"], 1):
        if not isinstance(waypoint, dict):
            raise ValueError("waypoint {}必须是对象".format(sequence))
        waypoint_id = str(waypoint.get("waypoint_id", ""))
        if not waypoint_id or waypoint_id in seen:
            raise ValueError("waypoint {}的唯一ID无效或重复".format(sequence))
        seen.add(waypoint_id)
        waypoint["waypoint_id"] = waypoint_id
        waypoint["sequence"] = sequence
        waypoint["joint_names"] = list(JOINT_NAMES)
        waypoint["joint_positions"] = _finite_vector(
            waypoint.get("joint_positions"), 7, "waypoint {}关节位置".format(sequence)
        )
        waypoint["link7_pose"] = _validate_pose(waypoint.get("link7_pose"), sequence)
        waypoint["ros_timestamp"] = _finite_number(
            waypoint.get("ros_timestamp"), "waypoint {} ROS时间戳".format(sequence)
        )

    config = result["trajectory_config"]
    if not isinstance(config, dict):
        raise ValueError("trajectory_config必须是对象")
    if config.get("interpolation_algorithm") != "piecewise_quintic_c2":
        raise ValueError("仅支持piecewise_quintic_c2插值")
    for key in ("speed_scale", "acceleration_scale"):
        value = _finite_number(config.get(key), key, positive=True)
        if value > 1.0:
            raise ValueError("{}必须在(0, 1]范围".format(key))
        config[key] = value
    config["sample_period_s"] = _finite_number(
        config.get("sample_period_s"), "sample_period_s", positive=True
    )

    if not isinstance(result["trajectory"], list):
        raise ValueError("trajectory必须是数组")
    previous_time = -1.0
    for index, point in enumerate(result["trajectory"]):
        if not isinstance(point, dict):
            raise ValueError("trajectory point {}必须是对象".format(index))
        for key in ("positions", "velocities", "accelerations"):
            point[key] = _finite_vector(point.get(key), 7, "trajectory point {} {}".format(index, key))
        stamp = _finite_number(point.get("time_from_start"), "trajectory point时间")
        if stamp < 0.0 or stamp <= previous_time:
            if not (index == 0 and stamp == 0.0):
                raise ValueError("trajectory time_from_start必须严格递增并从0开始")
        if index == 0 and stamp != 0.0:
            raise ValueError("trajectory必须从time_from_start=0开始")
        point["time_from_start"] = stamp
        previous_time = stamp

    status = result["trajectory_status"]
    if not isinstance(status, dict) or status.get("state") not in STATES:
        raise ValueError("trajectory_status.state无效")
    return result
