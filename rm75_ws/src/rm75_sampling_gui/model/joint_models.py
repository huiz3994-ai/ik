"""Thread-safe seven-axis feedback and preview-target models."""

import math
import threading
import time
from typing import Any, Dict, Iterable, Optional

import numpy as np

from .robot_model import JOINT_NAMES, RM75RobotModel, RobotModelError


class JointStateModel:
    """Name-mapped measured feedback; array ordering is never trusted."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self.positions: Optional[np.ndarray] = None
        self.velocities: Optional[np.ndarray] = None
        self.velocity_source = "unavailable"
        self.source_stamp: Optional[float] = None
        self.received_monotonic: Optional[float] = None

    def update_snapshot(self, snapshot: Dict[str, Any]) -> bool:
        values = snapshot.get("joints", {})
        fresh = bool(snapshot.get("fresh", {}).get("joints"))
        if not fresh or not all(name in values for name in JOINT_NAMES):
            return False
        positions = np.asarray([values[name] for name in JOINT_NAMES], dtype=float)
        if not np.isfinite(positions).all():
            return False
        velocity_data = snapshot.get("joint_velocities", {})
        velocity_source = str(snapshot.get("joint_velocity_source", "unavailable"))
        velocities = None
        if all(name in velocity_data for name in JOINT_NAMES):
            candidate = np.asarray([velocity_data[name] for name in JOINT_NAMES], dtype=float)
            if np.isfinite(candidate).all():
                velocities = candidate
        with self._lock:
            self.positions = positions
            self.velocities = velocities
            self.velocity_source = velocity_source if velocities is not None else "unavailable"
            self.source_stamp = snapshot.get("source_stamps", {}).get("joints")
            self.received_monotonic = time.monotonic()
        return True

    def vector(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self.positions is None else self.positions.copy()


class JointTargetModel:
    """Offline preview target constrained by the exact vendor URDF limits."""

    def __init__(self, robot: RM75RobotModel) -> None:
        self.robot = robot
        self._values = np.zeros(7, dtype=float)
        self._lock = threading.RLock()
        self._revision = 0

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    def vector(self) -> np.ndarray:
        with self._lock:
            return self._values.copy()

    def set_vector(self, values: Iterable[float]) -> np.ndarray:
        vector = self.robot.validate_vector(values)
        violations = self.robot.limit_violations(vector)
        if violations:
            raise RobotModelError("目标超出URDF限位：{}".format(", ".join(violations)))
        with self._lock:
            self._values = vector.copy()
            self._revision += 1
            return self._values.copy()

    def set_joint(self, index: int, value: float) -> np.ndarray:
        if index < 0 or index >= 7 or not math.isfinite(value):
            raise RobotModelError("无效关节目标")
        vector = self.vector()
        vector[index] = value
        return self.set_vector(vector)

