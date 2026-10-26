"""RM75-6FB URDF loader and forward kinematics.

All geometry, axes, origins and limits are read from the vendor xacro.  This
module deliberately contains no ROS publishers and no motion interface.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple
import xml.etree.ElementTree as ET

import numpy as np
import rospkg
import xacro
from urdf_parser_py.urdf import URDF


JOINT_NAMES = tuple("joint{}".format(index) for index in range(1, 8))


class RobotModelError(RuntimeError):
    """Raised when the selected vendor model is incomplete or inconsistent."""


@dataclass(frozen=True)
class VisualMesh:
    link: str
    filename: str
    scale: Tuple[float, float, float]
    origin: np.ndarray


def _rpy_matrix(rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def _origin_matrix(origin: object) -> np.ndarray:
    result = _rpy_matrix(getattr(origin, "rpy", None) or (0.0, 0.0, 0.0))
    result[:3, 3] = np.asarray(
        getattr(origin, "xyz", None) or (0.0, 0.0, 0.0), dtype=float
    )
    return result


def _axis_angle(axis: Iterable[float], angle: float) -> np.ndarray:
    vector = np.asarray(tuple(axis), dtype=float)
    norm = np.linalg.norm(vector)
    if not math.isfinite(norm) or norm <= 0.0:
        raise RobotModelError("URDF joint axis is zero or non-finite")
    x, y, z = vector / norm
    c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
        [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
        [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
    ])
    return result


class RM75RobotModel:
    """Parsed RM75-6FB model with deterministic base-to-Link7 FK."""

    def __init__(self, xacro_path: Optional[str] = None) -> None:
        if xacro_path is None:
            description = Path(rospkg.RosPack().get_path("rm_description"))
            xacro_path = str(
                description / "urdf/RM75_6FB/rm_75_6fb_description.urdf.xacro"
            )
        self.xacro_path = str(Path(xacro_path).resolve())
        try:
            xml = xacro.process_file(
                self.xacro_path, mappings={"link7_type": "Link7_6fb"}
            ).toxml()
            # urdf_parser_py does not model transmission actuator tags and
            # otherwise emits a warning for each joint. They are irrelevant to
            # visual/FK loading, so remove only those expanded top-level blocks.
            root = ET.fromstring(xml)
            for element in list(root):
                if element.tag in ("transmission", "gazebo"):
                    root.remove(element)
            self.urdf = URDF.from_xml_string(
                ET.tostring(root, encoding="unicode")
            )
        except Exception as error:
            raise RobotModelError(
                "无法展开RM75-6FB厂商xacro {}：{}".format(self.xacro_path, error)
            )

        self._joints = {joint.name: joint for joint in self.urdf.joints}
        missing = [name for name in JOINT_NAMES if name not in self._joints]
        if missing:
            raise RobotModelError("厂商URDF缺少关节：{}".format(", ".join(missing)))
        self.chain = [self._joints[name] for name in JOINT_NAMES]
        expected_parent = "base_link"
        for joint in self.chain:
            if joint.parent != expected_parent:
                raise RobotModelError(
                    "关节链不连续：{} parent={}，期望{}".format(
                        joint.name, joint.parent, expected_parent
                    )
                )
            expected_parent = joint.child
        if expected_parent != "Link7":
            raise RobotModelError("RM75-6FB末端不是Link7")

        self.limits: Dict[str, Tuple[float, float]] = {}
        self.velocity_limits: Dict[str, float] = {}
        for joint in self.chain:
            if joint.limit is None:
                raise RobotModelError("{}没有URDF限位".format(joint.name))
            lower, upper = float(joint.limit.lower), float(joint.limit.upper)
            velocity = float(joint.limit.velocity)
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise RobotModelError("{}限位无效".format(joint.name))
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise RobotModelError("{}的URDF速度上限无效".format(joint.name))
            self.limits[joint.name] = (lower, upper)
            self.velocity_limits[joint.name] = velocity

        self.visuals: List[VisualMesh] = []
        package_cache: Dict[str, str] = {}
        for link in self.urdf.links:
            for visual in link.visuals:
                geometry = visual.geometry
                uri = getattr(geometry, "filename", None)
                if not uri:
                    continue
                filename = self.resolve_resource(uri, package_cache)
                scale = getattr(geometry, "scale", None) or (1.0, 1.0, 1.0)
                self.visuals.append(
                    VisualMesh(
                        link.name,
                        filename,
                        tuple(float(value) for value in scale),
                        _origin_matrix(visual.origin),
                    )
                )
        required_links = {"base_link"}.union(
            "Link{}".format(index) for index in range(1, 8)
        )
        visual_links = {visual.link for visual in self.visuals}
        if not required_links.issubset(visual_links):
            raise RobotModelError(
                "厂商URDF缺少可视网格：{}".format(
                    ", ".join(sorted(required_links - visual_links))
                )
            )

    @staticmethod
    def resolve_resource(uri: str, package_cache: Optional[Dict[str, str]] = None) -> str:
        if not uri.startswith("package://"):
            return str(Path(uri).expanduser().resolve())
        package, relative = uri[len("package://"):].split("/", 1)
        cache = package_cache if package_cache is not None else {}
        if package not in cache:
            cache[package] = rospkg.RosPack().get_path(package)
        path = Path(cache[package]) / relative
        if not path.is_file():
            raise RobotModelError("URDF网格不存在：{}".format(path))
        return str(path)

    def validate_vector(self, values: Iterable[float]) -> np.ndarray:
        vector = np.asarray(tuple(values), dtype=float)
        if vector.shape != (7,) or not np.isfinite(vector).all():
            raise RobotModelError("关节值必须是有限的七维向量")
        return vector

    def limit_violations(self, values: Iterable[float], margin: float = 0.0) -> List[str]:
        vector = self.validate_vector(values)
        problems = []
        for index, name in enumerate(JOINT_NAMES):
            lower, upper = self.limits[name]
            if vector[index] < lower + margin or vector[index] > upper - margin:
                problems.append(name)
        return problems

    def forward_kinematics(self, values: Iterable[float]) -> Dict[str, np.ndarray]:
        """Return base-frame transforms for base, seven joints and Link7."""

        vector = self.validate_vector(values)
        transforms: Dict[str, np.ndarray] = {"base_link": np.eye(4)}
        parent_transform = transforms["base_link"]
        for index, joint in enumerate(self.chain):
            joint_frame = parent_transform.dot(_origin_matrix(joint.origin))
            transforms[joint.name] = joint_frame.copy()
            child = joint_frame.dot(_axis_angle(joint.axis, vector[index]))
            transforms[joint.child] = child
            parent_transform = child
        return transforms

    def tcp(self, values: Iterable[float]) -> Dict[str, float]:
        """Return the Link7 pose in base_link as XYZ + roll/pitch/yaw."""
        matrix = self.forward_kinematics(values)["Link7"]
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.asin(max(-1.0, min(1.0, -float(matrix[2, 0]))))
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        return {
            "x": float(matrix[0, 3]), "y": float(matrix[1, 3]),
            "z": float(matrix[2, 3]), "rx": roll, "ry": pitch, "rz": yaw,
        }

    def visual_transforms(self, values: Iterable[float]) -> List[Tuple[VisualMesh, np.ndarray]]:
        links = self.forward_kinematics(values)
        return [(visual, links[visual.link].dot(visual.origin)) for visual in self.visuals
                if visual.link in links]
