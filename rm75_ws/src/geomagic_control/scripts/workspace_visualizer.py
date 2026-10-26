#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Geomagic Touch 工作空间可视化 / 边界重建（RViz / ROS Noetic）

主要能力
1) 工作空间采样与可视化
- 点云（Marker.POINTS）
- 包络框（Marker.LINE_LIST）
- 统计文字（Marker.TEXT_VIEW_FACING，可选）

2) 边界重建（可选）
- hull：凸包三角面片（Marker.TRIANGLE_LIST，需要 scipy）
- voxel：体素边界（Marker.CUBE_LIST）

3) RViz 内一键开关
- points/bbox/text/boundary 分别发布到独立 Topic，
  在 RViz Displays 面板直接勾选/取消即可。

4) 自动随机采样（你提出的“程序代替我采样”）
- input_mode:=fk_random
  基于 URDF 关节限位做随机关节采样 + FK，生成 N 个工作空间点。
  典型用法：随机采样 50000 点，然后打开 RViz 查看结果。

5) 点云按 Z 轴渐变着色
- colorize_by_z:=true
  根据 z 轴范围对每个点填充颜色渐变（蓝->红），便于观察高度分布。

输入模式（~input_mode）
- tf（推荐）：从 TF 获取末端点
- pose：订阅 PoseStamped（可用 ~scale 做 mm->m）
- fk_random：离线随机采样（URDF FK），无需手动移动设备

采样模式（仅对 tf/pose 有效，~sampling_mode）
- fixed：固定频率 ~sample_rate_hz
- random_time：随机采样间隔（注意：这是“随机间隔”，不是“随机点”）
- distance：按位移触发采样 ~min_sample_dist

服务
- ~/reset (std_srvs/Empty)
- ~/set_publish_text (std_srvs/SetBool)
- ~/set_publish_bbox (std_srvs/SetBool)
- ~/set_publish_boundary (std_srvs/SetBool)

"""

from __future__ import annotations

import colorsys
import math
import random
import threading
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from visualization_msgs.msg import Marker

try:
    import tf2_ros
except Exception:  # pragma: no cover
    tf2_ros = None

try:
    from scipy.spatial import ConvexHull
except Exception:  # pragma: no cover
    ConvexHull = None

# FK random mode dependencies (ROS standard python libs)
try:
    from urdf_parser_py.urdf import URDF
except Exception:  # pragma: no cover
    URDF = None

try:
    from kdl_parser_py.urdf import treeFromUrdfModel
except Exception:  # pragma: no cover
    treeFromUrdfModel = None

try:
    import PyKDL as kdl
except Exception:  # pragma: no cover
    kdl = None


@dataclass
class Bounds:
    min_x: float = 0.0
    min_y: float = 0.0
    min_z: float = 0.0
    max_x: float = 0.0
    max_y: float = 0.0
    max_z: float = 0.0
    valid: bool = False

    def update(self, x: float, y: float, z: float) -> None:
        if not self.valid:
            self.min_x = self.max_x = x
            self.min_y = self.max_y = y
            self.min_z = self.max_z = z
            self.valid = True
            return
        self.min_x = min(self.min_x, x)
        self.min_y = min(self.min_y, y)
        self.min_z = min(self.min_z, z)
        self.max_x = max(self.max_x, x)
        self.max_y = max(self.max_y, y)
        self.max_z = max(self.max_z, z)

    def size(self) -> Tuple[float, float, float]:
        return (
            self.max_x - self.min_x,
            self.max_y - self.min_y,
            self.max_z - self.min_z,
        )


class WorkspaceVisualizer:
    def __init__(self) -> None:
        # ---------------- Parameters ----------------
        self.input_mode = rospy.get_param("~input_mode", "tf").strip().lower()
        self._is_fk_random = self.input_mode in ("fk_random", "fk", "offline_fk")

        # pose 模式参数
        self.pose_topic = rospy.get_param("~pose_topic", "/Geomagic/pose")
        self.scale = float(rospy.get_param("~scale", 0.001))

        # tf 模式参数
        self.tf_parent = rospy.get_param("~tf_parent", "base")
        self.tf_child = rospy.get_param("~tf_child", "stylus")
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))

        # 输出 frame
        self.out_frame = rospy.get_param("~frame_id", "")

        # 采样控制（仅 tf/pose）
        self.record = bool(rospy.get_param("~record", True))
        self.record_when_button_pressed = bool(rospy.get_param("~record_when_button_pressed", False))
        self.record_button = rospy.get_param("~record_button", "either")
        self.button_topic = rospy.get_param("~button_topic", "/Geomagic/button")

        # 采样模式与速率（仅 tf/pose）
        self.sampling_mode = rospy.get_param("~sampling_mode", "fixed").strip().lower()
        self.sample_rate_hz = float(rospy.get_param("~sample_rate_hz", 120.0))
        self.sample_tick_hz = float(rospy.get_param("~sample_tick_hz", 240.0))
        self.min_sample_dt = float(rospy.get_param("~min_sample_dt", 0.0))
        self.min_sample_dist = float(rospy.get_param("~min_sample_dist", 0.0))
        self.random_dt_min = float(rospy.get_param("~random_dt_min", 0.005))
        self.random_dt_max = float(rospy.get_param("~random_dt_max", 0.020))

        # 下采样（体素去重）
        self.downsample_resolution = float(rospy.get_param("~downsample_resolution", 0.002))
        self.max_points = int(rospy.get_param("~max_points", 20000))

        # Marker Topics：分 Topic 便于 RViz 内勾选开关
        self.points_topic = rospy.get_param("~points_topic", "/geomagic_workspace/points")
        self.bbox_topic = rospy.get_param("~bbox_topic", "/geomagic_workspace/bbox")
        self.text_topic = rospy.get_param("~text_topic", "/geomagic_workspace/text")
        self.boundary_topic = rospy.get_param("~boundary_topic", "/geomagic_workspace/boundary")

        # 可选合并输出
        self.publish_combined = bool(rospy.get_param("~publish_combined", False))
        self.marker_topic = rospy.get_param("~marker_topic", "/geomagic_workspace/markers")

        # Marker 样式
        self.point_size = float(rospy.get_param("~point_size", 0.0025))
        self.line_width = float(rospy.get_param("~line_width", 0.0015))
        self.text_size = float(rospy.get_param("~text_size", 0.03))
        self.alpha_points = float(rospy.get_param("~alpha_points", 0.85))
        self.alpha_box = float(rospy.get_param("~alpha_box", 0.90))
        self.alpha_boundary = float(rospy.get_param("~alpha_boundary", 0.35))

        # 文本/包围盒/边界开关（节点级；RViz 也可通过 checkbox 关闭显示）
        self.publish_text = bool(rospy.get_param("~publish_text", True))
        self.text_to_console = bool(rospy.get_param("~text_to_console", False))
        self.publish_bbox = bool(rospy.get_param("~publish_bbox", True))
        self.publish_boundary = bool(rospy.get_param("~publish_boundary", True))

        # 边界重建
        self.reconstruct_mode = rospy.get_param("~reconstruct_mode", "none").strip().lower()
        self.recon_min_points = int(rospy.get_param("~recon_min_points", 200))
        self.hull_throttle_sec = float(rospy.get_param("~hull_throttle_sec", 1.0))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.01))
        self.max_boundary_voxels = int(rospy.get_param("~max_boundary_voxels", 6000))

        # Z 轴渐变着色
        self.colorize_by_z = bool(rospy.get_param("~colorize_by_z", True))
        self.z_color_min = rospy.get_param("~z_color_min", None)
        self.z_color_max = rospy.get_param("~z_color_max", None)
        self.color_throttle_sec = float(rospy.get_param("~color_throttle_sec", 0.5))

        # 发布频率（Marker 发布）
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))

        # fk_random 参数
        self.robot_description_param = rospy.get_param("~robot_description_param", "robot_description")
        self.fk_base = rospy.get_param("~fk_base", "base")
        self.fk_tip = rospy.get_param("~fk_tip", "stylus")
        self.fk_samples = int(rospy.get_param("~fk_samples", 50000))
        self.fk_seed = int(rospy.get_param("~fk_seed", -1))
        self.fk_unique = bool(rospy.get_param("~fk_unique", True))
        self.fk_max_tries = int(rospy.get_param("~fk_max_tries", max(self.fk_samples * 6, 200000)))

        # ---------------- State ----------------
        self._lock = threading.Lock()
        self._button_grey = 0
        self._button_white = 0

        self._frame_id: str = "world"
        self._bounds = Bounds()

        self._points: Deque[Point] = deque()
        self._voxels: Dict[Tuple[int, int, int], Point] = {}
        self._voxel_fifo: Deque[Tuple[int, int, int]] = deque()

        # 采样 gating
        self._last_accept_time = 0.0
        self._next_random_time = 0.0
        self._last_accept_point: Optional[Point] = None

        # 重建缓存
        self._recon_dirty = True
        self._last_hull_time = 0.0
        self._hull_triangles: List[Point] = []
        self._voxel_boundary_centers: List[Point] = []
        self._recon_stats: str = ""

        # 颜色缓存
        self._colors_dirty = True
        self._cached_point_colors: List[ColorRGBA] = []
        self._cached_color_key: Tuple[int, float, float] = (0, 0.0, 0.0)  # (n, zmin, zmax)
        self._last_color_time = 0.0

        # ---------------- TF (optional) ----------------
        self._tf_buffer = None
        self._tf_listener = None
        if self.input_mode == "tf":
            if tf2_ros is None:
                raise RuntimeError("tf2_ros 不可用，但 input_mode=tf")
            self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # ---------------- ROS I/O ----------------
        # fk_random 建议 latch：RViz 后开也能立刻显示
        latch = bool(rospy.get_param("~latch_markers", self._is_fk_random))

        self._pub_points = rospy.Publisher(self.points_topic, Marker, queue_size=1, latch=latch)
        self._pub_bbox = rospy.Publisher(self.bbox_topic, Marker, queue_size=1, latch=latch)
        self._pub_text = rospy.Publisher(self.text_topic, Marker, queue_size=1, latch=latch)
        self._pub_boundary = rospy.Publisher(self.boundary_topic, Marker, queue_size=1, latch=latch)

        self._pub_combined = None
        if self.publish_combined:
            from visualization_msgs.msg import MarkerArray

            self._pub_combined = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=latch)

        # pose 模式订阅
        self._pose_sub = None
        if self.input_mode == "pose":
            self._pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self._on_pose, queue_size=200)

        # button 订阅（按需）
        if self.record_when_button_pressed and (not self._is_fk_random):
            from geomagic_control.msg import DeviceButtonEvent

            rospy.Subscriber(self.button_topic, DeviceButtonEvent, self._on_button, queue_size=50)

        # services
        rospy.Service("~reset", Empty, self._on_reset)
        rospy.Service("~set_publish_text", SetBool, self._on_set_publish_text)
        rospy.Service("~set_publish_bbox", SetBool, self._on_set_publish_bbox)
        rospy.Service("~set_publish_boundary", SetBool, self._on_set_publish_boundary)

        # timers
        self._publish_timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 1e-3)), self._on_publish_timer)

        self._sample_timer = None
        if self.input_mode == "tf":
            self._sample_timer = rospy.Timer(rospy.Duration(1.0 / max(self.sample_tick_hz, 1e-3)), self._on_sample_tick)

        if self.sampling_mode == "random_time":
            self._next_random_time = rospy.get_time() + self._draw_random_dt()

        # fk_random：启动即自动采样
        if self._is_fk_random:
            self.record = False
            self.record_when_button_pressed = False
            self._run_fk_random_sampling()

        rospy.loginfo(
            "[workspace_visualizer] input_mode=%s sampling_mode=%s sample_rate_hz=%.3f tick_hz=%.3f min_dt=%.4f min_dist=%.4f downsample=%.4f max_points=%d",
            self.input_mode,
            self.sampling_mode,
            self.sample_rate_hz,
            self.sample_tick_hz,
            self.min_sample_dt,
            self.min_sample_dist,
            self.downsample_resolution,
            self.max_points,
        )
        rospy.loginfo(
            "[workspace_visualizer] topics: points=%s bbox=%s text=%s boundary=%s (combined=%s)",
            self.points_topic,
            self.bbox_topic,
            self.text_topic,
            self.boundary_topic,
            (self.marker_topic if self.publish_combined else "off"),
        )
        rospy.loginfo(
            "[workspace_visualizer] publish: bbox=%s text=%s boundary=%s text_to_console=%s recon=%s colorize_by_z=%s",
            self.publish_bbox,
            self.publish_text,
            self.publish_boundary,
            self.text_to_console,
            self.reconstruct_mode,
            self.colorize_by_z,
        )
        if self.input_mode == "tf":
            rospy.loginfo("[workspace_visualizer] TF parent=%s child=%s", self.tf_parent, self.tf_child)
        elif self.input_mode == "pose":
            rospy.loginfo("[workspace_visualizer] pose_topic=%s scale=%s", self.pose_topic, self.scale)
        else:
            rospy.loginfo("[workspace_visualizer] fk_random base=%s tip=%s samples=%d unique=%s seed=%d", self.fk_base, self.fk_tip, self.fk_samples, self.fk_unique, self.fk_seed)

    # ---------------- services ----------------
    def _on_set_publish_text(self, req) -> SetBoolResponse:
        with self._lock:
            self.publish_text = bool(req.data)
        return SetBoolResponse(success=True, message=f"publish_text={self.publish_text}")

    def _on_set_publish_bbox(self, req) -> SetBoolResponse:
        with self._lock:
            self.publish_bbox = bool(req.data)
        return SetBoolResponse(success=True, message=f"publish_bbox={self.publish_bbox}")

    def _on_set_publish_boundary(self, req) -> SetBoolResponse:
        with self._lock:
            self.publish_boundary = bool(req.data)
        return SetBoolResponse(success=True, message=f"publish_boundary={self.publish_boundary}")

    def _on_reset(self, _req) -> EmptyResponse:
        with self._lock:
            self._points.clear()
            self._voxels.clear()
            self._voxel_fifo.clear()
            self._bounds = Bounds()
            self._recon_dirty = True
            self._hull_triangles.clear()
            self._voxel_boundary_centers.clear()
            self._recon_stats = ""
            self._last_accept_time = 0.0
            self._last_accept_point = None
            self._colors_dirty = True
            self._cached_point_colors = []
            if self.sampling_mode == "random_time":
                self._next_random_time = rospy.get_time() + self._draw_random_dt()
        rospy.loginfo("[workspace_visualizer] workspace samples cleared")
        return EmptyResponse()

    # ---------------- fk_random ----------------

    # ---------------- fk_random helpers ----------------
    @staticmethod
    def _voxel_key(x: float, y: float, z: float, reso: float) -> Tuple[int, int, int]:
        return (
            int(math.floor(x / reso)),
            int(math.floor(y / reso)),
            int(math.floor(z / reso)),
        )

    @staticmethod
    def _get_joint_limits(uj) -> Tuple[float, float]:
        # Extract limits from a urdf_parser_py Joint object.
        if uj is None:
            return -math.pi, math.pi
        jtype = getattr(uj, "type", "")
        if jtype == "continuous":
            return -math.pi, math.pi
        lim = getattr(uj, "limit", None)
        if lim is None:
            return -math.pi, math.pi
        try:
            lo = float(getattr(lim, "lower", -math.pi))
            hi = float(getattr(lim, "upper", math.pi))
        except Exception:
            return -math.pi, math.pi
        if (not math.isfinite(lo)) or (not math.isfinite(hi)) or (hi <= lo):
            return -math.pi, math.pi
        return lo, hi

    def _commit_fk_points(self, pts: List[Point], bounds: Bounds) -> None:
        # Commit FK results into internal buffers.
        if len(pts) > self.max_points:
            pts = pts[-self.max_points :]

        with self._lock:
            self._points = deque(pts)
            self._bounds = bounds
            self._frame_id = self.out_frame if self.out_frame else self.fk_base

            # Build voxel map for consistent behavior with online sampling.
            self._voxels.clear()
            self._voxel_fifo.clear()
            if self.downsample_resolution > 0.0:
                reso = self.downsample_resolution
                for p in pts:
                    key = self._voxel_key(p.x, p.y, p.z, reso)
                    self._voxels[key] = p
                    self._voxel_fifo.append(key)
            else:
                # If de-dup is disabled, keep a minimal index so other logic keeps working.
                for idx, p in enumerate(pts):
                    key = (idx, 0, 0)
                    self._voxels[key] = p
                    self._voxel_fifo.append(key)

            self._recon_dirty = True
            self._colors_dirty = True

    @staticmethod
    def _link_name(x) -> str:
        if isinstance(x, str):
            return x
        if hasattr(x, "link"):
            return str(getattr(x, "link"))
        return str(x)

    def _urdf_get_joint_chain(self, robot, base_link: str, tip_link: str):
        # Build base->tip joint chain from URDF (includes fixed joints).
        child_to_joint = {}
        for j in getattr(robot, "joints", []):
            try:
                child = self._link_name(getattr(j, "child", ""))
                parent = self._link_name(getattr(j, "parent", ""))
            except Exception:
                continue
            if child and parent:
                child_to_joint[child] = j

        chain = []
        cur = tip_link
        guard = 0
        while cur != base_link:
            guard += 1
            if guard > 300:
                raise RuntimeError("URDF chain too long / loop detected")
            if cur not in child_to_joint:
                raise KeyError(f"No joint connects to child link: {cur}")
            j = child_to_joint[cur]
            chain.append(j)
            cur = self._link_name(getattr(j, "parent", ""))
            if not cur:
                raise RuntimeError("Joint parent is empty")

        chain.reverse()
        return chain

    @staticmethod
    def _urdf_origin_xyz_rpy(j) -> Tuple[np.ndarray, np.ndarray]:
        o = getattr(j, "origin", None)
        if o is None:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        else:
            xyz = getattr(o, "xyz", None) or [0.0, 0.0, 0.0]
            rpy = getattr(o, "rpy", None) or [0.0, 0.0, 0.0]
        try:
            xyz = [float(v) for v in xyz]
            rpy = [float(v) for v in rpy]
        except Exception:
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
        if len(xyz) != 3:
            xyz = [0.0, 0.0, 0.0]
        if len(rpy) != 3:
            rpy = [0.0, 0.0, 0.0]
        return np.array(xyz, dtype=float), np.array(rpy, dtype=float)

    @staticmethod
    def _urdf_axis(j) -> np.ndarray:
        a = getattr(j, "axis", None)
        if a is None:
            vec = [1.0, 0.0, 0.0]
        else:
            if hasattr(a, "xyz"):
                a = getattr(a, "xyz")
            try:
                vec = [float(v) for v in a]
            except Exception:
                vec = [1.0, 0.0, 0.0]
        if len(vec) != 3:
            vec = [1.0, 0.0, 0.0]
        v = np.array(vec, dtype=float)
        n = float(np.linalg.norm(v))
        if n <= 0:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return v / n

    @staticmethod
    def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        return (Rz @ Ry) @ Rx

    @staticmethod
    def _rot_axis_angle(axis: np.ndarray, theta: float) -> np.ndarray:
        ax = axis.astype(float)
        n = float(np.linalg.norm(ax))
        if n <= 0:
            return np.eye(3, dtype=float)
        x, y, z = (ax / n).tolist()
        c = math.cos(theta)
        s = math.sin(theta)
        C = 1.0 - c
        return np.array(
            [
                [c + x*x*C, x*y*C - z*s, x*z*C + y*s],
                [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
                [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
            ],
            dtype=float,
        )

    def _fk_numpy_point(self, joint_infos, q: np.ndarray) -> Tuple[float, float, float]:
        # FK: accumulate origin transforms and joint motions; returns translation only.
        R = np.eye(3, dtype=float)
        t = np.zeros(3, dtype=float)

        for i, (jtype, R0, p0, axis) in enumerate(joint_infos):
            t = t + R @ p0
            R = R @ R0

            qi = float(q[i])
            if jtype in ("revolute", "continuous"):
                R = R @ self._rot_axis_angle(axis, qi)
            elif jtype == "prismatic":
                t = t + R @ (axis * qi)
            else:
                pass

        return float(t[0]), float(t[1]), float(t[2])

    # ---------------- fk_random ----------------
    def _run_fk_random_sampling(self) -> None:
        # Random sampling over URDF joint limits + FK to generate workspace points.
        # Prefer kdl_parser_py+PyKDL if available; fallback to pure numpy FK.

        if URDF is None:
            rospy.logerr(
                "[workspace_visualizer] fk_random: missing dependency urdf_parser_py (URDF=%s). Please install ROS Noetic urdfdom_py.",
                URDF,
            )
            return

        desc_key = self.robot_description_param
        if not desc_key.startswith("/"):
            desc_key = "/" + desc_key

        try:
            xml = rospy.get_param(desc_key)
        except Exception as e:
            rospy.logerr("[workspace_visualizer] fk_random: param %s not found: %s", desc_key, str(e))
            return

        try:
            robot = URDF.from_xml_string(xml)
        except Exception as e:
            rospy.logerr("[workspace_visualizer] fk_random: URDF parse failed: %s", str(e))
            return

        # -------------- KDL path --------------
        if treeFromUrdfModel is not None and kdl is not None:
            try:
                res = treeFromUrdfModel(robot)
                if isinstance(res, tuple):
                    ok, tree = res
                    if not ok:
                        raise RuntimeError("treeFromUrdfModel returned not-ok")
                else:
                    tree = res

                chain = tree.getChain(self.fk_base, self.fk_tip)
                fk_solver = kdl.ChainFkSolverPos_recursive(chain)

                joint_names = []
                joint_lows = []
                joint_highs = []

                none_type = getattr(kdl.Joint, "None", 0)
                for i in range(chain.getNrOfSegments()):
                    seg = chain.getSegment(i)
                    j = seg.getJoint()
                    if j.getType() == none_type:
                        continue
                    jname = j.getName()
                    joint_names.append(jname)
                    uj = robot.joint_map.get(jname)
                    lo, hi = self._get_joint_limits(uj)
                    joint_lows.append(lo)
                    joint_highs.append(hi)

                nj = len(joint_names)
                if nj <= 0:
                    raise RuntimeError(f"KDL chain has no joints: {self.fk_base}->{self.fk_tip}")

                target = max(1, int(self.fk_samples))
                rng = np.random.default_rng(None if self.fk_seed < 0 else self.fk_seed)
                jnt = kdl.JntArray(nj)
                frame = kdl.Frame()

                bounds = Bounds()
                pts = []
                voxel_seen = {}

                tries = 0
                last_log = rospy.get_time()
                lows = np.array(joint_lows, dtype=float)
                highs = np.array(joint_highs, dtype=float)

                while (len(pts) < target) and (tries < self.fk_max_tries) and (not rospy.is_shutdown()):
                    tries += 1
                    q = lows + (highs - lows) * rng.random(nj)
                    for k in range(nj):
                        jnt[k] = float(q[k])

                    ret = fk_solver.JntToCart(jnt, frame)
                    if ret < 0:
                        continue

                    x = float(frame.p[0]); y = float(frame.p[1]); z = float(frame.p[2])

                    if self.fk_unique and self.downsample_resolution > 0.0:
                        key = self._voxel_key(x, y, z, self.downsample_resolution)
                        if key in voxel_seen:
                            continue
                        voxel_seen[key] = 1

                    pts.append(Point(x=x, y=y, z=z))
                    bounds.update(x, y, z)

                    now = rospy.get_time()
                    if (now - last_log) > 1.0:
                        last_log = now
                        rospy.loginfo("[workspace_visualizer] fk_random(KDL) generating... %d/%d (tries=%d)", len(pts), target, tries)

                if len(pts) < target:
                    rospy.logwarn(
                        "[workspace_visualizer] fk_random(KDL): only got %d/%d points (tries=%d). You may lower downsample_resolution or set fk_unique:=false or increase fk_max_tries.",
                        len(pts), target, tries,
                    )
                else:
                    rospy.loginfo("[workspace_visualizer] fk_random(KDL) done: %d points (tries=%d)", len(pts), tries)

                self._commit_fk_points(pts, bounds)
                return

            except Exception as e:
                rospy.logwarn(
                    "[workspace_visualizer] fk_random: KDL path unavailable or failed (%s). Falling back to numpy FK.",
                    str(e),
                )

        # -------------- numpy fallback path --------------
        try:
            chain_joints = self._urdf_get_joint_chain(robot, self.fk_base, self.fk_tip)
        except Exception as e:
            rospy.logerr("[workspace_visualizer] fk_random(numpy): cannot build chain %s->%s: %s", self.fk_base, self.fk_tip, str(e))
            return

        joint_infos = []
        lows = []
        highs = []
        for j in chain_joints:
            jtype = getattr(j, "type", "fixed")
            p0, rpy = self._urdf_origin_xyz_rpy(j)
            R0 = self._rpy_to_R(rpy)
            axis = self._urdf_axis(j)

            if jtype in ("revolute", "prismatic", "continuous"):
                lo, hi = self._get_joint_limits(j)
            else:
                lo, hi = 0.0, 0.0

            joint_infos.append((jtype, R0, p0, axis))
            lows.append(float(lo))
            highs.append(float(hi))

        nj = len(joint_infos)
        if nj <= 0:
            rospy.logerr("[workspace_visualizer] fk_random(numpy): empty chain %s->%s", self.fk_base, self.fk_tip)
            return

        target = max(1, int(self.fk_samples))
        rng = np.random.default_rng(None if self.fk_seed < 0 else self.fk_seed)

        bounds = Bounds()
        pts = []
        voxel_seen = {}

        tries = 0
        last_log = rospy.get_time()
        lows = np.array(lows, dtype=float)
        highs = np.array(highs, dtype=float)

        while (len(pts) < target) and (tries < self.fk_max_tries) and (not rospy.is_shutdown()):
            tries += 1
            q = lows + (highs - lows) * rng.random(nj)
            x, y, z = self._fk_numpy_point(joint_infos, q)

            if self.fk_unique and self.downsample_resolution > 0.0:
                key = self._voxel_key(x, y, z, self.downsample_resolution)
                if key in voxel_seen:
                    continue
                voxel_seen[key] = 1

            pts.append(Point(x=x, y=y, z=z))
            bounds.update(x, y, z)

            now = rospy.get_time()
            if (now - last_log) > 1.0:
                last_log = now
                rospy.loginfo("[workspace_visualizer] fk_random(numpy) generating... %d/%d (tries=%d)", len(pts), target, tries)

        if len(pts) < target:
            rospy.logwarn(
                "[workspace_visualizer] fk_random(numpy): only got %d/%d points (tries=%d). You may lower downsample_resolution or set fk_unique:=false or increase fk_max_tries.",
                len(pts), target, tries,
            )
        else:
            rospy.loginfo("[workspace_visualizer] fk_random(numpy) done: %d points (tries=%d)", len(pts), tries)

        self._commit_fk_points(pts, bounds)

    # ---------------- button / gating ----------------
    def _on_button(self, msg) -> None:
        with self._lock:
            self._button_grey = int(msg.grey_button)
            self._button_white = int(msg.white_button)

    def _should_record_now(self) -> bool:
        if not self.record:
            return False
        if not self.record_when_button_pressed:
            return True
        if self.record_button == "grey":
            return self._button_grey == 1
        if self.record_button == "white":
            return self._button_white == 1
        return (self._button_grey == 1) or (self._button_white == 1)

    def _draw_random_dt(self) -> float:
        a = max(0.0, self.random_dt_min)
        b = max(a, self.random_dt_max)
        return random.uniform(a, b)

    def _time_gate_ok(self, now_sec: float) -> bool:
        if self.min_sample_dt > 0.0 and (now_sec - self._last_accept_time) < self.min_sample_dt:
            return False

        mode = self.sampling_mode
        if mode == "fixed":
            hz = max(self.sample_rate_hz, 1e-6)
            return (now_sec - self._last_accept_time) >= (1.0 / hz)
        if mode == "random_time":
            return now_sec >= self._next_random_time
        if mode == "distance":
            return True

        hz = max(self.sample_rate_hz, 1e-6)
        return (now_sec - self._last_accept_time) >= (1.0 / hz)

    def _distance_gate_ok(self, p: Point) -> bool:
        if self.min_sample_dist <= 0.0:
            return True
        if self._last_accept_point is None:
            return True
        dx = p.x - self._last_accept_point.x
        dy = p.y - self._last_accept_point.y
        dz = p.z - self._last_accept_point.z
        return (dx * dx + dy * dy + dz * dz) >= (self.min_sample_dist * self.min_sample_dist)

    # ---------------- live inputs ----------------
    def _on_pose(self, msg: PoseStamped) -> None:
        if self._is_fk_random:
            return

        with self._lock:
            if not self._should_record_now():
                return

        now = rospy.get_time()
        with self._lock:
            if not self._time_gate_ok(now):
                return

        frame_id = self.out_frame if self.out_frame else (msg.header.frame_id if msg.header.frame_id else "world")

        x = float(msg.pose.position.x) * self.scale
        y = float(msg.pose.position.y) * self.scale
        z = float(msg.pose.position.z) * self.scale
        p = Point(x=x, y=y, z=z)

        with self._lock:
            if self.sampling_mode == "distance" and not self._distance_gate_ok(p):
                return

        self._accept_sample(p, frame_id, now)

    def _on_sample_tick(self, _evt) -> None:
        if self._is_fk_random:
            return
        if self.input_mode != "tf" or self._tf_buffer is None:
            return

        with self._lock:
            if not self._should_record_now():
                return

        now = rospy.get_time()
        with self._lock:
            if not self._time_gate_ok(now):
                return

        try:
            trans = self._tf_buffer.lookup_transform(
                self.tf_parent,
                self.tf_child,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[workspace_visualizer] TF lookup failed (%s -> %s): %s", self.tf_parent, self.tf_child, str(e))
            return

        p = Point(
            x=float(trans.transform.translation.x),
            y=float(trans.transform.translation.y),
            z=float(trans.transform.translation.z),
        )

        with self._lock:
            if self.sampling_mode == "distance" and not self._distance_gate_ok(p):
                return

        frame_id = self.out_frame if self.out_frame else self.tf_parent
        self._accept_sample(p, frame_id, now)

    def _accept_sample(self, p: Point, frame_id: str, now_sec: float) -> None:
        with self._lock:
            if self.sampling_mode == "random_time":
                self._next_random_time = now_sec + self._draw_random_dt()
            self._last_accept_time = now_sec
            self._last_accept_point = Point(x=p.x, y=p.y, z=p.z)

        self._add_sample_point(p.x, p.y, p.z, frame_id)

    def _add_sample_point(self, x: float, y: float, z: float, frame_id: str) -> None:
        p = Point(x=x, y=y, z=z)

        with self._lock:
            self._bounds.update(x, y, z)
            self._frame_id = frame_id

            # 体素去重
            if self.downsample_resolution > 0.0:
                key = (
                    int(math.floor(x / self.downsample_resolution)),
                    int(math.floor(y / self.downsample_resolution)),
                    int(math.floor(z / self.downsample_resolution)),
                )
            else:
                key = (hash(round(x, 6)), hash(round(y, 6)), hash(round(z, 6)))

            if key in self._voxels:
                return

            self._voxels[key] = p
            self._voxel_fifo.append(key)
            self._points.append(p)
            self._recon_dirty = True
            self._colors_dirty = True

            # 容量上限（环形缓冲）
            while len(self._points) > self.max_points:
                self._points.popleft()
                old = self._voxel_fifo.popleft()
                self._voxels.pop(old, None)
                self._recon_dirty = True
                self._colors_dirty = True

    # ---------------- publish timer ----------------
    def _on_publish_timer(self, _evt) -> None:
        self._maybe_reconstruct_boundary()

        pts_marker = self._build_points_marker()
        bbox_marker = self._build_bbox_marker()
        text_marker = self._build_text_marker()
        boundary_marker = self._build_boundary_marker()

        if pts_marker is not None:
            self._pub_points.publish(pts_marker)
        if bbox_marker is not None:
            self._pub_bbox.publish(bbox_marker)
        if text_marker is not None:
            self._pub_text.publish(text_marker)
        if boundary_marker is not None:
            self._pub_boundary.publish(boundary_marker)

        if self._pub_combined is not None:
            from visualization_msgs.msg import MarkerArray

            ma = MarkerArray()
            if pts_marker is not None:
                ma.markers.append(pts_marker)
            if bbox_marker is not None:
                ma.markers.append(bbox_marker)
            if text_marker is not None:
                ma.markers.append(text_marker)
            if boundary_marker is not None:
                ma.markers.append(boundary_marker)
            self._pub_combined.publish(ma)

        if self.text_to_console:
            self._print_stats_throttled()

    # ---------------- reconstruction ----------------
    def _maybe_reconstruct_boundary(self) -> None:
        mode = self.reconstruct_mode
        if mode not in ("hull", "voxel"):
            return

        with self._lock:
            publish_boundary = self.publish_boundary
            dirty = self._recon_dirty
            pts = list(self._points)

        if not publish_boundary or (not dirty):
            return
        if len(pts) < self.recon_min_points:
            with self._lock:
                self._hull_triangles = []
                self._voxel_boundary_centers = []
                self._recon_stats = ""
                self._recon_dirty = False
            return

        if mode == "hull":
            self._reconstruct_hull(pts)
        else:
            self._reconstruct_voxel_boundary(pts)

        with self._lock:
            self._recon_dirty = False

    def _reconstruct_hull(self, pts: List[Point]) -> None:
        if ConvexHull is None:
            rospy.logwarn_throttle(5.0, "[workspace_visualizer] scipy 不可用，无法使用 hull 模式")
            with self._lock:
                self._hull_triangles = []
                self._recon_stats = ""
            return

        now = rospy.get_time()
        if (now - self._last_hull_time) < max(self.hull_throttle_sec, 0.0):
            return

        arr = np.array([[p.x, p.y, p.z] for p in pts], dtype=np.float64)

        try:
            hull = ConvexHull(arr)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "[workspace_visualizer] ConvexHull failed: %s", str(e))
            with self._lock:
                self._hull_triangles = []
                self._recon_stats = ""
            return

        tris: List[Point] = []
        for simplex in hull.simplices:
            v0 = arr[simplex[0]]
            v1 = arr[simplex[1]]
            v2 = arr[simplex[2]]
            tris.append(Point(x=float(v0[0]), y=float(v0[1]), z=float(v0[2])))
            tris.append(Point(x=float(v1[0]), y=float(v1[1]), z=float(v1[2])))
            tris.append(Point(x=float(v2[0]), y=float(v2[1]), z=float(v2[2])))

        stats = f"hull facets={len(hull.simplices)} area={hull.area:.6f} volume={hull.volume:.6f}"

        with self._lock:
            self._hull_triangles = tris
            self._voxel_boundary_centers = []
            self._recon_stats = stats
            self._last_hull_time = now

    def _reconstruct_voxel_boundary(self, pts: List[Point]) -> None:
        vs = max(self.voxel_size, 1e-6)
        occ: Dict[Tuple[int, int, int], int] = {}
        for p in pts:
            key = (
                int(math.floor(p.x / vs)),
                int(math.floor(p.y / vs)),
                int(math.floor(p.z / vs)),
            )
            occ[key] = 1

        boundary_keys: List[Tuple[int, int, int]] = []
        neighbors = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        for k in occ.keys():
            for dx, dy, dz in neighbors:
                nk = (k[0] + dx, k[1] + dy, k[2] + dz)
                if nk not in occ:
                    boundary_keys.append(k)
                    break

        if len(boundary_keys) > self.max_boundary_voxels:
            stride = int(math.ceil(len(boundary_keys) / float(self.max_boundary_voxels)))
            boundary_keys = boundary_keys[:: max(1, stride)]

        centers: List[Point] = []
        for k in boundary_keys:
            centers.append(Point(x=(k[0] + 0.5) * vs, y=(k[1] + 0.5) * vs, z=(k[2] + 0.5) * vs))

        with self._lock:
            self._voxel_boundary_centers = centers
            self._hull_triangles = []
            self._recon_stats = f"voxel boundary voxels={len(centers)} voxel_size={vs}"

    # ---------------- markers ----------------
    @staticmethod
    def _base_marker(frame_id: str, stamp: rospy.Time, ns: str, marker_id: int, marker_type: int) -> Marker:
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _get_z_range_for_color(self, bounds: Bounds) -> Tuple[float, float]:
        # 优先使用用户指定范围
        try:
            if self.z_color_min is not None and self.z_color_max is not None:
                zmin = float(self.z_color_min)
                zmax = float(self.z_color_max)
                if math.isfinite(zmin) and math.isfinite(zmax) and zmax > zmin:
                    return zmin, zmax
        except Exception:
            pass

        if bounds.valid:
            if bounds.max_z > bounds.min_z:
                return bounds.min_z, bounds.max_z
        return 0.0, 1.0

    def _compute_colors_by_z(self, pts: List[Point], zmin: float, zmax: float) -> List[ColorRGBA]:
        out: List[ColorRGBA] = []
        denom = max(zmax - zmin, 1e-9)
        a = max(0.0, min(1.0, self.alpha_points))

        # 蓝(240°) -> 红(0°)
        for p in pts:
            t = (p.z - zmin) / denom
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            hue = (1.0 - t) * (2.0 / 3.0)  # 0.666.. -> 0
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            out.append(ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a)))
        return out

    def _ensure_color_cache(self, pts: List[Point], bounds: Bounds) -> List[ColorRGBA]:
        if not self.colorize_by_z:
            return []

        now = rospy.get_time()
        zmin, zmax = self._get_z_range_for_color(bounds)
        key = (len(pts), float(zmin), float(zmax))

        with self._lock:
            dirty = self._colors_dirty
            cached = self._cached_point_colors
            cached_key = self._cached_color_key
            last_t = self._last_color_time

        # 如果缓存可用，直接返回
        if (not dirty) and (cached_key == key) and (len(cached) == len(pts)):
            return cached

        # 节流：点很多时避免每次 publish 都重新计算
        if (now - last_t) < max(self.color_throttle_sec, 0.0) and (len(cached) == len(pts)):
            return cached

        colors = self._compute_colors_by_z(pts, zmin, zmax)

        with self._lock:
            # 二次确认长度一致
            if len(self._points) == len(pts):
                self._cached_point_colors = colors
                self._cached_color_key = key
                self._colors_dirty = False
                self._last_color_time = now
                return self._cached_point_colors

        # 若期间点数变化，则返回临时结果（本次 marker 仍可用）
        return colors

    def _build_points_marker(self) -> Optional[Marker]:
        with self._lock:
            frame_id = self._frame_id
            pts = list(self._points)
            bounds = self._bounds

        now = rospy.Time.now()
        ns = "geomagic_workspace"
        m = self._base_marker(frame_id, now, ns, 0, Marker.POINTS)
        m.scale.x = self.point_size
        m.scale.y = self.point_size

        if len(pts) == 0:
            # 空 marker 也发布，便于 RViz 快速进入状态
            m.color.r = 0.1
            m.color.g = 0.9
            m.color.b = 0.2
            m.color.a = max(0.0, min(1.0, self.alpha_points))
            m.points = []
            return m

        m.points = pts

        if self.colorize_by_z:
            m.colors = self._ensure_color_cache(pts, bounds)
            # 全局色仍给一个合法值（RViz 以 colors[] 为准）
            m.color.a = max(0.0, min(1.0, self.alpha_points))
        else:
            m.color.r = 0.10
            m.color.g = 0.90
            m.color.b = 0.20
            m.color.a = max(0.0, min(1.0, self.alpha_points))

        return m

    def _build_bbox_marker(self) -> Optional[Marker]:
        with self._lock:
            frame_id = self._frame_id
            bounds = self._bounds
            enabled = self.publish_bbox

        now = rospy.Time.now()
        ns = "geomagic_workspace"
        m = self._base_marker(frame_id, now, ns, 1, Marker.LINE_LIST)

        if (not enabled) or (not bounds.valid):
            m.action = Marker.DELETE
            return m

        m.scale.x = self.line_width
        m.color.r = 1.0
        m.color.g = 0.7
        m.color.b = 0.1
        m.color.a = max(0.0, min(1.0, self.alpha_box))

        corners = [
            Point(bounds.min_x, bounds.min_y, bounds.min_z),
            Point(bounds.max_x, bounds.min_y, bounds.min_z),
            Point(bounds.max_x, bounds.max_y, bounds.min_z),
            Point(bounds.min_x, bounds.max_y, bounds.min_z),
            Point(bounds.min_x, bounds.min_y, bounds.max_z),
            Point(bounds.max_x, bounds.min_y, bounds.max_z),
            Point(bounds.max_x, bounds.max_y, bounds.max_z),
            Point(bounds.min_x, bounds.max_y, bounds.max_z),
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for i, j in edges:
            m.points.append(corners[i])
            m.points.append(corners[j])

        return m

    def _build_text_marker(self) -> Optional[Marker]:
        with self._lock:
            frame_id = self._frame_id
            pts = list(self._points)
            bounds = self._bounds
            enabled = self.publish_text
            recon_stats = self._recon_stats

        now = rospy.Time.now()
        ns = "geomagic_workspace"
        m = self._base_marker(frame_id, now, ns, 2, Marker.TEXT_VIEW_FACING)

        if not enabled:
            m.action = Marker.DELETE
            return m

        m.scale.z = self.text_size
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.95

        if bounds.valid:
            dx, dy, dz = bounds.size()
            pad = max(self.text_size * 2.0, 0.05)
            m.pose.position.x = bounds.max_x + pad
            m.pose.position.y = bounds.max_y + pad
            m.pose.position.z = bounds.max_z + pad

            recon_str = self.reconstruct_mode if self.reconstruct_mode in ("hull", "voxel") else "none"
            zcolor = "zcolor" if self.colorize_by_z else "mono"

            m.text = (
                "Geomagic Touch Workspace\n"
                f"samples: {len(pts)} (cap {self.max_points})\n"
                f"x:[{bounds.min_x:.4f},{bounds.max_x:.4f}] Δ={dx:.4f} m\n"
                f"y:[{bounds.min_y:.4f},{bounds.max_y:.4f}] Δ={dy:.4f} m\n"
                f"z:[{bounds.min_z:.4f},{bounds.max_z:.4f}] Δ={dz:.4f} m\n"
                f"mode={self.input_mode}/{self.sampling_mode} ds={self.downsample_resolution} {zcolor}\n"
                f"recon={recon_str} {recon_stats}"
            )
        else:
            m.pose.position.x = 0.0
            m.pose.position.y = 0.0
            m.pose.position.z = 0.0
            m.text = "Geomagic Touch Workspace\n(waiting for samples...)"

        return m

    def _build_boundary_marker(self) -> Optional[Marker]:
        with self._lock:
            frame_id = self._frame_id
            enabled = self.publish_boundary
            mode = self.reconstruct_mode
            hull_tris = list(self._hull_triangles)
            voxel_centers = list(self._voxel_boundary_centers)

        now = rospy.Time.now()
        ns = "geomagic_workspace"

        if (not enabled) or (mode not in ("hull", "voxel")):
            m = self._base_marker(frame_id, now, ns, 3, Marker.TRIANGLE_LIST)
            m.action = Marker.DELETE
            return m

        if mode == "hull":
            m = self._base_marker(frame_id, now, ns, 3, Marker.TRIANGLE_LIST)
            if len(hull_tris) == 0:
                m.action = Marker.DELETE
                return m
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.r = 0.2
            m.color.g = 0.6
            m.color.b = 1.0
            m.color.a = max(0.0, min(1.0, self.alpha_boundary))
            m.points = hull_tris
            return m

        m = self._base_marker(frame_id, now, ns, 3, Marker.CUBE_LIST)
        if len(voxel_centers) == 0:
            m.action = Marker.DELETE
            return m
        m.scale.x = self.voxel_size
        m.scale.y = self.voxel_size
        m.scale.z = self.voxel_size
        m.color.r = 0.2
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = max(0.0, min(1.0, self.alpha_boundary))
        m.points = voxel_centers
        return m

    def _print_stats_throttled(self) -> None:
        with self._lock:
            bounds = self._bounds
            n = len(self._points)
            stats = self._recon_stats

        if not bounds.valid:
            rospy.loginfo_throttle(2.0, "[workspace_visualizer] waiting for samples...")
            return

        dx, dy, dz = bounds.size()
        rospy.loginfo_throttle(
            1.0,
            "[workspace_visualizer] samples=%d x=[%.4f,%.4f] y=[%.4f,%.4f] z=[%.4f,%.4f] size=(%.4f,%.4f,%.4f) %s",
            n,
            bounds.min_x,
            bounds.max_x,
            bounds.min_y,
            bounds.max_y,
            bounds.min_z,
            bounds.max_z,
            dx,
            dy,
            dz,
            stats,
        )


def main() -> None:
    rospy.init_node("geomagic_workspace_visualizer", anonymous=False)
    _ = WorkspaceVisualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
