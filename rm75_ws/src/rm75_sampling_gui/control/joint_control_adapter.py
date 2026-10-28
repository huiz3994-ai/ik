"""Fail-closed GUI adapter for optional RM75 joint trajectory execution.

The GUI-side adapter never advertises or publishes ``/rm_driver/JointPos``.
Its motion endpoint is an independently gated FollowJointTrajectory server;
offline preview and validation remain available while execution is locked.
"""

from dataclasses import dataclass
import logging
import math
import threading
import time
import uuid
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel


LOG = logging.getLogger(__name__)


class JointControlError(RuntimeError):
    """Raised when an execution safety gate rejects a request."""


@dataclass
class _Ticket:
    token: str
    target: np.ndarray
    expires: float


class JointControlAdapter:
    """Validate, plan and optionally submit one seven-axis target."""

    CONFLICT_NODES = (
        "/rm75_touch_teleop",
        "/rm75_vertical_press",
        "/rm_control",
        "/rm75_low_speed_commissioning",
    )

    def __init__(
        self,
        robot: RM75RobotModel,
        config: Dict[str, Any],
        ros_interface: Any = None,
        state_machine: Any = None,
        collision_checker: Optional[Callable[[np.ndarray], bool]] = None,
        graph_checker: Optional[Callable[[], Iterable[str]]] = None,
    ) -> None:
        self.robot = robot
        self.config = dict(config.get("joint_control", {}))
        self.trajectory_config = dict(config.get("trajectory_library", {}))
        self.ros = ros_interface
        self.state_machine = state_machine
        self.execution_authorized = bool(
            config.get("joint_control_execution_authorized", False)
        )
        self.trajectory_execution_authorized = bool(
            config.get("trajectory_execution_authorized", False)
        )
        self.supervised_commissioning_authorized = bool(
            config.get("supervised_commissioning_authorized", False)
        )
        self._collision_checker = collision_checker
        self._graph_checker = graph_checker
        self._ticket: Optional[_Ticket] = None
        self._cancel = threading.Event()
        self._active = False
        self._lock = threading.RLock()
        self._action_client = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def armed(self) -> bool:
        if self.supervised_commissioning_authorized:
            return self.execution_authorized
        return (
            self.execution_authorized
            and self.trajectory_execution_authorized
            and bool(self.config.get("execution_path_verified", False))
            and bool(self.config.get("gui_loss_watchdog_verified", False))
            and bool(self.config.get("collision_scene_verified", False))
        )

    def _option(self, name: str, default: float) -> float:
        value = float(self.config.get(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise JointControlError("joint_control.{}必须为正有限值".format(name))
        return value

    @staticmethod
    def _actual(snapshot: Dict[str, Any]) -> np.ndarray:
        values = snapshot.get("joints", {})
        if not snapshot.get("fresh", {}).get("joints"):
            raise JointControlError("/joint_states反馈超时或不可用")
        if not all(name in values for name in JOINT_NAMES):
            raise JointControlError("/joint_states缺少joint1-joint7")
        vector = np.asarray([values[name] for name in JOINT_NAMES], dtype=float)
        if vector.shape != (7,) or not np.isfinite(vector).all():
            raise JointControlError("实际关节反馈不是有限七维向量")
        return vector

    def _live_nodes(self) -> set:
        if self._graph_checker is not None:
            return set(self._graph_checker())
        import rosnode
        return set(rosnode.get_node_names())

    def _control_source_check(
        self, snapshot: Dict[str, Any], control_mode: Optional[str]
    ) -> List[str]:
        problems = []
        if control_mode:
            problems.append("HMI控制源{}正在占用机械臂".format(control_mode))
        try:
            conflicts = sorted(set(self.CONFLICT_NODES).intersection(self._live_nodes()))
        except Exception as error:
            problems.append("无法核验ROS控制源：{}".format(error))
        else:
            if conflicts:
                problems.append("存在互斥控制节点：{}".format(", ".join(conflicts)))
        task_state = str((snapshot.get("task_status") or {}).get("state", "UNKNOWN"))
        if task_state not in ("", "IDLE", "NOT_RUNNING", "UNKNOWN"):
            problems.append("QR任务状态{}不是IDLE".format(task_state))
        return problems

    def _bridge_check(self) -> List[str]:
        if not self.execution_authorized:
            return ["JOINT_CONTROL_EXECUTION_AUTHORIZED=false"]
        if self.supervised_commissioning_authorized:
            try:
                if "/joint_control_server" not in self._live_nodes():
                    return ["受控七轴适配器未运行"]
            except Exception as error:
                return ["无法核验受控七轴适配器：{}".format(error)]
            return []
        if not self.trajectory_execution_authorized:
            return ["trajectory_execution_authorized=false"]
        commissioning = (
            ("execution_path_verified", "七轴完整执行链尚未验收"),
            ("gui_loss_watchdog_verified", "GUI失联停止看门狗尚未验收"),
            ("collision_scene_verified", "现场PlanningScene尚未验收"),
        )
        unverified = [message for key, message in commissioning
                      if not bool(self.config.get(key, False))]
        if unverified:
            return unverified
        try:
            import rospy
            nodes = self._live_nodes()
            if "/trajectory_bridge" not in nodes:
                return ["trajectory_bridge未运行"]
            required = (
                "hand_eye_calibrated",
                "tool_verified",
                "stop_verified",
                "gripper_feedback_verified",
            )
            problems = []
            if not bool(rospy.get_param("/trajectory_bridge/execute", False)):
                problems.append("trajectory_bridge/execute=false")
            for name in required:
                if not bool(rospy.get_param("/trajectory_bridge/" + name, False)):
                    problems.append("trajectory_bridge/{}未验收".format(name))
            return problems
        except Exception as error:
            return ["无法核验trajectory_bridge授权：{}".format(error)]

    def _collision_at(self, vector: np.ndarray) -> bool:
        if self._collision_checker is not None:
            return bool(self._collision_checker(vector.copy()))
        import rospy
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
        from sensor_msgs.msg import JointState

        service_name = str(self.config.get(
            "state_validity_service", "/check_state_validity"
        ))
        rospy.wait_for_service(service_name, timeout=self._option("service_timeout_s", 2.0))
        request = GetStateValidityRequest()
        request.group_name = str(self.config.get("move_group", "rm_group"))
        request.robot_state = RobotState(
            joint_state=JointState(name=list(JOINT_NAMES), position=vector.tolist())
        )
        response = rospy.ServiceProxy(service_name, GetStateValidity)(request)
        return not bool(response.valid)

    def _collision_path(
        self, start: np.ndarray, target: np.ndarray, required: bool
    ) -> Tuple[bool, str]:
        samples = max(2, int(self.config.get("collision_samples", 25)))
        try:
            for ratio in np.linspace(0.0, 1.0, samples):
                if self._collision_at(start + ratio * (target - start)):
                    return False, "MoveIt/FCL在路径{:.0f}%检测到碰撞".format(100.0 * ratio)
            return True, "MoveIt/FCL路径检查通过（{}点）".format(samples)
        except Exception as error:
            if required:
                return False, "MoveIt/FCL碰撞检查不可用：{}".format(error)
            return False, "碰撞检查未执行：{}".format(error)

    def validate(
        self,
        target: Iterable[float],
        snapshot: Dict[str, Any],
        state: Any,
        control_mode: Optional[str],
        for_execution: bool = False,
    ) -> Dict[str, Any]:
        """Run all local checks and issue a short-lived execution ticket."""

        vector = self.robot.validate_vector(target)
        actual = self._actual(snapshot)
        checks: List[Dict[str, Any]] = []

        def add(name: str, ok: bool, detail: str) -> None:
            checks.append({"name": name, "ok": bool(ok), "detail": detail})

        margin = self._option("joint_limit_margin_rad", 0.02)
        violations = self.robot.limit_violations(vector, margin)
        add("关节限位", not violations,
            "通过" if not violations else "越界或侵入余量：{}".format(", ".join(violations)))

        delta = np.abs(vector - actual)
        configured_delta = self.config.get("max_single_delta_rad")
        if configured_delta is not None:
            max_delta = self._option("max_single_delta_rad", math.radians(5.0))
            add("单次位移", bool(np.max(delta) <= max_delta),
                "最大{:.3f}°，限制{:.3f}°".format(
                    math.degrees(float(np.max(delta))), math.degrees(max_delta)))

        feedback_age = float(snapshot.get("ages", {}).get("joints", float("inf")))
        feedback_timeout = self._option("feedback_timeout_s", 0.25)
        add("反馈时效", 0.0 <= feedback_age <= feedback_timeout,
            "age={:.3f}s，限制{:.3f}s".format(feedback_age, feedback_timeout))

        state_name = getattr(state, "value", str(state))
        state_ok = state_name != "SAFE_STOP"
        add("SAFE_STOP", state_ok, "未锁存" if state_ok else "已锁存，必须先恢复")

        source_problems = self._control_source_check(snapshot, control_mode)
        add("控制源互斥", not source_problems,
            "通过" if not source_problems else "；".join(source_problems))

        collision_ok, collision_detail = self._collision_path(
            actual, vector, required=for_execution
        )
        # In preview-only mode an unavailable collision service is explicitly
        # reported as unchecked, while every positive collision is still a rejection.
        add("MoveIt/FCL碰撞", collision_ok, collision_detail)

        bridge_problems = self._bridge_check()
        add("独立执行授权", not bridge_problems,
            "通过" if not bridge_problems else "；".join(bridge_problems))

        mandatory = [check for check in checks if check["name"] != "独立执行授权"]
        if not for_execution and "碰撞检查未执行" in collision_detail:
            mandatory = [check for check in mandatory if check["name"] != "MoveIt/FCL碰撞"]
        if for_execution:
            mandatory = checks
        ok = all(check["ok"] for check in mandatory)
        token = None
        if for_execution and ok:
            ttl = self._option("command_ttl_s", 2.0)
            token = uuid.uuid4().hex
            with self._lock:
                self._ticket = _Ticket(token, vector.copy(), time.monotonic() + ttl)
        elif for_execution:
            with self._lock:
                self._ticket = None
        return {
            "ok": ok,
            "checks": checks,
            "token": token,
            "target": vector.tolist(),
            "actual": actual.tolist(),
            "delta": (vector - actual).tolist(),
            "execution_authorized": self.armed,
        }

    def build_trajectory(
        self, start: Iterable[float], target: Iterable[float]
    ) -> Tuple[List[np.ndarray], List[float]]:
        """Build a sampled quintic path using the vendor URDF velocity limits."""

        first = self.robot.validate_vector(start)
        last = self.robot.validate_vector(target)
        delta = np.abs(last - first)
        velocity_limits = np.asarray(
            [self.robot.velocity_limits[name] for name in JOINT_NAMES], dtype=float
        )
        if (velocity_limits.shape != (7,) or not np.isfinite(velocity_limits).all()
                or np.any(velocity_limits <= 0.0)):
            raise JointControlError("厂商URDF关节速度上限无效")
        duration = max(
            self._option("minimum_duration_s", 1.0),
            1.875 * float(np.max(delta / velocity_limits)),
        )
        period = self._option("trajectory_period_s", 0.05)
        count = max(2, int(math.ceil(duration / period)) + 1)
        times = np.linspace(0.0, duration, count)
        positions = []
        for stamp in times:
            u = stamp / duration
            scale = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
            positions.append(first + scale * (last - first))
        return positions, [float(value) for value in times]

    @staticmethod
    def _trajectory_arrays(points: Sequence[Dict[str, Any]]) -> Tuple[
            List[np.ndarray], List[np.ndarray], List[np.ndarray], List[float]]:
        if not isinstance(points, (list, tuple)) or len(points) < 2:
            raise JointControlError("轨迹至少需要两个密集点")
        positions: List[np.ndarray] = []
        velocities: List[np.ndarray] = []
        accelerations: List[np.ndarray] = []
        times: List[float] = []
        previous = -1.0
        for index, point in enumerate(points):
            if not isinstance(point, dict):
                raise JointControlError("轨迹点{}不是对象".format(index))
            vectors = []
            for key in ("positions", "velocities", "accelerations"):
                value = np.asarray(point.get(key, ()), dtype=float)
                if value.shape != (7,) or not np.isfinite(value).all():
                    raise JointControlError("轨迹点{}的{}不是有限七维向量".format(index, key))
                vectors.append(value)
            stamp = float(point.get("time_from_start", float("nan")))
            if not math.isfinite(stamp) or stamp < 0.0:
                raise JointControlError("轨迹点{}时间无效".format(index))
            if index == 0 and stamp != 0.0:
                raise JointControlError("轨迹必须从time_from_start=0开始")
            if index and stamp <= previous:
                raise JointControlError("轨迹时间必须严格递增")
            positions.append(vectors[0])
            velocities.append(vectors[1])
            accelerations.append(vectors[2])
            times.append(stamp)
            previous = stamp
        return positions, velocities, accelerations, times

    def to_joint_trajectory(self, points: Sequence[Dict[str, Any]]) -> Any:
        """Convert the stored dense representation to trajectory_msgs."""

        import rospy
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        positions, velocities, accelerations, times = self._trajectory_arrays(points)
        message = JointTrajectory()
        message.joint_names = list(JOINT_NAMES)
        for q, dq, ddq, stamp in zip(positions, velocities, accelerations, times):
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            point.velocities = dq.tolist()
            point.accelerations = ddq.tolist()
            point.time_from_start = rospy.Duration.from_sec(stamp)
            message.points.append(point)
        return message

    @staticmethod
    def feedback_progress(
        actual: Iterable[float], positions: Sequence[np.ndarray], minimum_index: int = 0,
    ) -> Dict[str, Any]:
        """Map actual feedback to a monotonic nearest dense point."""

        actual_vector = np.asarray(tuple(actual), dtype=float)
        if actual_vector.shape != (7,) or not np.isfinite(actual_vector).all():
            raise JointControlError("实际关节反馈不是有限七维向量")
        if not positions:
            raise JointControlError("轨迹位置为空")
        start = max(0, min(int(minimum_index), len(positions) - 1))
        errors = [float(np.max(np.abs(np.asarray(value) - actual_vector)))
                  for value in positions[start:]]
        index = start + int(np.argmin(errors))
        return {
            "current_point": index,
            "total_points": len(positions),
            "progress": float(index) / float(max(1, len(positions) - 1)),
            "max_error_rad": errors[index - start],
        }

    def validate_trajectory(
        self,
        points: Sequence[Dict[str, Any]],
        snapshot: Dict[str, Any],
        state: Any,
        control_mode: Optional[str],
    ) -> Dict[str, Any]:
        """Fail closed on start state, kinematic limits, scene and control source."""

        checks: List[Dict[str, Any]] = []

        def add(name: str, ok: bool, detail: str) -> None:
            checks.append({"name": name, "ok": bool(ok), "detail": detail})

        try:
            positions, velocities, accelerations, times = self._trajectory_arrays(points)
        except Exception as error:
            add("轨迹数据", False, str(error))
            return {"ok": False, "checks": checks}
        add("轨迹数据", True, "{}个密集点，{:.3f}s".format(len(points), times[-1]))

        try:
            actual = self._actual(snapshot)
        except Exception as error:
            add("反馈时效", False, str(error))
            actual = None
        else:
            tolerance = self._option("trajectory_start_tolerance_rad", 0.05)
            start_error = float(np.max(np.abs(actual - positions[0])))
            add("轨迹起点", start_error <= tolerance,
                "当前与起点误差{:.4f}rad，限制{:.4f}rad".format(start_error, tolerance))

        margin = self._option("joint_limit_margin_rad", 0.02)
        violations = []
        for index, vector in enumerate(positions):
            names = self.robot.limit_violations(vector, margin)
            if names:
                violations.append("点{}:{}".format(index, ",".join(names)))
                break
        add("整轨迹关节限位", not violations,
            "全部通过" if not violations else violations[0])

        velocity_limits = np.asarray(
            [self.robot.velocity_limits[name] for name in JOINT_NAMES], dtype=float
        )
        max_velocity = np.max(np.abs(np.asarray(velocities)), axis=0)
        velocity_ok = bool(np.all(max_velocity <= velocity_limits + 1e-9))
        add("整轨迹速度", velocity_ok,
            "最大{}".format([round(float(value), 4) for value in max_velocity]))

        acceleration_limits = np.asarray(
            self.trajectory_config.get("acceleration_limits_rad_s2", [1.5] * 7),
            dtype=float,
        )
        acceleration_ok = (
            acceleration_limits.shape == (7,)
            and np.isfinite(acceleration_limits).all()
            and np.all(acceleration_limits > 0.0)
            and np.all(np.max(np.abs(np.asarray(accelerations)), axis=0)
                       <= acceleration_limits + 1e-9)
        )
        add("整轨迹加速度", bool(acceleration_ok),
            "通过" if acceleration_ok else "超过配置的加速度限制")

        collision_failure = ""
        try:
            for index, vector in enumerate(positions):
                if self._collision_at(vector):
                    collision_failure = "MoveIt/FCL在密集点{}检测到碰撞".format(index)
                    break
        except Exception as error:
            collision_failure = "MoveIt/FCL碰撞检查不可用：{}".format(error)
        add("整轨迹MoveIt碰撞", not collision_failure,
            "{}个密集点全部通过".format(len(points)) if not collision_failure else collision_failure)

        state_name = getattr(state, "value", str(state))
        add("SAFE_STOP", state_name != "SAFE_STOP",
            "未锁存" if state_name != "SAFE_STOP" else "已锁存")
        source_problems = self._control_source_check(snapshot, control_mode)
        add("控制源互斥", not source_problems,
            "通过" if not source_problems else "；".join(source_problems))
        bridge_problems = self._bridge_check()
        add("独立执行授权", not bridge_problems,
            "通过" if not bridge_problems else "；".join(bridge_problems))
        return {"ok": all(item["ok"] for item in checks), "checks": checks}

    def execute_trajectory(
        self,
        points: Sequence[Dict[str, Any]],
        snapshot_provider: Callable[[], Dict[str, Any]],
        progress_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> Dict[str, Any]:
        self._trajectory_arrays(points)
        with self._lock:
            if self._active:
                raise JointControlError("关节控制已经在执行")
            self._active = True
            self._cancel.clear()
        try:
            return self._send_trajectory_points(points, snapshot_provider, progress_callback)
        finally:
            with self._lock:
                self._active = False

    def execute(
        self,
        token: str,
        target: Iterable[float],
        snapshot_provider: Callable[[], Dict[str, Any]],
    ) -> Dict[str, Any]:
        """Submit to the configured guarded action after consuming a fresh ticket."""

        vector = self.robot.validate_vector(target)
        with self._lock:
            ticket = self._ticket
            self._ticket = None
            if (ticket is None or token != ticket.token or time.monotonic() > ticket.expires
                    or not np.allclose(vector, ticket.target, atol=1e-10)):
                raise JointControlError("执行授权票据无效、过期或目标已改变")
            if self._active:
                raise JointControlError("关节控制已经在执行")
            self._active = True
            self._cancel.clear()
        try:
            start = self._actual(snapshot_provider())
            positions, times = self.build_trajectory(start, vector)
            return self._send_action(positions, times, snapshot_provider)
        finally:
            with self._lock:
                self._active = False

    def _send_action(
        self,
        positions: List[np.ndarray],
        times: List[float],
        snapshot_provider: Callable[[], Dict[str, Any]],
    ) -> Dict[str, Any]:
        points = [
            {"positions": values.tolist(), "velocities": [0.0] * 7,
             "accelerations": [0.0] * 7, "time_from_start": stamp}
            for values, stamp in zip(positions, times)
        ]
        return self._send_trajectory_points(points, snapshot_provider)

    def _send_trajectory_points(
        self,
        points: Sequence[Dict[str, Any]],
        snapshot_provider: Callable[[], Dict[str, Any]],
        progress_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> Dict[str, Any]:
        import actionlib
        import rospy
        from actionlib_msgs.msg import GoalStatus
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

        positions, _velocities, _accelerations, times = self._trajectory_arrays(points)

        action_name = str(self.config.get(
            "trajectory_action", "/rm75_sampling_gui/follow_joint_trajectory"
        ))
        client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
        self._action_client = client
        if not client.wait_for_server(rospy.Duration(self._option("action_server_timeout_s", 3.0))):
            raise JointControlError("七轴受控Action不可用：{}".format(action_name))
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.to_joint_trajectory(points)
        client.send_goal(goal)
        started = time.monotonic()
        next_source_check = started
        progress_index = 0
        deadline = time.monotonic() + times[-1] + self._option("result_timeout_s", 5.0)
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                client.cancel_goal()
                if not client.wait_for_result(rospy.Duration(
                        self._option("cancel_timeout_s", 4.0))):
                    if self.ros is not None:
                        self.ros.emergency_stop()
                    raise JointControlError("取消后停止未确认，已请求软件急停")
                if client.get_state() not in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                    raise JointControlError("取消结果未确认：{}".format(client.get_goal_status_text()))
                return {"cancelled": True, "status": client.get_goal_status_text()}
            snapshot = snapshot_provider()
            if not snapshot.get("fresh", {}).get("joints"):
                client.cancel_goal()
                if self.ros is not None:
                    self.ros.emergency_stop()
                raise JointControlError("执行期间关节反馈丢失，已停止")
            actual = self._actual(snapshot)
            progress = self.feedback_progress(actual, positions, progress_index)
            progress_index = int(progress["current_point"])
            if progress_callback is not None:
                progress_callback(progress)
            if time.monotonic() >= next_source_check:
                source_problems = self._control_source_check(snapshot, None)
                if source_problems:
                    client.cancel_goal()
                    if self.ros is not None:
                        self.ros.emergency_stop()
                    raise JointControlError(
                        "执行期间出现竞争控制源：{}".format("；".join(source_problems))
                    )
                next_source_check = time.monotonic() + 0.25
            fresh_status = snapshot.get("fresh", {})
            arm_error = snapshot.get("driver_arm_error")
            system_error = snapshot.get("driver_system_error")
            plan_ok = snapshot.get("driver_plan_ok")
            driver_fault = (
                (fresh_status.get("driver_arm_error") and arm_error not in (None, 0))
                or (fresh_status.get("driver_system_error") and system_error not in (None, 0))
                or (fresh_status.get("driver_plan_ok") and plan_ok is False)
            )
            if driver_fault:
                client.cancel_goal()
                if self.ros is not None:
                    self.ros.emergency_stop()
                raise JointControlError(
                    "驱动故障：ArmError={} SysError={} PlanState={}".format(
                        arm_error, system_error, plan_ok
                    )
                )
            # The guarded action server owns trajectory-time tracking checks.
            # Duplicating them here against a polled GUI snapshot introduced a
            # one-sample time lead at high speed and caused false SAFE_STOPs.
            if client.wait_for_result(rospy.Duration(0.05)):
                break
        else:
            client.cancel_goal()
            if self.ros is not None:
                self.ros.emergency_stop()
            raise JointControlError("轨迹执行超时，已请求停止")
        if client.get_state() != GoalStatus.SUCCEEDED:
            raise JointControlError(
                "轨迹桥拒绝/中止：{}".format(client.get_goal_status_text())
            )
        final = self._actual(snapshot_provider())
        tolerance = self._option("goal_tolerance_rad", 0.015)
        error = float(np.max(np.abs(final - positions[-1])))
        if error > tolerance:
            raise JointControlError(
                "执行结束跟踪误差{:.4f}rad超过{:.4f}rad".format(error, tolerance)
            )
        return {"cancelled": False, "max_error_rad": error, "status": "SUCCEEDED"}

    def cancel(self) -> Dict[str, Any]:
        self._cancel.set()
        client = self._action_client
        if client is not None:
            try:
                client.cancel_goal()
            except Exception:
                LOG.exception("发送关节控制取消失败")
        return {"cancel_requested": True, "active": self.active}
