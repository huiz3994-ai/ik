#!/usr/bin/env python3
"""Supervised, watchdog-protected RM75 joint trajectory adapter.

This is a separate ROS process.  The Qt slider never publishes JointPos.  The
adapter accepts only the private HMI action, validates the complete trajectory
against the vendor URDF and MoveIt/FCL, then becomes the sole JointPos source.
"""

import argparse
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
import numpy as np
import rosgraph
import rosnode
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, UInt16
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
import yaml

from rm_msgs.msg import JointPos, Plan_State
from rm75_sampling_gui.trajectory.interpolation import (
    evaluate_quintic, trajectory_coefficients, segment_check_times,
)


JOINTS = tuple("joint{}".format(index) for index in range(1, 8))


def load_joint_options(path: str) -> Dict:
    """Read only the joint-control section without loading GUI nulls into ROS."""

    with open(path, "r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    if not isinstance(root, dict):
        raise RuntimeError("GUI configuration must be a mapping")
    options = root.get("joint_control", {})
    if not isinstance(options, dict):
        raise RuntimeError("joint_control configuration must be a mapping")
    return options


class GuardedJointServer:
    """Fail-closed action-to-driver adapter for supervised commissioning."""

    def __init__(self, options: Optional[Dict] = None) -> None:
        self.authorized = bool(rospy.get_param("~authorized", False))
        if options is None:
            options = rospy.get_param("~joint_control", {})
        if not isinstance(options, dict):
            raise RuntimeError("~joint_control configuration must be a mapping")
        self.action_name = str(options.get(
            "trajectory_action", "/rm75_sampling_gui/follow_joint_trajectory"))
        self.command_topic = "/rm_driver/JointPos"
        self.feedback_timeout = float(options.get("feedback_timeout_s", 0.15))
        self.heartbeat_timeout = float(options.get("heartbeat_timeout_s", 0.35))
        configured_delta = options.get("max_single_delta_rad")
        self.max_delta = (None if configured_delta is None
                          else float(configured_delta))
        self.tracking_limit = float(options.get("max_tracking_error_rad", 0.03))
        self.tracking_latency_allowance = float(options.get(
            "tracking_latency_allowance_s", 0.02))
        self.settling_tracking_limit = float(options.get(
            "settling_tracking_limit_rad", 0.08))
        self.goal_tolerance = float(options.get("goal_tolerance_rad", 0.015))
        self.limit_margin = float(options.get("joint_limit_margin_rad", 0.02))
        self.collision_service = str(options.get(
            "state_validity_service", "/check_state_validity"))
        self.move_group = str(options.get("move_group", "arm"))
        positive = (
            self.feedback_timeout, self.heartbeat_timeout,
            self.tracking_limit, self.goal_tolerance, self.limit_margin,
            self.settling_tracking_limit,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise RuntimeError("invalid joint-control safety limits")
        if (self.max_delta is not None and
                (not math.isfinite(self.max_delta) or self.max_delta <= 0.0)):
            raise RuntimeError("invalid joint-control maximum delta")
        if (not math.isfinite(self.tracking_latency_allowance) or
                self.tracking_latency_allowance < 0.0):
            raise RuntimeError("invalid tracking latency allowance")
        if self.settling_tracking_limit < self.tracking_limit:
            raise RuntimeError("settling tracking limit must not be smaller")

        model = URDF.from_parameter_server("/robot_description")
        joints = {joint.name: joint for joint in model.joints}
        self.limits = []
        self.velocity_limits = []
        for name in JOINTS:
            joint = joints.get(name)
            if joint is None or joint.limit is None:
                raise RuntimeError("robot_description missing limit for " + name)
            self.limits.append((float(joint.limit.lower), float(joint.limit.upper)))
            velocity = float(joint.limit.velocity)
            if not math.isfinite(velocity) or velocity <= 0.0:
                raise RuntimeError("robot_description invalid velocity for " + name)
            self.velocity_limits.append(velocity)

        self.lock = threading.RLock()
        self.position: Optional[np.ndarray] = None
        self.feedback_stamp = 0.0
        self.heartbeat_stamp = 0.0
        self.stop_ack_sequence = 0
        self.stop_ack_value = False
        self.arm_error = 0
        self.system_error = 0
        self.plan_ok: Optional[bool] = None
        self.plan_stamp = 0.0

        self._assert_command_topic_unclaimed()
        self.command = rospy.Publisher(self.command_topic, JointPos, queue_size=1)
        self.stop = rospy.Publisher("/rm_driver/Emergency_Stop", Empty, queue_size=1)
        self.subscribers = [
            rospy.Subscriber("/joint_states", JointState, self._joint, queue_size=1),
            rospy.Subscriber("/rm75_sampling_gui/joint_control_heartbeat", Empty,
                             self._heartbeat, queue_size=1),
            rospy.Subscriber("/rm_driver/Set_Arm_Stop_Result", Bool,
                             self._stop_ack, queue_size=10),
            rospy.Subscriber("/rm_driver/ArmError", UInt16,
                             lambda msg: self._error("arm", msg.data), queue_size=10),
            rospy.Subscriber("/rm_driver/SysError", UInt16,
                             lambda msg: self._error("system", msg.data), queue_size=10),
            rospy.Subscriber("/rm_driver/Plan_State", Plan_State,
                             self._plan, queue_size=10),
        ]
        self.server = actionlib.SimpleActionServer(
            self.action_name, FollowJointTrajectoryAction,
            execute_cb=self._execute, auto_start=False,
        )
        self.server.start()
        rospy.logwarn(
            "Supervised joint adapter ready: authorized=%s delta_limit=%s velocity=vendor_URDF",
            self.authorized,
            "disabled" if self.max_delta is None else
            "{:.3f}deg".format(math.degrees(self.max_delta)),
        )

    def _assert_command_topic_unclaimed(self) -> None:
        _subscribers, publishers, _services = (None, None, None)
        publishers, _subscribers, _services = rosgraph.Master(
            rospy.get_name()).getSystemState()
        owners = []
        for topic, nodes in publishers:
            if topic == self.command_topic:
                owners.extend(nodes)
        if owners:
            raise RuntimeError(
                "JointPos already has publisher(s): {}".format(", ".join(owners))
            )

    def _joint(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        mapped = dict(zip(message.name, message.position))
        if not all(name in mapped and math.isfinite(mapped[name]) for name in JOINTS):
            return
        stamp = message.header.stamp.to_sec()
        now = rospy.Time.now().to_sec()
        if stamp <= 0.0 or not 0.0 <= now - stamp <= self.feedback_timeout:
            return
        with self.lock:
            self.position = np.asarray([mapped[name] for name in JOINTS], dtype=float)
            self.feedback_stamp = time.monotonic()

    def _heartbeat(self, _message: Empty) -> None:
        with self.lock:
            self.heartbeat_stamp = time.monotonic()

    def _stop_ack(self, message: Bool) -> None:
        with self.lock:
            self.stop_ack_sequence += 1
            self.stop_ack_value = bool(message.data)

    def _error(self, channel: str, value: int) -> None:
        with self.lock:
            if channel == "arm":
                self.arm_error = int(value)
            else:
                self.system_error = int(value)

    def _plan(self, message: Plan_State) -> None:
        with self.lock:
            self.plan_ok = bool(message.state)
            self.plan_stamp = time.monotonic()

    def _snapshot(self) -> Tuple[np.ndarray, float, float, int, int, Optional[bool], float]:
        with self.lock:
            if self.position is None:
                raise RuntimeError("joint feedback unavailable")
            return (
                self.position.copy(), self.feedback_stamp, self.heartbeat_stamp,
                self.arm_error, self.system_error, self.plan_ok, self.plan_stamp,
            )

    def _allowed_tracking_error(
        self, desired_velocity: np.ndarray, settling: bool = False
    ) -> np.ndarray:
        if settling:
            return np.full(7, self.settling_tracking_limit, dtype=float)
        velocity = np.asarray(desired_velocity, dtype=float)
        if velocity.shape != (7,) or not np.isfinite(velocity).all():
            raise RuntimeError("invalid desired velocity for tracking guard")
        return (self.tracking_limit +
                np.abs(velocity) * self.tracking_latency_allowance)

    def _halt(self, description: str) -> bool:
        with self.lock:
            baseline = self.stop_ack_sequence
        self.stop.publish(Empty())
        deadline = time.monotonic() + 3.0
        previous = None
        stable = 0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                current, feedback, _heartbeat, _arm, _system, _plan, _stamp = self._snapshot()
                good = time.monotonic() - feedback <= self.feedback_timeout
                if previous is not None:
                    good = good and float(np.max(np.abs(current - previous))) < 0.001
                stable = stable + 1 if good else 0
                previous = current
            except Exception:
                stable = 0
            with self.lock:
                acknowledged = (
                    self.stop_ack_sequence > baseline and self.stop_ack_value
                )
            if stable >= 5 and acknowledged:
                rospy.logwarn("%s: software stop ACK and stationary feedback confirmed", description)
                return True
            time.sleep(0.05)
        rospy.logerr("%s: STOP_UNCONFIRMED", description)
        return False

    def _abort(self, message: str, code: int = FollowJointTrajectoryResult.INVALID_GOAL) -> None:
        stopped = self._halt(message)
        result = FollowJointTrajectoryResult()
        result.error_code = code
        result.error_string = message + ("" if stopped else "; STOP_UNCONFIRMED")
        self.server.set_aborted(result, result.error_string)

    def _preflight(self, goal) -> Tuple[List[np.ndarray], List[float]]:
        if not self.authorized:
            raise RuntimeError("supervised commissioning authorization is false")
        current, feedback, heartbeat, arm, system, _plan, _plan_stamp = self._snapshot()
        now = time.monotonic()
        if now - feedback > self.feedback_timeout:
            raise RuntimeError("joint feedback timeout")
        if now - heartbeat > self.heartbeat_timeout:
            raise RuntimeError("GUI heartbeat timeout")
        if arm or system:
            raise RuntimeError("driver error ArmError={} SysError={}".format(arm, system))
        conflicts = set(rosnode.get_node_names()).intersection({
            "/rm75_touch_teleop", "/rm75_vertical_press", "/rm_control",
            "/rm75_low_speed_commissioning",
        })
        if conflicts:
            raise RuntimeError("competing control node: {}".format(", ".join(sorted(conflicts))))

        trajectory = goal.trajectory
        if len(trajectory.joint_names) != 7 or len(set(trajectory.joint_names)) != 7:
            raise RuntimeError("trajectory must name seven unique joints")
        order = []
        for name in JOINTS:
            if name not in trajectory.joint_names:
                raise RuntimeError("trajectory missing " + name)
            order.append(trajectory.joint_names.index(name))
        if len(trajectory.points) < 2:
            raise RuntimeError("trajectory needs at least two points")
        positions: List[np.ndarray] = []
        times: List[float] = []
        dense = []
        has_derivatives = any(point.velocities or point.accelerations
                              for point in trajectory.points)
        for point in trajectory.points:
            if len(point.positions) != 7:
                raise RuntimeError("invalid trajectory point dimensions")
            vector = np.asarray([point.positions[index] for index in order], dtype=float)
            stamp = float(point.time_from_start.to_sec())
            if not np.isfinite(vector).all() or not math.isfinite(stamp):
                raise RuntimeError("non-finite trajectory")
            if times and stamp <= times[-1]:
                raise RuntimeError("trajectory times are not increasing")
            for index, value in enumerate(vector):
                lower, upper = self.limits[index]
                if value < lower + self.limit_margin or value > upper - self.limit_margin:
                    raise RuntimeError("{} violates limit margin".format(JOINTS[index]))
            positions.append(vector)
            times.append(stamp)
            if has_derivatives:
                if len(point.velocities) != 7 or len(point.accelerations) != 7:
                    raise RuntimeError("trajectory derivative dimensions must be seven")
                dense.append({
                    "positions": vector.tolist(), "time_from_start": stamp,
                    "velocities": [point.velocities[index] for index in order],
                    "accelerations": [point.accelerations[index] for index in order],
                })
        if times[0] < 0.0 or float(np.max(np.abs(positions[0] - current))) > 0.03:
            raise RuntimeError("trajectory start does not match measured joints")
        if (self.max_delta is not None and
                float(np.max(np.abs(positions[-1] - current))) > self.max_delta):
            raise RuntimeError("target exceeds configured maximum delta")

        q = np.asarray(positions)
        t = np.asarray(times)
        dt = np.diff(t)
        velocity = np.diff(q, axis=0) / dt[:, None]
        vendor_velocity = np.asarray(self.velocity_limits, dtype=float)
        if np.any(np.abs(velocity) > vendor_velocity[None, :] + 1e-6):
            raise RuntimeError("trajectory exceeds vendor URDF velocity limit")

        self._segment_coefficients = trajectory_coefficients(dense) if has_derivatives else []
        checked_positions = list(positions)
        for coefficients, duration in zip(self._segment_coefficients, dt):
            for stamp in segment_check_times(coefficients, float(duration)):
                sample = evaluate_quintic(coefficients, stamp)
                if np.any(np.abs(sample["velocities"]) > vendor_velocity + 1e-6):
                    raise RuntimeError("quintic trajectory exceeds vendor URDF velocity limit")
                for axis, value in enumerate(sample["positions"]):
                    lower, upper = self.limits[axis]
                    if not lower + self.limit_margin <= value <= upper - self.limit_margin:
                        raise RuntimeError("quintic trajectory violates joint limit margin")
                checked_positions.append(sample["positions"])

        rospy.wait_for_service(self.collision_service, timeout=2.0)
        validity = rospy.ServiceProxy(self.collision_service, GetStateValidity)
        for index, vector in enumerate(checked_positions):
            request = GetStateValidityRequest()
            request.group_name = self.move_group
            request.robot_state = RobotState(
                joint_state=JointState(name=list(JOINTS), position=vector.tolist())
            )
            if not validity(request).valid:
                raise RuntimeError("MoveIt/FCL collision at trajectory point {}".format(index))
        return positions, times

    def _desired(self, positions, times, segment, elapsed):
        before, after = segment - 1, segment
        duration = times[after] - times[before]
        local_time = min(duration, max(0.0, elapsed - times[before]))
        if self._segment_coefficients:
            sample = evaluate_quintic(self._segment_coefficients[before], local_time)
            return sample["positions"], sample["velocities"]
        # Compatibility for legacy position-only goals. New taught paths carry
        # complete derivatives and always use the quintic branch above.
        return (positions[before] + local_time / duration * (positions[after] - positions[before]),
                (positions[after] - positions[before]) / duration)

    def _execute(self, goal) -> None:
        try:
            positions, times = self._preflight(goal)
        except Exception as error:
            result = FollowJointTrajectoryResult()
            result.error_code = result.INVALID_GOAL
            result.error_string = str(error)
            self.server.set_aborted(result, result.error_string)
            return

        started = time.monotonic()
        last_graph_check = 0.0
        segment = 1
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                if not self._halt("joint trajectory cancelled"):
                    self._abort("cancel STOP_UNCONFIRMED")
                    return
                self.server.set_preempted(FollowJointTrajectoryResult(), "stopped and acknowledged")
                return
            try:
                current, feedback_stamp, heartbeat_stamp, arm, system, plan, plan_stamp = self._snapshot()
            except Exception as error:
                self._abort(str(error))
                return
            now = time.monotonic()
            if now - feedback_stamp > self.feedback_timeout:
                self._abort("joint feedback timeout")
                return
            if now - heartbeat_stamp > self.heartbeat_timeout:
                self._abort("GUI heartbeat timeout")
                return
            if arm or system or (plan is False and plan_stamp >= started):
                self._abort("driver fault during execution")
                return
            if now - last_graph_check >= 0.25:
                last_graph_check = now
                conflicts = set(rosnode.get_node_names()).intersection({
                    "/rm75_touch_teleop", "/rm75_vertical_press", "/rm_control",
                    "/rm75_low_speed_commissioning",
                })
                if conflicts:
                    self._abort(
                        "competing control node appeared: {}".format(
                            ", ".join(sorted(conflicts))
                        )
                    )
                    return
            elapsed = now - started
            while segment + 1 < len(times) and elapsed > times[segment]:
                segment += 1
            if elapsed >= times[-1]:
                desired = positions[-1]
                desired_velocity = np.zeros(7)
                allowed_error = self._allowed_tracking_error(
                    desired_velocity, settling=True
                )
            else:
                desired, desired_velocity = self._desired(positions, times, segment, elapsed)
                allowed_error = self._allowed_tracking_error(desired_velocity)
            error = desired - current
            violation = np.abs(error) - allowed_error
            if float(np.max(violation)) > 0.0:
                axis = int(np.argmax(violation))
                self._abort(
                    "joint tracking error exceeded: {} error={:.4f} limit={:.4f}".format(
                        JOINTS[axis], abs(float(error[axis])),
                        float(allowed_error[axis])
                    )
                )
                return
            command = JointPos()
            command.joint = desired.tolist()
            command.expand = 0.0
            self.command.publish(command)
            feedback = FollowJointTrajectoryFeedback()
            feedback.header.stamp = rospy.Time.now()
            feedback.joint_names = list(JOINTS)
            feedback.actual.positions = current.tolist()
            feedback.desired.positions = desired.tolist()
            feedback.error.positions = error.tolist()
            self.server.publish_feedback(feedback)
            if elapsed >= times[-1]:
                final_error = float(np.max(np.abs(positions[-1] - current)))
                if final_error <= self.goal_tolerance:
                    result = FollowJointTrajectoryResult()
                    result.error_code = result.SUCCESSFUL
                    self.server.set_succeeded(result, "supervised target reached")
                    return
                if elapsed > times[-1] + 3.0:
                    self._abort("goal tolerance timeout",
                                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED)
                    return
            rate.sleep()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--authorized", choices=("true", "false"), default="false")
    parser.add_argument("--config", required=True)
    arguments = parser.parse_args(rospy.myargv()[1:])
    options = load_joint_options(arguments.config)
    rospy.init_node("rm75_joint_control_server")
    rospy.set_param("~authorized", arguments.authorized == "true")
    GuardedJointServer(options)
    rospy.spin()


if __name__ == "__main__":
    main()
