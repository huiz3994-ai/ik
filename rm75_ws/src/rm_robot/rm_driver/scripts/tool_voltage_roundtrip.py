#!/usr/bin/env python3
"""Move to a fixed tool pose, enable 24 V, and return to the start joints."""

import argparse
import math
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Empty

from rm_msgs.msg import (
    Arm_Current_State,
    GetArmState_Command,
    MoveJ,
    MoveJ_P,
    MoveL,
    Plan_State,
)


GET_STATE_COMMAND_TOPIC = "/rm_driver/GetArmState_Cmd"
STATE_TOPIC = "/rm_driver/Arm_Current_State"
MOVE_POSE_TOPIC = "/rm_driver/MoveJ_P_Cmd"
MOVE_JOINT_TOPIC = "/rm_driver/MoveJ_Cmd"
MOVE_LINEAR_TOPIC = "/rm_driver/MoveL_Cmd"
PLAN_STATE_TOPIC = "/rm_driver/Plan_State"
EMERGENCY_STOP_TOPIC = "/rm_driver/Emergency_Stop"
MOTION_COMMAND_TOPICS = {
    "/rm_driver/JointPos",
    "/rm_driver/MoveJ_Cmd",
    "/rm_driver/MoveJ_P_Cmd",
    "/rm_driver/MoveL_Cmd",
    "/rm_driver/MoveC_Cmd",
}

# Teach-pendant pose: X/Y/Z in millimetres and R/P/Y in radians.
TARGET_POSE = (-164.412, 157.617, 92.36, -3.112, -0.016, -0.568)
TOOL_Z_RETREAT_METRES = -0.210


class AutomationError(RuntimeError):
    """Raised when the requested sequence cannot be completed safely."""


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Save the current RM75 joints, move to the configured Cartesian pose, "
            "set the tool connector to 24 V, then return to the saved joints."
        )
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.10,
        help="motion speed ratio in (0, 1], default: 0.10",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for current robot state, default: 5",
    )
    parser.add_argument(
        "--motion-timeout",
        type=float,
        default=60.0,
        help="seconds to wait for each motion, default: 60",
    )
    parser.add_argument(
        "--position-tolerance-mm",
        type=float,
        default=3.0,
        help="Cartesian arrival tolerance in millimetres, default: 3",
    )
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=2.0,
        help="tool-orientation arrival tolerance in degrees, default: 2",
    )
    parser.add_argument(
        "--joint-tolerance-deg",
        type=float,
        default=1.5,
        help="joint arrival tolerance in degrees, default: 1.5",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="confirm that the real robot workspace is clear and allow motion",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if not 0.0 < args.speed <= 1.0:
        parser.error("--speed must be in (0, 1]")
    if args.state_timeout <= 0.0 or args.motion_timeout <= 0.0:
        parser.error("timeouts must be greater than zero")
    if (
        args.position_tolerance_mm <= 0.0
        or args.orientation_tolerance_deg <= 0.0
        or args.joint_tolerance_deg <= 0.0
    ):
        parser.error("arrival tolerances must be greater than zero")
    if not args.execute:
        parser.error("real-arm motion requires --execute")
    return args


def quaternion_from_rpy(roll, pitch, yaw):
    """Return a geometry_msgs quaternion tuple for fixed-axis roll/pitch/yaw."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RoundTrip:
    def __init__(self, args):
        self.args = args
        self._condition = threading.Condition()
        self._state_sequence = 0
        self._latest_state = None
        self._plan_sequence = 0
        self._latest_plan_ok = False

        self.state_subscriber = rospy.Subscriber(
            STATE_TOPIC, Arm_Current_State, self._state_callback, queue_size=1
        )
        self.plan_subscriber = rospy.Subscriber(
            PLAN_STATE_TOPIC, Plan_State, self._plan_callback, queue_size=10
        )
        self.state_publisher = rospy.Publisher(
            GET_STATE_COMMAND_TOPIC, GetArmState_Command, queue_size=1
        )
        self.pose_publisher = rospy.Publisher(MOVE_POSE_TOPIC, MoveJ_P, queue_size=1)
        self.joint_publisher = rospy.Publisher(MOVE_JOINT_TOPIC, MoveJ, queue_size=1)
        self.linear_publisher = rospy.Publisher(MOVE_LINEAR_TOPIC, MoveL, queue_size=1)
        self.stop_publisher = rospy.Publisher(EMERGENCY_STOP_TOPIC, Empty, queue_size=1)

    def _state_callback(self, message):
        with self._condition:
            self._latest_state = message
            self._state_sequence += 1
            self._condition.notify_all()

    def _plan_callback(self, message):
        with self._condition:
            self._latest_plan_ok = bool(message.state)
            self._plan_sequence += 1
            self._condition.notify_all()

    @staticmethod
    def ensure_exclusive_motion_control():
        """Reject startup while another ROS node can command arm motion."""
        try:
            code, message, system_state = rospy.get_master().getSystemState()
        except Exception as error:  # XML-RPC errors vary between ROS distributions.
            raise AutomationError("无法检查 ROS 命令源：{}".format(error))
        if code != 1:
            raise AutomationError("无法检查 ROS 命令源：{}".format(message))

        own_node = rospy.get_name()
        publishers = dict(system_state[0])
        conflicts = []
        for topic in sorted(MOTION_COMMAND_TOPICS):
            other_nodes = [node for node in publishers.get(topic, []) if node != own_node]
            if other_nodes:
                conflicts.append("{} <- {}".format(topic, ", ".join(other_nodes)))
        if conflicts:
            raise AutomationError(
                "检测到其他机械臂运动命令源，请先停止：{}".format("; ".join(conflicts))
            )

    @staticmethod
    def _wait_for_connections(handles, timeout, description):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown():
            if all(handle.get_num_connections() > 0 for handle in handles):
                return
            if time.monotonic() >= deadline:
                raise AutomationError("等待 {} ROS 连接超时".format(description))
            rospy.sleep(0.05)
        raise AutomationError("ROS 正在关闭")

    def capture_start_joints(self):
        self._wait_for_connections(
            [self.state_publisher, self.state_subscriber],
            self.args.state_timeout,
            "机械臂状态",
        )
        with self._condition:
            previous_sequence = self._state_sequence

        self.state_publisher.publish(
            GetArmState_Command(command="get_current_arm_state")
        )
        state = self._wait_for_new_state(previous_sequence, self.args.state_timeout)

        if state.dof != 7 or len(state.joint) != 7:
            raise AutomationError(
                "期望 RM75 的 7 个关节角，实际 dof={}、joint 数量={}".format(
                    state.dof, len(state.joint)
                )
            )
        if any(not math.isfinite(value) for value in state.joint):
            raise AutomationError("当前关节角包含 NaN/Inf")
        if any(error != 0 for error in state.err):
            raise AutomationError("机械臂当前存在错误码：{}".format(list(state.err)))

        # Arm_Current_State.joint is published in degrees; MoveJ expects radians.
        joints = [math.radians(value) for value in state.joint]
        rospy.loginfo("已保存起始关节角（deg）：%s", [round(v, 3) for v in state.joint])
        return joints

    def _wait_for_new_state(self, previous_sequence, timeout):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._state_sequence <= previous_sequence and not rospy.is_shutdown():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise AutomationError("获取机械臂当前状态超时")
                self._condition.wait(min(remaining, 0.2))
            if rospy.is_shutdown():
                raise AutomationError("ROS 正在关闭")
            return self._latest_state

    def _goal_error(self, command, state):
        """Return (arrived, details) using fresh controller state feedback."""
        if state.dof != 7 or len(state.joint) != 7:
            raise AutomationError(
                "运动中状态反馈异常：dof={}、joint 数量={}".format(
                    state.dof, len(state.joint)
                )
            )
        if any(error != 0 for error in state.err):
            raise AutomationError("运动中出现机械臂错误码：{}".format(list(state.err)))

        if hasattr(command, "Pose"):
            current_xyz = tuple(state.Pose[:3])
            target_xyz = (
                command.Pose.position.x,
                command.Pose.position.y,
                command.Pose.position.z,
            )
            if any(not math.isfinite(value) for value in state.Pose[:6]):
                raise AutomationError("末端位姿反馈包含 NaN/Inf")
            position_error = math.sqrt(
                sum((current - target) ** 2 for current, target in zip(current_xyz, target_xyz))
            )

            current_quaternion = quaternion_from_rpy(*state.Pose[3:6])
            target_quaternion = (
                command.Pose.orientation.x,
                command.Pose.orientation.y,
                command.Pose.orientation.z,
                command.Pose.orientation.w,
            )
            dot = abs(
                sum(
                    current * target
                    for current, target in zip(current_quaternion, target_quaternion)
                )
            )
            orientation_error = 2.0 * math.acos(max(-1.0, min(1.0, dot)))
            arrived = (
                position_error <= self.args.position_tolerance_mm / 1000.0
                and orientation_error
                <= math.radians(self.args.orientation_tolerance_deg)
            )
            return arrived, "位置误差={:.2f} mm，姿态误差={:.2f} deg".format(
                position_error * 1000.0, math.degrees(orientation_error)
            )

        if hasattr(command, "joint"):
            current_joints = [math.radians(value) for value in state.joint]
            if len(command.joint) != 7 or any(
                not math.isfinite(value) for value in current_joints
            ):
                raise AutomationError("关节目标或反馈无效")
            max_error = max(
                abs(current - target)
                for current, target in zip(current_joints, command.joint)
            )
            arrived = max_error <= math.radians(self.args.joint_tolerance_deg)
            return arrived, "最大关节误差={:.2f} deg".format(math.degrees(max_error))

        raise AutomationError("无法识别运动指令类型")

    def _publish_motion_and_wait(self, publisher, message, description):
        self._wait_for_connections(
            [
                publisher,
                self.plan_subscriber,
                self.state_publisher,
                self.state_subscriber,
            ],
            self.args.state_timeout,
            description,
        )
        with self._condition:
            previous_plan_sequence = self._plan_sequence
            processed_state_sequence = self._state_sequence
        rospy.loginfo("开始%s", description)
        publisher.publish(message)

        deadline = time.monotonic() + self.args.motion_timeout
        next_state_request = 0.0
        consecutive_arrivals = 0
        while not rospy.is_shutdown():
            now = time.monotonic()
            if now >= deadline:
                self.stop_publisher.publish(Empty())
                raise AutomationError(
                    "{}超时，已请求急停；不会继续发送运动指令".format(description)
                )

            if now >= next_state_request:
                self.state_publisher.publish(
                    GetArmState_Command(command="get_current_arm_state")
                )
                next_state_request = now + 0.5

            with self._condition:
                self._condition.wait(min(0.1, deadline - now))
                plan_sequence = self._plan_sequence
                plan_ok = self._latest_plan_ok
                state_sequence = self._state_sequence
                state = self._latest_state

            if plan_sequence > previous_plan_sequence and not plan_ok:
                raise AutomationError("{}失败（Plan_State=false）".format(description))

            if state_sequence > processed_state_sequence and state is not None:
                processed_state_sequence = state_sequence
                arrived, details = self._goal_error(message, state)
                consecutive_arrivals = consecutive_arrivals + 1 if arrived else 0
                if consecutive_arrivals >= 2:
                    rospy.loginfo("%s完成（真实反馈：%s）", description, details)
                    return

        raise AutomationError("ROS 正在关闭")

    def move_to_target(self):
        x_mm, y_mm, z_mm, roll, pitch, yaw = TARGET_POSE
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        command = MoveJ_P()
        command.Pose.position.x = x_mm / 1000.0
        command.Pose.position.y = y_mm / 1000.0
        command.Pose.position.z = z_mm / 1000.0
        command.Pose.orientation.x = qx
        command.Pose.orientation.y = qy
        command.Pose.orientation.z = qz
        command.Pose.orientation.w = qw
        command.speed = self.args.speed
        command.trajectory_connect = 0
        self._publish_motion_and_wait(self.pose_publisher, command, "目标位姿运动")

    def retreat_along_tool_z(self):
        """Move 150 mm along tool -Z while keeping the target orientation."""
        x_mm, y_mm, z_mm, roll, pitch, yaw = TARGET_POSE
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

        # Tool Z in the work frame is the third column of Rz(yaw)Ry(pitch)Rx(roll).
        tool_z_x = (
            math.cos(yaw) * math.sin(pitch) * math.cos(roll)
            + math.sin(yaw) * math.sin(roll)
        )
        tool_z_y = (
            math.sin(yaw) * math.sin(pitch) * math.cos(roll)
            - math.cos(yaw) * math.sin(roll)
        )
        tool_z_z = math.cos(pitch) * math.cos(roll)

        command = MoveL()
        command.Pose.position.x = x_mm / 1000.0 + TOOL_Z_RETREAT_METRES * tool_z_x
        command.Pose.position.y = y_mm / 1000.0 + TOOL_Z_RETREAT_METRES * tool_z_y
        command.Pose.position.z = z_mm / 1000.0 + TOOL_Z_RETREAT_METRES * tool_z_z
        command.Pose.orientation.x = qx
        command.Pose.orientation.y = qy
        command.Pose.orientation.z = qz
        command.Pose.orientation.w = qw
        command.speed = self.args.speed
        command.trajectory_connect = 0
        rospy.loginfo(
            "沿工具 -Z 退回 150 mm，目标 XYZ(mm)：[%.3f, %.3f, %.3f]",
            command.Pose.position.x * 1000.0,
            command.Pose.position.y * 1000.0,
            command.Pose.position.z * 1000.0,
        )
        self._publish_motion_and_wait(
            self.linear_publisher, command, "工具 -Z 方向退回"
        )

    def return_to_start(self, start_joints):
        command = MoveJ()
        command.joint = start_joints
        command.speed = self.args.speed
        command.trajectory_connect = 0
        self._publish_motion_and_wait(self.joint_publisher, command, "起始关节姿态返回")


def set_tool_voltage_24():
    rospy.loginfo("执行：rosrun rm_driver set_tool_voltage.py 24")
    try:
        completed = subprocess.run(
            ["rosrun", "rm_driver", "set_tool_voltage.py", "24"], check=False
        )
    except OSError as error:
        raise AutomationError("无法启动 set_tool_voltage.py：{}".format(error))
    if completed.returncode != 0:
        raise AutomationError(
            "24V 设置失败，set_tool_voltage.py 退出码 {}".format(
                completed.returncode
            )
        )


def main():
    args = parse_args()
    rospy.init_node("tool_voltage_roundtrip", anonymous=False)
    automation = RoundTrip(args)

    automation.ensure_exclusive_motion_control()
    start_joints = automation.capture_start_joints()
    automation.move_to_target()

    voltage_error = None
    try:
        set_tool_voltage_24()
    except AutomationError as error:
        voltage_error = error
        rospy.logerr("%s；仍将尝试返回起始姿态", error)

    if voltage_error is None:
        rospy.loginfo("24V 已开启，等待 2 秒后沿工具 -Z 退回")
        rospy.sleep(2.0)
        automation.retreat_along_tool_z()

    automation.return_to_start(start_joints)
    if voltage_error is not None:
        raise voltage_error
    rospy.loginfo("自动流程完成：24V 已设置，机械臂已返回起始姿态")


if __name__ == "__main__":
    try:
        main()
    except (AutomationError, rospy.ROSException) as error:
        rospy.logerr("自动流程终止：%s", error)
        raise SystemExit(1)
