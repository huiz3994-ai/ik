#!/usr/bin/env python3
"""Pick object A from an overhead approach, enable 24 V, and return."""

import math

import rospy
from rm_msgs.msg import MoveJ_P, MoveL

from tool_voltage_roundtrip import (
    AutomationError,
    RoundTrip,
    parse_args,
    quaternion_from_rpy,
    set_tool_voltage_24,
)


TARGET_POSE = (134.424, 183.920, 219.935, 3.128, 0.030, 1.560)
OVERHEAD_Z_OFFSET_METRES = 0.250


class OverheadRoundTrip(RoundTrip):
    def __init__(self, args):
        super().__init__(args)
        self.start_rpy = None

    def capture_start_joints(self):
        joints = super().capture_start_joints()
        with self._condition:
            state = self._latest_state
            self.start_rpy = tuple(state.Pose[3:6])
        if len(self.start_rpy) != 3 or any(
            not math.isfinite(value) for value in self.start_rpy
        ):
            raise AutomationError("起始末端姿态 RPY 无效：{}".format(self.start_rpy))
        rospy.loginfo(
            "已保存起始末端姿态 RPY(rad)：[%.3f, %.3f, %.3f]",
            *self.start_rpy
        )
        return joints

    @staticmethod
    def _fill_pose(message, z_offset_metres, rpy):
        x_mm, y_mm, z_mm, _, _, _ = TARGET_POSE
        qx, qy, qz, qw = quaternion_from_rpy(*rpy)
        message.Pose.position.x = x_mm / 1000.0
        message.Pose.position.y = y_mm / 1000.0
        message.Pose.position.z = z_mm / 1000.0 + z_offset_metres
        message.Pose.orientation.x = qx
        message.Pose.orientation.y = qy
        message.Pose.orientation.z = qz
        message.Pose.orientation.w = qw

    def _move_pose(self, z_offset_metres, rpy, description):
        command = MoveJ_P()
        self._fill_pose(command, z_offset_metres, rpy)
        command.speed = self.args.speed
        command.trajectory_connect = 0
        self._publish_motion_and_wait(self.pose_publisher, command, description)

    def _move_linear(self, z_offset_metres, rpy, description):
        command = MoveL()
        self._fill_pose(command, z_offset_metres, rpy)
        command.speed = self.args.speed
        command.trajectory_connect = 0
        self._publish_motion_and_wait(self.linear_publisher, command, description)

    def move_above_with_start_orientation(self):
        rospy.loginfo(
            "先移动到目标上方 250 mm：XYZ(mm)=[%.3f, %.3f, %.3f]",
            TARGET_POSE[0],
            TARGET_POSE[1],
            TARGET_POSE[2] + OVERHEAD_Z_OFFSET_METRES * 1000.0,
        )
        self._move_pose(
            OVERHEAD_Z_OFFSET_METRES,
            self.start_rpy,
            "保持初始姿态移动到上方预备点",
        )

    def descend_to_target(self):
        self._move_linear(
            0.0,
            TARGET_POSE[3:6],
            "下降 250 mm 并渐进旋转到目标姿态",
        )

    def rise_from_target(self):
        self._move_linear(
            OVERHEAD_Z_OFFSET_METRES,
            TARGET_POSE[3:6],
            "保持目标姿态直线上升 250 mm",
        )

    def restore_start_orientation_above(self):
        self._move_pose(
            OVERHEAD_Z_OFFSET_METRES,
            self.start_rpy,
            "在上方安全点恢复初始姿态",
        )


def main():
    args = parse_args()
    rospy.init_node("pick_object_a", anonymous=False)
    automation = OverheadRoundTrip(args)

    automation.ensure_exclusive_motion_control()
    start_joints = automation.capture_start_joints()
    automation.move_above_with_start_orientation()
    automation.descend_to_target()

    voltage_error = None
    try:
        set_tool_voltage_24()
    except AutomationError as error:
        voltage_error = error
        rospy.logerr("%s；仍将尝试按原路径返回", error)

    automation.rise_from_target()
    automation.restore_start_orientation_above()
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
