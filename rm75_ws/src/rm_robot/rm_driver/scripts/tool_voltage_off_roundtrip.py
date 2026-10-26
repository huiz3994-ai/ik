#!/usr/bin/env python3
"""Approach the work pose, disable tool voltage, and return safely."""

import math
import os
import sys
import subprocess

import rospy
from rm_msgs.msg import MoveJ_P, MoveL

# Force Python to import the real source script instead of the catkin relay wrapper.
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if SCRIPT_DIR in sys.path:
    sys.path.remove(SCRIPT_DIR)
sys.path.insert(0, SCRIPT_DIR)

from tool_voltage_roundtrip import (
    AutomationError,
    RoundTrip,
    TARGET_POSE,
    TOOL_Z_RETREAT_METRES,
    parse_args,
    quaternion_from_rpy,
)


class VoltageOffRoundTrip(RoundTrip):
    @staticmethod
    def _fill_pose(message, tool_z_offset_metres):
        x_mm, y_mm, z_mm, roll, pitch, yaw = TARGET_POSE
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        tool_z_x = (
            math.cos(yaw) * math.sin(pitch) * math.cos(roll)
            + math.sin(yaw) * math.sin(roll)
        )
        tool_z_y = (
            math.sin(yaw) * math.sin(pitch) * math.cos(roll)
            - math.cos(yaw) * math.sin(roll)
        )
        tool_z_z = math.cos(pitch) * math.cos(roll)

        message.Pose.position.x = x_mm / 1000.0 + tool_z_offset_metres * tool_z_x
        message.Pose.position.y = y_mm / 1000.0 + tool_z_offset_metres * tool_z_y
        message.Pose.position.z = z_mm / 1000.0 + tool_z_offset_metres * tool_z_z
        message.Pose.orientation.x = qx
        message.Pose.orientation.y = qy
        message.Pose.orientation.z = qz
        message.Pose.orientation.w = qw

    def move_to_approach(self):
        command = MoveJ_P()
        self._fill_pose(command, TOOL_Z_RETREAT_METRES)
        command.speed = self.args.speed
        command.trajectory_connect = 0
        rospy.loginfo(
            "移动到预备位姿 XYZ(mm)：[%.3f, %.3f, %.3f]",
            command.Pose.position.x * 1000.0,
            command.Pose.position.y * 1000.0,
            command.Pose.position.z * 1000.0,
        )
        self._publish_motion_and_wait(self.pose_publisher, command, "预备位姿运动")

    def move_linearly_to_target(self):
        command = MoveL()
        self._fill_pose(command, 0.0)
        command.speed = self.args.speed
        command.trajectory_connect = 0
        self._publish_motion_and_wait(
            self.linear_publisher, command, "工作位姿直线接近"
        )


def set_tool_voltage_zero():
    rospy.loginfo("执行：rosrun rm_driver set_tool_voltage.py 0")
    try:
        completed = subprocess.run(
            ["rosrun", "rm_driver", "set_tool_voltage.py", "0"], check=False
        )
    except OSError as error:
        raise AutomationError("无法启动 set_tool_voltage.py：{}".format(error))
    if completed.returncode != 0:
        raise AutomationError(
            "0V 设置失败，set_tool_voltage.py 退出码 {}".format(
                completed.returncode
            )
        )


def main():
    args = parse_args()
    rospy.init_node("tool_voltage_off_roundtrip", anonymous=False)
    automation = VoltageOffRoundTrip(args)

    automation.ensure_exclusive_motion_control()
    start_joints = automation.capture_start_joints()
    automation.move_to_approach()
    automation.move_linearly_to_target()

    voltage_error = None
    try:
        set_tool_voltage_zero()
    except AutomationError as error:
        voltage_error = error
        rospy.logerr("%s；仍将尝试沿原路径退回", error)

    automation.retreat_along_tool_z()
    automation.return_to_start(start_joints)
    if voltage_error is not None:
        raise voltage_error
    rospy.loginfo("自动流程完成：工具电压已关闭，机械臂已返回起始姿态")


if __name__ == "__main__":
    try:
        main()
    except (AutomationError, rospy.ROSException) as error:
        rospy.logerr("自动流程终止：%s", error)
        raise SystemExit(1)
