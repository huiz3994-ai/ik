#!/usr/bin/env python3
"""Restore the RM75 tool to one fixed teach-pendant pose."""

import argparse

import rospy

from rm_msgs.msg import MoveJ_P
from tool_voltage_roundtrip import AutomationError, RoundTrip, quaternion_from_rpy


# Teach-pendant convention: X/Y/Z in millimetres, R/P/Y in radians.
TARGET_POSE = (1.646, -0.201, 867.634, 0.0, 0.021, 0.003)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "将 RM75 末端恢复到固定示教器位姿："
            "[1.646, -0.201, 867.634] mm，[0, 0.021, 0.003] rad。"
        )
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.05,
        help="运动速度比例 (0, 0.10]，默认 0.05",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=5.0,
        help="等待机械臂状态的秒数，默认 5",
    )
    parser.add_argument(
        "--motion-timeout",
        type=float,
        default=90.0,
        help="等待运动完成的秒数，默认 90",
    )
    parser.add_argument(
        "--position-tolerance-mm",
        type=float,
        default=3.0,
        help="位置到位容差（毫米），默认 3",
    )
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=2.0,
        help="姿态到位容差（度），默认 2",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="确认现场净空、急停可用，并允许真机运动",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    if not 0.0 < args.speed <= 0.10:
        parser.error("--speed 必须在 (0, 0.10] 之间")
    if args.state_timeout <= 0.0 or args.motion_timeout <= 0.0:
        parser.error("超时时间必须大于 0")
    if args.position_tolerance_mm <= 0.0 or args.orientation_tolerance_deg <= 0.0:
        parser.error("到位容差必须大于 0")
    if not args.execute:
        parser.error("真机运动必须显式添加 --execute")

    # RoundTrip's shared feedback checker also expects this field.  This script
    # never sends a joint-space target, so the value is intentionally fixed.
    args.joint_tolerance_deg = 1.5
    return args


class FixedPoseRestore(RoundTrip):
    def restore(self):
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

        rospy.loginfo(
            "目标位姿 XYZ(mm)=[%.3f, %.3f, %.3f]，RPY(rad)=[%.3f, %.3f, %.3f]",
            *TARGET_POSE
        )
        self._publish_motion_and_wait(
            self.pose_publisher,
            command,
            "固定位置姿态恢复",
        )


def main():
    args = parse_args()
    rospy.init_node("restore_rm75_fixed_pose", anonymous=False)
    automation = FixedPoseRestore(args)

    automation.ensure_exclusive_motion_control()
    automation.capture_start_joints()  # Also validates DOF, feedback and errors.
    automation.restore()
    rospy.loginfo("位置姿态恢复完成")


if __name__ == "__main__":
    try:
        main()
    except (AutomationError, rospy.ROSException) as error:
        rospy.logerr("位置姿态恢复失败：%s", error)
        raise SystemExit(1)
