#!/usr/bin/env python3
"""Place object A using the camera-safe overhead path, then return."""

import rospy

from pick_object_a import OverheadRoundTrip, TARGET_POSE
from tool_voltage_off_roundtrip import set_tool_voltage_zero
from tool_voltage_roundtrip import AutomationError, parse_args


PLACE_OVERHEAD_Z_OFFSET_METRES = 0.200


class PlaceObjectARoundTrip(OverheadRoundTrip):
    def move_above_with_start_orientation(self):
        rospy.loginfo(
            "先移动到目标上方 200 mm：XYZ(mm)=[%.3f, %.3f, %.3f]",
            TARGET_POSE[0],
            TARGET_POSE[1],
            TARGET_POSE[2] + PLACE_OVERHEAD_Z_OFFSET_METRES * 1000.0,
        )
        self._move_pose(
            PLACE_OVERHEAD_Z_OFFSET_METRES,
            self.start_rpy,
            "保持初始姿态移动到上方 200 mm 预备点",
        )

    def rise_from_target(self):
        self._move_linear(
            PLACE_OVERHEAD_Z_OFFSET_METRES,
            TARGET_POSE[3:6],
            "保持目标姿态直线上升 200 mm",
        )

    def orient_above_target(self):
        self._move_pose(
            PLACE_OVERHEAD_Z_OFFSET_METRES,
            TARGET_POSE[3:6],
            "在上方 200 mm 安全点转换为目标姿态",
        )

    def descend_to_target(self):
        self._move_linear(
            0.0,
            TARGET_POSE[3:6],
            "保持目标姿态直线下降 200 mm",
        )

    def restore_start_orientation_above(self):
        self._move_pose(
            PLACE_OVERHEAD_Z_OFFSET_METRES,
            self.start_rpy,
            "在上方 200 mm 安全点恢复初始姿态",
        )


def main():
    args = parse_args()
    rospy.init_node("place_object_a", anonymous=False)
    automation = PlaceObjectARoundTrip(args)

    automation.ensure_exclusive_motion_control()
    start_joints = automation.capture_start_joints()
    automation.move_above_with_start_orientation()
    automation.orient_above_target()
    automation.descend_to_target()

    voltage_error = None
    try:
        set_tool_voltage_zero()
    except AutomationError as error:
        voltage_error = error
        rospy.logerr("%s；仍将尝试按原路径返回", error)

    automation.rise_from_target()
    automation.restore_start_orientation_above()
    automation.return_to_start(start_joints)
    if voltage_error is not None:
        raise voltage_error
    rospy.loginfo("放置物体 A 完成：工具电压已关闭，机械臂已返回起始姿态")


if __name__ == "__main__":
    try:
        main()
    except (AutomationError, rospy.ROSException) as error:
        rospy.logerr("自动流程终止：%s", error)
        raise SystemExit(1)
