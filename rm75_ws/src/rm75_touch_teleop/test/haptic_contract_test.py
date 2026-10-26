#!/usr/bin/env python3
import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from rm_msgs.msg import JointPos, Six_Force
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Empty


class HapticContract(unittest.TestCase):
    def setUp(self):
        self.commands = []
        self.robot_commands = []
        self.stops = 0
        self.fresh = False
        rospy.Subscriber('/haptic_contract/haptic_device_command', Vector3Stamped,
                         lambda m: self.commands.append(m))
        rospy.Subscriber('/haptic_contract/haptic_force_fresh', Bool,
                         lambda m: setattr(self, 'fresh', m.data))
        rospy.Subscriber('/test/haptic/robot_command', JointPos,
                         lambda m: self.robot_commands.append(m))
        rospy.Subscriber('/test/haptic/stop', Empty,
                         lambda _: setattr(self, 'stops', self.stops + 1))
        self.pose_pub = rospy.Publisher('/test/haptic/pose', PoseStamped, queue_size=1)
        self.joy_pub = rospy.Publisher('/test/haptic/joy', Joy, queue_size=1)
        self.twist_pub = rospy.Publisher('/test/haptic/twist', Twist, queue_size=1)
        self.joint_pub = rospy.Publisher('/test/haptic/joints', JointState, queue_size=1)
        self.force_pub = rospy.Publisher('/test/haptic/force', Six_Force, queue_size=1)

    def tick_inputs(self, force=None):
        pose = PoseStamped(); pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        self.joy_pub.publish(Joy(buttons=[0, 0]))
        self.twist_pub.publish(Twist())
        self.joint_pub.publish(JointState(
            name=['joint%d' % i for i in range(1, 8)],
            position=[0., .8395, 0., 1.4442, 0., .8513, 0.]))
        if force is not None:
            self.force_pub.publish(Six_Force(force_Fx=force))

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.tick_inputs()
            time.sleep(.02)
        self.assertTrue(predicate(), 'timed out')

    def test_disarmed_fake_force_and_watchdog(self):
        self.wait(lambda: self.force_pub.get_num_connections() > 0 and
                  self.joint_pub.get_num_connections() > 0)
        for _ in range(15):
            self.tick_inputs(10.0)
            time.sleep(.02)
        self.wait(lambda: self.fresh and self.commands and
                  abs(self.commands[-1].vector.z) > .05)
        command = self.commands[-1].vector
        self.assertAlmostEqual(command.x, 0.0, places=6)
        self.assertAlmostEqual(command.y, 0.0, places=6)
        self.assertLess(command.z, -0.05)  # robot +X -> Touch -Z via S^T
        self.wait(lambda: (not self.fresh) and self.commands and
                  math.sqrt(self.commands[-1].vector.x ** 2 +
                            self.commands[-1].vector.y ** 2 +
                            self.commands[-1].vector.z ** 2) < 1e-6)
        self.assertEqual(self.robot_commands, [])
        self.assertEqual(self.stops, 0)


if __name__ == '__main__':
    rospy.init_node('haptic_contract_test')
    rostest.rosrun('rm75_touch_teleop', 'haptic_contract_test', HapticContract)
