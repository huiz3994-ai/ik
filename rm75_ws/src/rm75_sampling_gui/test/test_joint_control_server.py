#!/usr/bin/env python3
"""Isolated preflight tests for the supervised JointPos adapter."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/joint_control_server.py"
SPEC = importlib.util.spec_from_file_location("rm75_guarded_joint_server", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GuardedServerPreflightTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RM75RobotModel()

    def server(self, heartbeat_age=0.0):
        server = MODULE.GuardedJointServer.__new__(MODULE.GuardedJointServer)
        server.authorized = True
        server.feedback_timeout = 0.15
        server.heartbeat_timeout = 0.35
        server.max_delta = None
        server.velocity_limits = [self.robot.velocity_limits[name]
                                  for name in JOINT_NAMES]
        server.tracking_limit = 0.05
        server.tracking_latency_allowance = 0.02
        server.settling_tracking_limit = 0.08
        server.goal_tolerance = 0.015
        server.limit_margin = 0.02
        server.collision_service = "/check_state_validity"
        server.move_group = "arm"
        server.limits = [self.robot.limits[name] for name in JOINT_NAMES]
        now = time.monotonic()
        server._snapshot = lambda: (
            np.zeros(7), now, now - heartbeat_age, 0, 0, True, now
        )
        return server

    @staticmethod
    def goal(target_degrees=0.5):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(reversed(JOINT_NAMES))
        duration = 4.0
        for stamp in np.linspace(0.0, duration, 81):
            u = stamp / duration
            scale = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
            values = np.full(7, math.radians(target_degrees) * scale)
            point = JointTrajectoryPoint()
            point.positions = list(reversed(values.tolist()))
            point.time_from_start = rospy.Duration.from_sec(float(stamp))
            goal.trajectory.points.append(point)
        return goal

    def test_valid_supervised_half_degree_path_passes(self):
        validity = lambda _request: SimpleNamespace(valid=True)
        with patch("rosnode.get_node_names", return_value=[]), \
                patch("rospy.wait_for_service"), \
                patch("rospy.ServiceProxy", return_value=validity):
            positions, times = self.server()._preflight(self.goal())
        self.assertEqual(len(positions), 81)
        self.assertAlmostEqual(math.degrees(positions[-1][0]), 0.5, places=5)
        self.assertEqual(times[0], 0.0)

    def test_target_delta_has_no_separate_maximum(self):
        validity = lambda _request: SimpleNamespace(valid=True)
        with patch("rosnode.get_node_names", return_value=[]), \
                patch("rospy.wait_for_service"), \
                patch("rospy.ServiceProxy", return_value=validity):
            positions, _times = self.server()._preflight(self.goal(1.1))
        self.assertAlmostEqual(math.degrees(positions[-1][0]), 1.1, places=5)

    def test_missing_gui_heartbeat_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "heartbeat"):
            self.server(heartbeat_age=1.0)._preflight(self.goal())

    def test_tracking_guard_compensates_one_command_period_and_settling(self):
        server = self.server()
        moving = server._allowed_tracking_error(np.full(7, 2.0))
        np.testing.assert_allclose(moving, np.full(7, 0.09))
        settling = server._allowed_tracking_error(np.zeros(7), settling=True)
        np.testing.assert_allclose(settling, np.full(7, 0.08))

    def test_full_gui_yaml_with_null_values_loads_only_joint_options(self):
        options = MODULE.load_joint_options(
            str(SCRIPT.parents[1] / "config" / "gui.yaml")
        )
        self.assertEqual(options["trajectory_action"],
                         "/rm75_sampling_gui/follow_joint_trajectory")
        self.assertEqual(options["max_tracking_error_rad"], 0.05)
        self.assertNotIn("max_single_delta_rad", options)


if __name__ == "__main__":
    unittest.main()
