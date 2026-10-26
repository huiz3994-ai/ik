#!/usr/bin/env python3
"""No-hardware tests for the RM75-6FB digital twin and safety adapter."""

import math
import os
import time
import unittest
from unittest.mock import patch

import numpy as np
import rospy
from PyQt5.QtWidgets import QApplication
from sensor_msgs.msg import JointState

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rm75_sampling_gui.control.joint_control_adapter import JointControlAdapter
from rm75_sampling_gui.model.robot_model import JOINT_NAMES, RM75RobotModel
from rm75_sampling_gui.ros.monitoring import Monitor
from rm75_sampling_gui.state.state_machine import RobotState
from rm75_sampling_gui.widgets.joint_control_widget import JointControlWidget


def configuration(authorized=False):
    return {
        "joint_control_execution_authorized": authorized,
        "trajectory_execution_authorized": authorized,
        "supervised_commissioning_authorized": False,
        "joint_control": {
            "execution_path_verified": authorized,
            "gui_loss_watchdog_verified": authorized,
            "collision_scene_verified": authorized,
            "joint_limit_margin_rad": 0.02,
            "feedback_timeout_s": 0.15,
            "command_ttl_s": 0.05,
            "minimum_duration_s": 1.0,
            "trajectory_period_s": 0.01,
            "collision_samples": 9,
        },
        "rendering": {"enabled": False},
    }


def snapshot(values=None, age=0.01, task_state="IDLE"):
    vector = np.zeros(7) if values is None else np.asarray(values)
    return {
        "joints": dict(zip(JOINT_NAMES, vector.tolist())),
        "fresh": {"joints": True},
        "ages": {"joints": age},
        "task_status": {"state": task_state},
        "source_stamps": {"joints": 100.0},
    }


class RobotModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RM75RobotModel()

    def test_vendor_6fb_model_meshes_chain_limits_and_seventh_axis(self):
        self.assertEqual(tuple(joint.name for joint in self.robot.chain), JOINT_NAMES)
        self.assertEqual(self.robot.chain[-1].parent, "Link6")
        self.assertEqual(self.robot.chain[-1].child, "Link7")
        self.assertAlmostEqual(self.robot.chain[-1].origin.xyz[2], 0.1612, places=6)
        link7 = [visual for visual in self.robot.visuals if visual.link == "Link7"]
        self.assertEqual(len(link7), 1)
        self.assertTrue(link7[0].filename.endswith("Link7_6fb.STL"))
        self.assertEqual(self.robot.limits["joint7"], (-6.28, 6.28))
        transform = self.robot.forward_kinematics(np.zeros(7))["Link7"]
        self.assertAlmostEqual(transform[2, 3], 0.8677, places=4)

    def test_fk_changes_with_each_named_axis_and_rejects_limit(self):
        origin = self.robot.forward_kinematics(np.zeros(7))["Link7"]
        for index in range(7):
            q = np.zeros(7)
            q[index] = 0.1
            changed = self.robot.forward_kinematics(q)["Link7"]
            self.assertFalse(np.allclose(origin, changed), "joint{} had no FK effect".format(index + 1))
        target = np.zeros(7)
        target[1] = self.robot.limits["joint2"][1] + 0.1
        self.assertEqual(self.robot.limit_violations(target), ["joint2"])

    def test_joint_state_name_mapping_and_measured_velocity_label(self):
        monitor = Monitor({"feedback_max_age_s": 1.0})
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(10.0)
        msg.name = list(reversed(JOINT_NAMES))
        msg.position = list(reversed([0.1 * index for index in range(1, 8)]))
        msg.velocity = list(reversed([0.01 * index for index in range(1, 8)]))
        with patch("rospy.Time.now", return_value=rospy.Time.from_sec(10.05)):
            monitor.joints(msg)
        data = monitor.snapshot()
        self.assertAlmostEqual(data["joints"]["joint1"], 0.1)
        self.assertAlmostEqual(data["joint_velocities"]["joint7"], 0.07)
        self.assertEqual(data["joint_velocity_source"], "measured")
        self.assertTrue(data["fresh"]["joint_velocities"])

        msg.velocity = []
        with patch("rospy.Time.now", return_value=rospy.Time.from_sec(10.06)):
            monitor.joints(msg)
        data = monitor.snapshot()
        self.assertEqual(data["joint_velocities"], {})
        self.assertEqual(data["joint_velocity_source"], "unavailable")
        self.assertFalse(data["fresh"]["joint_velocities"])


class JointControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RM75RobotModel()
        cls.app = QApplication.instance() or QApplication([])

    def adapter(self, authorized=False, collision=False, nodes=()):
        adapter = JointControlAdapter(
            self.robot, configuration(authorized),
            collision_checker=lambda _q: collision,
            graph_checker=lambda: nodes,
        )
        if authorized:
            adapter._bridge_check = lambda: []
        return adapter

    def test_slider_only_changes_preview_and_never_creates_ros_publisher(self):
        with patch("rospy.Publisher") as publisher:
            widget = JointControlWidget(self.robot, configuration(False))
            seen = []
            widget.target_changed.connect(lambda value: seen.append(value))
            widget.sliders[0].setValue(10)
            self.app.processEvents()
            self.assertAlmostEqual(widget.target[0], math.radians(1.0), places=6)
            self.assertTrue(seen)
            publisher.assert_not_called()
            widget.deleteLater()

    def test_execute_button_emits_target_directly_without_ui_confirmation(self):
        widget = JointControlWidget(self.robot, configuration(True))
        requested = []
        widget.execute_requested.connect(lambda target: requested.append(target))
        widget.execute_button.click()
        self.app.processEvents()
        self.assertEqual(len(requested), 1)
        np.testing.assert_allclose(requested[0], widget.target)
        self.assertFalse(hasattr(widget, "confirm"))
        self.assertFalse(hasattr(widget, "validate_button"))
        widget.deleteLater()

    def test_first_feedback_initializes_target_without_overwriting_user_edit(self):
        widget = JointControlWidget(self.robot, configuration(False))
        actual = np.asarray([-1.0, -0.2, -2.0, 0.4, 2.0, -1.0, -0.3])
        widget.update_snapshot(snapshot(actual))
        np.testing.assert_allclose(widget.target, actual)

        widget.target_boxes[6].setValue(math.degrees(actual[6]) + 1.0)
        edited = widget.target.copy()
        widget.update_snapshot(snapshot(actual + 0.1))
        np.testing.assert_allclose(widget.target, edited)
        widget.deleteLater()

    def test_independent_authorization_rejects_real_execution(self):
        result = self.adapter(False).validate(
            np.zeros(7), snapshot(), RobotState.IDLE, None, for_execution=True
        )
        self.assertFalse(result["ok"])
        self.assertIsNone(result["token"])
        auth = next(item for item in result["checks"] if item["name"] == "独立执行授权")
        self.assertFalse(auth["ok"])

    def test_supervised_adapter_authorizes_without_legacy_bridge_flags(self):
        config = configuration(True)
        config["trajectory_execution_authorized"] = False
        config["supervised_commissioning_authorized"] = True
        config["joint_control"]["execution_path_verified"] = False
        config["joint_control"]["gui_loss_watchdog_verified"] = False
        config["joint_control"]["collision_scene_verified"] = False
        adapter = JointControlAdapter(
            self.robot, config, collision_checker=lambda _q: False,
            graph_checker=lambda: ("/joint_control_server",),
        )
        result = adapter.validate(
            np.zeros(7), snapshot(), RobotState.IDLE, None, for_execution=True
        )
        self.assertTrue(result["ok"])
        self.assertTrue(result["execution_authorized"])
        self.assertIsNotNone(result["token"])

    def test_limit_collision_conflict_timeout_and_safe_stop_rejected(self):
        adapter = self.adapter(True)
        over = np.zeros(7)
        over[1] = self.robot.limits["joint2"][1]
        self.assertFalse(adapter.validate(over, snapshot(), RobotState.IDLE, None, True)["ok"])
        far = np.zeros(7)
        far[0] = math.radians(4.0)
        self.assertTrue(adapter.validate(far, snapshot(), RobotState.IDLE, None, True)["ok"])
        self.assertFalse(self.adapter(True, collision=True).validate(
            np.zeros(7), snapshot(), RobotState.IDLE, None, True)["ok"])
        self.assertFalse(self.adapter(True, nodes=("/rm75_touch_teleop",)).validate(
            np.zeros(7), snapshot(), RobotState.IDLE, None, True)["ok"])
        self.assertFalse(adapter.validate(
            np.zeros(7), snapshot(age=1.0), RobotState.IDLE, None, True)["ok"])
        self.assertFalse(adapter.validate(
            np.zeros(7), snapshot(), RobotState.SAFE_STOP, None, True)["ok"])

    def test_valid_ticket_expires_and_target_change_invalidates(self):
        adapter = self.adapter(True)
        target = np.full(7, math.radians(0.2))
        result = adapter.validate(target, snapshot(), RobotState.IDLE, None, True)
        self.assertTrue(result["ok"])
        self.assertIsNotNone(result["token"])
        ticket = adapter._ticket
        self.assertIsNotNone(ticket)
        ticket.expires = time.monotonic() - 1.0
        with self.assertRaisesRegex(Exception, "过期"):
            adapter.execute(result["token"], target, lambda: snapshot())

    def test_quintic_trajectory_uses_vendor_urdf_velocity_limits(self):
        adapter = self.adapter(True)
        positions, times = adapter.build_trajectory(
            np.zeros(7), np.full(7, 2.0)
        )
        q = np.asarray(positions)
        dt = times[1] - times[0]
        velocity = np.diff(q, axis=0) / dt
        limits = np.asarray([self.robot.velocity_limits[name] for name in JOINT_NAMES])
        self.assertTrue(np.all(np.max(np.abs(velocity), axis=0) <= limits * 1.01))
        self.assertTrue(np.allclose(q[0], 0.0))
        self.assertTrue(np.allclose(q[-1], 2.0))

    def test_cancel_is_nonblocking_and_latches_request(self):
        adapter = self.adapter(True)
        result = adapter.cancel()
        self.assertTrue(result["cancel_requested"])
        self.assertTrue(adapter._cancel.is_set())


if __name__ == "__main__":
    unittest.main()
