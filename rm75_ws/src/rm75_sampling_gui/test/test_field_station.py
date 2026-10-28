#!/usr/bin/env python3
"""Field-station regression suite: Qt offscreen, no ROS master or robot."""
import csv
import logging
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rospy
import yaml
from PyQt5.QtGui import QCloseEvent
from PyQt5.QtWidgets import QApplication, QMessageBox
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, String
from rm_msgs.msg import Six_Force
from rm75_sampling_gui.gui.main_window import MainWindow
from rm75_sampling_gui.ros.ros_interface import ROSInterface, ROSInterfaceError
from rm75_sampling_gui.ros.monitoring import Monitor
from rm75_sampling_gui.state.state_machine import RobotState, StateMachine, StateTransition, StateTransitionError
from rm75_sampling_gui.task.task_manager import TaskManager, ControlLaunchManager
from rm75_sampling_gui.task.system_manager import SystemLaunchManager, SystemLaunchError
from rm75_sampling_gui.logger.experiment_logger import ExperimentLogger
from rm75_sampling_gui.widgets.status_widget import StatusWidget
from rm75_sampling_gui.widgets.force_widget import ForceMonitorWidget
from rm75_sampling_gui.widgets.vision_widget import VisionWidget
from rm75_sampling_gui.widgets.diagnostic_widget import DiagnosticWidget
from rm75_sampling_gui.widgets.parameter_widget import ParameterWidget, load_parameter_sources

ROOT = Path(__file__).resolve().parents[1]


def config(directory):
    with (ROOT / "config/gui.yaml").open() as stream:
        value = yaml.safe_load(stream)
    with (ROOT / "config/tools.yaml").open() as stream:
        value["tools"] = yaml.safe_load(stream)["tools"]
    value["offline"] = True
    value["experiment"]["directory"] = directory
    value["parameter_sources"] = load_parameter_sources(value)
    return value


def sample():
    return {
        "timestamp": time.time(), "joints": {"joint{}".format(i): 0.1*i for i in range(1, 8)},
        "joint_limits": {"joint{}".format(i): (-2.0, 2.0) for i in range(1, 8)},
        "tcp": dict(x=0.1, y=0.2, z=0.3, rx=0.0, ry=0.0, rz=0.1),
        "wrench": dict(Fx=1.0, Fy=2.0, Fz=3.0, Tx=0.1, Ty=0.2, Tz=0.3),
        "force_z": -3.0, "filtered_force_z": -2.5, "force_setpoint": -5.0,
        "fresh": {key: True for key in ("joints", "tcp", "wrench", "force_z",
                  "filtered_force_z", "force_setpoint", "task_status", "vision_status")},
        "health": dict(master=True, driver=True, touch=True, qr=True, marker=True),
        "task_status": {"state": "HOLDING", "held_slot_id": 4},
        "tool_voltage": 24, "source_stamps": {"joints": 100.0, "tcp": 100.0},
    }


class StationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.config = config(self.temp.name)
        self.widgets = []

    def tearDown(self):
        for widget in self.widgets:
            widget.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def widget(self, value):
        self.widgets.append(value)
        return value

    def test_offline_main_window_has_all_pages_no_publishers(self):
        ros = ROSInterface(self.config)
        state = StateMachine()
        tasks = TaskManager(state, ros, self.config["tools"], self.config,
                            launch_manager=Mock())
        with patch("rospy.Publisher") as publisher:
            window = MainWindow(self.config, state, ros, tasks)
            window.show()
            self.app.processEvents()
            self.assertEqual(window.tabs.count(), 6)
            self.assertFalse(window._buttons["teleop_start"].isEnabled())
            publisher.assert_not_called()
            with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
                self.assertTrue(window.close())
            window.deleteLater()

    def test_system_controls_exist_and_are_offline_locked(self):
        ros = ROSInterface(self.config)
        state = StateMachine()
        tasks = TaskManager(state, ros, self.config["tools"], self.config,
                            launch_manager=Mock(), system_manager=Mock())
        window = MainWindow(self.config, state, ros, tasks)
        for name in ("base", "camera", "qr"):
            self.assertIn("system_{}_start".format(name), window._buttons)
            self.assertFalse(window._buttons["system_{}_start".format(name)].isEnabled())
        self.assertIn("camera_observation", window._buttons)
        self.assertEqual(window._buttons["camera_observation"].text(), "到达观察位")
        self.assertIn("target_return", window._buttons)
        self.assertEqual(window._buttons["target_return"].text(), "归还所持工具")
        self.assertIn("confirm_held_tool", window._buttons)
        self.assertEqual(
            window._buttons["confirm_held_tool"].text(), "确认持有选中工具")
        with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
            window.close()
        window.deleteLater()

    def test_system_manager_rejects_external_duplicate(self):
        manager = SystemLaunchManager(self.config, execution_enabled=False)
        with patch("rosnode.get_node_names", return_value=["/rm_driver"]):
            with self.assertRaises(SystemLaunchError):
                manager.start("base")

    def test_system_manager_rejects_conflicting_motion_source(self):
        manager = SystemLaunchManager(self.config, execution_enabled=False)
        with patch("rosnode.get_node_names", return_value=["/joint_control_server"]):
            with self.assertRaisesRegex(SystemLaunchError, "外部运动命令源"):
                manager.start("base")

    def test_status_does_not_report_owned_qr_node_as_external_pose(self):
        manager = SystemLaunchManager(self.config, execution_enabled=False)
        qr_process = Mock()
        qr_process.poll.return_value = None
        manager._processes["qr"] = qr_process
        live = ["/rm75_qr_marker", "/rm75_five_target_server"]

        with patch("rosnode.get_node_names", return_value=live):
            status = manager.status()

        self.assertTrue(status["qr"]["running"])
        self.assertTrue(status["qr"]["owned"])
        self.assertFalse(status["pose"]["running"])
        self.assertFalse(status["pose"]["owned"])
        self.assertTrue(status["pose"]["shadowed_by_owned_launch"])

    def test_system_manager_switches_only_one_control_path_while_stopped(self):
        manager = SystemLaunchManager(self.config, execution_enabled=False)
        with patch("rosnode.get_node_names", return_value=[]):
            manager.set_control_path("joint_control")
            self.assertTrue(manager._joint_control_execution_authorized)
            self.assertFalse(manager._cartesian_jog_execution_authorized)
            self.assertFalse(manager._trajectory_execution_authorized)
            self.assertFalse(manager._touch_z_force_authorized)
            self.assertFalse(manager._tool5_drill_authorized)
            manager.set_control_path("tool5_drill")
            self.assertFalse(manager._joint_control_execution_authorized)
            self.assertFalse(manager._cartesian_jog_execution_authorized)
            self.assertFalse(manager._trajectory_execution_authorized)
            self.assertFalse(manager._touch_z_force_authorized)
            self.assertTrue(manager._tool5_drill_authorized)

    def test_joint_base_uses_high_follow_without_changing_legacy_mode(self):
        launch = (ROOT / "launch/base_real.launch").read_text()
        self.assertIn(
            "default=\"$(eval arg('joint_control_execution_authorized') or "
            "arg('cartesian_jog_execution_authorized'))\"/>",
            launch,
        )
        self.assertIn('<arg name="Follow" value="$(arg driver_follow)"/>', launch)
        self.assertIn('<arg name="trajectory_mode" value="0"/>', launch)
        self.assertIn('<arg name="radio" value="0"/>', launch)
        self.assertIn(
            '<arg name="allow_trajectory_execution" value="$(arg moveit_execution_enabled)"/>',
            launch,
        )
        self.assertIn(
            'file="$(find rm_75_moveit_config)/launch/fake_moveit_controller_manager.launch.xml"',
            launch,
        )

    def test_real_base_maps_moveit_to_rm_group_bridge(self):
        launch = (ROOT / "launch/base_real.launch").read_text()
        self.assertIn(
            '<arg name="real_moveit_controller_manager" value="rm_75_real"/>',
            launch,
        )
        manager = (
            ROOT.parent
            / "rm_robot/rm_moveit_config/rm_75_moveit_config/launch"
            / "rm_75_real_moveit_controller_manager.launch.xml"
        ).read_text()
        self.assertIn('config/controllers.yaml', manager)
        controllers = yaml.safe_load((
            ROOT.parent
            / "rm_robot/rm_moveit_config/rm_75_moveit_config/config/controllers.yaml"
        ).read_text())
        controller = controllers["controller_list"][0]
        self.assertEqual(controller["name"], "rm_group")
        self.assertEqual(controller["action_ns"], "follow_joint_trajectory")
        self.assertIn('<param name="hand_eye_calibrated" value="true"/>', launch)
        self.assertIn('<param name="tool_verified" value="true"/>', launch)
        self.assertIn('<param name="stop_verified" value="true"/>', launch)
        self.assertIn(
            '<param name="require_gripper_feedback" value="false"/>', launch
        )

    def test_base_readiness_waits_for_moveit_collision_service(self):
        definition = self.config["system_launches"]["base"]
        self.assertEqual(definition["expected_services"], ["/check_state_validity"])
        with patch("rospy.wait_for_service") as wait:
            SystemLaunchManager._wait_for_services(
                definition, time.monotonic() + 1.0
            )
        wait.assert_called_once()
        self.assertEqual(wait.call_args.args[0], "/check_state_validity")
        self.assertGreater(wait.call_args.kwargs["timeout"], 0.0)

    def test_usb_recovery_waits_for_delayed_enumeration(self):
        definition = {
            "usb_recovery": {
                "device_name": "test camera",
                "vendor_id": "8086",
                "product_id": "0b3a",
                "enumerate_timeout_s": 1.0,
                "enumerate_poll_s": 0.05,
                "off_s": 0.0,
                "settle_s": 0.0,
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            device = Path(directory) / "2-1"
            (device / "power").mkdir(parents=True)
            (device / "idVendor").write_text("8086\n")
            (device / "idProduct").write_text("0b3a\n")
            (device / "authorized").write_text("1")
            (device / "power/control").write_text("auto")
            with patch.object(Path, "glob", side_effect=[[], [], [device]]), \
                    patch("time.sleep"):
                SystemLaunchManager._prepare_usb(definition)
            self.assertEqual((device / "authorized").read_text(), "1")
            self.assertEqual((device / "power/control").read_text(), "on")

    def test_usb_recovery_missing_device_has_actionable_hex_error(self):
        definition = {
            "usb_recovery": {
                "device_name": "Intel RealSense D435i",
                "vendor_id": "8086",
                "product_id": "0b3a",
                "enumerate_timeout_s": 0.0,
            }
        }
        with patch.object(Path, "glob", return_value=[]):
            with self.assertRaisesRegex(
                    SystemLaunchError,
                    r"未检测到Intel RealSense D435i 8086:0b3a.*lsusb"):
                SystemLaunchManager._prepare_usb(definition)

    def test_qr_detection_start_does_not_require_trajectory_bridge(self):
        definition = self.config["system_launches"]["qr"]
        self.assertEqual(definition["requires"], ["/rm_driver", "/move_group"])

    def test_joint_base_expected_nodes_do_not_depend_on_supervisor_gate(self):
        self.config["joint_control_execution_authorized"] = True
        self.config["supervised_commissioning_authorized"] = False
        manager = SystemLaunchManager(self.config, execution_enabled=False)
        expected = manager._expected_nodes(
            self.config["system_launches"]["base"])
        self.assertIn("/joint_control_server", expected)
        self.assertNotIn("/trajectory_bridge", expected)

    def test_z_force_base_has_no_competing_joint_command_source(self):
        self.config["touch_z_force_authorized"] = True
        self.config["joint_control_execution_authorized"] = False
        manager = SystemLaunchManager(self.config, execution_enabled=True)
        expected = manager._expected_nodes(
            self.config["system_launches"]["base"])
        self.assertIn("/rm_driver", expected)
        self.assertNotIn("/trajectory_bridge", expected)
        self.assertNotIn("/joint_control_server", expected)

    def test_z_force_profile_locks_xy_and_orientation(self):
        profile = yaml.safe_load((
            ROOT.parent / "rm75_touch_teleop/config/rm75_touch_z_force_haptic_real.yaml"
        ).read_text())
        self.assertTrue(profile["hybrid_control_enabled"])
        self.assertFalse(profile["translation_x_enabled"])
        self.assertFalse(profile["translation_y_enabled"])
        self.assertTrue(profile["translation_z_enabled"])
        self.assertFalse(profile["orientation_enabled"])
        self.assertLessEqual(profile["haptic_max_force_norm"], 0.5)

    def test_status_degrees_limit_margin_tf_and_tool5(self):
        widget = self.widget(StatusWidget(self.config))
        widget.update_snapshot(sample())
        self.assertAlmostEqual(float(widget.joints.item(0, 1).text()), math.degrees(0.1), places=3)
        self.assertAlmostEqual(float(widget.joints.item(0, 2).text()), math.degrees(1.9), places=3)
        self.assertIn("0.300", widget.tcp.text())
        self.assertIn("Tool5", widget.tool.text())
        data = sample()
        data["fresh"] = {}
        widget.update_snapshot(data)
        self.assertEqual(widget.joints.item(0, 1).text(), "—")

    def test_force_simulated_values_error_and_bounded_plot(self):
        self.config["monitoring"]["plot_samples"] = 20
        widget = self.widget(ForceMonitorWidget(self.config))
        for i in range(50):
            widget.update_snapshot(sample())
        self.assertEqual(len(widget.plot.samples), 20)
        self.assertEqual(widget.plot.samples[-1][1], (1.0, 2.0, -2.5))
        self.assertIn("误差：-2.50", widget.values.text())
        widget.update_snapshot({"timestamp": time.time(), "fresh": {}})
        self.assertEqual(widget.plot.samples[-1][1], (None, None, None))
        widget.resize(800, 400)
        self.assertFalse(widget.grab().isNull())

    def test_force_falls_back_to_raw_fz_without_force_controller(self):
        widget = self.widget(ForceMonitorWidget(self.config))
        data = sample()
        data["fresh"]["filtered_force_z"] = False
        data["fresh"]["force_z"] = False
        data["fresh"]["force_setpoint"] = False
        widget.update_snapshot(data)
        self.assertEqual(widget.plot.samples[-1][1], (1.0, 2.0, 3.0))
        self.assertIn("当前Fz：3.00", widget.values.text())

    def test_mark_pose_and_rgb_image(self):
        widget = self.widget(VisionWidget(self.config))
        msg = Image(width=2, height=1, encoding="rgb8", step=6, data=bytes([255, 0, 0, 0, 255, 0]))
        data = {"fresh": {"image": True, "vision_status": True, "marker_4": True},
                "sequences": {"image": 1}, "image_message": msg,
                "image_topic": "/camera/color/image_raw",
                "vision_status": {"detections": [{"slot_id": 4, "stable": True}]},
                "marker_4": {"translation": [1, 2, 3], "quaternion_xyzw": [0, 0, 0, 1],
                             "frame_id": "base_link"}}
        widget.update_snapshot(data)
        self.assertFalse(widget.image.pixmap().isNull())
        self.assertEqual(widget.table.item(4, 2).text(), "稳定")
        self.assertIn("base_link", widget.table.item(4, 3).text())
        data["fresh"] = {}
        widget.update_snapshot(data)
        self.assertIn("过期", widget.image.text())

    def test_diagnostic_disconnect_and_error_categories(self):
        widget = self.widget(DiagnosticWidget())
        widget.update_snapshot(sample())
        self.assertNotIn("ERROR", widget.summary.text())
        widget.update_snapshot({})
        self.assertIn("通信: ERROR", widget.summary.text())
        count = len(widget.history)
        widget.update_snapshot({})
        self.assertEqual(len(widget.history), count)
        widget.add_error("电压 ACK失败")
        widget.add_error("SAFE_STOP 已锁存")
        self.assertIn("[ACK失败]", widget.errors.toPlainText())
        self.assertIn("[SAFE_STOP]", widget.errors.toPlainText())

    def test_parameters_are_yaml_values_and_missing_is_explicit(self):
        widget = self.widget(ParameterWidget(self.config))
        self.assertEqual(widget.table.item(0, 1).text(), "0.8")
        self.assertEqual(widget.table.item(1, 1).text(), "-50.0")
        self.assertEqual(widget.table.item(4, 1).text(), "未配置")

    def test_active_window_close_guard_and_offline_estop(self):
        ros = ROSInterface(self.config)
        state = StateMachine()
        launches = Mock()
        launches.mode = "teleop"
        tasks = TaskManager(state, ros, self.config["tools"], self.config,
                            launch_manager=launches)
        window = MainWindow(self.config, state, ros, tasks)
        state.transition(RobotState.TELEOP)
        event = QCloseEvent()
        with patch.object(QMessageBox, "warning"):
            window.closeEvent(event)
        self.assertFalse(event.isAccepted())
        with patch.object(ros, "emergency_stop") as stop:
            window._submit("急停", tasks.emergency_stop, emergency=True)
            self.assertEqual(state.state, RobotState.SAFE_STOP)
            stop.assert_not_called()
        with self.assertRaises(StateTransitionError):
            state.transition(RobotState.IDLE)
        launches.mode = None
        with patch.object(QMessageBox, "question", return_value=QMessageBox.Yes):
            window.close()
        window.deleteLater()

    def test_slow_estop_ack_cannot_delay_state_latch(self):
        state = StateMachine(RobotState.TELEOP)
        ros = Mock()
        def check_latch():
            self.assertEqual(state.state, RobotState.SAFE_STOP)
            raise RuntimeError("ACK timeout")
        ros.emergency_stop.side_effect = check_latch
        tasks = TaskManager(state, ros, self.config["tools"], self.config, launch_manager=Mock())
        with self.assertRaises(RuntimeError):
            tasks.emergency_stop()
        self.assertEqual(state.state, RobotState.SAFE_STOP)


class RecordingTest(unittest.TestCase):
    def test_task_lifecycle_csv_yaml_ack_and_unique_directories(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = ExperimentLogger(directory, {"monitoring": {"wrench_topic": "/test"}})
            try:
                recorder.transition(StateTransition(RobotState.IDLE, RobotState.TELEOP, "start"))
                recorder.sample(sample())
                record = logging.LogRecord("rm75_sampling_gui", logging.INFO, "", 0, "ACK accepted=True", (), None)
                recorder.emit(record)
                recorder.transition(StateTransition(RobotState.TELEOP, RobotState.TOOL_DETECT, "detect"))
                recorder.sample({"timestamp": time.time(), "fresh": {}})
                recorder.transition(StateTransition(RobotState.TOOL_RETURN, RobotState.IDLE, "done"))
                recorder.flush_pending()
                first = recorder.directory
                self.assertFalse(recorder.active)
                self.assertEqual({p.name for p in first.iterdir()},
                                 {"robot_state.csv", "force.csv", "task.log", "config.yaml"})
                with (first / "robot_state.csv").open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(rows[0]["joint7"], "0.7000000000000001")
                self.assertEqual(rows[1]["joint1"], "")
                with (first / "force.csv").open() as stream:
                    force = list(csv.DictReader(stream))
                self.assertEqual(force[0]["Fz"], "3.0")
                self.assertEqual(force[0]["filtered_force_z"], "-2.5")
                self.assertIn("ACK accepted=True", (first / "task.log").read_text())
                recorder.transition(StateTransition(RobotState.IDLE, RobotState.TOOL_DETECT, "next"))
                recorder.transition(StateTransition(RobotState.TOOL_DETECT, RobotState.SAFE_STOP, "timeout"))
                recorder.transition(StateTransition(RobotState.SAFE_STOP, RobotState.SAFE_STOP, "late ACK"))
                recorder.flush_pending()
                self.assertEqual(len(list(Path(directory).iterdir())), 2)
                self.assertFalse(recorder.active)
            finally:
                recorder.close()

    def test_disk_failure_exposed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "not_a_directory"
            path.write_text("fixture")
            recorder = ExperimentLogger(str(path), {})
            recorder.transition(StateTransition(RobotState.IDLE, RobotState.TELEOP, "start"))
            recorder.flush_pending()
            self.assertIn("记录失败", recorder.error)
            recorder.close()


class TelemetryTest(unittest.TestCase):
    def test_touch_launch_uses_owned_subprocess_from_worker(self):
        manager = ControlLaunchManager(
            {"teleop_config": "config/rm75_touch_real_6f.yaml"}, False)
        with patch("rosnode.get_node_names", return_value=["/rm_driver", "/robot_state_publisher"]), \
                patch("rm75_sampling_gui.task.task_manager.subprocess.Popen") as spawn, \
                patch("rm75_sampling_gui.task.task_manager.os.killpg") as stop:
            parent = spawn.return_value
            parent.pid = 12345
            parent.poll.return_value = None
            manager.start("teleop")
            arguments = spawn.call_args[0][0]
            self.assertEqual(arguments[0], "roslaunch")
            self.assertIn("motion_enabled:=false", arguments)
            self.assertTrue(spawn.call_args[1]["start_new_session"])
            manager.stop()
            stop.assert_called_once()
            parent.wait.assert_called_once()
            self.assertIsNone(manager.mode)

    def test_z_force_launch_enables_haptics_with_dedicated_profile(self):
        manager = ControlLaunchManager({
            "z_force_teleop_config":
                "config/rm75_touch_z_force_haptic_real.yaml"
        }, True)
        with patch("rosnode.get_node_names", return_value=[
                "/rm_driver", "/robot_state_publisher"]), \
                patch("rm75_sampling_gui.task.task_manager.subprocess.Popen") as spawn:
            spawn.return_value.poll.return_value = None
            manager.start("z_force_teleop")
            arguments = spawn.call_args[0][0]
            self.assertIn("motion_enabled:=true", arguments)
            self.assertIn("haptic_feedback:=true", arguments)
            self.assertTrue(any(
                "rm75_touch_z_force_haptic_real.yaml" in value
                for value in arguments))

    def test_tool5_drill_launch_reuses_single_vertical_press_source_without_touch(self):
        manager = ControlLaunchManager({
            "force_control_config": "config/rm75_vertical_press_real.yaml",
            "tool5_drill": {"max_feed_travel_m": 0.05,
                            "desired_force_n": -10.0,
                            "max_z_velocity_m_s": 0.005},
        }, True)
        with patch("rosnode.get_node_names", return_value=[
                "/rm_driver", "/robot_state_publisher"]), \
                patch("rm75_sampling_gui.task.task_manager.subprocess.Popen") as spawn:
            spawn.return_value.poll.return_value = None
            manager.start("tool5_drill")
            arguments = spawn.call_args[0][0]
            self.assertIn("tool5_drill_enabled:=true", arguments)
            self.assertIn("force_limit:=60.0", arguments)
            self.assertFalse(any(value.startswith("desired_force:=")
                                 for value in arguments))
            self.assertIn("start_touch_driver:=false", arguments)
            self.assertIn("start_touch_helpers:=false", arguments)
            self.assertNotIn("/rm_driver/JointPos", arguments)

    def test_urdf_limits_and_ordered_joints(self):
        monitor = Monitor({})
        monitor.parse_limits('<robot><joint name="joint1"><limit lower="-1.5" upper="2.0"/></joint></robot>')
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(10)
        msg.name = ["joint{}".format(i) for i in range(7, 0, -1)]
        msg.position = [float(i)/10 for i in range(7, 0, -1)]
        with patch("rospy.Time.now", return_value=rospy.Time.from_sec(10.1)):
            monitor.joints(msg)
        self.assertEqual(monitor.snapshot()["joints"]["joint1"], 0.1)
        self.assertEqual(monitor.snapshot()["joint_limits"]["joint1"], (-1.5, 2.0))
        with patch("rospy.Time.now", return_value=rospy.Time.from_sec(100)):
            monitor.joints(msg)
        self.assertEqual(monitor.sequences["joints"], 1)

    def test_force_nan_and_stale_sensor_not_masked_by_target(self):
        ros = ROSInterface({})
        ros._force_callback("force_z", Float64(data=float("nan")))
        self.assertNotIn("force_z", ros.monitor.sequences)
        ros._force_callback("force_z", Float64(data=-3.0))
        ros.monitor.received["force_z"] = time.monotonic() - 10
        ros._force_callback("force_setpoint", Float64(data=-5.0))
        self.assertFalse(ros.telemetry_snapshot()["fresh"]["force_z"])
        self.assertTrue(ros.telemetry_snapshot()["fresh"]["force_setpoint"])
        msg = Six_Force(force_Fx=1, force_Fy=2, force_Fz=3, force_Mx=4, force_My=5, force_Mz=6)
        ros.monitor.wrench(msg)
        self.assertEqual(ros.telemetry_snapshot()["wrench"]["Tz"], 6)

    def test_no_joint_command_publisher(self):
        ros = ROSInterface({})
        with patch("rospy.Publisher") as pub, patch("rospy.Subscriber"):
            ros._bind_endpoints()
        names = [call.args[0] for call in pub.call_args_list]
        self.assertEqual(set(names), {"/rm_driver/SetToolVoltage", "/rm_driver/Emergency_Stop",
                                      "/rm_driver/Clear_System_Err",
                                      "/rm75_sampling_gui/joint_control_heartbeat",
                                      "/rm75_sampling_gui/cartesian_jog_heartbeat",
                                      "/rm75_vertical_press/drill_start_depth_m",
                                      "/rm75_vertical_press/drill_stop_feed"})
        self.assertNotIn("/rm_driver/JointPos", names)

    def test_tool5_motor_gate_waits_for_fresh_drilling_state(self):
        ros = ROSInterface({"ack_timeout_s": 0.1})
        ros._connected = True

        class DrillPublisher:
            @staticmethod
            def get_num_connections():
                return 1

            @staticmethod
            def publish(_message):
                ros._tool5_drill_state_callback(String(data="DRILLING"))

        ros._tool5_drill_start_pub = DrillPublisher()
        ros.start_tool5_drill(0.01)
        self.assertEqual(ros.telemetry_snapshot()["tool5_drill_state"],
                         "DRILLING")

    def test_tool5_motor_gate_rejects_bottom_safe_stop(self):
        ros = ROSInterface({"ack_timeout_s": 0.1})
        ros._connected = True
        ros._set_telemetry("stop_reason", "injected drill rejection")

        class DrillPublisher:
            @staticmethod
            def get_num_connections():
                return 1

            @staticmethod
            def publish(_message):
                ros._tool5_drill_state_callback(String(data="SAFE_STOP"))

        ros._tool5_drill_start_pub = DrillPublisher()
        with self.assertRaisesRegex(ROSInterfaceError,
                                    "injected drill rejection"):
            ros.start_tool5_drill(0.01)


if __name__ == "__main__":
    unittest.main()
