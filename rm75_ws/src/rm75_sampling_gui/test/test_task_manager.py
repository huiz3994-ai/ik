#!/usr/bin/env python3
"""TaskManager tests with no ROS graph and no robot commands."""

import unittest
from unittest.mock import Mock, patch

from rm75_sampling_gui.state.state_machine import RobotState, StateMachine
from rm75_sampling_gui.task.task_manager import TaskManager, TaskManagerError


class FakeLaunchManager:
    def __init__(self):
        self.mode = None
        self.fail_start = False

    def start(self, mode):
        if self.fail_start:
            raise RuntimeError("injected launch failure")
        self.mode = mode

    def stop(self):
        self.mode = None

    def set_execution_enabled(self, enabled):
        self.execution_enabled = bool(enabled)


class FakeSystemManager:
    def __init__(self):
        self.control_path = "observe"

    def status(self):
        return {"base": {"running": True}}

    def set_control_path(self, path):
        self.control_path = path

    def shutdown(self):
        return None


class FakeROS:
    def __init__(self):
        self.active = False
        self.waited_nodes = []
        self.tool_voltage = None
        self.emergency_calls = 0
        self.fail_pick = False
        self.five_target_calls = []
        self.qr_calls = []
        self.qr_online = False
        self.holding_slot = None
        self.drill_depth = None
        self.drill_hold_calls = 0

    def wait_for_node(self, _name, _timeout):
        self.waited_nodes.append(_name)
        return None

    def wait_for_force_feedback(self, _timeout):
        return None

    def detect_tool(self, tools, preferred, timeout):
        del timeout
        selected = preferred or tools[0]["name"]
        return next(dict(tool) for tool in tools if tool["name"] == selected)

    def call_qr_task(
        self, command, slot_id, execute, supervised_confirm=False
    ):
        del execute, supervised_confirm
        self.qr_calls.append((command, slot_id))
        if command == "pick":
            if self.fail_pick:
                raise RuntimeError("injected pick failure")
            return {"state": "HOLDING", "slot_id": slot_id}
        if command == "return":
            return {"state": "IDLE", "slot_id": slot_id}
        if command == "preview":
            return {"state": "IDLE", "slot_id": slot_id}
        if command == "stop":
            return {"state": "SAFE_STOP"}
        if command == "status":
            return {"state": "IDLE"}
        return {"state": "IDLE"}

    def set_tool_voltage(self, voltage):
        self.tool_voltage = voltage
        return {"ack": True, "voltage": voltage}

    def call_five_target_task(self, command, slot_id=0, execute=False,
                              supervised_confirm=False):
        del supervised_confirm
        self.five_target_calls.append((command, slot_id, execute))
        if command == "status":
            return {"command": command, "holding_slot": self.holding_slot}
        result = {"command": command, "slot_id": slot_id, "executed": execute}
        if command == "execute_selected" and execute:
            result["holding_slot"] = slot_id
        if command == "move_camera_observation" and execute:
            result["voltage_v"] = 0
        if command == "return_held_tool" and execute:
            result["voltage_v"] = 0
            result["holding_slot"] = None
        if command == "confirm_held_tool":
            result["voltage_v"] = 24
            result["holding_slot"] = slot_id
        return result

    def telemetry_snapshot(self):
        return {
            "teleop_active": self.active,
            "control_mode": "CONTACT" if self.active else "APPROACH",
            "tool_voltage": self.tool_voltage,
            "health": {"qr": self.qr_online},
        }

    def emergency_stop(self):
        self.emergency_calls += 1
        return {"ack": True}

    def clear_system_error(self):
        return {"ack": True}

    def start_tool5_drill(self, depth_m):
        self.drill_depth = depth_m

    def stop_tool5_drill_feed(self):
        self.drill_hold_calls += 1


class TaskManagerTest(unittest.TestCase):
    def setUp(self):
        self.node_names = patch("rosnode.get_node_names", return_value=[])
        self.node_names.start()
        self.addCleanup(self.node_names.stop)
        self.machine = StateMachine()
        self.ros = FakeROS()
        self.launches = FakeLaunchManager()
        self.manager = TaskManager(
            self.machine,
            self.ros,
            [{"name": "Tool{}".format(i), "slot_id": i - 1} for i in range(1, 6)],
            {
                "node_start_timeout_s": 1.0,
                "marker_timeout_s": 1.0,
                "feedback_timeout_s": 1.0,
            },
            execution_enabled=True,
            launch_manager=self.launches,
            system_manager=FakeSystemManager(),
        )

    def test_complete_real_workflow(self):
        self.manager.start_teleop()
        self.assertEqual(self.machine.state, RobotState.TELEOP)
        self.manager.detect_tool()
        self.assertEqual(self.machine.state, RobotState.TOOL_DETECT)
        self.manager.pick_tool()
        self.assertEqual(self.machine.state, RobotState.TOOL_HOLD)
        self.manager.set_voltage(24)
        self.manager.enter_force_control()
        self.assertEqual(self.machine.state, RobotState.FORCE_CONTROL)
        self.ros.active = True
        self.manager.start_sampling()
        self.ros.active = False
        self.manager.stop_sampling()
        self.assertEqual(self.machine.state, RobotState.IDLE)
        self.manager.return_tool()
        self.assertEqual(self.machine.state, RobotState.IDLE)

    def test_control_path_switch_updates_mutually_exclusive_authorization(self):
        result = self.manager.set_control_path("qr_pickup")
        self.assertEqual(result["control_path"], "qr_pickup")
        self.assertTrue(self.manager.config["execution_enabled"])
        self.assertTrue(self.manager.config["qr_execution_authorized"])
        self.assertTrue(self.manager.config["trajectory_execution_authorized"])
        self.assertFalse(self.manager.config["joint_control_execution_authorized"])
        self.assertFalse(self.manager.config["touch_z_force_authorized"])
        self.assertFalse(self.manager.config["tool5_drill_authorized"])

    def test_pose_pickup_authorizes_trajectory_without_qr(self):
        result = self.manager.set_control_path("pose_pickup")
        self.assertEqual(result["control_path"], "pose_pickup")
        self.assertTrue(self.manager.config["execution_enabled"])
        self.assertTrue(self.manager.config["trajectory_execution_authorized"])
        self.assertFalse(self.manager.config["qr_execution_authorized"])

    def test_force_control_can_start_without_tool_workflow_or_voltage(self):
        result = self.manager.enter_force_control()
        self.assertEqual(result["mode"], "force_control")
        self.assertEqual(self.machine.state, RobotState.FORCE_CONTROL)
        self.assertIn("/rm75_vertical_press", self.ros.waited_nodes)

    def test_tool5_drill_uses_dedicated_vertical_source_and_hold_command(self):
        self.manager.config["tool5_drill_authorized"] = True
        prepared = self.manager.prepare_tool5_drill()
        self.assertEqual(prepared["mode"], "tool5_drill")
        self.assertEqual(self.launches.mode, "tool5_drill")
        self.manager.engage_tool5_drill(0.03)
        self.assertEqual(self.ros.drill_depth, 0.03)
        held = self.manager.hold_tool5_drill()
        self.assertTrue(held["holding"])
        self.assertEqual(self.ros.drill_hold_calls, 1)

    def test_z_force_teleop_is_z_only_haptic_and_waits_for_force(self):
        self.manager.config["touch_z_force_authorized"] = True
        result = self.manager.start_z_force_teleop()
        self.assertEqual(self.launches.mode, "z_force_teleop")
        self.assertEqual(self.machine.state, RobotState.FORCE_CONTROL)
        self.assertEqual(result["translation_axes"], ["z"])
        self.assertTrue(result["orientation_locked"])
        self.assertTrue(result["haptic_feedback"])
        self.assertIn("/rm75_touch_teleop", self.ros.waited_nodes)
        stopped = self.manager.stop_z_force_teleop()
        self.assertTrue(stopped["stopped"])
        self.assertEqual(self.machine.state, RobotState.IDLE)

    def test_z_force_teleop_requires_dedicated_authorization(self):
        with self.assertRaises(TaskManagerError):
            self.manager.start_z_force_teleop()
        self.assertIsNone(self.launches.mode)

    def test_voltage_is_not_gated_by_tool_workflow_state(self):
        self.machine.transition(RobotState.TELEOP)
        self.assertTrue(self.manager.set_voltage(24)["ack"])
        self.assertTrue(self.manager.set_voltage(0)["ack"])

    def test_pick_exception_enters_safe_stop(self):
        self.manager.detect_tool()
        self.ros.fail_pick = True
        with self.assertRaises(RuntimeError):
            self.manager.pick_tool()
        self.assertEqual(self.machine.state, RobotState.SAFE_STOP)
        self.assertEqual(self.ros.emergency_calls, 1)

    def test_teleop_launch_exception_enters_safe_stop(self):
        self.launches.fail_start = True
        with self.assertRaises(RuntimeError):
            self.manager.start_teleop()
        self.assertEqual(self.machine.state, RobotState.SAFE_STOP)
        self.assertEqual(self.ros.emergency_calls, 1)

    def test_touch_cannot_preempt_active_joint_control(self):
        self.manager.joint_adapter = Mock(active=True)
        with self.assertRaises(TaskManagerError):
            self.manager.start_teleop()
        self.assertIsNone(self.launches.mode)
        self.assertEqual(self.ros.emergency_calls, 0)

    def test_active_sampling_stop_requires_touch_release(self):
        self.manager.detect_tool()
        self.manager.pick_tool()
        self.manager.set_voltage(24)
        self.manager.enter_force_control()
        self.ros.active = True
        with self.assertRaises(TaskManagerError):
            self.manager.stop_sampling()
        self.assertEqual(self.machine.state, RobotState.FORCE_CONTROL)
        self.assertEqual(self.ros.emergency_calls, 0)

    def test_active_z_force_stop_requires_touch_release(self):
        self.manager.config["touch_z_force_authorized"] = True
        self.manager.start_z_force_teleop()
        self.ros.active = True
        with self.assertRaises(TaskManagerError):
            self.manager.stop_z_force_teleop()
        self.assertEqual(self.launches.mode, "z_force_teleop")
        self.assertEqual(self.ros.emergency_calls, 0)

    def test_disarmed_voltage_is_rejected_without_dispatch(self):
        manager = TaskManager(
            StateMachine(RobotState.TOOL_HOLD),
            self.ros,
            [{"name": "Tool1", "slot_id": 0}],
            {},
            execution_enabled=False,
            launch_manager=self.launches,
            system_manager=FakeSystemManager(),
        )
        with self.assertRaises(TaskManagerError):
            manager.set_voltage(24)
        self.assertIsNone(self.ros.tool_voltage)

    def test_disarmed_force_control_is_rejected(self):
        manager = TaskManager(
            StateMachine(),
            self.ros,
            [{"name": "Tool1", "slot_id": 0}],
            {"node_start_timeout_s": 1.0, "feedback_timeout_s": 1.0},
            execution_enabled=False,
            launch_manager=self.launches,
            system_manager=FakeSystemManager(),
        )
        with self.assertRaises(TaskManagerError):
            manager.enter_force_control()
        self.assertIsNone(self.launches.mode)

    def test_recover_without_qr_clears_driver_and_returns_idle(self):
        self.machine.fail(RuntimeError("tracking"), "teleop")
        result = self.manager.recover()
        self.assertEqual(self.machine.state, RobotState.IDLE)
        self.assertTrue(result["task"]["skipped"])

    def test_emergency_stop_skips_qr_command_when_five_target_is_offline(self):
        result = self.manager.emergency_stop()

        self.assertTrue(result["ack"])
        self.assertEqual(self.machine.state, RobotState.SAFE_STOP)
        self.assertEqual(self.ros.five_target_calls, [])
        self.assertEqual(self.ros.qr_calls, [])

    def test_emergency_stop_uses_live_five_target_service(self):
        self.ros.qr_online = True

        result = self.manager.emergency_stop()

        self.assertTrue(result["ack"])
        self.assertEqual(self.machine.state, RobotState.SAFE_STOP)
        self.assertEqual(self.ros.five_target_calls[-1], ("stop", 0, False))
        self.assertEqual(self.ros.qr_calls, [])

    def test_idle_recovery_clears_stale_driver_stop_without_motion(self):
        result = self.manager.recover()
        self.assertEqual(self.machine.state, RobotState.IDLE)
        self.assertTrue(result["driver"]["ack"])
        self.assertTrue(result["task"]["skipped"])

    def test_recover_queries_live_five_target_service_not_legacy_qr_service(self):
        self.ros.qr_online = True
        self.machine.fail(RuntimeError("tracking"), "configured pickup")

        result = self.manager.recover()

        self.assertEqual(self.machine.state, RobotState.IDLE)
        self.assertEqual(result["task"]["holding_slot"], None)
        self.assertEqual(self.ros.five_target_calls[-1], ("status", 0, False))
        self.assertEqual(self.ros.qr_calls, [])

    def test_recover_preserves_five_target_held_tool_state(self):
        manager = TaskManager(
            self.machine,
            self.ros,
            [{"name": "Tool{}".format(i), "slot_id": i} for i in range(1, 6)],
            {"node_start_timeout_s": 1.0, "feedback_timeout_s": 1.0},
            execution_enabled=True,
            launch_manager=self.launches,
            system_manager=FakeSystemManager(),
        )
        self.ros.qr_online = True
        self.ros.holding_slot = 5
        self.machine.fail(RuntimeError("tracking"), "configured pickup")

        result = manager.recover()

        self.assertEqual(self.machine.state, RobotState.TOOL_HOLD)
        self.assertEqual(manager.selected_tool, "Tool5")
        self.assertEqual(result["task"]["holding_slot"], 5)
        self.assertEqual(self.ros.qr_calls, [])

    def test_single_marker_preview_uses_selected_one_based_tool_slot(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool{}".format(i), "slot_id": i, "marker_id": 0}
             for i in range(1, 6)],
            {"qr_execution_authorized": False,
             "trajectory_execution_authorized": False},
            execution_enabled=False, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool5")
        result = manager.preview_selected_target()
        self.assertEqual(result["slot_id"], 5)
        self.assertEqual(self.ros.five_target_calls[-1], ("preview", 5, False))

    def test_single_marker_execute_requires_all_three_authorizations(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool1", "slot_id": 1, "marker_id": 0}],
            {"qr_execution_authorized": True,
             "trajectory_execution_authorized": False},
            execution_enabled=True, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool1")
        with self.assertRaises(TaskManagerError):
            manager.execute_selected_target()
        self.assertEqual(self.ros.five_target_calls, [])

    def test_single_marker_execute_transitions_to_tool_hold(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool3", "slot_id": 3, "marker_id": 0}],
            {"qr_execution_authorized": True,
             "trajectory_execution_authorized": True},
            execution_enabled=True, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool3")
        result = manager.execute_selected_target()
        self.assertEqual(result["holding_slot"], 3)
        self.assertEqual(manager.state_machine.state, RobotState.TOOL_HOLD)
        self.assertEqual(
            self.ros.five_target_calls[-1], ("execute_selected", 3, True))

    def test_move_to_camera_observation_uses_real_execution_gate(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool3", "slot_id": 3, "marker_id": 0}],
            {"qr_execution_authorized": True,
             "trajectory_execution_authorized": True},
            execution_enabled=True, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool3")
        result = manager.move_to_camera_observation()
        self.assertTrue(result["executed"])
        self.assertEqual(
            self.ros.five_target_calls[-1],
            ("move_camera_observation", 3, True))

    def test_return_selected_target_releases_and_returns_idle(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool3", "slot_id": 3, "marker_id": 0}],
            {"qr_execution_authorized": True,
             "trajectory_execution_authorized": True},
            execution_enabled=True, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool3")
        manager.state_machine.transition(RobotState.TOOL_PICK, "test pickup")
        manager.state_machine.transition(RobotState.TOOL_HOLD, "test holding")
        result = manager.return_selected_target()
        self.assertEqual(result["voltage_v"], 0)
        self.assertIsNone(result["holding_slot"])
        self.assertEqual(manager.state_machine.state, RobotState.IDLE)
        self.assertEqual(
            self.ros.five_target_calls[-1],
            ("return_held_tool", 3, True))

    def test_confirm_selected_tool_held_recovers_tool_hold(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool1", "slot_id": 1, "marker_id": 0}],
            {"qr_execution_authorized": True,
             "supervised_commissioning_authorized": True},
            execution_enabled=True, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool1")
        result = manager.confirm_selected_tool_held()
        self.assertEqual(result["holding_slot"], 1)
        self.assertEqual(result["voltage_v"], 24)
        self.assertEqual(manager.state_machine.state, RobotState.TOOL_HOLD)

    def test_physical_validation_confirmation_requires_supervised_gate(self):
        manager = TaskManager(
            StateMachine(), self.ros,
            [{"name": "Tool1", "slot_id": 1, "marker_id": 0}],
            {"supervised_commissioning_authorized": False},
            execution_enabled=False, launch_manager=self.launches,
            system_manager=FakeSystemManager())
        manager.select_tool("Tool1")
        with self.assertRaises(TaskManagerError):
            manager.confirm_five_target_validation()
        manager.config["supervised_commissioning_authorized"] = True
        result = manager.confirm_five_target_validation()
        self.assertEqual(result["command"], "confirm_physical_validation")


if __name__ == "__main__":
    unittest.main()
