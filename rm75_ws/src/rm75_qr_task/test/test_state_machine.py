#!/usr/bin/env python3
import os
import tempfile
import unittest
import numpy as np

from rm75_qr_task.adapters import (MockHoldFeedback, MockMotionAdapter,
                                   MockVoltageAdapter)
from rm75_qr_task.marker_manager import MarkerManager
from rm75_qr_task.safety_checker import REQUIRED_VERIFICATIONS, SafetyChecker
from rm75_qr_task.task_state_machine import TaskError, TaskStateMachine
from rm75_qr_task.task_storage import TaskStorage


def transform_config(x=0, y=0, z=0):
    return {'translation': [x, y, z], 'quaternion_xyzw': [0, 0, 0, 1],
            'verified': True}


class TaskStateMachineTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.slot = {
            'id': 0, 'enabled': True, 'calibrated': True, 'size_m': 0.08,
            'marker_to_object': transform_config(0.04, 0, 0),
            'marker_to_pick_tcp': transform_config(0.04, 0, 0.1),
            'approach_direction_marker': [0, 0, 1],
            'approach_distance_m': 0.08, 'retreat_distance_m': 0.1,
            'station_fixed_verified': True, 'allow_relocalization': False,
            'collision_size_m': [0.04, 0.04, 0.04],
        }
        self.slots = {0: self.slot, 1: dict(self.slot, id=1),
                      2: dict(self.slot, id=2), 3: dict(self.slot, id=3)}
        self.now = 100.0
        self.markers = MarkerManager(self.slots, stability_samples=1,
                                     clock=lambda: self.now)
        for slot_id in range(4):
            marker = np.eye(4)
            marker[0, 3] = slot_id * 0.2
            marker[2, 3] = 0.5
            self.markers.add(slot_id, self.now, marker, 0.1)
        self.motion = MockMotionAdapter()
        self.voltage = MockVoltageAdapter()
        self.feedback = MockHoldFeedback(True, True)
        self.storage = TaskStorage(os.path.join(self.directory.name, 'state.json'))
        self.safety = SafetyChecker(True, True,
                                    {name: True for name in REQUIRED_VERIFICATIONS})
        self.config = {'allow_supervised_recovery': True}
        self.machine = TaskStateMachine(
            self.slots, self.markers, self.motion, self.voltage, self.feedback,
            self.storage, self.safety, self.config)

    def tearDown(self):
        self.directory.cleanup()

    def test_preview_is_non_mutating_and_sends_no_voltage(self):
        result = self.machine.pick(0, execute=False)
        self.assertEqual(self.machine.status()['state'], 'IDLE')
        self.assertEqual(self.voltage.commands, [])
        self.assertFalse(result['plan']['executed'])

    def test_complete_pick_and_return_restores_each_command_start(self):
        pick = self.machine.pick(2, execute=True)
        self.assertEqual(pick['state'], 'HOLDING')
        self.assertEqual(pick['held_slot_id'], 2)
        self.assertEqual(self.voltage.commands, [24])
        result = self.machine.return_object(2, execute=True)
        self.assertEqual(result['state'], 'IDLE')
        self.assertEqual(self.voltage.commands, [24, 0])
        actions = [name for name, _ in self.motion.actions]
        self.assertIn('attach', actions)
        self.assertIn('detach', actions)
        self.assertIn('move_pick_start', actions)
        self.assertIn('move_return_start', actions)

    def test_wrong_return_slot_and_second_pick_are_rejected(self):
        self.machine.pick(2, execute=True)
        with self.assertRaises(TaskError):
            self.machine.return_object(1, execute=False)
        with self.assertRaises(TaskError):
            self.machine.pick(0, execute=False)

    def test_hold_failure_preserves_power_and_requires_recovery(self):
        self.feedback.holding_value = False
        with self.assertRaises(TaskError):
            self.machine.pick(0, execute=True)
        status = self.machine.status()
        self.assertEqual(status['state'], 'RECOVERY_REQUIRED')
        self.assertEqual(status['power_commanded_voltage'], 24)
        self.assertEqual(self.voltage.commands, [24])

    def test_release_failure_does_not_retreat_or_hide_power_state(self):
        self.machine.pick(0, execute=True)
        self.feedback.released_value = False
        with self.assertRaises(TaskError):
            self.machine.return_object(0, execute=True)
        status = self.machine.status()
        self.assertEqual(status['state'], 'RECOVERY_REQUIRED')
        self.assertEqual(status['power_commanded_voltage'], 0)
        self.assertNotIn('move_retreat', [name for name, _ in self.motion.actions])

    def test_fixed_station_motion_requires_revalidation(self):
        original = np.eye(4)
        original[2, 3] = 0.5
        self.machine.record_station_pose(0, self.now, original, 0.1)
        moved = original.copy()
        moved[0, 3] += 0.05
        self.machine.record_station_pose(0, self.now + 1, moved, 0.1)
        with self.assertRaises(TaskError):
            self.machine.preview_pick(0)

    def test_planning_failure_never_touches_voltage(self):
        self.motion.fail_action = 'preview_sequence'
        with self.assertRaises(TaskError):
            self.machine.pick(0, execute=True)
        self.assertEqual(self.machine.status()['state'], 'FAULT')
        self.assertEqual(self.voltage.commands, [])

    def test_restart_from_holding_demands_confirmation(self):
        self.machine.pick(0, execute=True)
        restarted = TaskStateMachine(
            self.slots, self.markers, self.motion, self.voltage, self.feedback,
            self.storage, self.safety, self.config)
        self.assertEqual(restarted.status()['state'], 'RECOVERY_REQUIRED')
        recovered = restarted.recover('holding', 0, supervised_confirm=True)
        self.assertEqual(recovered['state'], 'HOLDING')

    def test_execution_authorization_is_separate(self):
        locked = SafetyChecker(False, True,
                               {name: True for name in REQUIRED_VERIFICATIONS})
        machine = TaskStateMachine(
            self.slots, self.markers, self.motion, self.voltage, self.feedback,
            TaskStorage(os.path.join(self.directory.name, 'locked.json')),
            locked, self.config)
        with self.assertRaises(TaskError):
            machine.pick(0, execute=True)
        self.assertEqual(self.voltage.commands, [])


if __name__ == '__main__':
    unittest.main()
