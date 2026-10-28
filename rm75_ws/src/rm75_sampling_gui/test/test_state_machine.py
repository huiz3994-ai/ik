#!/usr/bin/env python3
"""State-transition regression tests for the sampling HMI."""

import unittest

from rm75_sampling_gui.state.state_machine import (
    RobotState,
    StateMachine,
    StateTransitionError,
)


class StateMachineTest(unittest.TestCase):
    def test_complete_sampling_path(self):
        machine = StateMachine()
        path = [
            RobotState.TELEOP,
            RobotState.IDLE,
            RobotState.TOOL_DETECT,
            RobotState.TOOL_PICK,
            RobotState.IDLE,
            RobotState.FORCE_CONTROL,
            RobotState.IDLE,
            RobotState.TOOL_RETURN,
            RobotState.IDLE,
        ]
        for state in path:
            machine.transition(state, "test")
        self.assertEqual(machine.state, RobotState.IDLE)

    def test_normal_functions_do_not_require_a_fixed_workflow(self):
        machine = StateMachine()
        machine.transition(RobotState.FORCE_CONTROL, "direct operator selection")
        machine.transition(RobotState.TOOL_DETECT, "independent function")
        self.assertEqual(machine.state, RobotState.TOOL_DETECT)

    def test_exception_latches_safe_stop(self):
        machine = StateMachine(RobotState.TOOL_PICK)
        machine.fail(RuntimeError("injected"), "pick")
        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        with self.assertRaises(StateTransitionError):
            machine.transition(RobotState.IDLE, "unsafe bypass")


if __name__ == "__main__":
    unittest.main()
