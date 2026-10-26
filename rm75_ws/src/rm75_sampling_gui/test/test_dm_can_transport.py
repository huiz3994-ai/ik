import importlib.util
from pathlib import Path
import struct
import sys

import pytest


def _load_dm_can():
    workspace = Path(__file__).resolve().parents[3]
    source = workspace / "work/dm-motor-sdk/Python例程/u2can/DM_CAN.py"
    spec = importlib.util.spec_from_file_location("rm75_test_dm_can", str(source))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_motor_control():
    workspace = Path(__file__).resolve().parents[3]
    staged_python = str(workspace / ".rm75_hardware/python")
    if staged_python not in sys.path:
        sys.path.insert(0, staged_python)
    source = workspace / ".rm75_hardware/motor_control.py"
    spec = importlib.util.spec_from_file_location("rm75_test_motor_control", str(source))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_parameter_receiver_reassembles_fragmented_serial_reply():
    dm_can = _load_dm_can()

    class FakeSerial:
        def __init__(self, chunks):
            self.chunks = iter(chunks)

        def read_all(self):
            return next(self.chunks, b"")

    class FakeMotor:
        def __init__(self):
            self.temp_param_dict = {}

    # USB-to-CAN reply: AA, command, reserved, CAN ID, payload, 55.
    # Payload acknowledges a float-valued RID 3 write; this test exercises
    # transport reassembly independently of that register's valid range.
    payload = bytes((1, 0, 0x55, 3)) + struct.pack("f", 1.5)
    frame = bytes((0xAA, 0x11, 0, 0, 0, 0, 0)) + payload + bytes((0x55,))
    motor = FakeMotor()
    control = dm_can.MotorControl.__new__(dm_can.MotorControl)
    control.serial_ = FakeSerial((frame[:9], frame[9:]))
    control.motors_map = {1: motor}
    control.data_save = b""

    control.recv_set_param_data()
    assert motor.temp_param_dict == {}
    control.recv_set_param_data()
    assert motor.temp_param_dict[3] == 1.5


def test_ampere_limit_is_converted_to_normalized_oc_value():
    motor_control = _load_motor_control()
    ratio, full_scale_current = motor_control.SafeMotor.over_current_ratio(1.5, {
        "torque_constant": 0.0,
        "pole_pairs": 14,
        "flux": 0.0045345681719481945,
        "gear_ratio": 40.0,
        "gear_efficiency": 1.0,
        "tmax": 28.0,
    })
    assert 0.0 < ratio < 1.0
    expected_full_scale = 28.0 / (1.5 * 14 * 0.0045345681719481945 * 40.0)
    assert full_scale_current == pytest.approx(expected_full_scale)
    assert ratio == pytest.approx(1.5 / full_scale_current)

    # A prior command leaves the volatile TMAX register at the enforced 5 N.m
    # limit; conversion must still use the motor model's 28 N.m full scale.
    _, repeated_full_scale = motor_control.SafeMotor.over_current_ratio(1.5, {
        "torque_constant": 0.0,
        "pole_pairs": 14,
        "flux": 0.0045345681719481945,
        "gear_ratio": 40.0,
        "gear_efficiency": 1.0,
        "tmax": 5.0,
    })
    assert repeated_full_scale == pytest.approx(expected_full_scale)
