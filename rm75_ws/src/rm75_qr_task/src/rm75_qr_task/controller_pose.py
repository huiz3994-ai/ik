"""Conversion of RM controller pose feedback into explicit SE(3) values.

The vendor driver converts controller Euler values with tf2::Quaternion.setRPY,
which is the fixed-axis XYZ (roll, pitch, yaw) convention represented by
Rz(yaw) * Ry(pitch) * Rx(roll).  This module does not decide whether an
arbitrary pendant recording used the base work frame or the flange TCP; that
fact must be explicitly confirmed in the calibration configuration.
"""

import math
import numpy as np

from .pose_manager import matrix_quaternion, transform_matrix


class ControllerPoseError(ValueError):
    pass


SUPPORTED_CONVENTION = 'fixed_xyz_rpy'


def rpy_matrix(roll, pitch, yaw):
    values = np.asarray([roll, pitch, yaw], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ControllerPoseError('Euler values must be finite')
    cr, sr = math.cos(values[0]), math.sin(values[0])
    cp, sp = math.cos(values[1]), math.sin(values[1])
    cy, sy = math.cos(values[2]), math.sin(values[2])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return np.dot(rz, np.dot(ry, rx))


def controller_pose_to_transform(values, position_unit='mm',
                                 angle_unit='rad',
                                 convention=SUPPORTED_CONVENTION):
    values = np.asarray(values, dtype=float)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ControllerPoseError('controller pose must contain six finite values')
    if convention != SUPPORTED_CONVENTION:
        raise ControllerPoseError('unsupported controller rotation convention {}'.format(
            convention))
    scales = {'m': 1.0, 'mm': 1e-3}
    if position_unit not in scales:
        raise ControllerPoseError('unsupported position unit {}'.format(position_unit))
    if angle_unit == 'deg':
        angles = np.deg2rad(values[3:])
    elif angle_unit == 'rad':
        angles = values[3:]
    else:
        raise ControllerPoseError('unsupported angle unit {}'.format(angle_unit))
    rotation = rpy_matrix(*angles)
    return transform_matrix(values[:3] * scales[position_unit],
                            matrix_quaternion(rotation))
