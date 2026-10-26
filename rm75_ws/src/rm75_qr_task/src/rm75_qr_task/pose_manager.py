"""Small, dependency-light SE(3) helpers using xyzw quaternions."""

import math
import numpy as np


def _normalized_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('quaternion must contain four finite values')
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError('zero quaternion')
    return q / norm


def quaternion_matrix(q):
    x, y, z, w = _normalized_quaternion(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def matrix_quaternion(rotation):
    r = np.asarray(rotation, dtype=float)
    if r.shape != (3, 3) or not np.all(np.isfinite(r)):
        raise ValueError('rotation must be a finite 3x3 matrix')
    # Stable branch-based conversion, returned as xyzw.
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
             (r[1, 0]-r[0, 1])/s, 0.25*s]
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(1+r[0, 0]-r[1, 1]-r[2, 2])*2
            q = [0.25*s, (r[0, 1]+r[1, 0])/s,
                 (r[0, 2]+r[2, 0])/s, (r[2, 1]-r[1, 2])/s]
        elif i == 1:
            s = math.sqrt(1+r[1, 1]-r[0, 0]-r[2, 2])*2
            q = [(r[0, 1]+r[1, 0])/s, 0.25*s,
                 (r[1, 2]+r[2, 1])/s, (r[0, 2]-r[2, 0])/s]
        else:
            s = math.sqrt(1+r[2, 2]-r[0, 0]-r[1, 1])*2
            q = [(r[0, 2]+r[2, 0])/s, (r[1, 2]+r[2, 1])/s,
                 0.25*s, (r[1, 0]-r[0, 1])/s]
    q = _normalized_quaternion(q)
    return q if q[3] >= 0 else -q


def transform_matrix(translation, quaternion):
    t = np.asarray(translation, dtype=float)
    if t.shape != (3,) or not np.all(np.isfinite(t)):
        raise ValueError('translation must contain three finite values')
    result = np.eye(4)
    result[:3, :3] = quaternion_matrix(quaternion)
    result[:3, 3] = t
    return result


def transform_parts(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError('transform must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError('invalid homogeneous transform')
    return matrix[:3, 3].copy(), matrix_quaternion(matrix[:3, :3])


def validate_transform(matrix, orthogonality_tolerance=1e-8):
    """Validate and return a copy of a proper right-handed SE(3) transform."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError('transform must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError('invalid homogeneous transform bottom row')
    rotation = matrix[:3, :3]
    if not np.allclose(np.dot(rotation.T, rotation), np.eye(3),
                       atol=float(orthogonality_tolerance)):
        raise ValueError('rotation matrix is not orthogonal')
    determinant = float(np.linalg.det(rotation))
    if not np.isclose(determinant, 1.0, atol=float(orthogonality_tolerance)):
        raise ValueError('rotation matrix determinant is not +1')
    return matrix.copy()


def compose(*transforms):
    result = np.eye(4)
    for transform in transforms:
        transform = np.asarray(transform, dtype=float)
        validate_transform(transform)
        result = np.dot(result, transform)
    return validate_transform(result, orthogonality_tolerance=1e-7)


def inverse(transform):
    transform = validate_transform(transform)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -np.dot(result[:3, :3], transform[:3, 3])
    return validate_transform(result)


def rotation_distance(a, b):
    qa = matrix_quaternion(np.asarray(a)[:3, :3])
    qb = matrix_quaternion(np.asarray(b)[:3, :3])
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(qa, qb)))))


def offset_along_marker(transform, direction, distance):
    direction = np.asarray(direction, dtype=float)
    norm = np.linalg.norm(direction)
    if direction.shape != (3,) or norm < 1e-12:
        raise ValueError('approach direction must be a non-zero 3-vector')
    offset = np.eye(4)
    offset[:3, 3] = direction / norm * float(distance)
    return compose(transform, offset)


def serialize_transform(transform):
    t, q = transform_parts(transform)
    return {'translation': t.tolist(), 'quaternion_xyzw': q.tolist()}


def deserialize_transform(data):
    return transform_matrix(data['translation'], data['quaternion_xyzw'])
