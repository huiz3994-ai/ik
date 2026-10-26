"""Planar square-marker PnP shared by AprilTag and QR detection."""

import numpy as np


class PnPError(RuntimeError):
    pass


def square_object_points(size_m):
    half = float(size_m) / 2.0
    if not np.isfinite(half) or half <= 0:
        raise PnPError('marker physical size must be positive')
    # TL, TR, BR, BL as seen from the marker front; +X right, +Y up.
    return np.array([[-half, half, 0], [half, half, 0],
                     [half, -half, 0], [-half, -half, 0]], np.float64)


def solve_square_pose(corners, size_m, camera_matrix, distortion, cv2_module=None):
    if cv2_module is None:
        import cv2 as cv2_module
    corners = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1, 1)
    if not (np.all(np.isfinite(corners)) and
            np.all(np.isfinite(camera_matrix)) and
            np.all(np.isfinite(distortion))):
        raise PnPError('PnP input contains non-finite values')
    if camera_matrix[0, 0] <= 0 or camera_matrix[1, 1] <= 0:
        raise PnPError('invalid focal length')
    object_points = square_object_points(size_m)
    flag = getattr(cv2_module, 'SOLVEPNP_IPPE_SQUARE', cv2_module.SOLVEPNP_ITERATIVE)
    ok, rotation_vector, translation = cv2_module.solvePnP(
        object_points, corners, camera_matrix, distortion, flags=flag)
    if not ok:
        raise PnPError('solvePnP failed')
    rotation, _ = cv2_module.Rodrigues(rotation_vector)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation).reshape(3)
    if transform[2, 3] <= 0:
        raise PnPError('marker solution is behind the optical camera')
    projected, _ = cv2_module.projectPoints(
        object_points, rotation_vector, translation, camera_matrix, distortion)
    residual = projected.reshape(4, 2) - corners
    rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
    return transform, rms
