#!/usr/bin/env python3
import math
import unittest
import numpy as np

from rm75_qr_task.pnp import solve_square_pose, square_object_points
from rm75_qr_task.pose_manager import (compose, inverse, rotation_distance,
                                       transform_matrix, transform_parts)


class PoseManagerTest(unittest.TestCase):
    def test_transform_round_trip_and_inverse(self):
        q = [0.0, 0.0, math.sin(0.3), math.cos(0.3)]
        transform = transform_matrix([0.1, -0.2, 0.3], q)
        translation, recovered = transform_parts(transform)
        self.assertTrue(np.allclose(translation, [0.1, -0.2, 0.3]))
        self.assertAlmostEqual(abs(float(np.dot(q, recovered))), 1.0, places=8)
        self.assertTrue(np.allclose(compose(transform, inverse(transform)), np.eye(4)))

    def test_rotation_is_not_replaced_by_identity(self):
        a = transform_matrix([0, 0, 0], [0, 0, 0, 1])
        b = transform_matrix([0, 0, 0], [0, math.sin(0.25), 0, math.cos(0.25)])
        self.assertAlmostEqual(rotation_distance(a, b), 0.5, places=7)

    def test_pnp_recovers_six_d_pose(self):
        import cv2
        camera = np.array([[600., 0, 320], [0, 610., 240], [0, 0, 1.]])
        distortion = np.zeros(5)
        rotation_vector = np.array([[0.18], [-0.12], [0.3]])
        translation = np.array([[0.04], [-0.03], [0.7]])
        points, _ = cv2.projectPoints(square_object_points(0.08),
                                      rotation_vector, translation,
                                      camera, distortion)
        transform, error = solve_square_pose(points, 0.08, camera, distortion)
        expected_rotation, _ = cv2.Rodrigues(rotation_vector)
        expected = np.eye(4)
        expected[:3, :3] = expected_rotation
        expected[:3, 3] = translation.reshape(3)
        self.assertLess(error, 1e-5)
        self.assertLess(np.linalg.norm(transform[:3, 3] - expected[:3, 3]), 1e-5)
        self.assertLess(rotation_distance(transform, expected), 1e-5)


if __name__ == '__main__':
    unittest.main()
