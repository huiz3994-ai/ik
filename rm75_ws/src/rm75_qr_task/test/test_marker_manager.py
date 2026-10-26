#!/usr/bin/env python3
import unittest
import numpy as np

from rm75_qr_task.marker_manager import MarkerManager, MarkerQualityError
from rm75_qr_task.pose_manager import transform_matrix


class MarkerManagerTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        slots = {index: {'enabled': True, 'calibrated': True, 'size_m': 0.0225}
                 for index in range(5)}
        self.manager = MarkerManager(slots, stability_samples=3, max_age=0.5,
                                     clock=lambda: self.now)

    def test_all_five_ids_are_distinct_and_stable(self):
        for slot in range(5):
            for sample in range(3):
                transform = transform_matrix([slot * 0.1 + sample * 0.0001, 0, 0.5],
                                             [0, 0, 0, 1])
                self.manager.add(slot, 9.8 + sample * 0.05, transform, 0.2)
            self.assertTrue(self.manager.stable(slot))
            self.assertAlmostEqual(self.manager.latest(slot)[1][0, 3],
                                   slot * 0.1 + 0.0002)

    def test_jump_resets_window_and_stale_rejected(self):
        identity = transform_matrix([0, 0, 0.5], [0, 0, 0, 1])
        for stamp in (9.8, 9.85, 9.9):
            self.manager.add(0, stamp, identity, 0.2)
        self.manager.add(0, 9.95,
                         transform_matrix([0.1, 0, 0.5], [0, 0, 0, 1]), 0.2)
        self.assertFalse(self.manager.stable(0))
        self.now = 11.0
        with self.assertRaises(MarkerQualityError):
            self.manager.latest(0)

    def test_bad_id_reprojection_and_uncalibrated_are_rejected(self):
        identity = np.eye(4)
        with self.assertRaises(MarkerQualityError):
            self.manager.add(9, 10, identity, 0.2)
        with self.assertRaises(MarkerQualityError):
            self.manager.add(0, 10, identity, 3.0)
        locked = MarkerManager({0: {'enabled': False, 'calibrated': False,
                                    'size_m': 0.0}})
        with self.assertRaises(MarkerQualityError):
            locked.add(0, 10, identity, 0.2)


if __name__ == '__main__':
    unittest.main()
