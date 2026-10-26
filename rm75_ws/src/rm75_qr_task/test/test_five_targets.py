import copy
import os
import tempfile
import unittest

import numpy as np
import yaml

from rm75_qr_task.controller_pose import (controller_pose_to_transform,
                                          rpy_matrix)
from rm75_qr_task.five_targets import (CalibrationStore, FiveTargetComputer,
                                       FiveTargetError, build_calibration,
                                       joint_target_reached,
                                       observation_joint_target,
                                       reference_tcp_transforms,
                                       validate_calibration)
from rm75_qr_task.pose_manager import (compose, matrix_quaternion,
                                       transform_matrix)


POSES = [
    [-120.806, 254.971, 217.749, 3.073, -0.047, 1.497],
    [-189.102, 258.388, 213.392, 3.135, -0.026, -1.618],
    [-119.581, 326.464, 211.952, -3.126, 0.021, 1.553],
    [-189.641, 328.648, 211.018, -3.124, -0.019, -1.728],
    [-249.211, 116.644, 177.064, -3.137, 0.019, -1.608],
]


def configuration(tcp_offset=None):
    if tcp_offset is None:
        tcp_offset = transform_matrix([0, 0, 0], [0, 0, 0, 1])
    from rm75_qr_task.pose_manager import serialize_transform
    link7_tcp = serialize_transform(tcp_offset)
    link7_tcp.update({'verified': True, 'source': 'unit-test'})
    return {
        'calibration_version': 'test-v1',
        'base_frame': 'base_link', 'link7_frame': 'Link7',
        'tcp_frame': 'test_tcp', 'marker_type': 'apriltag16h5',
        'marker_id': 0, 'marker_size_m': 0.0225,
        'marker_size_measurement_verified': True,
        'handeye': {
            'translation': [0.02, 0.09, 0.025],
            'quaternion_xyzw': [0, 0, 0, 1],
            'verified': True, 'source': 'unit-test'},
        'controller_pose_definition': {
            'position_unit': 'mm', 'angle_unit': 'rad',
            'rotation_convention': 'fixed_xyz_rpy',
            'controller_frame': 'base_link', 'pose_reference': 'tcp',
            'definition_confirmed': True,
        },
        'reference_controller_poses': [
            {'slot_id': index + 1, 'values': value}
            for index, value in enumerate(POSES)],
        'link7_to_tcp': link7_tcp,
    }


class FiveTargetTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.marker = transform_matrix(
            [0.31, -0.12, 0.48], matrix_quaternion(rpy_matrix(0.2, -0.1, 0.4)))

    def _computer(self, config=None):
        calibration = build_calibration(config or configuration(), self.marker,
                                        created_at='unit-test')
        return FiveTargetComputer(calibration, max_marker_age_s=0.5,
                                  clock=lambda: self.now), calibration

    def test_controller_mm_fixed_xyz_conversion(self):
        result = controller_pose_to_transform([1000, 2000, 3000, 0.2, -0.3, 0.4])
        np.testing.assert_allclose(result[:3, 3], [1, 2, 3], atol=1e-12)
        np.testing.assert_allclose(result[:3, :3], rpy_matrix(0.2, -0.3, 0.4),
                                   atol=1e-12)

    def test_reference_marker_round_trip_recovers_all_five_teach_poses(self):
        computer, _ = self._computer()
        result = computer.compute_all(0, self.now, self.marker)
        references = reference_tcp_transforms(configuration())
        for slot_id in range(1, 6):
            np.testing.assert_allclose(result['targets'][slot_id]['tcp'],
                                       references[slot_id], atol=1e-10)

    def test_link7_reference_does_not_require_a_separate_tcp_transform(self):
        cfg = configuration()
        cfg['controller_pose_definition']['pose_reference'] = 'link7'
        cfg['tcp_frame'] = 'Link7'
        del cfg['link7_to_tcp']
        computer, calibration = self._computer(cfg)
        self.assertEqual(calibration['target_reference'], 'link7')
        self.assertEqual(calibration['link7_to_tcp']['source'],
                         'not_applicable_pose_reference_is_Link7')
        result = computer.compute_all(0, self.now, self.marker)
        references = reference_tcp_transforms(cfg)
        for slot_id in range(1, 6):
            np.testing.assert_allclose(result['targets'][slot_id]['link7'],
                                       references[slot_id], atol=1e-10)
            np.testing.assert_allclose(result['targets'][slot_id]['tcp'],
                                       result['targets'][slot_id]['link7'],
                                       atol=1e-12)

    def test_camera_observation_changes_only_joint7(self):
        current = [0.1 * index for index in range(1, 8)]
        target = observation_joint_target(
            current, ['joint{}'.format(index) for index in range(1, 8)],
            {'joint_name': 'joint7', 'position_rad': -1.25,
             'verified': True})
        self.assertEqual(target[:6], current[:6])
        self.assertEqual(target[6], -1.25)
        with self.assertRaises(FiveTargetError):
            observation_joint_target(
                current, ['joint{}'.format(index) for index in range(1, 8)],
                {'joint_name': 'joint7', 'position_rad': -1.25,
                 'verified': False})

    def test_camera_observation_at_target_is_a_verified_noop(self):
        current = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, -1.899990966]
        target = observation_joint_target(
            current, ['joint{}'.format(index) for index in range(1, 8)],
            {'joint_name': 'joint7', 'position_rad': -1.899990966,
             'verified': True})
        self.assertTrue(joint_target_reached(current, target, 0.02))
        target[-1] = -1.85
        self.assertFalse(joint_target_reached(current, target, 0.02))
        with self.assertRaises(FiveTargetError):
            joint_target_reached(current, target, 0.0)

    def test_camera_observation_full_pose_restores_every_joint(self):
        current = [0.1 * index for index in range(1, 8)]
        names = ['joint{}'.format(index) for index in range(1, 8)]
        taught = [-0.1 * index for index in range(1, 8)]
        target = observation_joint_target(
            current, names,
            {'joint_name': 'joint7', 'position_rad': taught[-1],
             'joint_names': names, 'positions_rad': taught,
             'verified': True})
        self.assertEqual(target, taught)
        with self.assertRaises(FiveTargetError):
            observation_joint_target(
                current, names,
                {'joint_name': 'joint7', 'position_rad': 0.0,
                 'joint_names': names, 'positions_rad': taught,
                 'verified': True})

    def test_marker_rigid_motion_moves_all_targets_as_se3(self):
        computer, _ = self._computer()
        references = reference_tcp_transforms(configuration())
        for angles in ((0.3, 0, 0), (0, -0.4, 0), (0, 0, 0.5)):
            delta = transform_matrix([0.08, -0.03, 0.02],
                                     matrix_quaternion(rpy_matrix(*angles)))
            moved_marker = compose(delta, self.marker)
            result = computer.compute_all(0, self.now, moved_marker)
            for slot_id in range(1, 6):
                np.testing.assert_allclose(result['targets'][slot_id]['tcp'],
                                           compose(delta, references[slot_id]),
                                           atol=1e-9)

    def test_near_pi_euler_is_not_added_componentwise(self):
        cfg = configuration()
        cfg['reference_controller_poses'][0]['values'][3:] = [3.14159, -3.13, 3.12]
        computer, _ = self._computer(cfg)
        value = computer.compute_selected(1, 0, self.now, self.marker)['tcp']
        self.assertAlmostEqual(np.linalg.det(value[:3, :3]), 1.0, places=9)
        np.testing.assert_allclose(np.dot(value[:3, :3].T, value[:3, :3]),
                                   np.eye(3), atol=1e-9)

    def test_nonzero_tcp_offset_produces_distinct_link7_target(self):
        offset = transform_matrix([0.02, -0.01, 0.11],
                                  matrix_quaternion(rpy_matrix(0.1, 0.2, -0.1)))
        computer, _ = self._computer(configuration(offset))
        target = computer.compute_selected(3, 0, self.now, self.marker)
        self.assertFalse(np.allclose(target['tcp'], target['link7']))
        np.testing.assert_allclose(compose(target['link7'], offset),
                                   target['tcp'], atol=1e-9)

    def test_every_slot_is_distinct_and_selectable(self):
        computer, _ = self._computer()
        positions = []
        for slot_id in range(1, 6):
            positions.append(computer.compute_selected(
                slot_id, 0, self.now, self.marker)['tcp'][:3, 3])
        self.assertEqual(len({tuple(np.round(item, 8)) for item in positions}), 5)
        with self.assertRaises(FiveTargetError):
            computer.compute_selected(0, 0, self.now, self.marker)

    def test_wrong_marker_and_stale_snapshot_are_rejected(self):
        computer, _ = self._computer()
        with self.assertRaises(FiveTargetError):
            computer.compute_all(1, self.now, self.marker)
        with self.assertRaises(FiveTargetError):
            computer.compute_all(0, self.now - 0.51, self.marker)

    def test_unconfirmed_controller_definition_and_tcp_fail_closed(self):
        cfg = configuration()
        cfg['controller_pose_definition']['definition_confirmed'] = False
        with self.assertRaises(FiveTargetError):
            build_calibration(cfg, self.marker)
        cfg = configuration()
        cfg['link7_to_tcp']['verified'] = False
        with self.assertRaises(FiveTargetError):
            build_calibration(cfg, self.marker)

    def test_corrupt_incomplete_or_mismatched_calibration_is_rejected(self):
        _, calibration = self._computer()
        broken = copy.deepcopy(calibration)
        broken['marker_to_tcp'].pop()
        with self.assertRaises(FiveTargetError):
            validate_calibration(broken)
        broken = copy.deepcopy(calibration)
        broken['base_marker_reference']['quaternion_xyzw'] = [0, 0, 0, 0]
        with self.assertRaises(ValueError):
            validate_calibration(broken)
        with self.assertRaises(FiveTargetError):
            validate_calibration(calibration, {'calibration_version': 'other'})

    def test_atomic_yaml_round_trip_leaves_no_temporary_file(self):
        _, calibration = self._computer()
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'calibration.yaml')
            store = CalibrationStore(path)
            store.save(calibration)
            loaded = store.load({'calibration_version': 'test-v1'})
            self.assertEqual(loaded['version'], 'test-v1')
            self.assertEqual(os.listdir(directory), ['calibration.yaml'])
            with open(path) as stream:
                self.assertEqual(yaml.safe_load(stream)['schema'],
                                 'rm75_single_marker_five_targets/v1')


if __name__ == '__main__':
    unittest.main()
