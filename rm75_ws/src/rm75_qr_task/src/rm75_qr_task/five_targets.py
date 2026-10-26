"""Fail-closed calibration and SE(3) math for one marker and five TCP targets."""

import copy
import datetime
import os
import tempfile
import time

import numpy as np
import yaml

from .controller_pose import controller_pose_to_transform
from .pose_manager import (compose, deserialize_transform, inverse,
                           rotation_distance, serialize_transform,
                           validate_transform)


SCHEMA = 'rm75_single_marker_five_targets/v1'
SLOT_IDS = (1, 2, 3, 4, 5)


class FiveTargetError(RuntimeError):
    pass


def observation_joint_target(current_positions, active_joint_names,
                             camera_observation):
    """Return the taught observation joints, or legacy J7-only fallback."""
    names = list(active_joint_names)
    positions = [float(value) for value in current_positions]
    if len(names) != 7 or len(positions) != 7 or len(set(names)) != 7:
        raise FiveTargetError('observation move requires seven unique active joints')
    if not all(np.isfinite(positions)):
        raise FiveTargetError('current joint state contains a non-finite value')
    if not isinstance(camera_observation, dict):
        raise FiveTargetError('camera observation J7 position is not calibrated')
    joint_name = str(camera_observation.get('joint_name', ''))
    position = float(camera_observation.get('position_rad', np.nan))
    if joint_name != 'joint7' or joint_name not in names or not np.isfinite(position):
        raise FiveTargetError('camera observation must contain a finite joint7 position')
    if not camera_observation.get('verified', False):
        raise FiveTargetError('camera observation J7 position is not verified')
    taught_names = camera_observation.get('joint_names')
    taught_positions = camera_observation.get('positions_rad')
    if taught_names is not None or taught_positions is not None:
        taught_names = list(taught_names or [])
        taught_positions = [float(value) for value in (taught_positions or [])]
        if (len(taught_names) != 7 or len(taught_positions) != 7 or
                len(set(taught_names)) != 7 or set(taught_names) != set(names) or
                not all(np.isfinite(taught_positions))):
            raise FiveTargetError(
                'camera observation full joint position is invalid')
        taught = dict(zip(taught_names, taught_positions))
        if abs(taught['joint7'] - position) > 1e-6:
            raise FiveTargetError(
                'camera observation joint7 disagrees with full joint position')
        return [taught[name] for name in names]
    target = list(positions)
    target[names.index(joint_name)] = position
    return target


def joint_target_reached(current_positions, target_positions, tolerance_rad):
    """Return whether every joint is already inside the requested tolerance."""
    current = [float(value) for value in current_positions]
    target = [float(value) for value in target_positions]
    tolerance = float(tolerance_rad)
    if (not current or len(current) != len(target) or
            not all(np.isfinite(current)) or not all(np.isfinite(target)) or
            not np.isfinite(tolerance) or tolerance <= 0.0):
        raise FiveTargetError('invalid joint target tolerance check')
    return max(abs(actual - expected) for actual, expected in zip(
        current, target)) <= tolerance


def _require(mapping, key):
    if key not in mapping:
        raise FiveTargetError('missing calibration field {}'.format(key))
    return mapping[key]


def _iso_utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def reference_tcp_transforms(config):
    """Convert the five pendant records after their definition is confirmed."""
    definition = _require(config, 'controller_pose_definition')
    if not definition.get('definition_confirmed', False):
        raise FiveTargetError('坐标标定未完成: controller pose definition is not confirmed')
    if definition.get('controller_frame') != config.get('base_frame', 'base_link'):
        raise FiveTargetError('controller work frame is not confirmed equal to base_link')
    if definition.get('pose_reference') not in ('tcp', 'link7'):
        raise FiveTargetError(
            'controller pose reference must be confirmed as TCP or Link7')
    records = _require(config, 'reference_controller_poses')
    result = {}
    for item in records:
        slot_id = int(item['slot_id'])
        if slot_id in result:
            raise FiveTargetError('duplicate reference slot {}'.format(slot_id))
        try:
            result[slot_id] = controller_pose_to_transform(
                item['values'], definition.get('position_unit', 'mm'),
                definition.get('angle_unit', 'rad'),
                definition.get('rotation_convention', 'fixed_xyz_rpy'))
        except (ValueError, KeyError) as exc:
            raise FiveTargetError('invalid reference slot {}: {}'.format(slot_id, exc))
    if tuple(sorted(result)) != SLOT_IDS:
        raise FiveTargetError('reference poses must contain slots 1 through 5 exactly once')
    return result


def build_calibration(config, base_marker_reference, quality=None,
                      created_at=None):
    """Build a calibration document; no physical verification is inferred."""
    base_marker_reference = validate_transform(base_marker_reference)
    if not config.get('marker_size_measurement_verified', False):
        raise FiveTargetError(
            'marker encoded-square size has not been physically measured')
    handeye = _require(config, 'handeye')
    if not handeye.get('verified', False):
        raise FiveTargetError('hand-eye calibration is not verified')
    validate_transform(deserialize_transform(handeye))
    references = reference_tcp_transforms(config)
    pose_reference = config['controller_pose_definition'].get('pose_reference')
    if pose_reference == 'link7':
        # The teach-pendant values already describe the MoveIt endpoint.  A
        # separate tool transform is neither needed nor allowed to block this
        # mode; keep an explicit identity only for the persisted schema and
        # the existing ROS JSON contract.
        link7_tcp = {
            'translation': [0.0, 0.0, 0.0],
            'quaternion_xyzw': [0.0, 0.0, 0.0, 1.0],
            'verified': True,
            'source': 'not_applicable_pose_reference_is_Link7',
        }
    else:
        link7_tcp = _require(config, 'link7_to_tcp')
        if not link7_tcp.get('verified', False):
            raise FiveTargetError(
                '坐标标定未完成: Link7 to TCP transform is not verified')
    link7_tcp_transform = deserialize_transform(link7_tcp)
    marker_targets = {
        slot_id: compose(inverse(base_marker_reference), transform)
        for slot_id, transform in references.items()
    }
    calibration = {
        'schema': SCHEMA,
        'version': str(config.get('calibration_version', 'unversioned')),
        'created_at': created_at or _iso_utc_now(),
        'mode': 'single_marker_five_targets',
        'base_frame': str(config.get('base_frame', 'base_link')),
        'link7_frame': str(config.get('link7_frame', 'Link7')),
        'tcp_frame': str(config.get('tcp_frame', 'controller_tcp')),
        'target_reference': str(pose_reference),
        'marker': {
            'family': str(config.get('marker_type', 'apriltag16h5')),
            'id': int(config.get('marker_id', 0)),
            'size_m': float(config.get('marker_size_m', 0.0)),
            'size_measurement_verified': bool(
                config.get('marker_size_measurement_verified', False)),
        },
        'controller_pose_definition': copy.deepcopy(
            config['controller_pose_definition']),
        'base_marker_reference': serialize_transform(base_marker_reference),
        'link7_to_tcp': copy.deepcopy(link7_tcp),
        'handeye': copy.deepcopy(handeye),
        'reference_tcp': [
            {'slot_id': slot_id, 'name': 'Tool{}'.format(slot_id),
             **serialize_transform(references[slot_id])}
            for slot_id in SLOT_IDS
        ],
        'marker_to_tcp': [
            {'slot_id': slot_id, 'name': 'Tool{}'.format(slot_id),
             **serialize_transform(marker_targets[slot_id])}
            for slot_id in SLOT_IDS
        ],
        'quality': copy.deepcopy(quality or {}),
        'mathematical_validation': {'passed': True, 'max_error': 0.0},
        'physical_validation': {'passed': False,
                                'reason': 'requires supervised on-site validation'},
    }
    validate_calibration(calibration)
    # Mathematical round trip is part of creation, not a claimed physical result.
    maximum = 0.0
    for slot_id in SLOT_IDS:
        rebuilt = compose(base_marker_reference, marker_targets[slot_id])
        maximum = max(maximum, float(np.max(np.abs(rebuilt - references[slot_id]))))
    calibration['mathematical_validation']['max_error'] = maximum
    calibration['mathematical_validation']['passed'] = maximum <= 1e-9
    return calibration


def validate_calibration(data, expected=None):
    if not isinstance(data, dict) or data.get('schema') != SCHEMA:
        raise FiveTargetError('unsupported or missing calibration schema')
    marker = _require(data, 'marker')
    if marker.get('family') != 'apriltag16h5' or int(marker.get('id', -1)) != 0:
        raise FiveTargetError('calibration must use apriltag16h5 ID 0')
    if not np.isfinite(float(marker.get('size_m', 0))) or float(marker['size_m']) <= 0:
        raise FiveTargetError('marker encoded-square size is invalid')
    reference = deserialize_transform(_require(data, 'base_marker_reference'))
    validate_transform(reference)
    link7_tcp = _require(data, 'link7_to_tcp')
    validate_transform(deserialize_transform(link7_tcp))
    handeye = _require(data, 'handeye')
    if not handeye.get('verified', False):
        raise FiveTargetError('calibration hand-eye transform is not verified')
    validate_transform(deserialize_transform(handeye))
    camera_observation = data.get('camera_observation')
    if camera_observation is not None:
        observation_joint_target(
            [0.0] * 7,
            ['joint{}'.format(index) for index in range(1, 8)],
            camera_observation)
    targets = {}
    for item in _require(data, 'marker_to_tcp'):
        slot_id = int(item['slot_id'])
        if slot_id in targets:
            raise FiveTargetError('duplicate target slot {}'.format(slot_id))
        targets[slot_id] = validate_transform(deserialize_transform(item))
    if tuple(sorted(targets)) != SLOT_IDS:
        raise FiveTargetError('calibration must contain targets 1 through 5')
    if expected:
        for key in ('base_frame', 'link7_frame', 'tcp_frame'):
            if expected.get(key) and data.get(key) != expected.get(key):
                raise FiveTargetError('{} mismatch: {} != {}'.format(
                    key, data.get(key), expected.get(key)))
        if expected.get('calibration_version') and data.get('version') != str(
                expected['calibration_version']):
            raise FiveTargetError('calibration version mismatch')
        if expected.get('handeye'):
            actual_handeye = deserialize_transform(data['handeye'])
            configured_handeye = deserialize_transform(expected['handeye'])
            if not np.allclose(actual_handeye, configured_handeye, atol=1e-10):
                raise FiveTargetError('hand-eye transform changed since calibration')
    return data


class CalibrationStore(object):
    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(path))

    def load(self, expected=None):
        try:
            with open(self.path, 'r') as stream:
                data = yaml.safe_load(stream)
        except (OSError, yaml.YAMLError) as exc:
            raise FiveTargetError('cannot load calibration: {}'.format(exc))
        return validate_calibration(data, expected)

    def save(self, data):
        validate_calibration(data)
        directory = os.path.dirname(self.path)
        os.makedirs(directory, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix='.five-targets-', suffix='.yaml.tmp', dir=directory)
        try:
            with os.fdopen(descriptor, 'w') as stream:
                yaml.safe_dump(data, stream, default_flow_style=False,
                               sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            try:
                directory_fd = os.open(directory, os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except OSError:
                pass
        except Exception:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            raise


class FiveTargetComputer(object):
    def __init__(self, calibration, max_marker_age_s=0.5, clock=None):
        self.calibration = validate_calibration(copy.deepcopy(calibration))
        self.max_marker_age_s = float(max_marker_age_s)
        self.clock = clock or time.time
        self.marker_id = int(self.calibration['marker']['id'])
        self.marker_targets = {
            int(item['slot_id']): deserialize_transform(item)
            for item in self.calibration['marker_to_tcp']
        }
        self.link7_tcp = deserialize_transform(self.calibration['link7_to_tcp'])

    def _validate_observation(self, marker_id, stamp, base_marker):
        if int(marker_id) != self.marker_id:
            raise FiveTargetError('wrong marker ID {}; expected {}'.format(
                marker_id, self.marker_id))
        age = float(self.clock()) - float(stamp)
        if age < 0 or age > self.max_marker_age_s:
            raise FiveTargetError('marker observation is stale ({:.3f}s)'.format(age))
        return validate_transform(base_marker), age

    def compute_all(self, marker_id, stamp, base_marker):
        base_marker, age = self._validate_observation(marker_id, stamp, base_marker)
        result = {}
        for slot_id in SLOT_IDS:
            tcp = compose(base_marker, self.marker_targets[slot_id])
            link7 = compose(tcp, inverse(self.link7_tcp))
            result[slot_id] = {'tcp': tcp, 'link7': link7}
        return {'marker_id': self.marker_id, 'marker_stamp': float(stamp),
                'marker_age_s': age, 'calibration_version': self.calibration['version'],
                'targets': result}

    def compute_selected(self, slot_id, marker_id, stamp, base_marker):
        slot_id = int(slot_id)
        if slot_id not in SLOT_IDS:
            raise FiveTargetError('tool slot must be in 1..5')
        return self.compute_all(marker_id, stamp, base_marker)['targets'][slot_id]


def pose_error(target, actual):
    target = validate_transform(target)
    actual = validate_transform(actual)
    return {
        'translation_m': float(np.linalg.norm(target[:3, 3] - actual[:3, 3])),
        'rotation_rad': float(rotation_distance(target, actual)),
    }
