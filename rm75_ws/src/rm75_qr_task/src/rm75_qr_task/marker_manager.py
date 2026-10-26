"""Marker validation, freshness, and multi-frame stability tracking."""

from collections import defaultdict, deque
import time
import numpy as np

from .pose_manager import rotation_distance


class MarkerQualityError(RuntimeError):
    pass


class MarkerManager(object):
    def __init__(self, slots, stability_samples=5, max_age=0.5,
                 max_translation_jump=0.006, max_rotation_jump_deg=2.0,
                 max_reprojection_error=2.0, clock=None):
        self.slots = {int(key): value for key, value in slots.items()}
        self.stability_samples = int(stability_samples)
        self.max_age = float(max_age)
        self.max_translation_jump = float(max_translation_jump)
        self.max_rotation_jump = np.deg2rad(float(max_rotation_jump_deg))
        self.max_reprojection_error = float(max_reprojection_error)
        self.clock = clock or time.time
        self.samples = defaultdict(lambda: deque(maxlen=self.stability_samples))

    def validate_slot(self, slot_id):
        slot_id = int(slot_id)
        if slot_id not in self.slots:
            raise MarkerQualityError('unknown slot {}'.format(slot_id))
        config = self.slots[slot_id]
        if not config.get('enabled', False):
            raise MarkerQualityError('slot {} is disabled'.format(slot_id))
        if not config.get('calibrated', False):
            raise MarkerQualityError('slot {} calibration is incomplete'.format(slot_id))
        size = float(config.get('size_m', 0.0))
        if size <= 0:
            raise MarkerQualityError('slot {} marker size is not calibrated'.format(slot_id))
        return config

    def add(self, slot_id, stamp, transform, reprojection_error):
        config = self.validate_slot(slot_id)
        stamp = float(stamp)
        transform = np.asarray(transform, dtype=float)
        error = float(reprojection_error)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise MarkerQualityError('invalid marker transform')
        if not np.isfinite(error) or error > self.max_reprojection_error:
            raise MarkerQualityError('reprojection error {:.3f}px exceeds limit'.format(error))
        queue = self.samples[int(slot_id)]
        if queue:
            previous = queue[-1][1]
            translation = np.linalg.norm(transform[:3, 3] - previous[:3, 3])
            rotation = rotation_distance(transform, previous)
            if translation > self.max_translation_jump or rotation > self.max_rotation_jump:
                queue.clear()
        queue.append((stamp, transform.copy(), error))
        return config

    def stable(self, slot_id, now=None):
        now = self.clock() if now is None else float(now)
        queue = self.samples[int(slot_id)]
        if len(queue) < self.stability_samples:
            return False
        if now - queue[-1][0] < 0 or now - queue[-1][0] > self.max_age:
            return False
        reference = queue[0][1]
        return all(
            np.linalg.norm(item[1][:3, 3] - reference[:3, 3]) <= self.max_translation_jump
            and rotation_distance(item[1], reference) <= self.max_rotation_jump
            for item in list(queue)[1:])

    def latest(self, slot_id, now=None):
        if not self.stable(slot_id, now):
            raise MarkerQualityError('slot {} has no fresh stable pose'.format(slot_id))
        stamp, transform, error = self.samples[int(slot_id)][-1]
        return stamp, transform.copy(), error

    def statistics(self, slot_id):
        """Return measured multi-frame spread without claiming absolute accuracy."""
        queue = list(self.samples[int(slot_id)])
        if not queue:
            return {'sample_count': 0}
        reference = queue[0][1]
        translations = [float(np.linalg.norm(item[1][:3, 3] - reference[:3, 3]))
                        for item in queue]
        rotations = [float(rotation_distance(item[1], reference)) for item in queue]
        return {
            'sample_count': len(queue),
            'translation_span_m': max(translations),
            'rotation_span_deg': float(np.rad2deg(max(rotations))),
            'mean_reprojection_error_px': float(np.mean([item[2] for item in queue])),
        }
