"""Persistent PICK/RETURN orchestration with fail-closed transitions."""

import copy
import threading
import time
import numpy as np

from .pose_manager import (compose, deserialize_transform, inverse,
                           offset_along_marker, rotation_distance,
                           serialize_transform)
from .safety_checker import SafetyError


class TaskError(RuntimeError):
    pass


ACTIVE_STATES = {
    'DETECT_MARKER', 'PLAN_PICK', 'MOVE_PRE_PICK', 'MOVE_PICK', 'POWER_ON',
    'VERIFY_HOLD', 'LIFT_OBJECT', 'RETURN_PICK_START', 'PLAN_RETURN',
    'MOVE_PRE_PLACE', 'MOVE_PLACE', 'POWER_OFF', 'VERIFY_RELEASE', 'RETREAT',
    'RETURN_RETURN_START',
}


class TaskStateMachine(object):
    def __init__(self, slots, marker_manager, motion, voltage, hold_feedback,
                 storage, safety, config=None, clock=None):
        self.slots = {int(key): value for key, value in slots.items()}
        self.markers = marker_manager
        self.motion = motion
        self.voltage = voltage
        self.hold_feedback = hold_feedback
        self.storage = storage
        self.safety = safety
        self.config = dict(config or {})
        self.clock = clock or time.time
        self.lock = threading.Lock()
        self.stop_requested = threading.Event()
        try:
            self.data = storage.load()
        except Exception as exc:
            self.data = storage.default_state()
            self.data['state'] = 'RECOVERY_REQUIRED'
            self.data['last_error'] = 'state file could not be trusted: {}'.format(exc)
            storage.save(self.data)
        previous = self.data.get('state')
        if previous in ACTIVE_STATES or previous in ('HOLDING', 'SAFE_STOP'):
            self.data['state'] = 'RECOVERY_REQUIRED'
            self.data['last_error'] = (
                'restart with physical state requiring independent confirmation; '
                'no automatic power action was taken')
            self.storage.save(self.data)

    def record_station_pose(self, slot_id, stamp, transform,
                            reprojection_error):
        """Persist a commissioned fixed station without silently accepting motion."""
        slot_id = int(slot_id)
        slot = self.slots.get(slot_id)
        if not slot or not slot.get('station_fixed_verified', False):
            return
        with self.lock:
            records = self.data.setdefault('station_records', {})
            key = str(slot_id)
            previous = records.get(key)
            moved = False
            if previous:
                old = deserialize_transform(previous['base_marker'])
                moved = (
                    np.linalg.norm(transform[:3, 3] - old[:3, 3]) >
                    self.config.get('station_motion_tolerance_m', 0.005) or
                    rotation_distance(transform, old) >
                    np.deg2rad(self.config.get('station_motion_tolerance_deg', 2.0)))
            if moved and not slot.get('allow_relocalization', False):
                previous['movement_detected'] = True
                previous['last_rejected_observation_stamp'] = float(stamp)
            else:
                records[key] = {
                    'base_marker': serialize_transform(transform),
                    'stamp': float(stamp),
                    'reprojection_error_px': float(reprojection_error),
                    'movement_detected': False,
                }
            self.storage.save(self.data)

    def _station_pose(self, slot_id, require_fresh=False):
        saved = self.data.get('station_records', {}).get(str(int(slot_id)))
        if saved and saved.get('movement_detected', False):
            raise TaskError('slot {} movement was detected; revalidation required'.format(slot_id))
        try:
            return self.markers.latest(slot_id)
        except Exception:
            slot = self._slot(slot_id)
            if (require_fresh or not slot.get('station_fixed_verified', False)
                    or not saved):
                raise TaskError('slot {} has no fresh stable pose'.format(slot_id))
            return (saved['stamp'], deserialize_transform(saved['base_marker']),
                    saved['reprojection_error_px'])

    def _sync_world_objects(self):
        for slot_id, slot in sorted(self.slots.items()):
            if not slot.get('enabled', False):
                continue
            if self.data.get('held_slot_id') == slot_id:
                continue
            _, marker, _ = self._station_pose(slot_id)
            marker_object = self._configured_transform(slot, 'marker_to_object')
            self.motion.add_world_object(
                slot_id, compose(marker, marker_object), slot)

    def _transition(self, state, pending=None, successful=None):
        self.data['state'] = state
        self.data['pending_action'] = pending
        if successful is not None:
            self.data['last_successful_action'] = successful
        self.storage.save(self.data)

    def _slot(self, slot_id):
        slot_id = int(slot_id)
        if slot_id not in self.slots:
            raise TaskError('unknown slot {}'.format(slot_id))
        self.markers.validate_slot(slot_id)
        return self.slots[slot_id]

    @staticmethod
    def _configured_transform(config, key):
        value = config.get(key)
        if not value or not value.get('verified', False):
            raise TaskError('{} is not calibrated and verified'.format(key))
        return deserialize_transform(value)

    def _geometry(self, slot_id, marker_transform):
        slot = self._slot(slot_id)
        marker_object = self._configured_transform(slot, 'marker_to_object')
        marker_tcp = self._configured_transform(slot, 'marker_to_pick_tcp')
        direction = slot.get('approach_direction_marker')
        if not isinstance(direction, list) or len(direction) != 3:
            raise TaskError('approach direction is not configured')
        approach = float(slot.get('approach_distance_m', 0))
        retreat = float(slot.get('retreat_distance_m', 0))
        if approach <= 0 or retreat <= 0:
            raise TaskError('safe approach/retreat distances are not configured')
        base_object = compose(marker_transform, marker_object)
        base_pick_tcp = compose(marker_transform, marker_tcp)
        base_pre_pick = offset_along_marker(base_pick_tcp, direction, -approach)
        base_lift = offset_along_marker(base_pick_tcp, direction, -retreat)
        return slot, marker_object, marker_tcp, base_object, base_pick_tcp, base_pre_pick, base_lift

    @staticmethod
    def _snapshot_record(snapshot):
        return {'joints': list(snapshot['joints']),
                'tcp': serialize_transform(snapshot['tcp']),
                'timestamp': float(snapshot['timestamp'])}

    def _feedback_holding(self, supervised_confirm):
        try:
            return self.hold_feedback.holding()
        except Exception:
            if (supervised_confirm and
                    self.config.get('allow_supervised_confirmation', False)):
                return True
            raise

    def _feedback_released(self, supervised_confirm):
        try:
            return self.hold_feedback.released()
        except Exception:
            if (supervised_confirm and
                    self.config.get('allow_supervised_confirmation', False)):
                return True
            raise

    def _check_stop(self):
        if self.stop_requested.is_set():
            raise TaskError('stop requested')

    def status(self):
        with self.lock:
            return copy.deepcopy(self.data)

    def status_report(self):
        report = self.status()
        try:
            snapshot = self.motion.snapshot()
            report['current_robot'] = self._snapshot_record(snapshot)
        except Exception as exc:
            report['current_robot'] = {'available': False, 'reason': str(exc)}
        try:
            report['tool_power_feedback'] = self.voltage.power_status()
        except Exception as exc:
            report['tool_power_feedback'] = {'available': False, 'reason': str(exc)}
        try:
            report['object_feedback'] = self.hold_feedback.feedback_status()
        except Exception as exc:
            report['object_feedback'] = {'available': False, 'reason': str(exc)}
        return report

    def preview_pick(self, slot_id):
        with self.lock:
            if self.data['state'] != 'IDLE':
                raise TaskError('PICK requires IDLE, current state is {}'.format(self.data['state']))
            self.safety.check_preview()
            _, marker, reprojection = self._station_pose(slot_id, require_fresh=True)
            slot, _, _, base_object, base_pick, base_pre, base_lift = self._geometry(slot_id, marker)
            start = self.motion.snapshot()
            self._sync_world_objects()
            plan = self.motion.preview_sequence([base_pre, base_pick, base_lift], start['joints'])
            return {'slot_id': int(slot_id), 'reprojection_error_px': reprojection,
                    'object_pose': serialize_transform(base_object),
                    'pick_tcp': serialize_transform(base_pick),
                    'pre_pick_tcp': serialize_transform(base_pre),
                    'lift_tcp': serialize_transform(base_lift), 'plan': plan,
                    'warning': 'planning preview only; no motion or voltage command was sent'}

    def pick(self, slot_id, execute=False, supervised_confirm=False):
        if not execute:
            return self.preview_pick(slot_id)
        with self.lock:
            power_on = False
            touch_allowed = False
            try:
                self.stop_requested.clear()
                if self.data['state'] != 'IDLE':
                    raise TaskError('PICK requires IDLE, current state is {}'.format(self.data['state']))
                self.safety.check_execute(execute)
                self._transition('DETECT_MARKER', 'detect slot {}'.format(slot_id))
                marker_stamp, marker, reprojection = self._station_pose(
                    slot_id, require_fresh=True)
                (slot, marker_object, marker_tcp, base_object, base_pick,
                 base_pre, base_lift) = self._geometry(slot_id, marker)
                start = self.motion.snapshot()
                if len(start['joints']) != 7:
                    raise TaskError('joint snapshot does not contain seven joints')
                # The grasp relation must correspond to the configured grasp TCP.
                tcp_object = compose(inverse(base_pick), base_object)
                self.data['record'] = {
                    'slot_id': int(slot_id), 'marker_stamp': float(marker_stamp),
                    'reprojection_error_px': float(reprojection),
                    'pick_start': self._snapshot_record(start),
                    'base_marker_pick': serialize_transform(marker),
                    'marker_object_original': serialize_transform(marker_object),
                    'marker_tcp_pick': serialize_transform(marker_tcp),
                    'base_object_original': serialize_transform(base_object),
                    'tcp_object': serialize_transform(tcp_object),
                }
                self.storage.save(self.data)
                self._transition('PLAN_PICK', 'plan pick sequence')
                self._sync_world_objects()
                self.motion.preview_sequence([base_pre], start['joints'])
                self._check_stop()
                self._transition('MOVE_PRE_PICK', 'move to pre-pick')
                self.motion.move_pose(base_pre, 'pre_pick')
                self._check_stop()
                self.motion.set_target_contact(slot_id, True, slot)
                touch_allowed = True
                self._transition('MOVE_PICK', 'collision-checked approach')
                self.motion.move_pose(base_pick, 'pick')
                self.motion.verify_pose(
                    base_pick, self.config.get('target_pose_tolerance_m', 0.004),
                    np.deg2rad(self.config.get('target_pose_tolerance_deg', 2.0)))
                self._check_stop()
                self._transition('POWER_ON', 'set tool 24V')
                self.voltage.set_voltage(24)
                power_on = True
                self.data['power_commanded_voltage'] = 24
                self.storage.save(self.data)
                self._transition('VERIFY_HOLD', 'verify independent hold feedback', 'power_on_ack')
                if not self._feedback_holding(supervised_confirm):
                    raise TaskError('independent feedback reports object not held')
                self.data['hold_confirmed'] = True
                self.storage.save(self.data)
                self.motion.attach(slot_id, base_object, slot)
                self.motion.set_target_contact(slot_id, False, slot)
                touch_allowed = False
                self._transition('LIFT_OBJECT', 'lift attached object', 'hold_confirmed')
                self.motion.move_pose(base_lift, 'lift')
                self._check_stop()
                self._transition('RETURN_PICK_START', 'replan to PICK start', 'lift_complete')
                self.motion.move_joints(start['joints'], 'pick_start')
                self.motion.verify(start, self.config.get('joint_return_tolerance_rad', 0.02),
                                   self.config.get('tcp_return_tolerance_m', 0.005),
                                   np.deg2rad(self.config.get('tcp_return_tolerance_deg', 2.0)))
                self.data['held_slot_id'] = int(slot_id)
                self.data['last_error'] = ''
                self._transition('HOLDING', None, 'pick_complete')
                return self.status_unlocked()
            except Exception as exc:
                if touch_allowed:
                    try:
                        self.motion.set_target_contact(slot_id, False, slot)
                    except Exception:
                        pass
                self.data['last_error'] = str(exc)
                self.data['state'] = 'RECOVERY_REQUIRED' if power_on else 'FAULT'
                self.data['pending_action'] = 'operator recovery; power state preserved'
                self.storage.save(self.data)
                raise TaskError(str(exc))

    def preview_return(self, slot_id):
        with self.lock:
            result = self._prepare_return(slot_id)
            self.safety.check_preview()
            start = self.motion.snapshot()
            self._sync_world_objects()
            plan = self.motion.preview_sequence(
                [result['pre_place'], result['place'], result['retreat']],
                start['joints'])
            return {'slot_id': int(slot_id), 'place_tcp': serialize_transform(result['place']),
                    'pre_place_tcp': serialize_transform(result['pre_place']),
                    'retreat_tcp': serialize_transform(result['retreat']),
                    'object_pose': serialize_transform(result['object']), 'plan': plan,
                    'warning': 'planning preview only; no motion or voltage command was sent'}

    def _prepare_return(self, slot_id):
        slot_id = int(slot_id)
        if self.data['state'] != 'HOLDING':
            raise TaskError('RETURN requires HOLDING, current state is {}'.format(self.data['state']))
        if self.data.get('held_slot_id') != slot_id:
            raise TaskError('held object came from slot {}, not {}'.format(
                self.data.get('held_slot_id'), slot_id))
        record = self.data.get('record')
        if not record or record.get('slot_id') != slot_id:
            raise TaskError('persistent PICK record is missing or mismatched')
        slot = self._slot(slot_id)
        original_marker = deserialize_transform(record['base_marker_pick'])
        base_object = deserialize_transform(record['base_object_original'])
        try:
            _, current_marker, _ = self._station_pose(slot_id)
            translation_change = np.linalg.norm(
                current_marker[:3, 3] - original_marker[:3, 3])
            rotation_change = rotation_distance(current_marker, original_marker)
            if translation_change > self.config.get('station_motion_tolerance_m', 0.005) or \
                    rotation_change > np.deg2rad(self.config.get('station_motion_tolerance_deg', 2.0)):
                if not slot.get('allow_relocalization', False):
                    raise TaskError('station moved; relocalization is not authorized')
                marker_object = deserialize_transform(record['marker_object_original'])
                base_object = compose(current_marker, marker_object)
        except TaskError:
            raise
        except Exception:
            if not slot.get('station_fixed_verified', False):
                raise TaskError('marker unavailable and station fixation is not verified')
        tcp_object = deserialize_transform(record['tcp_object'])
        place = compose(base_object, inverse(tcp_object))
        direction = slot['approach_direction_marker']
        pre = offset_along_marker(place, direction, -float(slot['approach_distance_m']))
        retreat = offset_along_marker(place, direction, -float(slot['retreat_distance_m']))
        return {'slot': slot, 'object': base_object, 'place': place,
                'pre_place': pre, 'retreat': retreat}

    def return_object(self, slot_id, execute=False, supervised_confirm=False):
        if not execute:
            return self.preview_return(slot_id)
        with self.lock:
            power_off = False
            try:
                self.stop_requested.clear()
                self.safety.check_execute(execute)
                result = self._prepare_return(slot_id)
                start = self.motion.snapshot()
                self.data['record']['return_start'] = self._snapshot_record(start)
                self.storage.save(self.data)
                self._transition('PLAN_RETURN', 'plan return sequence')
                self._sync_world_objects()
                self.motion.preview_sequence(
                    [result['pre_place']],
                    start['joints'])
                self._check_stop()
                self._transition('MOVE_PRE_PLACE', 'move to pre-place')
                self.motion.move_pose(result['pre_place'], 'pre_place')
                self._check_stop()
                self._transition('MOVE_PLACE', 'collision-checked place approach')
                self.motion.move_pose(result['place'], 'place')
                self.motion.verify_pose(
                    result['place'], self.config.get('target_pose_tolerance_m', 0.004),
                    np.deg2rad(self.config.get('target_pose_tolerance_deg', 2.0)))
                self._check_stop()
                self._transition('POWER_OFF', 'set tool 0V')
                self.voltage.set_voltage(0)
                power_off = True
                self.data['power_commanded_voltage'] = 0
                self.storage.save(self.data)
                self._transition('VERIFY_RELEASE', 'verify independent release feedback', 'power_off_ack')
                if not self._feedback_released(supervised_confirm):
                    raise TaskError('independent feedback reports object still held')
                self.motion.detach(slot_id, result['object'], result['slot'])
                self.data['hold_confirmed'] = False
                self.storage.save(self.data)
                self._transition('RETREAT', 'retreat after confirmed release', 'release_confirmed')
                self.motion.move_pose(result['retreat'], 'retreat')
                self._check_stop()
                self._transition('RETURN_RETURN_START', 'replan to RETURN start', 'retreat_complete')
                self.motion.move_joints(start['joints'], 'return_start')
                self.motion.verify(start, self.config.get('joint_return_tolerance_rad', 0.02),
                                   self.config.get('tcp_return_tolerance_m', 0.005),
                                   np.deg2rad(self.config.get('tcp_return_tolerance_deg', 2.0)))
                archived = copy.deepcopy(self.data)
                self.storage.archive(archived)
                station_records = copy.deepcopy(self.data.get('station_records', {}))
                self.data = self.storage.default_state()
                self.data['station_records'] = station_records
                self.data['last_successful_action'] = 'return_complete'
                self.storage.save(self.data)
                return self.status_unlocked()
            except Exception as exc:
                self.data['last_error'] = str(exc)
                self.data['state'] = 'RECOVERY_REQUIRED'
                self.data['pending_action'] = (
                    'verify physical object/power state; no automatic voltage action was taken')
                if power_off:
                    self.data['hold_confirmed'] = False
                self.storage.save(self.data)
                raise TaskError(str(exc))

    def stop(self):
        # Signal/cancel before waiting for the task lock: MoveIt execute() is
        # blocking and must be interruptible by a concurrent service request.
        self.stop_requested.set()
        self.motion.stop()
        with self.lock:
            if self.data['state'] != 'RECOVERY_REQUIRED':
                self.data['state'] = 'SAFE_STOP'
            self.data['pending_action'] = 'operator inspection and explicit recovery'
            self.data['last_error'] = 'stop requested; tool voltage was not changed'
            self.storage.save(self.data)
            return self.status_unlocked()

    def recover(self, target, slot_id=None, supervised_confirm=False):
        with self.lock:
            if self.data['state'] not in ('FAULT', 'SAFE_STOP', 'RECOVERY_REQUIRED'):
                raise TaskError('recovery is only allowed from an error/recovery state')
            if not (supervised_confirm and
                    self.config.get('allow_supervised_recovery', False)):
                raise TaskError('supervised recovery is disabled or not confirmed')
            if target == 'holding':
                if slot_id is None or int(slot_id) not in self.slots:
                    raise TaskError('recovery to HOLDING requires a valid slot')
                if not self.hold_feedback.holding():
                    raise TaskError('independent feedback does not confirm holding')
                self.data['held_slot_id'] = int(slot_id)
                self.data['hold_confirmed'] = True
                self.data['last_error'] = ''
                self._transition('HOLDING', None, 'supervised_recovery_holding')
            elif target == 'idle':
                if not self.hold_feedback.released():
                    raise TaskError('independent feedback does not confirm released state')
                self.storage.archive(copy.deepcopy(self.data))
                station_records = copy.deepcopy(self.data.get('station_records', {}))
                self.data = self.storage.default_state()
                self.data['station_records'] = station_records
                self.data['last_successful_action'] = 'supervised_recovery_idle'
                self.storage.save(self.data)
            else:
                raise TaskError('recovery target must be holding or idle')
            return self.status_unlocked()

    def status_unlocked(self):
        return copy.deepcopy(self.data)
