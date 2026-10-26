#!/usr/bin/env python3
"""Persistent ROS service server for four-station PICK/RETURN commands."""

import json
import os
import sys
import threading
import time

import numpy as np
import rospy
import rosnode
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, UInt8

from rm75_qr_task.adapters import (MockHoldFeedback, MockMotionAdapter,
                                   MockVoltageAdapter, MoveItMotionAdapter,
                                   RosHoldFeedback, RosVoltageAdapter)
from rm75_qr_task.marker_manager import MarkerManager
from rm75_qr_task.pose_manager import transform_matrix
from rm75_qr_task.safety_checker import SafetyChecker
from rm75_qr_task.srv import TaskCommand, TaskCommandResponse
from rm75_qr_task.task_state_machine import TaskError, TaskStateMachine
from rm75_qr_task.task_storage import TaskStorage


def load_yaml(path):
    with open(os.path.expanduser(path), 'r') as stream:
        return yaml.safe_load(stream)


class RuntimeMonitor(object):
    def __init__(self, require_map=True, timeout=0.5):
        self.require_map = bool(require_map)
        self.timeout = float(timeout)
        self.joint_wall = None
        self.map_wall = None
        self.joint_names = set()
        self.lock = threading.Lock()
        rospy.Subscriber('/joint_states', JointState, self._joint, queue_size=1)
        rospy.Subscriber('/mapping/processed', Header, self._map, queue_size=1)

    def _joint(self, message):
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        required = {'joint{}'.format(index) for index in range(1, 8)}
        if not required.issubset(values) or not all(np.isfinite(values[name]) for name in required):
            return
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        if age < 0 or age > self.timeout:
            return
        with self.lock:
            self.joint_wall = time.monotonic()
            self.joint_names = set(values)

    def _map(self, message):
        age = (rospy.Time.now() - message.stamp).to_sec()
        if 0 <= age <= self.timeout:
            with self.lock:
                self.map_wall = time.monotonic()

    def check(self):
        now = time.monotonic()
        with self.lock:
            if self.joint_wall is None or now - self.joint_wall > self.timeout:
                return False, 'seven-joint feedback is missing or stale'
            if self.require_map and (self.map_wall is None or
                                     now - self.map_wall > self.timeout):
                return False, 'planning-scene cloud heartbeat is missing or stale'
        try:
            names = set(rosnode.get_node_names())
        except Exception as exc:
            return False, 'cannot audit command sources: {}'.format(exc)
        conflicts = sorted(name for name in names if
                           name in ('/rm_control', '/rm75_touch_teleop') or
                           'touch_teleop' in name)
        if conflicts:
            return False, 'competing command source present: ' + ', '.join(conflicts)
        return True, ''


class TaskServer(object):
    def __init__(self):
        markers_config = load_yaml(rospy.get_param('~markers_config'))
        task_config = load_yaml(rospy.get_param('~task_config'))
        safety_config = load_yaml(rospy.get_param('~safety_config'))
        slots = {int(item['id']): item for item in markers_config['slots']}
        self.manager = MarkerManager(
            slots,
            stability_samples=markers_config.get('stability_samples', 5),
            max_age=markers_config.get('max_age_s', 0.5),
            max_translation_jump=markers_config.get('max_translation_jump_m', 0.006),
            max_rotation_jump_deg=markers_config.get('max_rotation_jump_deg', 2.0),
            max_reprojection_error=markers_config.get('max_reprojection_error_px', 2.0),
            clock=lambda: rospy.Time.now().to_sec())
        use_mock = rospy.get_param('~use_mock_adapters', False)
        if use_mock:
            motion = MockMotionAdapter()
            voltage = MockVoltageAdapter()
            feedback = MockHoldFeedback(
                holding=rospy.get_param('~mock_holding', False),
                released=rospy.get_param('~mock_released', True))
            runtime_check = lambda: (True, '')
        else:
            motion = MoveItMotionAdapter(
                group_name=task_config.get('move_group', 'arm'),
                tcp_link=task_config.get('tcp_link', 'grasp_tcp'),
                base_frame=markers_config.get('base_frame', 'base_link'))
            voltage = RosVoltageAdapter(rospy, task_config.get('voltage_timeout_s', 5.0))
            feedback = RosHoldFeedback(
                rospy, task_config.get('hold_feedback_topic',
                                       '/qr_task/object_held'),
                task_config.get('feedback_timeout_s', 0.5))
            runtime = RuntimeMonitor(task_config.get('require_map', True),
                                     task_config.get('feedback_timeout_s', 0.5))
            runtime_check = runtime.check
            self.runtime = runtime
        safety = SafetyChecker(
            execution_authorized=rospy.get_param('~execution_authorized', False),
            simulation=rospy.get_param('~simulation', False),
            verifications=safety_config.get('verifications', {}),
            runtime_check=runtime_check)
        storage = TaskStorage(
            rospy.get_param('~state_file', task_config['state_file']),
            rospy.get_param('~archive_dir', task_config['archive_dir']))
        self.machine = TaskStateMachine(
            slots, self.manager, motion, voltage, feedback, storage, safety,
            config=task_config)
        self.target_pub = rospy.Publisher('/qr_task/target_slot', UInt8,
                                          queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/qr_task/task_status', String,
                                          queue_size=1, latch=True)
        rospy.Subscriber('/qr_task/marker_observation', String,
                         self._observation, queue_size=20)
        self.service = rospy.Service('/qr_task/command', TaskCommand,
                                     self._command)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._publish_status)
        self._publish_status(None)

    def _observation(self, message):
        try:
            value = json.loads(message.data)
            transform = transform_matrix(value['translation'],
                                         value['quaternion_xyzw'])
            self.manager.add(value['slot_id'], value['stamp'], transform,
                             value['reprojection_error_px'])
            if self.manager.stable(value['slot_id']):
                self.machine.record_station_pose(
                    value['slot_id'], value['stamp'], transform,
                    value['reprojection_error_px'])
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Rejected marker observation: %s', exc)

    def _publish_status(self, _event):
        self.status_pub.publish(String(data=json.dumps(
            self.machine.status_report(), sort_keys=True)))

    def _command(self, request):
        command = request.command.strip().lower()
        try:
            if command in ('pick', 'preview'):
                self.target_pub.publish(UInt8(data=request.slot_id))
                details = (self.machine.preview_pick(request.slot_id)
                           if command == 'preview' else
                           self.machine.pick(request.slot_id, request.execute,
                                             request.supervised_confirm))
            elif command == 'return':
                self.target_pub.publish(UInt8(data=request.slot_id))
                details = self.machine.return_object(
                    request.slot_id, request.execute, request.supervised_confirm)
            elif command == 'status':
                details = self.machine.status_report()
            elif command == 'stop':
                details = self.machine.stop()
            elif command == 'recover_holding':
                details = self.machine.recover(
                    'holding', request.slot_id, request.supervised_confirm)
            elif command == 'recover_idle':
                details = self.machine.recover(
                    'idle', supervised_confirm=request.supervised_confirm)
            else:
                raise TaskError('unknown command {}'.format(command))
            self._publish_status(None)
            return TaskCommandResponse(True, details.get('state', self.machine.status()['state']),
                                       'accepted', json.dumps(details, sort_keys=True))
        except Exception as exc:
            self._publish_status(None)
            return TaskCommandResponse(False, self.machine.status()['state'],
                                       str(exc), json.dumps(self.machine.status(), sort_keys=True))


def main():
    rospy.init_node('rm75_qr_task_server')
    try:
        TaskServer()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('%s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
