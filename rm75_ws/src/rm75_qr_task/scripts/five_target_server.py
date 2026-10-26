#!/usr/bin/env python3
"""ROS façade for fail-closed common-marker five-target positioning."""

import copy
import hashlib
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rosnode
import rospy
import yaml
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from rm75_qr_task.adapters import RosVoltageAdapter
from rm75_qr_task.controller_pose import rpy_matrix
from rm75_qr_task.five_targets import (CalibrationStore, FiveTargetComputer,
                                       FiveTargetError, SLOT_IDS,
                                       build_calibration,
                                       joint_target_reached,
                                       observation_joint_target, pose_error)
from rm75_qr_task.pose_manager import (compose, rotation_distance,
                                       deserialize_transform,
                                       matrix_quaternion, serialize_transform, transform_matrix,
                                       transform_parts)
from rm75_qr_task.srv import TaskCommand, TaskCommandResponse


def load_yaml(path):
    with open(os.path.expanduser(path), 'r') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise FiveTargetError('configuration must be a YAML mapping')
    return value


class FiveTargetServer(object):
    def __init__(self):
        self.config = load_yaml(rospy.get_param('~target_config'))
        self.tool_config_path = os.path.expanduser(rospy.get_param(
            '~tool_config', ''))
        if self.config.get('mode') != 'single_marker_five_targets':
            raise FiveTargetError('target config mode is not single_marker_five_targets')
        self.execution_authorized = bool(
            rospy.get_param('~execution_authorized', False))
        self.trajectory_execution_authorized = bool(
            rospy.get_param('~trajectory_execution_authorized', False))
        self.tool_voltage_authorized = bool(
            rospy.get_param('~tool_voltage_authorized', False))
        self.supervised_commissioning_authorized = bool(
            rospy.get_param('~supervised_commissioning_authorized', False))
        self.store = CalibrationStore(rospy.get_param(
            '~calibration_file', '/root/.ros/rm75_qr_task/single_marker_five_targets.yaml'))
        self.lock = threading.RLock()
        self.observation = None
        self.observation_wall = None
        self.joint_wall = None
        self.joint_positions = None
        self.joint_names = None
        self.controller_pose = None
        self.controller_pose_wall = None
        self.computer = None
        self.calibration_error = ''
        self.selected_slot = 1
        self.last_targets = None
        self.last_plan = None
        self.cancel_requested = threading.Event()
        self.group = None
        self.holding_state_param = str(rospy.get_param(
            '~holding_state_param', '/qr_task/five_targets/holding_slot'))
        saved_holding = int(rospy.get_param(self.holding_state_param, 0))
        self.holding_slot = saved_holding if saved_holding in SLOT_IDS else None
        self.staged_pickup = None
        self.configured_plan = None
        self.configured_pick_status = {
            'status': 'IDLE', 'step': 0, 'step_name': '等待执行',
            'slot_id': None, 'grasp_confirmed': False,
        }
        self.voltage = RosVoltageAdapter(
            rospy, self.config.get('voltage_timeout_s', 5.0))
        self._load_calibration()

        self.status_pub = rospy.Publisher('/qr_task/five_targets/status', String,
                                          queue_size=1, latch=True)
        self.targets_pub = rospy.Publisher('/qr_task/five_targets/targets', String,
                                           queue_size=1, latch=True)
        self.pose_array_pub = rospy.Publisher(
            '/qr_task/five_targets/tcp_poses', PoseArray, queue_size=1, latch=True)
        self.selected_tcp_pub = rospy.Publisher(
            '/qr_task/five_targets/selected_tcp', PoseStamped,
            queue_size=1, latch=True)
        self.selected_link7_pub = rospy.Publisher(
            '/qr_task/five_targets/selected_link7', PoseStamped,
            queue_size=1, latch=True)
        rospy.Subscriber('/qr_task/marker_observation', String,
                         self._observation, queue_size=20)
        rospy.Subscriber('/joint_states', JointState, self._joints, queue_size=1)
        try:
            from rm_msgs.msg import Joint_PoseEuler
            rospy.Subscriber('/rm_driver/Udp_Joint_PoseEuler', Joint_PoseEuler,
                             self._controller, queue_size=1)
        except ImportError:
            rospy.logwarn('rm_msgs/Joint_PoseEuler unavailable; controller audit disabled')
        self.service = rospy.Service('/qr_task/five_targets/command', TaskCommand,
                                     self._command)
        self.timer = rospy.Timer(rospy.Duration(0.5), self._timer)
        self._publish_status()

    def _expected(self):
        result = {key: self.config.get(key) for key in
                  ('base_frame', 'link7_frame', 'tcp_frame', 'calibration_version')}
        result['handeye'] = self.config.get('handeye')
        return result

    def _load_calibration(self):
        try:
            calibration = self.store.load(self._expected())
            self.computer = FiveTargetComputer(
                calibration, self.config.get('max_marker_age_s', 0.5),
                clock=lambda: rospy.Time.now().to_sec())
            self.calibration_error = ''
        except Exception as exc:
            self.computer = None
            self.calibration_error = str(exc)

    def _observation(self, message):
        try:
            value = json.loads(message.data)
            marker_id = int(value.get('marker_id', value.get('slot_id', -1)))
            if marker_id != int(self.config.get('marker_id', 0)):
                return
            transform = transform_matrix(value['translation'],
                                         value['quaternion_xyzw'])
            stamp = float(value['stamp'])
            age = rospy.Time.now().to_sec() - stamp
            if age < 0 or age > float(self.config.get('max_marker_age_s', 0.5)):
                return
            with self.lock:
                self.observation = {
                    'marker_id': marker_id, 'stamp': stamp, 'transform': transform,
                    'reprojection_error_px': float(value.get(
                        'reprojection_error_px', math.nan)),
                    'pixel_size': float(value.get('pixel_size', math.nan)),
                    'stability': copy.deepcopy(value.get('stability', {})),
                }
                self.observation_wall = time.monotonic()
            self._compute_and_publish()
        except (KeyError, TypeError, ValueError) as exc:
            rospy.logwarn_throttle(2.0, 'Rejected common-marker observation: %s', exc)

    def _joints(self, message):
        required = {'joint{}'.format(index) for index in range(1, 8)}
        values = dict(zip(message.name, message.position))
        if required.issubset(values) and all(np.isfinite(values[name]) for name in required):
            age = (rospy.Time.now() - message.header.stamp).to_sec()
            if 0 <= age <= float(self.config.get('joint_feedback_max_age_s', 0.25)):
                with self.lock:
                    self.joint_wall = time.monotonic()
                    self.joint_names = [
                        'joint{}'.format(index) for index in range(1, 8)]
                    self.joint_positions = [
                        float(values[name]) for name in self.joint_names]

    def _controller(self, message):
        values = list(message.position) + list(message.euler)
        if len(values) == 6 and all(np.isfinite(values)):
            with self.lock:
                self.controller_pose = values
                self.controller_pose_wall = time.monotonic()

    @staticmethod
    def _pose(transform, frame, stamp=None):
        translation, quaternion = transform_parts(transform)
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = stamp or rospy.Time.now()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = translation
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = quaternion
        return message

    def _fresh_observation(self, after_wall=None):
        with self.lock:
            observation = copy.deepcopy(self.observation)
            observation_wall = self.observation_wall
        if observation is None:
            raise FiveTargetError('no stable common marker ID 0 observation')
        if after_wall is not None and (
                observation_wall is None or observation_wall <= float(after_wall)):
            raise FiveTargetError('no new marker observation after camera observation move')
        age = rospy.Time.now().to_sec() - observation['stamp']
        if age < 0 or age > float(self.config.get('max_marker_age_s', 0.5)):
            raise FiveTargetError('common marker observation is stale')
        minimum_pixels = float(self.config.get('minimum_marker_pixels', 0.0))
        pixels = observation.get('pixel_size', math.nan)
        if not np.isfinite(pixels) or pixels < minimum_pixels:
            raise FiveTargetError(
                'marker is too small in image ({:.1f}px < {:.1f}px)'.format(
                    pixels, minimum_pixels))
        return observation

    def _fresh_joints(self):
        with self.lock:
            wall = self.joint_wall
            names = copy.deepcopy(self.joint_names)
            positions = copy.deepcopy(self.joint_positions)
        maximum_age = float(self.config.get('joint_feedback_max_age_s', 0.25))
        if (wall is None or names is None or positions is None or
                time.monotonic() - wall > maximum_age):
            raise FiveTargetError('fresh seven-joint feedback is unavailable')
        return names, positions

    def _compute(self, slot_id=None):
        if self.computer is None:
            raise FiveTargetError('坐标标定未完成: {}'.format(self.calibration_error))
        observation = self._fresh_observation()
        target_marker = observation['transform']
        marker_reference_error = None
        target_source = 'live_marker_observation'
        if self.config.get('target_pose_source') == (
                'calibrated_base_marker_reference'):
            reference = deserialize_transform(
                self.computer.calibration['base_marker_reference'])
            marker_reference_error = pose_error(reference, target_marker)
            maximum_translation = float(self.config.get(
                'marker_reference_gate_translation_m', 0.006))
            maximum_rotation = np.deg2rad(float(self.config.get(
                'marker_reference_gate_rotation_deg', 12.0)))
            if marker_reference_error['translation_m'] > maximum_translation:
                raise FiveTargetError(
                    'marker moved beyond calibrated reference ({:.3f}mm)'.format(
                        marker_reference_error['translation_m'] * 1000.0))
            if marker_reference_error['rotation_rad'] > maximum_rotation:
                raise FiveTargetError(
                    'marker rotated beyond calibrated reference ({:.3f}deg)'.format(
                        np.rad2deg(marker_reference_error['rotation_rad'])))
            target_marker = reference
            target_source = 'calibrated_base_marker_reference'
        result = self.computer.compute_all(
            observation['marker_id'], observation['stamp'], target_marker)
        # Freeze the actual live observation for the pre-execution movement
        # check even when the target itself comes from the fixed calibration.
        result['base_marker'] = observation['transform'].copy()
        result['target_pose_source'] = target_source
        if marker_reference_error is not None:
            result['marker_reference_error'] = marker_reference_error
        result['observation_quality'] = {
            key: copy.deepcopy(observation.get(key))
            for key in ('reprojection_error_px', 'pixel_size', 'stability')}
        if slot_id is not None:
            slot_id = int(slot_id)
            if slot_id not in SLOT_IDS:
                raise FiveTargetError('tool slot must be in 1..5')
            result['selected_slot'] = slot_id
        return result

    def _serialize_targets(self, result):
        return {
            'ok': True,
            'marker_id': result['marker_id'],
            'marker_stamp': result['marker_stamp'],
            'marker_age_s': result['marker_age_s'],
            'calibration_version': result['calibration_version'],
            'selected_slot': result.get('selected_slot', self.selected_slot),
            'observation_quality': result.get('observation_quality', {}),
            'target_pose_source': result.get('target_pose_source'),
            'marker_reference_error': copy.deepcopy(
                result.get('marker_reference_error')),
            'targets': [
                {'slot_id': slot_id, 'name': 'Tool{}'.format(slot_id),
                 'tcp': serialize_transform(result['targets'][slot_id]['tcp']),
                 'link7': serialize_transform(result['targets'][slot_id]['link7'])}
                for slot_id in SLOT_IDS
            ],
        }

    def _publish_targets(self, result):
        value = self._serialize_targets(result)
        self.targets_pub.publish(String(data=json.dumps(value, sort_keys=True)))
        array = PoseArray()
        array.header.frame_id = str(self.config.get('base_frame', 'base_link'))
        array.header.stamp = rospy.Time.from_sec(result['marker_stamp'])
        for slot_id in SLOT_IDS:
            array.poses.append(self._pose(result['targets'][slot_id]['tcp'],
                                          array.header.frame_id,
                                          array.header.stamp).pose)
        self.pose_array_pub.publish(array)
        selected = int(result.get('selected_slot', self.selected_slot))
        self.selected_tcp_pub.publish(self._pose(
            result['targets'][selected]['tcp'], array.header.frame_id, array.header.stamp))
        self.selected_link7_pub.publish(self._pose(
            result['targets'][selected]['link7'], array.header.frame_id, array.header.stamp))

    def _compute_and_publish(self):
        try:
            result = self._compute(self.selected_slot)
            with self.lock:
                self.last_targets = result
            self._publish_targets(result)
        except Exception:
            # Status reports the reason; a detection must not turn into a stale target.
            with self.lock:
                self.last_targets = None
        self._publish_status()

    def _calibration_status(self):
        pose_reference = self.config.get('controller_pose_definition', {}).get(
            'pose_reference')
        link7_to_tcp_required = pose_reference != 'link7'
        status = {
            'loaded': self.computer is not None,
            'error': self.calibration_error,
            'file': self.store.path,
            'expected_version': str(self.config.get('calibration_version', '')),
            'controller_definition_confirmed': bool(
                self.config.get('controller_pose_definition', {}).get(
                    'definition_confirmed', False)),
            'link7_to_tcp_required': link7_to_tcp_required,
            'link7_to_tcp_verified': (
                not link7_to_tcp_required or bool(
                    self.config.get('link7_to_tcp', {}).get('verified', False))),
            'marker_size_measurement_verified': bool(
                self.config.get('marker_size_measurement_verified', False)),
        }
        if self.computer is not None:
            status['version'] = self.computer.calibration['version']
            status['camera_observation'] = copy.deepcopy(
                self.computer.calibration.get('camera_observation'))
            status['physical_validation'] = copy.deepcopy(
                self.computer.calibration.get('physical_validation', {}))
        return status

    def _status(self):
        try:
            observation = self._fresh_observation()
            vision = {'ok': True, 'marker_id': observation['marker_id'],
                      'stamp': observation['stamp'],
                      'pixel_size': observation['pixel_size'],
                      'reprojection_error_px': observation['reprojection_error_px'],
                      'stability': observation['stability']}
        except Exception as exc:
            vision = {'ok': False, 'reason': str(exc)}
        return {
            'mode': 'single_marker_five_targets',
            'selected_slot': self.selected_slot,
            'calibration': self._calibration_status(),
            'vision': vision,
            'targets_valid': self.last_targets is not None,
            'joint_feedback_fresh': self.joint_wall is not None and
            time.monotonic() - self.joint_wall <= float(
                self.config.get('joint_feedback_max_age_s', 0.25)),
            'controller_pose_feedback_fresh': self.controller_pose_wall is not None and
            time.monotonic() - self.controller_pose_wall <= 0.5,
            'execution_authorized': self.execution_authorized,
            'trajectory_execution_authorized': self.trajectory_execution_authorized,
            'tool_voltage_authorized': self.tool_voltage_authorized,
            'supervised_commissioning_authorized': (
                self.supervised_commissioning_authorized),
            'holding_slot': self.holding_slot,
            'staged_pickup_slot': (
                None if self.staged_pickup is None else
                self.staged_pickup['slot_id']),
            'configured_pick': copy.deepcopy(self.configured_pick_status),
            'environment_collision_geometry_required': False,
            'execution_ready': self._execution_missing() == [],
            'execution_rejections': self._execution_missing(),
        }

    def _publish_status(self):
        if hasattr(self, 'status_pub'):
            self.status_pub.publish(String(data=json.dumps(self._status(), sort_keys=True)))

    def _set_configured_step(self, status, step, step_name, slot_id=None,
                             error=None, grasp_confirmed=False):
        self.configured_pick_status = {
            'status': str(status), 'step': int(step),
            'step_name': str(step_name),
            'slot_id': None if slot_id is None else int(slot_id),
            'grasp_confirmed': bool(grasp_confirmed),
            'updated_at': rospy.Time.now().to_sec(),
        }
        if error:
            self.configured_pick_status['error'] = str(error)
        self._publish_status()

    def _set_holding_slot(self, slot_id):
        self.holding_slot = None if slot_id is None else int(slot_id)
        rospy.set_param(self.holding_state_param,
                        0 if self.holding_slot is None else self.holding_slot)

    def _timer(self, _event):
        self._publish_status()

    def _execution_missing(self):
        missing = []
        if not self.execution_authorized:
            missing.append('execution_authorized=false')
        if not self.trajectory_execution_authorized:
            missing.append('trajectory_execution_authorized=false')
        elif not bool(rospy.get_param('/trajectory_bridge/enabled', False)):
            missing.append('trajectory_bridge/enabled=false')
        if not self.tool_voltage_authorized:
            missing.append('tool_voltage_authorized=false')
        verifications = self.config.get('verifications', {})
        for name in ('controller_frames_verified', 'tcp_verified',
                     'marker_geometry_verified', 'hand_eye_verified',
                     'rack_rigid_verified', 'voltage_interface_verified'):
            if not verifications.get(name, False):
                missing.append(name)
        if self.computer is None:
            missing.append('valid_calibration')
        else:
            if not self.computer.calibration.get('camera_observation', {}).get(
                    'verified', False):
                missing.append('camera_observation_joint7')
            if not self.computer.calibration.get(
                    'physical_validation', {}).get('passed', False):
                missing.append('calibration_physical_validation')
        if self.holding_slot is not None:
            missing.append('tool_already_held')
        if self.joint_wall is None or time.monotonic() - self.joint_wall > float(
                self.config.get('joint_feedback_max_age_s', 0.25)):
            missing.append('fresh_joint_feedback')
        return missing

    def _move_group(self):
        if self.group is None:
            import moveit_commander
            moveit_commander.roscpp_initialize([])
            self.group = moveit_commander.MoveGroupCommander(
                self.config.get('move_group', 'arm'))
            self.group.set_pose_reference_frame(
                self.config.get('base_frame', 'base_link'))
            self.group.set_end_effector_link(
                self.config.get('link7_frame', 'Link7'))
            self.group.set_max_velocity_scaling_factor(float(
                self.config.get('motion', {}).get('speed_scale', 0.05)))
            self.group.set_max_acceleration_scaling_factor(float(
                self.config.get('motion', {}).get('speed_scale', 0.05)))
        return self.group

    @staticmethod
    def _trajectory(result):
        if isinstance(result, tuple):
            if not result[0] or not result[1].joint_trajectory.points:
                raise FiveTargetError('MoveIt planning failed: {}'.format(result[-1]))
            return result[1]
        if not result or not result.joint_trajectory.points:
            raise FiveTargetError('MoveIt planning returned an empty trajectory')
        return result

    @staticmethod
    def _robot_state(joint_names, positions):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState as MoveItJointState
        state = RobotState()
        state.is_diff = True
        state.joint_state = MoveItJointState(
            name=list(joint_names), position=list(positions))
        return state

    @staticmethod
    def _trajectory_endpoint(trajectory, active_joints):
        endpoint = trajectory.joint_trajectory.points[-1]
        values = dict(zip(trajectory.joint_trajectory.joint_names,
                          endpoint.positions))
        if not set(active_joints).issubset(values):
            raise FiveTargetError('planned endpoint does not contain all arm joints')
        return [float(values[name]) for name in active_joints]

    def _plan_cartesian_lift(self, group, start_names, start_positions,
                             start_transform):
        lift_distance = float(self.config.get(
            'motion', {}).get('vertical_lift_m', 0.20))
        if not np.isfinite(lift_distance) or lift_distance <= 0.0:
            raise FiveTargetError('vertical lift distance must be positive')
        rigid_lift_target = start_transform.copy()
        rigid_lift_target[2, 3] += lift_distance
        start_state = self._robot_state(start_names, start_positions)
        group.set_start_state(start_state)
        step = float(self.config.get(
            'motion', {}).get('cartesian_step_m', 0.005))
        trajectory, fraction = group.compute_cartesian_path(
            [self._pose(rigid_lift_target, self.config.get(
                'base_frame', 'base_link')).pose],
            step, avoid_collisions=True)
        minimum = float(self.config.get(
            'motion', {}).get('minimum_cartesian_fraction', 0.999))
        lift_target = rigid_lift_target
        orientation_adjustment_deg = 0.0
        orientation_adjustment_axis = None
        chosen_clearance = lift_distance
        if fraction < minimum:
            configured_clearance = float(self.config.get(
                'motion', {}).get('vertical_straight_clearance_m', 0.17))
            tilt_deg = float(self.config.get(
                'motion', {}).get('vertical_lift_final_tilt_deg', 10.0))
            if (not np.isfinite(configured_clearance) or
                    configured_clearance <= 0.0 or
                    configured_clearance >= lift_distance or
                    not np.isfinite(tilt_deg) or
                    tilt_deg <= 0.0):
                raise FiveTargetError(
                    'vertical lift Cartesian path incomplete ({:.3f})'.format(
                        fraction))
            # Keep every waypoint on the same world-Z line.  Near workspace
            # limits a rigid wrist orientation can become unreachable before
            # the requested 20 cm clearance is achieved, so try the smallest
            # late wrist adjustment that preserves strictly vertical TCP
            # translation.  Prefer the longest rigid portion first.
            # Use one deterministic late-adjustment candidate so a field
            # planning request cannot spend tens of seconds exploring several
            # unreachable IK branches.  The tool first clears the slot by
            # 7 cm; its TCP then remains on the same world-Z line to 20 cm.
            clearances = [min(configured_clearance, 0.07)]
            best = (float(fraction), trajectory, rigid_lift_target,
                    0.0, None, lift_distance)
            angle = np.deg2rad(tilt_deg)
            c, s = np.cos(angle), np.sin(angle)
            rotations = (
                ('y-', np.array([[c, 0.0, -s], [0.0, 1.0, 0.0],
                                 [s, 0.0, c]])),
            )
            solved = False
            for clearance in clearances:
                straight_target = start_transform.copy()
                straight_target[2, 3] += clearance
                for axis, rotation in rotations:
                    if self.cancel_requested.is_set():
                        raise FiveTargetError(
                            'execution cancelled during vertical lift planning')
                    tilt = np.eye(4)
                    tilt[:3, :3] = rotation
                    candidate_target = compose(rigid_lift_target, tilt)
                    group.set_start_state(start_state)
                    candidate_trajectory, candidate_fraction = (
                        group.compute_cartesian_path(
                            [self._pose(straight_target, self.config.get(
                                'base_frame', 'base_link')).pose,
                             self._pose(candidate_target, self.config.get(
                                'base_frame', 'base_link')).pose],
                            step, avoid_collisions=True))
                    if candidate_fraction > best[0]:
                        best = (float(candidate_fraction),
                                candidate_trajectory, candidate_target,
                                tilt_deg, axis, clearance)
                    if (candidate_fraction >= minimum and
                            candidate_trajectory.joint_trajectory.points):
                        best = (float(candidate_fraction),
                                candidate_trajectory, candidate_target,
                                tilt_deg, axis, clearance)
                        solved = True
                        break
                if solved:
                    break
            (fraction, trajectory, lift_target,
             orientation_adjustment_deg, orientation_adjustment_axis,
             chosen_clearance) = best
        if fraction < minimum or not trajectory.joint_trajectory.points:
            raise FiveTargetError(
                'vertical lift Cartesian path incomplete ({:.3f})'.format(fraction))
        scale = float(self.config.get('motion', {}).get('speed_scale', 0.05))
        trajectory = group.retime_trajectory(
            start_state, trajectory, velocity_scaling_factor=scale,
            acceleration_scaling_factor=scale,
            algorithm='iterative_time_parameterization')
        if not trajectory.joint_trajectory.points:
            raise FiveTargetError('vertical lift retiming returned an empty trajectory')
        return (trajectory, lift_target, float(fraction),
                float(orientation_adjustment_deg),
                orientation_adjustment_axis, float(chosen_clearance))

    def _plan_cartesian_placement_descent(
            self, group, start_names, start_positions, start_transform,
            slot_transform, orientation_adjustment_deg,
            vertical_straight_clearance_m=None):
        """Plan the requested vertical placement from 20 cm to 2 cm high."""
        motion = self.config.get('motion', {})
        above_distance = float(motion.get('vertical_lift_m', 0.20))
        descent_distance = float(motion.get('placement_descent_m', 0.18))
        if (not np.isfinite(above_distance) or above_distance <= 0.0 or
                not np.isfinite(descent_distance) or descent_distance <= 0.0 or
                descent_distance >= above_distance):
            raise FiveTargetError(
                'placement descent must be positive and less than above-slot height')
        release_height = above_distance - descent_distance
        release_target = slot_transform.copy()
        release_target[2, 3] += release_height
        waypoints = []
        # Reverse the small wrist adjustment used for the last part of the
        # 20 cm pickup lift, while every waypoint remains on the same world-Z
        # line above the slot.
        if orientation_adjustment_deg > 0.0:
            clearance = float(
                vertical_straight_clearance_m
                if vertical_straight_clearance_m is not None else
                motion.get('vertical_straight_clearance_m', 0.17))
            if release_height < clearance < above_distance:
                straight_target = slot_transform.copy()
                straight_target[2, 3] += clearance
                waypoints.append(self._pose(
                    straight_target,
                    self.config.get('base_frame', 'base_link')).pose)
        waypoints.append(self._pose(
            release_target,
            self.config.get('base_frame', 'base_link')).pose)
        start_state = self._robot_state(start_names, start_positions)
        group.set_start_state(start_state)
        step = float(motion.get('cartesian_step_m', 0.005))
        trajectory, fraction = group.compute_cartesian_path(
            waypoints, step, avoid_collisions=True)
        minimum = float(motion.get('minimum_cartesian_fraction', 0.999))
        if fraction < minimum or not trajectory.joint_trajectory.points:
            raise FiveTargetError(
                'placement descent Cartesian path incomplete ({:.3f})'.format(
                    fraction))
        scale = float(motion.get('speed_scale', 0.05))
        trajectory = group.retime_trajectory(
            start_state, trajectory, velocity_scaling_factor=scale,
            acceleration_scaling_factor=scale,
            algorithm='iterative_time_parameterization')
        if not trajectory.joint_trajectory.points:
            raise FiveTargetError(
                'placement descent retiming returned an empty trajectory')
        return (trajectory, release_target, float(fraction),
                descent_distance, release_height)

    def _camera_observation_target(self, current_names=None,
                                   current_positions=None):
        if self.computer is None:
            raise FiveTargetError('valid calibration is required')
        if current_names is None or current_positions is None:
            current_names, current_positions = self._fresh_joints()
        observation = copy.deepcopy(
            self.computer.calibration.get('camera_observation'))
        configured = self.config.get('camera_observation', {})
        # Calibrations created before the full-pose fix contain only J7.  Use
        # the audited stable observation joints from the deployment config as
        # a migration fallback; newly captured calibrations persist all seven.
        if ('joint_names' not in observation and
                configured.get('joint_names') is not None):
            observation['joint_names'] = copy.deepcopy(
                configured.get('joint_names'))
            observation['positions_rad'] = copy.deepcopy(
                configured.get('positions_rad'))
        return observation_joint_target(
            current_positions, current_names, observation)

    def _execute_observation_joint_target(self, group, current, target,
                                          tolerance, label):
        if joint_target_reached(current, target, tolerance):
            rospy.loginfo('%s already within %.6f rad; skipping motion',
                          label, tolerance)
            return list(current)
        group.set_start_state_to_current_state()
        group.set_joint_value_target(target)
        trajectory = self._trajectory(group.plan())
        if self.cancel_requested.is_set():
            raise FiveTargetError(
                'execution cancelled before {}'.format(label))
        if not group.execute(trajectory, wait=True):
            raise FiveTargetError('{} execution failed'.format(label))
        group.stop()
        return self._wait_for_joint_target(
            group, target, tolerance, label)

    def _move_to_camera_observation(self):
        group = self._move_group()
        active = list(group.get_active_joints())
        current = list(group.get_current_joint_values())
        target = self._camera_observation_target(active, current)
        tolerance = float(self.config.get(
            'camera_observation', {}).get('joint_tolerance_rad', 0.02))
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before camera observation move')
        # J7 always moves first as required by the eye-in-hand camera setup.
        # Then restore the other six taught observation joints while holding
        # J7 fixed; J7 alone cannot reproduce a camera viewpoint.
        observation = self.computer.calibration.get('camera_observation')
        joint7_target = observation_joint_target(
            current, active,
            {'joint_name': 'joint7',
             'position_rad': float(observation['position_rad']),
             'verified': True})
        measured = self._execute_observation_joint_target(
            group, current, joint7_target, tolerance,
            'camera observation J7')
        measured = self._execute_observation_joint_target(
            group, measured, target, tolerance,
            'camera observation full pose')
        # Discard every image acquired during motion.  Only a stable detection
        # received after settling may drive a pickup target.
        settle = float(self.config.get(
            'camera_observation', {}).get('settle_s', 0.5))
        with self.lock:
            self.observation = None
            self.observation_wall = None
        if settle > 0:
            rospy.sleep(settle)
        ready_wall = time.monotonic()
        deadline = ready_wall + float(self.config.get(
            'camera_observation', {}).get('marker_wait_timeout_s', 8.0))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.cancel_requested.is_set():
                raise FiveTargetError('execution cancelled while waiting for marker')
            try:
                self._fresh_observation(after_wall=ready_wall)
                return target, ready_wall
            except FiveTargetError:
                rospy.sleep(0.05)
        raise FiveTargetError('no stable marker after camera observation J7 move')

    def _plan(self, slot_id, return_joints=None):
        result = self._compute(slot_id)
        group = self._move_group()
        final = result['targets'][slot_id]['link7']
        active = list(group.get_active_joints())
        current = list(group.get_current_joint_values())
        if return_joints is None:
            return_joints = self._camera_observation_target(
                active, current)
        # Resolve the desired IK branch first.  The resulting J7 value is then
        # reached on its own before any of the other joints descend toward the
        # rack, protecting the eye-in-hand RGB camera.
        group.set_start_state_to_current_state()
        group.set_pose_target(self._pose(final, self.config.get('base_frame', 'base_link')),
                              self.config.get('link7_frame', 'Link7'))
        ik_plan = self._trajectory(group.plan())
        group.clear_pose_targets()
        pickup_joints = self._trajectory_endpoint(ik_plan, active)
        pre_rotate_joints = list(current)
        pre_rotate_joints[active.index('joint7')] = pickup_joints[
            active.index('joint7')]
        group.set_start_state(self._robot_state(active, current))
        group.set_joint_value_target(pre_rotate_joints)
        pre_rotate_plan = self._trajectory(group.plan())
        group.set_start_state(self._robot_state(active, pre_rotate_joints))
        group.set_joint_value_target(pickup_joints)
        pickup_plan = self._trajectory(group.plan())
        (lift_plan, lift_target, lift_fraction,
         lift_orientation_adjustment, lift_orientation_axis,
         lift_straight_clearance) = self._plan_cartesian_lift(
            group, active, pickup_joints, final)
        lift_joints = self._trajectory_endpoint(lift_plan, active)
        group.set_start_state(self._robot_state(active, lift_joints))
        group.set_joint_value_target(list(return_joints))
        retreat_plan = self._trajectory(group.plan())
        group.set_start_state_to_current_state()
        plan = {'slot_id': slot_id, 'marker_stamp': result['marker_stamp'],
                'calibration_version': result['calibration_version'],
                'target_link7': serialize_transform(final),
                'lift_target_link7': serialize_transform(lift_target),
                'vertical_lift_m': float(self.config.get(
                    'motion', {}).get('vertical_lift_m', 0.20)),
                'vertical_lift_cartesian_fraction': lift_fraction,
                'vertical_lift_orientation_adjustment_deg': (
                    lift_orientation_adjustment),
                'vertical_lift_orientation_adjustment_axis': (
                    lift_orientation_axis),
                'vertical_straight_clearance_m': lift_straight_clearance,
                'return_observation_joints': list(return_joints),
                'segments': 4,
                'sequence': ['j7_pre_rotate', 'target_descent',
                             'vertical_lift', 'return_observation'],
                'moveit_self_collision_checked': True,
                'environment_collision_geometry_used': False,
                'executed': False,
                '_trajectories': (pre_rotate_plan, pickup_plan,
                                  lift_plan, retreat_plan),
                '_pickup_joints': pickup_joints,
                '_pre_rotate_joints': pre_rotate_joints,
                '_lift_joints': lift_joints,
                '_lift_target': lift_target,
                '_targets': result}
        with self.lock:
            self.last_plan = plan
        return plan

    def _public_plan(self, plan):
        return {key: value for key, value in plan.items() if not key.startswith('_')}

    def _validate_frozen_snapshot(self, plan):
        # MoveIt planning can briefly monopolise the process long enough for
        # the last subscriber sample to cross max_marker_age_s.  Do not turn
        # that scheduling delay into a false SAFE_STOP: wait for the next
        # genuinely fresh camera sample, while keeping the original age and
        # rack-motion tolerances unchanged.
        deadline = time.monotonic() + float(self.config.get(
            'pre_execution_marker_wait_s', 2.0))
        last_error = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                current = self._fresh_observation()
                break
            except FiveTargetError as error:
                last_error = error
                rospy.sleep(0.02)
        else:
            if last_error is not None:
                raise last_error
            raise FiveTargetError(
                'no fresh marker observation before execution')
        frozen = plan['_targets']['base_marker']
        translation = float(np.linalg.norm(
            current['transform'][:3, 3] - frozen[:3, 3]))
        rotation = float(rotation_distance(current['transform'], frozen))
        if translation > float(self.config.get(
                'execution_marker_translation_tolerance_m', 0.003)):
            raise FiveTargetError('rack marker moved since planning ({:.4f}m)'.format(
                translation))
        if rotation > np.deg2rad(float(self.config.get(
                'execution_marker_rotation_tolerance_deg', 1.0))):
            raise FiveTargetError('rack marker rotated since planning ({:.3f}deg)'.format(
                np.rad2deg(rotation)))
        if self.computer.calibration['version'] != plan['calibration_version']:
            raise FiveTargetError('calibration changed before execution')

    def _execute_pose_from_measured(self, group, transform, label):
        group.set_start_state_to_current_state()
        group.set_pose_target(
            self._pose(transform, self.config.get('base_frame', 'base_link')),
            self.config.get('link7_frame', 'Link7'))
        trajectory = self._trajectory(group.plan())
        group.clear_pose_targets()
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before {}'.format(label))
        if not group.execute(trajectory, wait=True):
            raise FiveTargetError('{} MoveIt execution failed'.format(label))
        group.stop()
        pose = group.get_current_pose(self.config.get('link7_frame', 'Link7')).pose
        actual = transform_matrix(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])
        error = pose_error(transform, actual)
        if error['translation_m'] > float(self.config.get(
                'target_translation_tolerance_m', 0.004)):
            raise FiveTargetError('{} position feedback tolerance exceeded'.format(label))
        if error['rotation_rad'] > np.deg2rad(float(self.config.get(
                'target_rotation_tolerance_deg', 2.0))):
            raise FiveTargetError('{} rotation feedback tolerance exceeded'.format(label))
        return actual

    def _execute_joints_from_measured(self, group, target, label):
        active = list(group.get_active_joints())
        if len(active) != 7 or len(target) != 7:
            raise FiveTargetError('{} requires a seven-joint target'.format(label))
        group.set_start_state_to_current_state()
        group.set_joint_value_target(list(target))
        trajectory = self._trajectory(group.plan())
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before {}'.format(label))
        if not group.execute(trajectory, wait=True):
            raise FiveTargetError('{} MoveIt execution failed'.format(label))
        group.stop()
        tolerance = float(self.config.get(
            'camera_observation', {}).get('joint_tolerance_rad', 0.02))
        return self._wait_for_joint_target(
            group, target, tolerance, label)

    def _wait_for_joint_target(self, group, target, tolerance, label):
        timeout = float(self.config.get(
            'motion', {}).get('joint_settle_timeout_s', 2.0))
        deadline = time.monotonic() + timeout
        measured = list(group.get_current_joint_values())
        while not rospy.is_shutdown():
            if joint_target_reached(measured, target, tolerance):
                return measured
            if self.cancel_requested.is_set():
                raise FiveTargetError(
                    'execution cancelled while settling {}'.format(label))
            if time.monotonic() >= deadline:
                maximum = max(abs(actual - expected) for actual, expected in zip(
                    measured, target))
                raise FiveTargetError(
                    '{} feedback tolerance exceeded ({:.4f}deg)'.format(
                        label, np.rad2deg(maximum)))
            rospy.sleep(0.05)
            measured = list(group.get_current_joint_values())

    def _measured_link7(self, group):
        pose = group.get_current_pose(
            self.config.get('link7_frame', 'Link7')).pose
        return transform_matrix(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])

    def _settle_feedback(self):
        delay = float(self.config.get(
            'motion', {}).get('feedback_settle_s', 0.25))
        if delay > 0.0:
            rospy.sleep(delay)

    def _wait_before_power_on(self):
        delay = float(self.config.get(
            'motion', {}).get('pre_power_settle_s', 2.0))
        if not np.isfinite(delay) or delay < 0.0:
            raise FiveTargetError('pre-power settle time is invalid')
        deadline = time.monotonic() + delay
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.cancel_requested.is_set():
                raise FiveTargetError(
                    'execution cancelled during pre-power settle')
            rospy.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return delay

    def _execute_approach_from_plan(self, group, plan):
        current = list(group.get_current_joint_values())
        tolerance = float(self.config.get(
            'motion', {}).get('pickup_j7_tolerance_rad', 0.01))
        self._execute_observation_joint_target(
            group, current, plan['_pre_rotate_joints'], tolerance,
            'pickup J7 pre-rotation')
        self._execute_joints_from_measured(
            group, plan['_pickup_joints'], 'pickup-target descent')
        self._settle_feedback()
        actual = self._measured_link7(group)
        target = plan['_targets']['targets'][plan['slot_id']]['link7']
        error = pose_error(target, actual)
        if error['translation_m'] > float(self.config.get(
                'target_translation_tolerance_m', 0.004)):
            raise FiveTargetError(
                'pickup-target descent position feedback tolerance exceeded')
        if error['rotation_rad'] > np.deg2rad(float(self.config.get(
                'target_rotation_tolerance_deg', 2.0))):
            raise FiveTargetError(
                'pickup-target descent rotation feedback tolerance exceeded')
        return actual

    def _execute_vertical_lift(self, group):
        current = self._measured_link7(group)
        active = list(group.get_active_joints())
        joints = list(group.get_current_joint_values())
        (trajectory, lift_target, fraction,
         orientation_adjustment, _orientation_axis,
         _straight_clearance) = self._plan_cartesian_lift(
            group, active, joints, current)
        group.set_start_state_to_current_state()
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before vertical lift')
        if not group.execute(trajectory, wait=True):
            raise FiveTargetError('vertical lift MoveIt execution failed')
        group.stop()
        self._settle_feedback()
        actual = self._measured_link7(group)
        error = pose_error(lift_target, actual)
        if error['translation_m'] > float(self.config.get(
                'target_translation_tolerance_m', 0.004)):
            raise FiveTargetError('vertical lift position feedback tolerance exceeded')
        if error['rotation_rad'] > np.deg2rad(float(self.config.get(
                'target_rotation_tolerance_deg', 2.0))):
            raise FiveTargetError('vertical lift rotation feedback tolerance exceeded')
        return actual, fraction, orientation_adjustment

    def _configured_target(self, slot_id):
        """Load one persisted absolute Link7 target without caching stale values."""
        if int(slot_id) not in SLOT_IDS:
            raise FiveTargetError('tool slot must be in 1..5')
        if not self.tool_config_path or not os.path.isfile(self.tool_config_path):
            raise FiveTargetError('five-tool configuration file is unavailable')
        document = load_yaml(self.tool_config_path)
        matches = [item for item in document.get('tools', [])
                   if int(item.get('slot_id', 0)) == int(slot_id)]
        if len(matches) != 1:
            raise FiveTargetError('configured Tool{} is missing or duplicated'.format(slot_id))
        configured = matches[0].get('configured_pick') or {}
        position = configured.get('position_m')
        rpy = configured.get('rpy_rad')
        height = configured.get('lift_height_m')
        try:
            position = [float(value) for value in position]
            rpy = [float(value) for value in rpy]
            height = float(height)
        except (TypeError, ValueError) as exc:
            raise FiveTargetError('Tool{} configured pose is incomplete'.format(slot_id)) from exc
        if (len(position) != 3 or len(rpy) != 3 or
                not all(np.isfinite(position + rpy + [height])) or height <= 0.0):
            raise FiveTargetError('Tool{} configured pose is invalid'.format(slot_id))
        target = transform_matrix(position, matrix_quaternion(rpy_matrix(*rpy)))
        pregrasp = target.copy()
        pregrasp[2, 3] += height
        public = {
            'slot_id': int(slot_id), 'name': 'Tool{}'.format(slot_id),
            'frame': self.config.get('base_frame', 'base_link'),
            'pose_reference': self.config.get('link7_frame', 'Link7'),
            'position_m': position, 'rpy_rad': rpy, 'lift_height_m': height,
            'target': serialize_transform(target),
            'pregrasp': serialize_transform(pregrasp),
            'updated_at': configured.get('updated_at'),
        }
        digest = hashlib.sha256(json.dumps(
            public, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
        return target, pregrasp, public, digest

    def _plan_strict_cartesian(self, group, names, positions,
                               start_transform, target_transform, label):
        """Plan a collision-checked world-Z segment with exactly fixed orientation."""
        delta = target_transform[:3, 3] - start_transform[:3, 3]
        if abs(float(delta[0])) > 1e-9 or abs(float(delta[1])) > 1e-9:
            raise FiveTargetError('{} is not parallel to base-frame Z'.format(label))
        if not np.allclose(start_transform[:3, :3], target_transform[:3, :3],
                           atol=1e-10, rtol=0.0):
            raise FiveTargetError('{} changes end-effector orientation'.format(label))
        start_state = self._robot_state(names, positions)
        group.set_start_state(start_state)
        step = float(self.config.get('motion', {}).get('cartesian_step_m', 0.005))
        trajectory, fraction = group.compute_cartesian_path(
            [self._pose(target_transform, self.config.get(
                'base_frame', 'base_link')).pose], step, avoid_collisions=True)
        minimum = float(self.config.get(
            'motion', {}).get('minimum_cartesian_fraction', 0.999))
        if fraction < minimum or not trajectory.joint_trajectory.points:
            raise FiveTargetError(
                '{} Cartesian path incomplete ({:.3f})'.format(label, fraction))
        scale = float(self.config.get('motion', {}).get('speed_scale', 0.05))
        trajectory = group.retime_trajectory(
            start_state, trajectory, velocity_scaling_factor=scale,
            acceleration_scaling_factor=scale,
            algorithm='iterative_time_parameterization')
        if not trajectory.joint_trajectory.points:
            raise FiveTargetError('{} retiming returned an empty trajectory'.format(label))
        endpoint = self._trajectory_endpoint(trajectory, names)
        return trajectory, endpoint, float(fraction)

    def _plan_configured_pick(self, slot_id):
        self.cancel_requested.clear()
        self._set_configured_step('PLANNING', 0, 'Dry Run：检查预抓取和严格Z向路径', slot_id)
        target, pregrasp, public_config, digest = self._configured_target(slot_id)
        group = self._move_group()
        active = list(group.get_active_joints())
        current = list(group.get_current_joint_values())
        if len(active) != 7 or len(current) != 7:
            raise FiveTargetError('configured pickup requires a seven-joint MoveIt group')

        group.set_start_state_to_current_state()
        group.set_pose_target(
            self._pose(pregrasp, self.config.get('base_frame', 'base_link')),
            self.config.get('link7_frame', 'Link7'))
        pregrasp_plan = self._trajectory(group.plan())
        group.clear_pose_targets()
        pregrasp_joints = self._trajectory_endpoint(pregrasp_plan, active)
        descent_plan, pickup_joints, descent_fraction = self._plan_strict_cartesian(
            group, active, pregrasp_joints, pregrasp, target,
            'configured pickup descent')
        lift_plan, lifted_joints, lift_fraction = self._plan_strict_cartesian(
            group, active, pickup_joints, target, pregrasp,
            'configured pickup lift')
        plan = {
            'slot_id': int(slot_id), 'configuration': public_config,
            'configuration_digest': digest,
            'sequence': ['move_to_pregrasp', 'vertical_descent',
                         'tool_action_24v_ack', 'grasp_result_unverified',
                         'vertical_lift', 'hold_position'],
            'speed_scale': float(self.config.get('motion', {}).get('speed_scale', 0.05)),
            'cartesian_step_m': float(self.config.get(
                'motion', {}).get('cartesian_step_m', 0.005)),
            'descent_cartesian_fraction': descent_fraction,
            'lift_cartesian_fraction': lift_fraction,
            'moveit_collision_checked': True,
            'dry_run': True, 'executed': False,
            '_start_joints': current, '_pregrasp_joints': pregrasp_joints,
            '_pickup_joints': pickup_joints, '_lifted_joints': lifted_joints,
            '_target': target, '_pregrasp': pregrasp,
            '_trajectories': (pregrasp_plan, descent_plan, lift_plan),
        }
        self.configured_plan = plan
        self._set_configured_step('DRY_RUN_OK', 0, 'Dry Run规划通过，等待人工确认', slot_id)
        return plan

    @staticmethod
    def _public_configured_plan(plan):
        return {key: value for key, value in plan.items() if not key.startswith('_')}

    def _configured_execution_missing(self):
        missing = []
        if not self.execution_authorized:
            missing.append('execution_authorized=false')
        if not self.trajectory_execution_authorized:
            missing.append('trajectory_execution_authorized=false')
        elif not bool(rospy.get_param('/trajectory_bridge/enabled', False)):
            missing.append('trajectory_bridge/enabled=false')
        if not self.tool_voltage_authorized:
            missing.append('tool_voltage_authorized=false')
        verifications = self.config.get('verifications', {})
        for name in ('controller_frames_verified', 'tcp_verified',
                     'voltage_interface_verified'):
            if not verifications.get(name, False):
                missing.append(name)
        if self.holding_slot is not None:
            missing.append('tool_already_held')
        if self.joint_wall is None or time.monotonic() - self.joint_wall > float(
                self.config.get('joint_feedback_max_age_s', 0.25)):
            missing.append('fresh_joint_feedback')
        return missing

    def _assert_configured_execution_ready(self):
        missing = self._configured_execution_missing()
        if missing:
            raise FiveTargetError('configured execution rejected: ' + ', '.join(missing))
        live_nodes = set(rosnode.get_node_names())
        if '/trajectory_bridge' not in live_nodes:
            raise FiveTargetError(
                'trajectory_bridge is not running; restart base in trajectory pickup mode')
        conflicts = sorted(name for name in live_nodes
                           if name in ('/joint_control_server', '/rm75_touch_teleop',
                                       '/rm75_vertical_press', '/rm_control'))
        if conflicts:
            raise FiveTargetError('competing command source present: ' + ', '.join(conflicts))

    def _execute_configured_pick(self, slot_id):
        self._assert_configured_execution_ready()
        plan = self.configured_plan
        if plan is None or int(plan.get('slot_id', 0)) != int(slot_id):
            raise FiveTargetError('run Dry Run for this tool before execution')
        _target, _pregrasp, _public, digest = self._configured_target(slot_id)
        if digest != plan.get('configuration_digest'):
            self.configured_plan = None
            raise FiveTargetError('tool configuration changed after Dry Run; plan again')
        group = self._move_group()
        current = list(group.get_current_joint_values())
        start_tolerance = float(self.config.get(
            'camera_observation', {}).get('joint_tolerance_rad', 0.02))
        if not joint_target_reached(current, plan['_start_joints'], start_tolerance):
            self.configured_plan = None
            raise FiveTargetError('robot moved after Dry Run; plan again')
        self.cancel_requested.clear()
        try:
            self.voltage.set_voltage(0)
            self._set_configured_step('RUNNING', 1, '运动到预抓取位置', slot_id)
            if not group.execute(plan['_trajectories'][0], wait=True):
                raise FiveTargetError('pregrasp MoveIt execution failed')
            group.stop()
            self._wait_for_joint_target(group, plan['_pregrasp_joints'],
                                        start_tolerance, 'configured pregrasp')
            if self.cancel_requested.is_set():
                raise FiveTargetError('execution cancelled before vertical descent')

            self._set_configured_step('RUNNING', 2, '保持姿态，沿基坐标Z轴缓慢下降', slot_id)
            group.set_start_state_to_current_state()
            if not group.execute(plan['_trajectories'][1], wait=True):
                raise FiveTargetError('configured vertical descent execution failed')
            group.stop()
            self._wait_for_joint_target(group, plan['_pickup_joints'],
                                        start_tolerance, 'configured target')
            actual = self._measured_link7(group)
            error = pose_error(plan['_target'], actual)
            if error['translation_m'] > float(self.config.get(
                    'target_translation_tolerance_m', 0.004)):
                raise FiveTargetError('configured target position feedback tolerance exceeded')
            if error['rotation_rad'] > np.deg2rad(float(self.config.get(
                    'target_rotation_tolerance_deg', 2.0))):
                raise FiveTargetError('configured target rotation feedback tolerance exceeded')

            self._set_configured_step('RUNNING', 3, '调用现有末端24V取物动作', slot_id)
            pre_power_settle = self._wait_before_power_on()
            self.voltage.set_voltage(24)
            self._set_holding_slot(slot_id)
            self._set_configured_step(
                'RUNNING', 4, '24V ACK已确认；无独立物体在位传感器', slot_id,
                grasp_confirmed=False)
            if self.cancel_requested.is_set():
                raise FiveTargetError('execution cancelled before vertical lift')

            self._set_configured_step('RUNNING', 5, '保持姿态，沿基坐标Z轴抬升', slot_id)
            group.set_start_state_to_current_state()
            if not group.execute(plan['_trajectories'][2], wait=True):
                raise FiveTargetError('configured vertical lift execution failed')
            group.stop()
            self._wait_for_joint_target(group, plan['_lifted_joints'],
                                        start_tolerance, 'configured lift')
            lifted = self._measured_link7(group)
            lift_error = pose_error(plan['_pregrasp'], lifted)
            if lift_error['translation_m'] > float(self.config.get(
                    'target_translation_tolerance_m', 0.004)):
                raise FiveTargetError('configured lift position feedback tolerance exceeded')
            if lift_error['rotation_rad'] > np.deg2rad(float(self.config.get(
                    'target_rotation_tolerance_deg', 2.0))):
                raise FiveTargetError('configured lift rotation feedback tolerance exceeded')
            result = self._public_configured_plan(plan)
            result.update({
                'executed': True, 'status': 'COMPLETED_UNVERIFIED',
                'voltage_v': 24, 'voltage_ack_confirmed': True,
                'grasp_confirmed': False,
                'grasp_confirmation_note': (
                    '24V ACK only; no independent object-present sensor is available'),
                'holding_slot': self.holding_slot,
                'pre_power_settle_s': pre_power_settle,
                'final_link7': serialize_transform(lifted),
                'held_at_pregrasp': True,
            })
            self._set_configured_step(
                'COMPLETED_UNVERIFIED', 6,
                '运动完成并保持；取物结果未独立确认', slot_id,
                grasp_confirmed=False)
            self.configured_plan = None
            return result
        except Exception as exc:
            self._set_configured_step('FAILED', self.configured_pick_status.get(
                'step', 0), '执行失败，已终止后续步骤', slot_id, error=exc)
            self.configured_plan = None
            try:
                group.stop()
            except Exception:
                pass
            raise

    def _assert_execution_path_ready(self, allow_holding=False):
        missing = self._execution_missing()
        if allow_holding:
            missing = [item for item in missing if item != 'tool_already_held']
        if missing:
            raise FiveTargetError('real execution rejected: ' + ', '.join(missing))
        live_nodes = set(rosnode.get_node_names())
        if '/trajectory_bridge' not in live_nodes:
            raise FiveTargetError(
                'trajectory_bridge is not running; restart base in QR pickup mode')
        conflicts = sorted(name for name in live_nodes
                           if name in ('/joint_control_server', '/rm75_touch_teleop',
                                       '/rm75_vertical_press', '/rm_control'))
        if conflicts:
            raise FiveTargetError('competing command source present: ' + ', '.join(conflicts))

    def _execute_camera_observation_only(self):
        """Commissioning step: restore the camera pose, reacquire, then stop."""
        self._assert_execution_path_ready()
        self.cancel_requested.clear()
        self.voltage.set_voltage(0)
        observation_joints, ready_wall = self._move_to_camera_observation()
        observation = self._fresh_observation(after_wall=ready_wall)
        return {
            'executed': True,
            'motion': 'camera_observation_pose',
            'observation_joints': list(observation_joints),
            'marker_stamp': observation['stamp'],
            'marker_pixel_size': observation['pixel_size'],
            'marker_reprojection_error_px': observation['reprojection_error_px'],
            'pre_motion_voltage_off_ack': True,
            'voltage_v': 0,
        }

    def _move_selected_target_only(self, slot_id):
        """Commissioning step: approach a computed target at 0 V and stop."""
        self._assert_execution_path_ready()
        self.cancel_requested.clear()
        self.staged_pickup = None
        self.voltage.set_voltage(0)
        group = self._move_group()
        observation_joints, _ = self._move_to_camera_observation()
        plan = self._plan(slot_id, return_joints=observation_joints)
        self._validate_frozen_snapshot(plan)
        target_link7 = plan['_targets']['targets'][slot_id]['link7']
        actual_link7 = self._execute_approach_from_plan(group, plan)
        target_tcp = plan['_targets']['targets'][slot_id]['tcp']
        actual_tcp = compose(actual_link7, self.computer.link7_tcp)
        self.staged_pickup = {
            'slot_id': int(slot_id),
            'target_link7': target_link7.copy(),
            'target_tcp': target_tcp.copy(),
            'observation_joints': list(observation_joints),
        }
        public = self._public_plan(plan)
        public.update({
            'executed': True,
            'staged_at_target': True,
            'actual_tcp': serialize_transform(actual_tcp),
            'error': pose_error(target_tcp, actual_tcp),
            'pre_motion_voltage_off_ack': True,
            'voltage_command_sent': False,
            'voltage_v': 0,
        })
        return public

    def _engage_staged_and_retreat(self, slot_id):
        """Power a visually confirmed staged pickup, then return to observation."""
        self._assert_execution_path_ready()
        if self.staged_pickup is None:
            raise FiveTargetError('no staged pickup target is awaiting confirmation')
        if int(slot_id) != int(self.staged_pickup['slot_id']):
            raise FiveTargetError(
                'staged pickup is Tool{}, not Tool{}'.format(
                    self.staged_pickup['slot_id'], slot_id))
        group = self._move_group()
        pose = group.get_current_pose(
            self.config.get('link7_frame', 'Link7')).pose
        actual_link7 = transform_matrix(
            [pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])
        staged_error = pose_error(
            self.staged_pickup['target_link7'], actual_link7)
        if staged_error['translation_m'] > float(self.config.get(
                'target_translation_tolerance_m', 0.004)):
            raise FiveTargetError('robot is no longer at staged pickup position')
        if staged_error['rotation_rad'] > np.deg2rad(float(self.config.get(
                'target_rotation_tolerance_deg', 2.0))):
            raise FiveTargetError('robot is no longer at staged pickup orientation')
        self.cancel_requested.clear()
        pre_power_settle = self._wait_before_power_on()
        self.voltage.set_voltage(24)
        self._set_holding_slot(slot_id)
        (lifted_link7, lift_fraction,
         lift_orientation_adjustment) = self._execute_vertical_lift(group)
        retreat_joints = self._execute_joints_from_measured(
            group, self.staged_pickup['observation_joints'], 'pickup-retreat')
        result = {
            'executed': True,
            'staged_target_confirmed': True,
            'voltage_command_sent': True,
            'voltage_v': 24,
            'pre_power_settle_s': pre_power_settle,
            'hold_feedback_confirmed': False,
            'hold_feedback_note': (
                'voltage ACK confirms command acceptance only; '
                'no independent object-present sensor was used'),
            'holding_slot': self.holding_slot,
            'vertical_lift_m': float(self.config.get(
                'motion', {}).get('vertical_lift_m', 0.20)),
            'vertical_lift_cartesian_fraction': lift_fraction,
            'vertical_lift_orientation_adjustment_deg': (
                lift_orientation_adjustment),
            'lifted_link7': serialize_transform(lifted_link7),
            'retreat_observation_joints': retreat_joints,
        }
        self.staged_pickup = None
        return result

    def _plan_return_held_tool(self, slot_id, observation_joints=None):
        """Plan placement above the slot without descending through the rack."""
        group = self._move_group()
        active = list(group.get_active_joints())
        current = list(group.get_current_joint_values())
        if observation_joints is None:
            observation_joints = self._camera_observation_target(active, current)
        pickup_plan = self._plan(slot_id, return_joints=observation_joints)
        high_joints = list(pickup_plan['_lift_joints'])
        pre_rotate_joints = list(current)
        pre_rotate_joints[active.index('joint7')] = high_joints[
            active.index('joint7')]

        group.set_start_state(self._robot_state(active, current))
        group.set_joint_value_target(pre_rotate_joints)
        pre_rotate_plan = self._trajectory(group.plan())
        group.set_start_state(self._robot_state(active, pre_rotate_joints))
        group.set_joint_value_target(high_joints)
        above_slot_plan = self._trajectory(group.plan())

        slot_transform = pickup_plan['_targets']['targets'][slot_id]['link7']
        (descent_plan, release_target, descent_fraction,
         descent_distance, release_height) = (
            self._plan_cartesian_placement_descent(
            group, active, high_joints, pickup_plan['_lift_target'],
                slot_transform,
                pickup_plan['vertical_lift_orientation_adjustment_deg'],
                pickup_plan['vertical_straight_clearance_m']))
        release_joints = self._trajectory_endpoint(descent_plan, active)
        group.set_start_state(self._robot_state(active, release_joints))
        group.set_joint_value_target(list(observation_joints))
        retreat_plan = self._trajectory(group.plan())
        group.set_start_state_to_current_state()

        plan = {
            'slot_id': int(slot_id),
            'marker_stamp': pickup_plan['marker_stamp'],
            'calibration_version': pickup_plan['calibration_version'],
            'placement_above_slot_m': float(self.config.get(
                'motion', {}).get('vertical_lift_m', 0.20)),
            'placement_descent_m': descent_distance,
            'release_height_above_slot_m': release_height,
            'release_target_link7': serialize_transform(release_target),
            'placement_descent_cartesian_fraction': descent_fraction,
            'return_observation_joints': list(observation_joints),
            'segments': 4,
            'sequence': ['placement_j7_pre_rotate',
                         'placement_above_slot_20cm',
                         'vertical_descent_18cm_release_0v',
                         'return_observation'],
            'moveit_self_collision_checked': True,
            'environment_collision_geometry_used': False,
            'executed': False,
            '_trajectories': (pre_rotate_plan, above_slot_plan,
                              descent_plan, retreat_plan),
            '_pre_rotate_joints': pre_rotate_joints,
            '_above_slot_joints': high_joints,
            '_release_joints': release_joints,
            '_release_target': release_target,
            '_targets': pickup_plan['_targets'],
        }
        with self.lock:
            self.last_plan = plan
        return plan

    def _return_held_tool(self, slot_id):
        """Return the recorded tool to its calibrated slot and clear it."""
        self._assert_execution_path_ready(allow_holding=True)
        if self.holding_slot is None:
            raise FiveTargetError('no held tool is recorded')
        if int(slot_id) != int(self.holding_slot):
            raise FiveTargetError(
                'held tool is Tool{}, not Tool{}'.format(
                    self.holding_slot, slot_id))
        self.cancel_requested.clear()
        # Prove that holding power is still accepted before moving the long
        # tool.  A fresh ACK is required; this is not object-present feedback.
        self.voltage.set_voltage(24)
        group = self._move_group()
        active = list(group.get_active_joints())
        current = list(group.get_current_joint_values())
        observation_joints = self._camera_observation_target(active, current)
        tolerance = float(self.config.get(
            'camera_observation', {}).get('joint_tolerance_rad', 0.02))
        if not joint_target_reached(current, observation_joints, tolerance):
            raise FiveTargetError(
                'held-tool return must start from the camera observation pose')
        # The marker must be visible at the observation pose.  Freeze its
        # target before J7 turns away from the camera view.
        plan = self._plan_return_held_tool(
            slot_id, observation_joints=observation_joints)
        self._validate_frozen_snapshot(plan)
        placement_j7_tolerance = float(self.config.get(
            'motion', {}).get('pickup_j7_tolerance_rad', 0.01))
        self._execute_observation_joint_target(
            group, current, plan['_pre_rotate_joints'],
            placement_j7_tolerance, 'placement J7 pre-rotation')
        self._execute_joints_from_measured(
            group, plan['_above_slot_joints'],
            'placement above-slot 20cm approach')
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before placement descent')
        descent_plan = plan['_trajectories'][2]
        group.set_start_state_to_current_state()
        if not group.execute(descent_plan, wait=True):
            raise FiveTargetError('placement vertical descent execution failed')
        group.stop()
        self._settle_feedback()
        actual_link7 = self._measured_link7(group)
        release_error = pose_error(plan['_release_target'], actual_link7)
        if release_error['translation_m'] > float(self.config.get(
                'target_translation_tolerance_m', 0.004)):
            raise FiveTargetError(
                'placement release position feedback tolerance exceeded')
        if release_error['rotation_rad'] > np.deg2rad(float(self.config.get(
                'target_rotation_tolerance_deg', 2.0))):
            raise FiveTargetError(
                'placement release rotation feedback tolerance exceeded')
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before tool release')
        # Release immediately at 18 cm of descent, then return directly to the
        # original observation pose.  There is intentionally no post-release
        # wait and no second vertical lift.
        self.voltage.set_voltage(0)
        self._set_holding_slot(None)
        retreat_joints = self._execute_joints_from_measured(
            group, observation_joints, 'return-tool retreat')
        return {
            'executed': True,
            'returned_slot': int(slot_id),
            'placed_link7': serialize_transform(actual_link7),
            'voltage_hold_ack_before_motion': True,
            'voltage_off_ack': True,
            'voltage_v': 0,
            'placement_above_slot_m': plan['placement_above_slot_m'],
            'placement_descent_m': plan['placement_descent_m'],
            'release_height_above_slot_m': (
                plan['release_height_above_slot_m']),
            'placement_descent_cartesian_fraction': (
                plan['placement_descent_cartesian_fraction']),
            'release_immediate_no_settle': True,
            'retreat_observation_joints': retreat_joints,
            'holding_slot': None,
        }

    def _execute(self, slot_id):
        self._assert_execution_path_ready()
        self.cancel_requested.clear()
        # Always prove that the electromagnet is off before any approach
        # motion.  The adapter requires an ACK newer than this exact command.
        self.voltage.set_voltage(0)
        group = self._move_group()
        observation_joints, _ = self._move_to_camera_observation()
        plan = self._plan(slot_id, return_joints=observation_joints)
        # The target is derived only from a stable image acquired after J7
        # reached its taught camera observation position.
        self._validate_frozen_snapshot(plan)
        target_link7 = plan['_targets']['targets'][slot_id]['link7']
        actual_link7 = self._execute_approach_from_plan(group, plan)
        if self.cancel_requested.is_set():
            raise FiveTargetError('execution cancelled before tool power on')
        pre_power_settle = self._wait_before_power_on()
        self.voltage.set_voltage(24)
        self._set_holding_slot(slot_id)
        (lifted_link7, lift_fraction,
         lift_orientation_adjustment) = self._execute_vertical_lift(group)
        retreat_joints = self._execute_joints_from_measured(
            group, observation_joints, 'pickup-retreat')
        actual_tcp = compose(actual_link7, self.computer.link7_tcp)
        target_tcp = plan['_targets']['targets'][slot_id]['tcp']
        public = self._public_plan(plan)
        public.update({'executed': True, 'actual_tcp': serialize_transform(actual_tcp),
                       'error': pose_error(target_tcp, actual_tcp),
                       'pre_pick_voltage_off_ack': True,
                       'voltage_command_sent': True,
                       'voltage_v': 24,
                       'pre_power_settle_s': pre_power_settle,
                       'hold_feedback_confirmed': False,
                       'hold_feedback_note': (
                           'voltage ACK confirms command acceptance only; '
                           'no independent object-present sensor was used'),
                       'holding_slot': self.holding_slot,
                       'vertical_lift_m': float(self.config.get(
                           'motion', {}).get('vertical_lift_m', 0.20)),
                       'vertical_lift_cartesian_fraction': lift_fraction,
                       'vertical_lift_orientation_adjustment_deg': (
                           lift_orientation_adjustment),
                       'lifted_link7': serialize_transform(lifted_link7),
                       'retreat_observation_joints': retreat_joints})
        return public

    def _command(self, request):
        command = request.command.strip().lower()
        slot_id = int(request.slot_id)
        try:
            with self.lock:
                if slot_id in SLOT_IDS:
                    self.selected_slot = slot_id
            if command == 'calibration_status':
                details = self._calibration_status()
            elif command == 'calibrate_reference':
                observation = self._fresh_observation()
                joint_names, joint_positions = self._fresh_joints()
                calibration = build_calibration(
                    self.config, observation['transform'],
                    quality={'reprojection_error_px': observation['reprojection_error_px'],
                             'pixel_size': observation['pixel_size'],
                             'stability': observation['stability']})
                calibration['camera_observation'] = {
                    'joint_name': 'joint7',
                    'position_rad': float(joint_positions[
                        joint_names.index('joint7')]),
                    'joint_names': list(joint_names),
                    'positions_rad': [float(value) for value in joint_positions],
                    'verified': True,
                    'source': 'captured_with_reference_calibration',
                    'recorded_at': rospy.Time.now().to_sec(),
                }
                self.store.save(calibration)
                self._load_calibration()
                details = self._calibration_status()
            elif command == 'confirm_physical_validation':
                if not self.supervised_commissioning_authorized:
                    raise FiveTargetError(
                        'supervised commissioning is not authorized')
                if not request.supervised_confirm:
                    raise FiveTargetError(
                        'supervised physical validation confirmation is missing')
                if self.computer is None:
                    raise FiveTargetError('valid calibration is required')
                calibration = copy.deepcopy(self.computer.calibration)
                calibration['physical_validation'] = {
                    'passed': True,
                    'source': 'supervised_operator_confirmation',
                    'confirmed_at': rospy.Time.now().to_sec(),
                }
                self.store.save(calibration)
                self._load_calibration()
                details = self._calibration_status()
            elif command == 'confirm_held_tool':
                if not self.supervised_commissioning_authorized:
                    raise FiveTargetError(
                        'supervised commissioning is not authorized')
                if not request.supervised_confirm:
                    raise FiveTargetError(
                        'supervised held-tool confirmation is missing')
                if slot_id not in SLOT_IDS:
                    raise FiveTargetError('held-tool slot must be 1..5')
                if self.holding_slot is not None:
                    raise FiveTargetError(
                        'Tool{} is already recorded as held'.format(
                            self.holding_slot))
                # Recovery is deliberately coupled to a fresh 24V ACK.  It is
                # used only after the operator has visually confirmed that a
                # tool remained attached across a ROS/HMI restart.
                self.voltage.set_voltage(24)
                self._set_holding_slot(slot_id)
                details = {
                    'recovered': True,
                    'holding_slot': self.holding_slot,
                    'voltage_v': 24,
                    'voltage_hold_ack': True,
                    'hold_feedback_confirmed': False,
                    'hold_feedback_note': (
                        'supervised visual confirmation plus voltage ACK; '
                        'no independent object-present sensor was used'),
                }
            elif command == 'detect':
                observation = self._fresh_observation()
                details = {'marker_id': observation['marker_id'],
                           'stamp': observation['stamp'],
                           'base_marker': serialize_transform(observation['transform']),
                           'pixel_size': observation['pixel_size'],
                           'reprojection_error_px': observation['reprojection_error_px'],
                           'stability': observation['stability']}
            elif command in ('compute_targets', 'preview'):
                result = self._compute(slot_id if slot_id in SLOT_IDS else None)
                with self.lock:
                    self.last_targets = result
                self._publish_targets(result)
                details = self._serialize_targets(result)
            elif command == 'plan':
                details = self._public_plan(self._plan(slot_id))
            elif command == 'plan_configured':
                details = self._public_configured_plan(
                    self._plan_configured_pick(slot_id))
            elif command == 'execute_configured':
                if not request.execute or not request.supervised_confirm:
                    raise FiveTargetError(
                        'execute and supervised confirmation are required')
                details = self._execute_configured_pick(slot_id)
            elif command == 'plan_return_held_tool':
                if self.holding_slot is None:
                    raise FiveTargetError('no held tool is recorded')
                if int(slot_id) != int(self.holding_slot):
                    raise FiveTargetError(
                        'held tool is Tool{}, not Tool{}'.format(
                            self.holding_slot, slot_id))
                active = list(self._move_group().get_active_joints())
                current = list(self._move_group().get_current_joint_values())
                observation_joints = self._camera_observation_target(
                    active, current)
                tolerance = float(self.config.get(
                    'camera_observation', {}).get(
                        'joint_tolerance_rad', 0.02))
                if not joint_target_reached(
                        current, observation_joints, tolerance):
                    raise FiveTargetError(
                        'held-tool return must start from the camera observation pose')
                details = self._public_plan(self._plan_return_held_tool(
                    slot_id, observation_joints=observation_joints))
            elif command == 'move_camera_observation':
                if not request.execute:
                    raise FiveTargetError('request execute flag is false')
                details = self._execute_camera_observation_only()
            elif command == 'move_selected_target':
                if not request.execute:
                    raise FiveTargetError('request execute flag is false')
                details = self._move_selected_target_only(slot_id)
            elif command == 'engage_and_retreat':
                if not request.execute or not request.supervised_confirm:
                    raise FiveTargetError(
                        'execute and supervised confirmation are required')
                details = self._engage_staged_and_retreat(slot_id)
            elif command == 'release_held_tool':
                if not request.execute or not request.supervised_confirm:
                    raise FiveTargetError(
                        'execute and supervised confirmation are required')
                if self.holding_slot is None:
                    raise FiveTargetError('no held tool is recorded')
                released_slot = self.holding_slot
                self.voltage.set_voltage(0)
                self._set_holding_slot(None)
                details = {'executed': True,
                           'released_slot': released_slot,
                           'voltage_v': 0,
                           'voltage_off_ack': True}
            elif command == 'return_held_tool':
                if not request.execute or not request.supervised_confirm:
                    raise FiveTargetError(
                        'execute and supervised confirmation are required')
                details = self._return_held_tool(slot_id)
            elif command == 'execute_selected':
                if not request.execute:
                    raise FiveTargetError('request execute flag is false')
                details = self._execute(slot_id)
            elif command in ('cancel', 'stop'):
                self.cancel_requested.set()
                if self.group is not None:
                    self.group.stop()
                if self.configured_pick_status.get('status') in ('PLANNING', 'RUNNING'):
                    self._set_configured_step(
                        'CANCELLED', self.configured_pick_status.get('step', 0),
                        '用户取消，已停止后续步骤',
                        self.configured_pick_status.get('slot_id'))
                self.configured_plan = None
                details = {'cancelled': True}
            elif command == 'status':
                details = self._status()
            else:
                raise FiveTargetError('unknown command {}'.format(command))
            self._publish_status()
            return TaskCommandResponse(True, 'READY', 'accepted',
                                       json.dumps(details, sort_keys=True))
        except Exception as exc:
            if command in ('plan_configured', 'execute_configured') and self.configured_pick_status.get(
                    'status') not in ('FAILED', 'CANCELLED'):
                self._set_configured_step(
                    'FAILED', self.configured_pick_status.get('step', 0),
                    'Dry Run或执行失败，未继续运动', slot_id, error=exc)
            if command == 'plan_configured':
                self.configured_plan = None
            self._publish_status()
            return TaskCommandResponse(False, 'REJECTED', str(exc),
                                       json.dumps(self._status(), sort_keys=True))


def main():
    rospy.init_node('rm75_five_target_server')
    try:
        FiveTargetServer()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('%s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
