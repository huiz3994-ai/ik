#!/usr/bin/env python3
"""Run an easy_handeye eye-in-hand calibration without the rqt workflow."""

from __future__ import division, print_function

import math
import sys

import rospy
import tf2_ros
from easy_handeye.handeye_client import HandeyeClient


class CalibrationError(RuntimeError):
    pass


def _sample_count(samples):
    return len(samples.hand_world_samples)


def _rotation_distance(a, b):
    dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


class RM75HandeyeAuto(object):
    def __init__(self):
        self.execute_motion = rospy.get_param('~execute_motion', False)
        self.pose_indices = [int(i) for i in rospy.get_param('~pose_indices', list(range(17)))]
        self.settle_seconds = float(rospy.get_param('~settle_seconds', 2.0))
        self.min_samples = int(rospy.get_param('~min_samples', 8))
        self.algorithm = rospy.get_param('~algorithm', 'OpenCV/Tsai-Lenz')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.marker_frame = rospy.get_param('~marker_frame', 'camera_marker')
        self.tf_timeout = float(rospy.get_param('~tf_timeout', 10.0))
        self.stability_samples = int(rospy.get_param('~stability_samples', 5))
        self.stability_period = float(rospy.get_param('~stability_period', 0.15))
        self.max_marker_translation = float(rospy.get_param('~max_marker_translation', 0.004))
        self.max_marker_rotation = math.radians(float(rospy.get_param('~max_marker_rotation_deg', 1.5)))
        self.abort_on_pose_failure = rospy.get_param('~abort_on_pose_failure', False)
        self.reset_samples = rospy.get_param('~reset_samples', True)
        self.start_pose_plan_attempts = int(rospy.get_param('~start_pose_plan_attempts', 5))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = HandeyeClient(rospy.get_namespace())

    def _clear_samples(self):
        samples = self.client.get_sample_list()
        while _sample_count(samples):
            samples = self.client.remove_sample(_sample_count(samples) - 1)
        rospy.loginfo('Cleared previous easy_handeye samples')

    def _wait_for_stable_marker(self):
        deadline = rospy.Time.now() + rospy.Duration(self.tf_timeout)
        transforms = []
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.camera_frame, self.marker_frame, rospy.Time(0), rospy.Duration(1.0))
                transforms.append(transform.transform)
                transforms = transforms[-self.stability_samples:]
                if len(transforms) == self.stability_samples:
                    reference = transforms[0]
                    stable = True
                    for current in transforms[1:]:
                        dx = current.translation.x - reference.translation.x
                        dy = current.translation.y - reference.translation.y
                        dz = current.translation.z - reference.translation.z
                        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
                        angle = _rotation_distance(current.rotation, reference.rotation)
                        if distance > self.max_marker_translation or angle > self.max_marker_rotation:
                            stable = False
                            break
                    if stable:
                        return
                rospy.sleep(self.stability_period)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                transforms = []
                rospy.sleep(self.stability_period)
        raise CalibrationError(
            'ArUco TF {} -> {} is missing or unstable'.format(self.camera_frame, self.marker_frame))

    def _take_sample(self, label):
        self._wait_for_stable_marker()
        samples = self.client.take_sample()
        rospy.loginfo('Captured sample %d at %s', _sample_count(samples), label)
        return samples

    def _move_and_sample(self, pose_index):
        selected = self.client.select_target_pose(pose_index)
        if not selected.success:
            raise CalibrationError('Could not select target pose {}'.format(pose_index))
        planned = self.client.plan_to_selected_target_pose()
        if not planned.success:
            raise CalibrationError('MoveIt planning failed for target pose {}'.format(pose_index))
        executed = self.client.execute_plan()
        if not executed.success:
            raise CalibrationError('MoveIt execution failed for target pose {}'.format(pose_index))
        rospy.sleep(self.settle_seconds)
        return self._take_sample('target pose {}'.format(pose_index))

    def run(self):
        if not self.execute_motion:
            raise CalibrationError(
                'Automatic motion is locked. Re-run with execute_motion:=true after clearing the workspace.')
        if len(self.pose_indices) + 1 < self.min_samples:
            raise CalibrationError('pose_indices cannot produce the requested minimum sample count')

        algorithms = self.client.list_algorithms()
        if self.algorithm not in algorithms.algorithms:
            raise CalibrationError(
                'Algorithm {} is unavailable; choose one of {}'.format(self.algorithm, algorithms.algorithms))
        if not self.client.set_algorithm(self.algorithm).success:
            raise CalibrationError('Failed to select calibration algorithm {}'.format(self.algorithm))

        if self.reset_samples:
            self._clear_samples()

        rospy.logwarn('RM75 AUTOMATIC MOTION STARTING. Keep the emergency stop within reach.')
        start = None
        for attempt in range(1, self.start_pose_plan_attempts + 1):
            start = self.client.check_starting_pose()
            if start.can_calibrate:
                break
            rospy.logwarn('Calibration-envelope planning attempt %d/%d failed; retrying',
                          attempt, self.start_pose_plan_attempts)
        if not start.can_calibrate:
            raise CalibrationError(
                'The current RM75 pose cannot reach the calibration motion envelope. Reposition it and retry.')

        target_count = len(start.target_poses.target_poses)
        invalid = [i for i in self.pose_indices if i < 0 or i >= target_count]
        if invalid:
            raise CalibrationError('Invalid target pose indices {} (available: 0..{})'.format(
                invalid, target_count - 1))

        rospy.sleep(self.settle_seconds)
        samples = self._take_sample('starting pose')
        for pose_index in self.pose_indices:
            if rospy.is_shutdown():
                return
            try:
                samples = self._move_and_sample(pose_index)
            except (CalibrationError, rospy.ServiceException) as exc:
                rospy.logerr('Pose %d skipped: %s', pose_index, exc)
                if self.abort_on_pose_failure:
                    raise

        count = _sample_count(samples)
        if count < self.min_samples:
            raise CalibrationError('Only {} valid samples were captured; at least {} are required'.format(
                count, self.min_samples))

        result = self.client.compute_calibration()
        if not result.valid:
            raise CalibrationError('easy_handeye could not compute a valid calibration from {} samples'.format(count))
        self.client.save()

        transform = result.calibration.transform
        t = transform.transform.translation
        q = transform.transform.rotation
        rospy.loginfo(
            'CALIBRATION SAVED: %s -> %s | xyz=[%.9f, %.9f, %.9f] qxyzw=[%.9f, %.9f, %.9f, %.9f]',
            transform.header.frame_id, transform.child_frame_id,
            t.x, t.y, t.z, q.x, q.y, q.z, q.w)


def main():
    rospy.init_node('rm75_handeye_auto')
    try:
        RM75HandeyeAuto().run()
    except (CalibrationError, rospy.ROSException, rospy.ServiceException) as exc:
        rospy.logfatal('%s', exc)
        sys.exit(1)


if __name__ == '__main__':
    main()
