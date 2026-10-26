#!/usr/bin/env python3
"""Detect configured AprilTag/QR stations and publish stable base-frame 6D poses."""

import json
import os
import sys
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String, UInt8
from tf2_geometry_msgs import do_transform_pose

from rm75_qr_task.marker_manager import MarkerManager, MarkerQualityError
from rm75_qr_task.pnp import PnPError, solve_square_pose
from rm75_qr_task.pose_manager import transform_matrix, transform_parts


class MarkerNode(object):
    def __init__(self):
        config_path = rospy.get_param('~markers_config')
        with open(os.path.expanduser(config_path), 'r') as stream:
            config = yaml.safe_load(stream)
        self.marker_type = config.get('marker_type', 'apriltag36h11')
        if self.marker_type not in ('apriltag16h5', 'apriltag36h11', 'qr'):
            raise RuntimeError(
                'marker_type must be apriltag16h5, apriltag36h11 or qr')
        self.base_frame = config.get('base_frame', 'base_link')
        self.slots = {int(item['id']): item for item in config['slots']}
        self.marker_to_slot = {
            int(item.get('marker_id', item['id'])): int(item['id'])
            for item in config['slots']
        }
        if len(self.marker_to_slot) != len(self.slots):
            raise RuntimeError('every configured marker_id must be unique')
        self.manager = MarkerManager(
            self.slots,
            stability_samples=config.get('stability_samples', 5),
            max_age=config.get('max_age_s', 0.5),
            max_translation_jump=config.get('max_translation_jump_m', 0.006),
            max_rotation_jump_deg=config.get('max_rotation_jump_deg', 2.0),
            max_reprojection_error=config.get('max_reprojection_error_px', 2.0),
            clock=lambda: rospy.Time.now().to_sec())
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.handeye_publishers = {}
        rospy.Subscriber('/tf_static', TFMessage, self._tf_publishers,
                         queue_size=20)
        rospy.Subscriber('/tf', TFMessage, self._tf_publishers, queue_size=20)
        self.pose_publishers = {
            slot_id: rospy.Publisher('/qr_task/slot_{}/pose'.format(slot_id),
                                     PoseStamped, queue_size=1)
            for slot_id in self.slots}
        self.markers_pub = rospy.Publisher('/qr_task/markers', PoseArray,
                                           queue_size=1)
        self.target_pub = rospy.Publisher('/qr_task/target_pose', PoseStamped,
                                          queue_size=1)
        self.observation_pub = rospy.Publisher('/qr_task/marker_observation',
                                               String, queue_size=10)
        self.status_pub = rospy.Publisher('/qr_task/vision_status', String,
                                          queue_size=10)
        self.debug_image_pub = rospy.Publisher('/qr_task/debug_image', Image,
                                               queue_size=1)
        self.target_slot = None
        rospy.Subscriber('/qr_task/target_slot', UInt8, self._target, queue_size=1)
        image = Subscriber(config.get('image_topic', '/camera/color/image_raw'), Image)
        info = Subscriber(config.get('camera_info_topic', '/camera/color/camera_info'),
                          CameraInfo)
        self.sync = ApproximateTimeSynchronizer([image, info], 8, 0.05)
        self.sync.registerCallback(self._callback)
        self.qr = cv2.QRCodeDetector()
        self.dictionary = None
        if self.marker_type.startswith('apriltag'):
            dictionary_name = {
                'apriltag16h5': 'DICT_APRILTAG_16h5',
                'apriltag36h11': 'DICT_APRILTAG_36h11',
            }[self.marker_type]
            if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco,
                                                        dictionary_name):
                raise RuntimeError(
                    'OpenCV ArUco {} support is unavailable'.format(
                        self.marker_type))
            self.dictionary = cv2.aruco.getPredefinedDictionary(
                getattr(cv2.aruco, dictionary_name))

    def _target(self, message):
        self.target_slot = int(message.data)

    def _tf_publishers(self, message):
        caller = str((getattr(message, '_connection_header', None) or {}).get(
            'callerid', 'unknown'))
        topic = str((getattr(message, '_connection_header', None) or {}).get(
            'topic', ''))
        now = time.monotonic()
        for transform in message.transforms:
            if transform.child_frame_id.lstrip('/') == 'camera_link':
                self.handeye_publishers[caller] = None if topic.endswith(
                    '/tf_static') else now

    def _handeye_sources(self):
        now = time.monotonic()
        self.handeye_publishers = {
            name: stamp for name, stamp in self.handeye_publishers.items()
            if stamp is None or now - stamp <= 15.0}
        return sorted(self.handeye_publishers)

    def _detections(self, image):
        if self.marker_type.startswith('apriltag'):
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary)
            if ids is None:
                return []
            return [(self.marker_to_slot[int(marker_id)], int(marker_id),
                     np.asarray(corner).reshape(4, 2))
                    for marker_id, corner in zip(ids.reshape(-1), corners)
                    if int(marker_id) in self.marker_to_slot]
        result = self.qr.detectAndDecodeMulti(image)
        if not result or not result[0]:
            return []
        _, decoded, points, _ = result
        by_text = {slot['text']: slot_id for slot_id, slot in self.slots.items()}
        return [(by_text[text], int(by_text[text]), np.asarray(corners).reshape(4, 2))
                for text, corners in zip(decoded, points) if text in by_text]

    @staticmethod
    def _matrix_pose(matrix, header):
        translation, quaternion = transform_parts(matrix)
        pose = PoseStamped()
        pose.header = header
        (pose.pose.position.x, pose.pose.position.y,
         pose.pose.position.z) = translation
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        return pose

    @staticmethod
    def _pose_matrix(pose):
        p, q = pose.pose.position, pose.pose.orientation
        return transform_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def _callback(self, image_message, info):
        age = (rospy.Time.now() - image_message.header.stamp).to_sec()
        if age < 0 or age > self.manager.max_age:
            self.status_pub.publish(String(data=json.dumps(
                {'ok': False, 'reason': 'image stale', 'age_s': age})))
            return
        sources = self._handeye_sources()
        if len(sources) > 1:
            self.status_pub.publish(String(data=json.dumps({
                'ok': False,
                'reason': 'duplicate Link7/camera hand-eye TF publishers',
                'handeye_tf_publishers': sources})))
            return
        if (image_message.header.frame_id != info.header.frame_id or
                image_message.width != info.width or
                image_message.height != info.height):
            self.status_pub.publish(String(data=json.dumps(
                {'ok': False, 'reason': 'image/CameraInfo geometry or frame mismatch'})))
            return
        try:
            image = self.bridge.imgmsg_to_cv2(image_message, 'bgr8')
            detections = self._detections(image)
        except Exception as exc:
            self.status_pub.publish(String(data=json.dumps(
                {'ok': False, 'reason': 'decode failed: {}'.format(exc)})))
            return
        pose_array = PoseArray()
        pose_array.header.stamp = image_message.header.stamp
        pose_array.header.frame_id = self.base_frame
        seen = []
        annotated = image.copy()
        for slot_id, marker_id, corners in detections:
            if slot_id not in self.slots:
                continue
            slot = self.slots[slot_id]
            try:
                camera_marker, error = solve_square_pose(
                    corners, slot.get('size_m', 0.0), np.asarray(info.K).reshape(3, 3),
                    info.D)
                optical_pose = self._matrix_pose(camera_marker, image_message.header)
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, image_message.header.frame_id,
                    image_message.header.stamp, rospy.Duration(0.1))
                base_pose = do_transform_pose(optical_pose, transform)
                base_matrix = self._pose_matrix(base_pose)
                self.manager.add(slot_id, image_message.header.stamp.to_sec(),
                                 base_matrix, error)
                stable = self.manager.stable(slot_id)
                edges = [float(np.linalg.norm(corners[(index + 1) % 4] - corners[index]))
                         for index in range(4)]
                pixel_size = float(np.mean(edges))
                statistics = self.manager.statistics(slot_id)
                seen.append({'slot_id': slot_id, 'marker_id': marker_id,
                             'reprojection_error_px': error,
                             'pixel_size': pixel_size, 'corners_px': corners.tolist(),
                             'stable': stable, 'stability': statistics})
                color = (0, 200, 0) if stable else (0, 180, 255)
                points = np.rint(corners).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated, [points], True, color, 2)
                cv2.putText(annotated, 'ID {} {:.1f}px'.format(marker_id, pixel_size),
                            tuple(points[0, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            color, 1, cv2.LINE_AA)
                if not stable:
                    continue
                self.pose_publishers[slot_id].publish(base_pose)
                pose_array.poses.append(base_pose.pose)
                if self.target_slot == slot_id:
                    self.target_pub.publish(base_pose)
                tf_message = TransformStamped()
                tf_message.header = base_pose.header
                tf_message.child_frame_id = 'qr_slot_{}_marker'.format(slot_id)
                tf_message.transform.translation.x = base_pose.pose.position.x
                tf_message.transform.translation.y = base_pose.pose.position.y
                tf_message.transform.translation.z = base_pose.pose.position.z
                tf_message.transform.rotation = base_pose.pose.orientation
                self.tf_broadcaster.sendTransform(tf_message)
                observation = {
                    'slot_id': slot_id,
                    'marker_id': marker_id,
                    'stamp': base_pose.header.stamp.to_sec(),
                    'frame_id': self.base_frame,
                    'reprojection_error_px': error,
                    'pixel_size': pixel_size,
                    'corners_px': corners.tolist(),
                    'stability': statistics,
                    'translation': [base_pose.pose.position.x,
                                    base_pose.pose.position.y,
                                    base_pose.pose.position.z],
                    'quaternion_xyzw': [base_pose.pose.orientation.x,
                                        base_pose.pose.orientation.y,
                                        base_pose.pose.orientation.z,
                                        base_pose.pose.orientation.w],
                }
                self.observation_pub.publish(String(data=json.dumps(observation)))
            except (PnPError, MarkerQualityError, tf2_ros.TransformException,
                    ValueError) as exc:
                seen.append({'slot_id': slot_id, 'stable': False,
                             'reason': str(exc)})
        try:
            debug = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug.header = image_message.header
            self.debug_image_pub.publish(debug)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, 'Cannot publish marker debug image: %s', exc)
        self.markers_pub.publish(pose_array)
        self.status_pub.publish(String(data=json.dumps(
            {'ok': bool(pose_array.poses), 'marker_type': self.marker_type,
             'detections': seen, 'stamp': image_message.header.stamp.to_sec(),
             'handeye_tf_publishers': sources,
             'handeye_tf_unique': len(sources) <= 1},
            sort_keys=True)))


def main():
    rospy.init_node('rm75_qr_marker')
    try:
        MarkerNode()
        rospy.spin()
    except Exception as exc:
        rospy.logfatal('%s', exc)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
