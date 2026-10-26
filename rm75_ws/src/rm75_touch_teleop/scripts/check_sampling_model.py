#!/usr/bin/env python3
"""Read-only FK comparison. Requires controller World and zero-offset Arm_Tip.
Controller pose uses its configured TCP: nonzero tool/work frames invalidate this comparison.
"""
import json
import xml.etree.ElementTree as ET
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from rm_msgs.msg import Joint_PoseEuler
from tf.transformations import euler_matrix, rotation_matrix
rospy.init_node('sampling_readonly_model_check', anonymous=True)
js = rospy.wait_for_message('/joint_states', JointState, timeout=5)
pose = rospy.wait_for_message('/rm_driver/Udp_Joint_PoseEuler', Joint_PoseEuler, timeout=5)
q = dict(zip(js.name, js.position))
robot = ET.fromstring(rospy.get_param('/robot_description'))
transform = np.eye(4)
for i in range(1, 8):
    joint = robot.find('joint[@name="joint%d"]' % i)
    origin = joint.find('origin')
    fixed = euler_matrix(*map(float, origin.get('rpy', '0 0 0').split()))
    fixed[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    turn = rotation_matrix(q[joint.get('name')], list(map(float, joint.find('axis').get('xyz').split())))
    transform = transform.dot(fixed).dot(turn)
error = transform[:3, 3] - pose.position
rotation = transform[:3, :3].T.dot(euler_matrix(*pose.euler)[:3, :3])
print(json.dumps({'fk_position_m': transform[:3, 3].tolist(),
                  'controller_position_m': list(pose.position),
                  'position_error_mm': float(np.linalg.norm(error)*1000),
                  'orientation_error_deg': float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1)))),
                  'joint7_origin': robot.find('joint[@name="joint7"]/origin').attrib,
                  'assumption': 'World identity and zero TCP offset; stationary robot'}, indent=2))
