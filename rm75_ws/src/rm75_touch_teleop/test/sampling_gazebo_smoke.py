#!/usr/bin/env python3
"""Opt-in test for an isolated sampling_sim launch with both Touch nodes disabled."""
import json
import math
import statistics
import time
import rospy
import rosnode
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String, Bool
from rm_msgs.msg import Six_Force
from std_srvs.srv import Trigger

rospy.init_node('sampling_gazebo_smoke')
assert rospy.get_param('/use_sim_time', False), 'Simulation clock required'
assert rospy.get_param('/rm75_touch_teleop/output_mode') == 'trajectory'
assert rospy.get_param('/rm75_touch_teleop/force_topic') == '/sampling_sim/wrench'
assert '/gazebo' in rosnode.get_node_names(), 'Gazebo required'
assert not any(n.startswith('/rm_driver') or n.startswith('/Geomagic') or n == '/touch_pubforce'
               for n in rosnode.get_node_names()), 'Real driver/Touch must not be running'
pose_pub = rospy.Publisher('/Geomagic/pose_raw', PoseStamped, queue_size=1)
joy_pub = rospy.Publisher('/Geomagic/joy', Joy, queue_size=1)
inc_pub = rospy.Publisher('/Touch_Pose_increment', Float64MultiArray, queue_size=1)
state = {'mode': '', 'active': False, 'force': 0., 'reason': '',
         'haptic_raw': 0., 'haptic_filtered': 0., 'haptic_command': 0.}
rospy.Subscriber('/rm75_touch_teleop/control_mode', String, lambda m: state.update(mode=m.data))
rospy.Subscriber('/rm75_touch_teleop/active', Bool, lambda m: state.update(active=m.data))
rospy.Subscriber('/rm75_touch_teleop/stop_reason', String, lambda m: state.update(reason=m.data))
rospy.Subscriber('/sampling_sim/wrench', Six_Force, lambda m: state.update(force=m.force_Fz))
norm = lambda m: math.sqrt(m.vector.x*m.vector.x + m.vector.y*m.vector.y + m.vector.z*m.vector.z)
rospy.Subscriber('/rm75_touch_teleop/haptic_raw_force', Vector3Stamped,
                 lambda m: state.update(haptic_raw=norm(m)))
rospy.Subscriber('/rm75_touch_teleop/haptic_filtered_force', Vector3Stamped,
                 lambda m: state.update(haptic_filtered=norm(m)))
rospy.Subscriber('/rm75_touch_teleop/haptic_command', Vector3Stamped,
                 lambda m: state.update(haptic_command=norm(m)))

def publish(y, grey):
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.pose.orientation.w = 1.
    p.pose.position.y = y
    pose_pub.publish(p)
    inc_pub.publish(Float64MultiArray(data=[0., y, 0.]))
    joy_pub.publish(Joy(buttons=[int(grey), 0]))

# Launch auto-initialization needs to finish before surface calibration.
for _ in range(500):
    publish(0., False)
    time.sleep(.01)
rospy.wait_for_service('/sampling_soil/set_surface', timeout=10)
response = rospy.ServiceProxy('/sampling_soil/set_surface', Trigger)()
assert response.success, response.message
for _ in range(40):
    publish(0., False)
    time.sleep(.01)
start = time.monotonic()
y = 0.
contact_time = None
samples = []
max_force = 0.
max_haptic = {'raw': 0., 'filtered': 0., 'command': 0.}
try:
    while time.monotonic() - start < 35:
        if state['mode'] == 'SAFE_STOP' and time.monotonic() - start > .5:
            raise AssertionError('SAFE_STOP: '+state['reason'])
        if state['active'] and state['mode'] == 'POSITION_MODE':
            y -= .025  # 2.5 mm/s in Touch coordinates at 100 Hz
        if state['mode'] == 'CONTACT_MODE' and contact_time is None:
            contact_time = time.monotonic()
        publish(y, True)
        max_force = max(max_force, abs(state['force']))
        for key in max_haptic:
            max_haptic[key] = max(max_haptic[key], state['haptic_'+key])
        if contact_time is not None and time.monotonic()-contact_time > 12:
            samples.append(state['force'])
            if len(samples) >= 200:
                break
        time.sleep(.01)
    assert contact_time is not None, 'No contact reached'
    assert len(samples) >= 200, 'Insufficient settled contact samples'
    assert max_force < 100, max_force
    assert abs(statistics.mean(samples)+50) < 3, statistics.mean(samples)
    assert max_haptic['raw'] > 10., max_haptic
    assert max_haptic['filtered'] > 5., max_haptic
    assert max_haptic['command'] > .1, max_haptic
    print(json.dumps({'result':'PASS', 'settled_mean_N': statistics.mean(samples),
                      'settled_min_N': min(samples), 'settled_max_N': max(samples),
                      'peak_abs_N': max_force, 'samples': len(samples),
                      'max_haptic': max_haptic}, indent=2))
finally:
    for _ in range(25):
        publish(y, False)
        time.sleep(.01)
