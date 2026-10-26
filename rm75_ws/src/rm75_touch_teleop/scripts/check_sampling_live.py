#!/usr/bin/env python3
"""Read-only live diagnostics. Never publishes, calls robot services or changes params."""
import argparse
import json
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String, UInt16, Float64MultiArray
from rm_msgs.msg import Six_Force, JointPos
from rosgraph_msgs.msg import Log

parser = argparse.ArgumentParser()
parser.add_argument('--duration', type=float, default=10.0)
args = parser.parse_args(rospy.myargv()[1:])
rospy.init_node('sampling_readonly_check', anonymous=True)
records = {}
subscribers = []

def callback(message, topic):
    now = time.monotonic()
    row = records[topic]
    if row['count']:
        row['max_gap_s'] = max(row['max_gap_s'], now-row['last_time'])
    else:
        row['first_time'] = now
    row['last_time'] = now
    row['count'] += 1
    if isinstance(message, Six_Force):
        value = [getattr(message, k) for k in ('force_Fx','force_Fy','force_Fz','force_Mx','force_My','force_Mz')]
    elif isinstance(message, JointState):
        value = list(message.position)
    elif isinstance(message, JointPos):
        value = list(message.joint)
    elif isinstance(message, Joy):
        value = list(message.buttons)
    elif isinstance(message, PoseStamped):
        p = message.pose.position
        value = [p.x, p.y, p.z]
    else:
        value = message.data
    row['latest'] = value
    if isinstance(value, list) and all(isinstance(v, (int,float)) for v in value):
        if not all(math.isfinite(v) for v in value):
            row['nonfinite_messages'] += 1
            return
        if 'min' not in row or len(row['min']) != len(value):
            row['min'], row['max'] = list(value), list(value)
        else:
            row['min'] = [min(a,b) for a,b in zip(row['min'],value)]
            row['max'] = [max(a,b) for a,b in zip(row['max'],value)]

for topic, msg in [('/Geomagic/pose_raw',PoseStamped),('/Geomagic/joy',Joy),
                   ('/Touch_Pose_increment',Float64MultiArray),('/joint_states',JointState),
                   ('/rm_driver/UdpSixZeroForce',Six_Force),('/rm_driver/Udp_Coordinate',UInt16),
                   ('/rm_driver/JointPos',JointPos),('/rm75_touch_teleop/active',Bool),
                   ('/rm75_touch_teleop/control_mode',String),('/rm75_touch_teleop/stop_reason',String)]:
    records[topic] = dict(count=0,max_gap_s=0.,nonfinite_messages=0)
    subscribers.append(rospy.Subscriber(topic,msg,callback,callback_args=topic,queue_size=100))
logs = []
def log_callback(m):
    if m.level >= Log.WARN and len(logs) < 30:
        logs.append({'node':m.name,'message':m.msg})
subscribers.append(rospy.Subscriber('/rosout_agg',Log,log_callback,queue_size=100))
start = time.monotonic()
while time.monotonic()-start < args.duration and not rospy.is_shutdown():
    time.sleep(.05)
for subscriber in subscribers:
    subscriber.unregister()
end = time.monotonic()
for row in records.values():
    if row['count']:
        span = row['last_time']-row['first_time']
        row['rate_hz'] = (row['count']-1)/span if span > 0 else 0
        row['last_age_s'] = end-row.pop('last_time')
        row.pop('first_time')
print(json.dumps({'duration_s':end-start,'parameters':rospy.get_param('/rm75_touch_teleop',{}),
                  'topics':records,'warnings':logs},indent=2,ensure_ascii=False))
