#!/usr/bin/env python3
"""Inject a Gazebo-only obstacle during approach. No PlanningScene API injection.
The D435i point cloud and OctoMap must invalidate the running trajectory.
Run against a freshly started sim.launch (one task per reset).
"""
import json,time,threading,sys
import rospy,actionlib
from rm75_autonomous_grasp.msg import GraspAction,GraspGoal
from gazebo_msgs.srv import SpawnModel,DeleteModel,GetModelState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String
rospy.init_node('dynamic_test')
start=time.monotonic()
while not rospy.get_param('/observation_ready',False):
    if time.monotonic()-start>60:sys.exit('Observation pose timeout')
    time.sleep(.1)
get_model=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
z_before=get_model('target','world').pose.position.z
records=[];cancel_times=[];spawned=[];count=0;injecting=False
mode=rospy.get_param('~mode','obstacle')
def inject():
    global injecting
    injecting=True
    if mode=='cloud_loss':
        import rosnode
        rosnode.kill_nodes(['/obstacle_map']);spawned.append(time.monotonic());return
    # Fixed test geometry remains ahead of the approaching TCP.
    p=Pose();p.position.x=.34;p.position.y=.045;p.position.z=.19;p.orientation.w=1
    xml='<sdf version="1.6"><model name="vision_dynamic"><static>true</static><link name="link"><collision name="c"><geometry><box><size>0.075 0.09 0.035</size></box></geometry></collision><visual name="v"><geometry><box><size>0.075 0.09 0.035</size></box></geometry><material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material></visual></link></model></sdf>'
    r=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)('vision_dynamic',xml,'',p,'world')
    if r.success:spawned.append(time.monotonic())
def execution(_):
    global count
    count+=1
    if count==2 and not injecting:threading.Thread(target=inject,daemon=True).start()
rospy.Subscriber('/execute_trajectory/goal',ExecuteTrajectoryActionGoal,execution)
def on_cancel(_):
    cancel_times.append(time.monotonic())
    if mode=='recovery' and spawned:
        def remove():
            time.sleep(.2)
            rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)('vision_dynamic')
        threading.Thread(target=remove,daemon=True).start()
rospy.Subscriber('/execute_trajectory/cancel',GoalID,on_cancel)
rospy.Subscriber('/grasp/state',String,lambda m:records.append(m.data))
c=actionlib.SimpleActionClient('/grasp',GraspAction);c.wait_for_server(rospy.Duration(30));c.send_goal(GraspGoal(grasp_mode='top'))
if not c.wait_for_result(rospy.Duration(120)):c.cancel_goal();sys.exit('Task timeout')
r=c.get_result();latency=next((t-spawned[0] for t in cancel_times if spawned and t>=spawned[0]),None)
lift=get_model('target','world').pose.position.z-z_before
report={'lift_m':lift,'mode':mode,'injected':bool(spawned),'task_success':bool(r and r.success),'error':r.error if r else 'none','cancel_latency_wall_s':latency,'states':records,'execute_calls':count,'obstacle_source':'Gazebo RGB-D only; no CollisionObject insertion'}
print(json.dumps(report,indent=2))
# A blocked goal is expected to fail safely. Recovery success is tested separately.
expected='TIMEOUT' if mode=='cloud_loss' else 'COLLISION'
passed=bool(spawned) and latency is not None and any(expected in s for s in records) and (report['task_success'] and count>=3 and lift>0.08 if mode=='recovery' else not report['task_success'])
sys.exit(0 if passed else 1)
