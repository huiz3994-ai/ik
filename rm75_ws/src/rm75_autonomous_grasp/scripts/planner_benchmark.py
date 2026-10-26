#!/usr/bin/env python3
"""Construct an actual blocked joint-space straight path, compare OMPL planners.
Planning only: does not execute trajectories. Use a fresh simulation.
"""
import json,sys,time,math,copy
import rospy,moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState,CollisionObject
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionFK,GetPositionFKRequest,GetStateValidity,GetStateValidityRequest
rospy.init_node('planner_benchmark');moveit_commander.roscpp_initialize(sys.argv)
group=moveit_commander.MoveGroupCommander('arm');scene=moveit_commander.PlanningSceneInterface(synchronous=True)
start=time.monotonic()
while not rospy.get_param('/observation_ready',False):
    if time.monotonic()-start>60:sys.exit('Observation timeout')
    time.sleep(.1)
names=['joint%d'%i for i in range(1,8)];q0=group.get_current_joint_values()
fk=rospy.ServiceProxy('/compute_fk',GetPositionFK);validity=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
def state(q):
    r=RobotState();r.joint_state=JointState(name=names,position=q);return r
def valid(q):
    req=GetStateValidityRequest();req.robot_state=state(q);req.group_name='arm';return validity(req).valid
def tcp(q):
    req=GetPositionFKRequest();req.header.frame_id='base_link';req.fk_link_names=['grasp_tcp'];req.robot_state=state(q);r=fk(req)
    if r.error_code.val!=1:raise RuntimeError('FK failed')
    return r.pose_stamped[0]
def straight(a,b):
    n=max(1,int(math.ceil(max(abs(x-y) for x,y in zip(a,b))/.035)))
    return [valid([x+(y-x)*i/n for x,y in zip(a,b)]) for i in range(n+1)]
results=[];found=False
try:
    for delta in [.8,-.8,1.2,-1.2]:
        q1=q0[:];q1[0]+=delta;mid=[(a+b)/2 for a,b in zip(q0,q1)];pose=tcp(mid)
        scene.add_box('benchmark_blocker',pose,size=(.045,.045,.045))
        direct=straight(q0,q1)
        if valid(q0) and valid(q1) and not all(direct):found=True;break
        scene.remove_world_object('benchmark_blocker')
    if not found:raise RuntimeError('Cannot construct valid endpoints with blocked straight path')
    for planner in rospy.get_param('~planners',['RRTConnect','RRTstar','PRM','KPIECE','EST']):
        group.set_start_state(state(q0));group.set_joint_value_target(q1);group.set_planner_id(planner);group.set_planning_time(rospy.get_param('~planning_time',3.0));group.set_num_planning_attempts(1)
        begin=time.monotonic();ok,trajectory,planning_time,code=group.plan();wall=time.monotonic()-begin
        safe=ok;length=0;points=trajectory.joint_trajectory.points
        # Interpolate every returned segment independently with MoveIt's state validity service.
        for a,b in zip(points,points[1:]):
            length+=math.sqrt(sum((x-y)**2 for x,y in zip(a.positions,b.positions)))
            if not all(straight(a.positions,b.positions)):safe=False;break
        results.append({'planner':planner,'success':bool(ok),'validated_collision_free':bool(safe),'planning_seconds':planning_time if ok else None,'wall_seconds':wall,'joint_path_length_rad':length,'waypoints':len(points),'error_code':code.val})
    report={'straight_path_blocked':True,'valid_start_and_goal':True,'start':q0,'goal':q1,'results':results};print(json.dumps(report,indent=2));passed=all(x['success'] and x['validated_collision_free'] for x in results)
finally:scene.remove_world_object('benchmark_blocker');moveit_commander.roscpp_shutdown()
sys.exit(0 if passed else 1)
