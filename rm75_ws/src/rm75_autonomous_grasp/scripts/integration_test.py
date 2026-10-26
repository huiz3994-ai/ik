#!/usr/bin/env python3
"""Runtime acceptance: perception, task result, physically lifted Gazebo object."""
import json,sys,time,rospy,actionlib
from gazebo_msgs.srv import GetModelState
from rm75_autonomous_grasp.msg import GraspAction,GraspGoal
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ExecuteTrajectoryActionGoal,RobotState
from moveit_msgs.srv import GetStateValidity,GetStateValidityRequest
from sensor_msgs.msg import JointState
import math
rospy.init_node('grasp_integration_test')
rospy.wait_for_service('/gazebo/get_model_state',60);get=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
deadline=time.monotonic()+60
while not rospy.get_param('/observation_ready',False):
    if time.monotonic()>deadline:sys.exit('Observation initialization failed')
    time.sleep(.1)
detected=rospy.wait_for_message('/perception/object_pose',PoseStamped,timeout=60).pose.position
truth=get('target','world').pose.position
z0=truth.z
xy_error=((detected.x-truth.x)**2+(detected.y-truth.y)**2)**0.5
surface_z_error=abs(detected.z-(truth.z+.02))
client=actionlib.SimpleActionClient('/grasp',GraspAction)
if not client.wait_for_server(rospy.Duration(60)):sys.exit('No task server')
start=time.monotonic();states=[];straight_collisions=[];checked=[]
def check_direct(msg):
    if checked:return
    checked.append(True)
    t=msg.goal.trajectory.joint_trajectory
    a=t.points[0].positions;b=t.points[-1].positions
    n=max(2,int(math.ceil(max(abs(x-y) for x,y in zip(a,b))/.02)))
    service=rospy.ServiceProxy('/check_state_validity',GetStateValidity)
    for i in range(n+1):
        q=[x+(y-x)*i/n for x,y in zip(a,b)]
        req=GetStateValidityRequest();req.group_name='arm';req.robot_state=RobotState();req.robot_state.joint_state=JointState(name=t.joint_names,position=q)
        r=service(req)
        if not r.valid:
            straight_collisions.extend(sorted(set((c.contact_body_1,c.contact_body_2) for c in r.contacts)))
            break
rospy.Subscriber('/execute_trajectory/goal',ExecuteTrajectoryActionGoal,check_direct)
client.send_goal(GraspGoal(grasp_mode='top'),feedback_cb=lambda f:states.append(f.state))
if not client.wait_for_result(rospy.Duration(180)):client.cancel_goal();sys.exit('Task timeout')
r=client.get_result();z1=get('target','world').pose.position.z
report={'success':bool(r and r.success),'error':r.error if r else 'no result','lift_m':z1-z0,'wall_seconds':time.monotonic()-start,'perception_xy_error_m':xy_error,'perception_top_surface_z_error_m':surface_z_error,'states':states,'direct_pregrasp_collisions':straight_collisions,'gripper_model':'idealized fixed attachment'}
print(json.dumps(report,indent=2));sys.exit(0 if report['success'] and z1-z0>0.08 and xy_error<.015 and surface_z_error<.01 else 1)
