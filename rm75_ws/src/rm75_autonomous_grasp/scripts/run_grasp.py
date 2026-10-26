#!/usr/bin/env python3
import sys,rospy,actionlib
from rm75_autonomous_grasp.msg import GraspAction,GraspGoal
rospy.init_node('run_grasp');c=actionlib.SimpleActionClient('/grasp',GraspAction)
if not c.wait_for_server(rospy.Duration(30)):sys.exit('Grasp server unavailable')
c.send_goal(GraspGoal(grasp_mode=rospy.get_param('~mode','top')),feedback_cb=lambda f:rospy.loginfo('%s %s',f.state,f.detail))
try:
    if not c.wait_for_result(rospy.Duration(180)):c.cancel_goal();sys.exit('Task timeout; cancel requested')
    r=c.get_result();print(r);sys.exit(0 if r and r.success else 1)
except KeyboardInterrupt:c.cancel_goal()
