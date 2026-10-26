#!/usr/bin/env python3
"""Protocol tests against a mock driver on rostest's isolated ROS master.
Never launch this script on a live robot master.
"""
import unittest,time,threading
import rospy,rostest,actionlib
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty,Bool
from rm_msgs.msg import JointPos
from actionlib_msgs.msg import GoalStatus
class BridgeTest(unittest.TestCase):
    def test_contract(self):
        self.q=[0.0]*7;self.commands=0;self.stops=0;self.publish_feedback=True;self.running=True
        names=['joint%d'%i for i in range(1,8)]
        jp=rospy.Publisher('/joint_states',JointState,queue_size=1)
        ack=rospy.Publisher('/rm_driver/Set_Arm_Stop_Result',Bool,queue_size=1)
        def command(m):self.q=list(m.joint);self.commands+=1
        def stop(_):self.stops+=1;ack.publish(Bool(data=True))
        subs=[rospy.Subscriber('/rm_driver/JointPos',JointPos,command),rospy.Subscriber('/rm_driver/Emergency_Stop',Empty,stop)]
        def feedback():
            while self.running and not rospy.is_shutdown():
                if self.publish_feedback:
                    m=JointState();m.header.stamp=rospy.Time.now();m.name=names;m.position=self.q;jp.publish(m)
                time.sleep(.01)
        thread=threading.Thread(target=feedback,daemon=True);thread.start()
        client=actionlib.SimpleActionClient('/rm_group/follow_joint_trajectory',FollowJointTrajectoryAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(8)));time.sleep(.3)
        def goal(delta,duration=1):
            g=FollowJointTrajectoryGoal();g.trajectory.joint_names=list(reversed(names));a=JointTrajectoryPoint();a.positions=list(reversed(self.q));a.time_from_start=rospy.Duration(0);b=JointTrajectoryPoint();b.positions=[v+delta for v in a.positions];b.time_from_start=rospy.Duration(duration);g.trajectory.points=[a,b];return g
        try:
            client.send_goal(goal(.03));self.assertTrue(client.wait_for_result(rospy.Duration(3)));self.assertEqual(client.get_state(),GoalStatus.SUCCEEDED);self.assertGreater(self.commands,10)
            # Invalid dimensionality cannot produce any new position command.
            bad=goal(.01);bad.trajectory.points[1].positions=[0];before=self.commands;client.send_goal(bad);self.assertTrue(client.wait_for_result(rospy.Duration(5)));self.assertEqual(client.get_state(),GoalStatus.ABORTED);self.assertEqual(self.commands,before)
            # Cancel returns only after measured stationarity and stop acknowledgement.
            client.send_goal(goal(.1,4));time.sleep(.25);client.cancel_goal();self.assertTrue(client.wait_for_result(rospy.Duration(5)));self.assertEqual(client.get_state(),GoalStatus.PREEMPTED);self.assertGreaterEqual(self.stops,2)
            # Stale feedback cannot complete a trajectory or report a confirmed stop.
            client.send_goal(goal(.06,3));time.sleep(.15);self.publish_feedback=False;self.assertTrue(client.wait_for_result(rospy.Duration(6)));self.assertEqual(client.get_state(),GoalStatus.ABORTED);self.assertIn('STOP_UNCONFIRMED',client.get_result().error_string)
        finally:self.running=False;thread.join(timeout=1)
if __name__=='__main__':
    rospy.init_node('test_bridge');rostest.rosrun('rm75_autonomous_grasp','bridge_contract',BridgeTest)
