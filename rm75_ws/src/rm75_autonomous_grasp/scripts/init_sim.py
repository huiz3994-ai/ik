#!/usr/bin/env python3
"""Move to the observation pose through MoveIt, after shared scene setup."""
import sys,time,rospy,moveit_commander
rospy.init_node('init_sim')
if not rospy.get_param('/use_sim_time',False):sys.exit('Simulation only')
start=time.monotonic()
while not rospy.is_shutdown() and not rospy.get_param('/scene_ready',False):
    if time.monotonic()-start>60:sys.exit('Scene readiness timeout')
    time.sleep(.1)
moveit_commander.roscpp_initialize(sys.argv)
group=moveit_commander.MoveGroupCommander('arm',wait_for_servers=60)
group.set_max_velocity_scaling_factor(.3);group.set_max_acceleration_scaling_factor(.2)
group.set_planning_time(10);group.set_joint_value_target([0,.2,0,.9,0,2.04159,0])
if not group.go(wait=True):sys.exit('Observation pose failed')
rospy.set_param('/observation_ready',True)
rospy.loginfo('Observation pose reached via MoveIt')
moveit_commander.roscpp_shutdown()
