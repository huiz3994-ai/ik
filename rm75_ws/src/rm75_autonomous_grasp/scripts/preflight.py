#!/usr/bin/env python3
import sys,rospy,tf2_ros
from sensor_msgs.msg import JointState,Image,CameraInfo,PointCloud2
from moveit_msgs.srv import GetPlanningScene,GetPlanningSceneRequest
rospy.init_node('grasp_preflight');b=tf2_ros.Buffer();l=tf2_ros.TransformListener(b);errors=[]
for target,source in [('world','base_link'),('base_link','camera_link'),('base_link','grasp_tcp')]:
    try:b.lookup_transform(target,source,rospy.Time(0),rospy.Duration(5))
    except Exception as e:errors.append('TF %s <- %s: %s'%(target,source,e))
for topic,typ in [('/joint_states',JointState),('/camera/color/image_raw',Image),('/camera/aligned_depth_to_color/image_raw',Image),('/camera/color/camera_info',CameraInfo),('/mapping/cloud',PointCloud2)]:
    try:
        m=rospy.wait_for_message(topic,typ,timeout=5)
        if (rospy.Time.now()-m.header.stamp).to_sec()>1:errors.append('Stale '+topic)
    except Exception as e:errors.append(str(e))
try:
    rospy.wait_for_service('/get_planning_scene',5);req=GetPlanningSceneRequest();req.components.components=32
    s=rospy.ServiceProxy('/get_planning_scene',GetPlanningScene)(req).scene
    if not s.world.octomap.octomap.data:errors.append('OctoMap empty')
except Exception as e:errors.append(str(e))
print('\n'.join(errors) if errors else 'PASS: TF, images, CameraInfo, joints, cloud and OctoMap available')
sys.exit(bool(errors))
