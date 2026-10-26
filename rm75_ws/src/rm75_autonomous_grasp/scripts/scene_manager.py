#!/usr/bin/env python3
"""One source of geometry for Gazebo and MoveIt, plus deterministic dynamic obstacle injection."""
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool, SetBoolResponse

def collision(spec):
    c = CollisionObject(); c.id = spec['id']; c.header.frame_id = 'base_link'
    p = SolidPrimitive(); p.type = p.BOX if spec['type'] == 'box' else p.CYLINDER; p.dimensions = spec['size']
    pose = Pose(); pose.position.x, pose.position.y, pose.position.z = spec['xyz']; pose.orientation.w = 1
    c.primitives = [p]; c.primitive_poses = [pose]; c.operation = c.ADD
    return c

def sdf(spec, static=True):
    d = spec['size']; geom = '<box><size>%s</size></box>' % ' '.join(map(str,d)) if spec['type']=='box' else '<cylinder><radius>%s</radius><length>%s</length></cylinder>'%(d[1],d[0])
    color=' '.join(map(str,spec['color']))
    return """<sdf version="1.6"><model name="%s"><static>%s</static><link name="link"><inertial><mass>0.05</mass><inertia><ixx>0.000014</ixx><iyy>0.000014</iyy><izz>0.000014</izz></inertia></inertial><collision name="collision"><geometry>%s</geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision><visual name="visual"><geometry>%s</geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual></link></model></sdf>""" % (spec['id'],str(static).lower(),geom,geom,color,color)

class SceneManager:
    def __init__(self):
        for name in ['/gazebo/spawn_sdf_model','/apply_planning_scene']: rospy.wait_for_service(name,60)
        self.spawn=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel);self.delete=rospy.ServiceProxy('/gazebo/delete_model',DeleteModel);self.apply=rospy.ServiceProxy('/apply_planning_scene',ApplyPlanningScene)
        for spec in rospy.get_param('~objects'):self.add(spec)
        self.add(rospy.get_param('~target'),target=True)
        rospy.set_param('/scene_ready',True)
        self.dynamic=False
        self.service=rospy.Service('/scene/dynamic_obstacle',SetBool,self.inject)
        rospy.loginfo('Scene loaded: shared Gazebo/MoveIt geometry; target pose is not sent to perception')
    def add(self,spec,target=False):
        c=collision(spec);res=self.spawn(spec['id'],sdf(spec,not target),'',c.primitive_poses[0],'world')
        if not res.success and 'already exists' not in res.status_message:raise RuntimeError(res.status_message)
        if not target:
            scene=PlanningScene();scene.is_diff=True;scene.world.collision_objects=[c]
            if not self.apply(scene).success:raise RuntimeError('Scene rejected')
    def inject(self,req):
        spec={'id':'dynamic_obstacle','type':'box','size':rospy.get_param('~dynamic_size',[0.12,0.12,0.18]),'xyz':rospy.get_param('~dynamic_xyz',[0.34,0.045,0.30]),'color':[0.8,0.6,0.1,1]}
        if req.data and not self.dynamic:self.add(spec);self.dynamic=True
        elif not req.data and self.dynamic:
            self.delete(spec['id']);c=collision(spec);c.operation=c.REMOVE;s=PlanningScene();s.is_diff=True;s.world.collision_objects=[c];self.apply(s);self.dynamic=False
        return SetBoolResponse(True,'inserted' if self.dynamic else 'removed; OctoMap clears only observed free space')
if __name__=='__main__':
    rospy.init_node('scene_manager')
    SceneManager();rospy.spin()
