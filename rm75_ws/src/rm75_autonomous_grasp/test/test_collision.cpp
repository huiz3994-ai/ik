#include <gtest/gtest.h>
#include <rm75_autonomous_grasp/core.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
TEST(FCL, InterpolatedSegmentHitsObstacle) {
  const std::string xml =
      R"(<robot name="test"><link name="base"/><link name="tool"><collision><geometry><sphere radius="0.02"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>)";
  auto u = urdf::parseURDF(xml);
  auto srdf = std::make_shared<srdf::Model>();
  ASSERT_TRUE(
      srdf->initString(*u, "<robot name='test'><group name='arm'><joint "
                           "name='slide'/></group></robot>"));
  auto model = std::make_shared<moveit::core::RobotModel>(u, srdf);
  planning_scene::PlanningScene s(model);
  s.setActiveCollisionDetector(
      collision_detection::CollisionDetectorAllocatorFCL::create(), true);
  moveit_msgs::CollisionObject o;
  o.id = "obstacle";
  o.header.frame_id = "base";
  o.operation = o.ADD;
  shape_msgs::SolidPrimitive shape;
  shape.type = shape.BOX;
  shape.dimensions = {.04, .2, .2};
  o.primitives.push_back(shape);
  geometry_msgs::Pose p;
  p.orientation.w = 1;
  p.position.x = .5;
  o.primitive_poses.push_back(p);
  ASSERT_TRUE(s.processCollisionObjectMsg(o));
  moveit_msgs::RobotTrajectory t;
  t.joint_trajectory.joint_names = {"slide"};
  trajectory_msgs::JointTrajectoryPoint a, b;
  a.positions = {0};
  b.positions = {1};
  t.joint_trajectory.points = {a, b};
  std::string error;
  EXPECT_FALSE(rm75::collisionFree(s, t, 0, .01, error));
  EXPECT_EQ(error.substr(0, 9), "COLLISION");
  // Cubic controller interpolation can hit an obstacle although the straight
  // line between the two positions is free.
  t.joint_trajectory.points[1].positions={0.1};
  t.joint_trajectory.points[0].velocities={2.0};
  t.joint_trajectory.points[1].velocities={-2.0};
  t.joint_trajectory.points[1].time_from_start=ros::Duration(1.0);
  EXPECT_FALSE(rm75::collisionFree(s,t,0,.01,error));
  o.operation = o.REMOVE;
  s.processCollisionObjectMsg(o);
  EXPECT_TRUE(rm75::collisionFree(s, t, 0, .01, error));
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "collision_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
