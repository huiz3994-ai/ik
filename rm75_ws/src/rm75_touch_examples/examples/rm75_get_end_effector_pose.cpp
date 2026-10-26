#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

// Adapted from z1_examples/examples/get_end_effector_pose.cpp.
// Z1 used group "manipulator"; RM75 uses group "arm" and tip "Link7".
int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_get_end_effector_pose");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string planning_group;
  std::string tip_link;
  double publish_rate;
  private_nh.param<std::string>("planning_group", planning_group, "arm");
  private_nh.param<std::string>("tip_link", tip_link, "Link7");
  private_nh.param("publish_rate", publish_rate, 100.0);

  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  ros::Publisher pose_pub =
      nh.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 1);
  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    geometry_msgs::PoseStamped pose = move_group.getCurrentPose(tip_link);
    if (pose.header.frame_id.empty()) pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose_pub.publish(pose);
    rate.sleep();
  }
  return 0;
}
