#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_initialize_forward_pose");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  std::string command_topic;
  double move_duration;
  private_nh.param<std::string>("command_topic", command_topic,
                                "/arm/arm_joint_controller/command");
  private_nh.param("move_duration", move_duration, 3.0);

  ros::Publisher pub =
      nh.advertise<trajectory_msgs::JointTrajectory>(command_topic, 1, true);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(20.0);
  while (ros::ok() && pub.getNumSubscribers() == 0 &&
         ros::WallTime::now() < deadline) {
    ros::spinOnce();
    ros::WallDuration(0.05).sleep();
  }
  if (pub.getNumSubscribers() == 0) {
    ROS_ERROR("RM75 trajectory controller did not become ready");
    return 1;
  }

  trajectory_msgs::JointTrajectory trajectory;
  trajectory.header.stamp = ros::Time::now() + ros::Duration(0.1);
  trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4",
                            "joint5", "joint6", "joint7"};
  trajectory_msgs::JointTrajectoryPoint point;
  // Official RM75 SRDF "forward" state, away from the zero-pose singularity.
  point.positions = {0.0, 0.8395, 0.0, 1.4442, 0.0, 0.8513, 0.0};
  point.time_from_start = ros::Duration(move_duration);
  trajectory.points.push_back(point);
  pub.publish(trajectory);
  ROS_INFO("Moving RM75 to the Touch-ready forward pose (%.1f s)", move_duration);
  ros::WallDuration(move_duration + 0.5).sleep();
  return 0;
}
