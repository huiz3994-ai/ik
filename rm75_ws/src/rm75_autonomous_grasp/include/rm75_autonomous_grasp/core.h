#pragma once
#include <actionlib/client/simple_action_client.h>
#include <atomic>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/SetBool.h>
namespace rm75 {
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
struct Candidate {
  geometry_msgs::PoseStamped pre, grasp, lift;
};
class GraspPlanner {
public:
  std::vector<Candidate> generate(const geometry_msgs::PoseStamped &,
                                  const std::string &, double, double,
                                  double) const;
};
class TrajectoryOptimizer {
public:
  bool optimize(const std::string &, Plan &,
                const moveit::core::RobotModelConstPtr &, double, double,
                std::string &) const;
};
// FCL evaluates interpolated joint-space segments, including attached bodies.
bool collisionFree(const planning_scene::PlanningScene &,
                   const moveit_msgs::RobotTrajectory &, size_t start,
                   double step, std::string &);
class MoveItPlanner {
  ros::NodeHandle nh_, p_;
  ros::Subscriber cloud_sub_, scene_sub_, stop_ack_sub_, filtered_sub_;
  std::atomic<bool> stop_acknowledged_{false};
  ros::WallTime cloud_received_, scene_received_;
  ros::Time cloud_stamp_;
  std::mutex mutex_;
  actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>
      execution_{"/execute_trajectory", true};
  ros::Publisher stop_pub_, metrics_;
  size_t attempts_ = 0, successes_ = 0;
  bool simulation_, execute_, require_map_;
  double map_timeout_, step_, velocity_, acceleration_;
  std::string optimizer_, planner_;

public:
  moveit::planning_interface::MoveGroupInterface group{"arm"};
  moveit::planning_interface::PlanningSceneInterface scene;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor;
  MoveItPlanner(ros::NodeHandle &);
  bool ready(std::string &);
  bool mapReady(std::string &);
  bool plan(const geometry_msgs::PoseStamped &, Plan &, std::string &);
  bool run(const Plan &, const std::function<bool()> &, std::string &);
  bool stopAndWait(std::string &);
  bool enabled() const { return execute_; }
};
class Gripper {
  ros::NodeHandle nh_, p_;
  ros::Publisher open_, close_;
  ros::Subscriber ack_, holding_;
  ros::ServiceClient sim_;
  std::mutex mutex_;
  bool ack = false, ok = false, held = false;
  ros::WallTime held_time;
  bool simulation;
  double timeout;
  int speed, force;
  bool verified;

public:
  explicit Gripper(ros::NodeHandle &);
  bool command(bool close, std::string &);
  bool holding(std::string &);
};
} // namespace rm75
