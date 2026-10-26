#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/Joint_PoseEuler.h>

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_motion_planning/moveit_collision_checker.h"

namespace {

bool orderedJointVector(const sensor_msgs::JointState& message,
                        Eigen::VectorXd* joints, std::string* error) {
  if (message.name.size() != message.position.size()) {
    *error = "joint name/position lengths differ";
    return false;
  }
  std::unordered_map<std::string, double> values;
  for (std::size_t i = 0; i < message.name.size(); ++i) {
    if (!std::isfinite(message.position[i])) {
      *error = "non-finite joint position: " + message.name[i];
      return false;
    }
    if (!values.emplace(message.name[i], message.position[i]).second) {
      *error = "duplicate joint name: " + message.name[i];
      return false;
    }
  }
  joints->resize(7);
  for (int index = 0; index < 7; ++index) {
    const std::string name = "joint" + std::to_string(index + 1);
    const auto found = values.find(name);
    if (found == values.end()) {
      *error = "missing joint: " + name;
      return false;
    }
    (*joints)[index] = found->second;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_live_state_audit",
            ros::init_options::AnonymousName);
  ros::NodeHandle node("~");
  std::string topic{"/joint_states"};
  std::string controller_pose_topic{"/rm_driver/Udp_Joint_PoseEuler"};
  int requested_samples = 100;
  double timeout_sec = 2.0;
  double minimum_margin_rad = 0.02;
  double max_stationary_span_rad = 0.01;
  double max_fk_position_error_m = 0.003;
  double max_fk_orientation_error_rad = 0.034906585;
  node.param("joint_state_topic", topic, topic);
  node.param("controller_pose_topic", controller_pose_topic,
             controller_pose_topic);
  node.param("samples", requested_samples, requested_samples);
  node.param("timeout_sec", timeout_sec, timeout_sec);
  node.param("minimum_margin_rad", minimum_margin_rad, minimum_margin_rad);
  node.param("max_stationary_span_rad", max_stationary_span_rad,
             max_stationary_span_rad);
  node.param("max_fk_position_error_m", max_fk_position_error_m,
             max_fk_position_error_m);
  node.param("max_fk_orientation_error_rad", max_fk_orientation_error_rad,
             max_fk_orientation_error_rad);
  if (requested_samples < 2 || timeout_sec <= 0.0 || minimum_margin_rad < 0.0 ||
      max_stationary_span_rad <= 0.0 ||
      max_fk_position_error_m <= 0.0 || max_fk_orientation_error_rad <= 0.0) {
    ROS_ERROR("Invalid audit parameters");
    return 2;
  }

  try {
    rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
    rm75_motion_planning::MoveitCollisionChecker collision(
        RM75_DEFAULT_URDF, RM75_DEFAULT_SRDF);
    const auto limits = model.limits();
    std::mutex sample_mutex;
    std::condition_variable samples_ready;
    std::vector<sensor_msgs::JointStateConstPtr> messages;
    std::vector<ros::WallTime> receipts;
    rm_msgs::Joint_PoseEulerConstPtr controller_pose;
    messages.reserve(requested_samples);
    receipts.reserve(requested_samples);
    const ros::Subscriber subscriber = node.subscribe<sensor_msgs::JointState>(
        topic, std::max(10, requested_samples),
        [&](const sensor_msgs::JointStateConstPtr& message) {
          std::lock_guard<std::mutex> lock(sample_mutex);
          if (static_cast<int>(messages.size()) >= requested_samples) return;
          messages.push_back(message);
          receipts.push_back(ros::WallTime::now());
          if (static_cast<int>(messages.size()) >= requested_samples)
            samples_ready.notify_one();
        });
    const ros::Subscriber pose_subscriber =
        node.subscribe<rm_msgs::Joint_PoseEuler>(
            controller_pose_topic, 10,
            [&](const rm_msgs::Joint_PoseEulerConstPtr& message) {
              std::lock_guard<std::mutex> lock(sample_mutex);
              controller_pose = message;
              if (static_cast<int>(messages.size()) >= requested_samples)
                samples_ready.notify_one();
            });
    ros::AsyncSpinner spinner(1);
    spinner.start();
    {
      std::unique_lock<std::mutex> lock(sample_mutex);
      if (!samples_ready.wait_for(
              lock, std::chrono::duration<double>(timeout_sec), [&] {
                return static_cast<int>(messages.size()) >= requested_samples &&
                       static_cast<bool>(controller_pose);
              })) {
        ROS_ERROR_STREAM("Timed out waiting for " << requested_samples
                         << " samples on " << topic << " and a controller pose; received "
                         << messages.size() << " joint samples, pose="
                         << static_cast<bool>(controller_pose));
        return 2;
      }
    }
    spinner.stop();
    (void)subscriber;
    (void)pose_subscriber;

    Eigen::VectorXd previous;
    Eigen::VectorXd first;
    Eigen::VectorXd latest;
    double min_margin = std::numeric_limits<double>::infinity();
    double max_step = 0.0;
    double max_stationary_span = 0.0;
    int collision_count = 0;
    std::string first_collision;
    for (int sample = 0; sample < requested_samples; ++sample) {
      Eigen::VectorXd joints;
      std::string error;
      if (!orderedJointVector(*messages[sample], &joints, &error)) {
        ROS_ERROR_STREAM("Invalid joint state: " << error);
        return 2;
      }
      min_margin = std::min(
          min_margin,
          std::min((joints - limits.lower).minCoeff(),
                   (limits.upper - joints).minCoeff()));
      if (previous.size() == joints.size())
        max_step = std::max(max_step, (joints - previous).norm());
      if (sample == 0) first = joints;
      max_stationary_span =
          std::max(max_stationary_span, (joints - first).norm());
      std::string detail;
      if (collision.inCollision(joints, &detail)) {
        ++collision_count;
        if (first_collision.empty())
          first_collision = "sample " + std::to_string(sample) + ": " + detail;
      }
      previous = joints;
      latest = joints;
    }

    const double elapsed = (receipts.back() - receipts.front()).toSec();
    const double rate = elapsed > 0.0 ? (requested_samples - 1) / elapsed : 0.0;
    const auto flange = model.forward(latest);
    const Eigen::Quaterniond orientation(flange.rotation);
    const Eigen::Vector3d reported_position(
        controller_pose->position[0], controller_pose->position[1],
        controller_pose->position[2]);
    const Eigen::Matrix3d reported_rotation =
        (Eigen::AngleAxisd(controller_pose->euler[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(controller_pose->euler[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(controller_pose->euler[0], Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const double fk_position_error =
        (flange.position - reported_position).norm();
    const double fk_orientation_error = Eigen::Quaterniond(flange.rotation)
                                            .angularDistance(
                                                Eigen::Quaterniond(reported_rotation));
    const bool safe = min_margin >= minimum_margin_rad && collision_count == 0 &&
                      max_stationary_span <= max_stationary_span_rad &&
                      fk_position_error <= max_fk_position_error_m &&
                      fk_orientation_error <= max_fk_orientation_error_rad;
    std::cout << std::fixed << std::setprecision(6)
              << "samples=" << requested_samples << '\n'
              << "observed_rate_hz=" << rate << '\n'
              << "max_sample_joint_step_rad=" << max_step << '\n'
              << "start_to_end_joint_delta_rad=" << (latest - first).norm() << '\n'
              << "max_stationary_span_rad=" << max_stationary_span << '\n'
              << "min_joint_limit_margin_rad=" << min_margin << '\n'
              << "collision_samples=" << collision_count << '\n'
              << "latest_q_rad=" << latest.transpose() << '\n'
              << "base_T_Link7_translation_m=" << flange.position.transpose() << '\n'
              << "base_T_Link7_quaternion_xyzw=" << orientation.x() << ' '
              << orientation.y() << ' ' << orientation.z() << ' '
              << orientation.w() << '\n'
              << "controller_fk_position_error_m=" << fk_position_error << '\n'
              << "controller_fk_orientation_error_rad=" << fk_orientation_error
              << '\n';
    if (!first_collision.empty())
      std::cout << "first_collision=" << first_collision << '\n';
    std::cout << "audit=" << (safe ? "PASS" : "FAIL") << '\n';
    return safe ? 0 : 1;
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Live-state audit failed: " << error.what());
    return 2;
  }
}
