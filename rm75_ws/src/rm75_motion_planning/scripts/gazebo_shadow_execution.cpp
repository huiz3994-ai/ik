#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_motion_planning/command_source_manager.h"
#include "rm75_motion_planning/moveit_collision_checker.h"
#include "rm75_motion_planning/runtime_guard.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

namespace {

bool orderedJoints(const sensor_msgs::JointState& message,
                   Eigen::VectorXd* joints) {
  if (message.name.size() != message.position.size()) return false;
  std::unordered_map<std::string, double> values;
  for (std::size_t i = 0; i < message.name.size(); ++i)
    values[message.name[i]] = message.position[i];
  joints->resize(7);
  for (int i = 0; i < 7; ++i) {
    const auto found = values.find("joint" + std::to_string(i + 1));
    if (found == values.end() || !std::isfinite(found->second)) return false;
    (*joints)[i] = found->second;
  }
  return true;
}

trajectory_msgs::JointTrajectory commandFor(const Eigen::VectorXd& q) {
  trajectory_msgs::JointTrajectory message;
  for (int i = 1; i <= 7; ++i)
    message.joint_names.push_back("joint" + std::to_string(i));
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.assign(q.data(), q.data() + q.size());
  point.time_from_start = ros::Duration(.04);
  message.points.push_back(point);
  return message;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_gazebo_shadow_execution");
  ros::NodeHandle node("~");
  bool execute = false;
  bool add_obstacle = false;
  int moving_joint = 6;
  double delta = .10;
  std::string feedback_topic = "/arm/joint_states";
  std::string command_topic = "/arm/arm_joint_controller/command";
  node.param("execute", execute, execute);
  node.param("add_obstacle", add_obstacle, add_obstacle);
  node.param("moving_joint", moving_joint, moving_joint);
  node.param("delta_rad", delta, delta);
  node.param("feedback_topic", feedback_topic, feedback_topic);
  node.param("command_topic", command_topic, command_topic);
  bool use_sim_time = false;
  ros::param::param("/use_sim_time", use_sim_time, false);
  if (!use_sim_time || feedback_topic.find("/arm/") != 0 ||
      command_topic.find("/arm/") != 0 ||
      feedback_topic.find("rm_driver") != std::string::npos ||
      command_topic.find("rm_driver") != std::string::npos) {
    ROS_ERROR("Shadow execution requires Gazebo /use_sim_time and /arm topics");
    return 2;
  }
  if (moving_joint < 0 || moving_joint >= 7 || !std::isfinite(delta) ||
      std::abs(delta) > .5 || std::abs(delta) < .001) {
    ROS_ERROR("Invalid shadow trajectory parameters");
    return 2;
  }

  try {
    std::mutex mutex;
    Eigen::VectorXd measured;
    ros::WallTime stamp;
    std::size_t feedback_count = 0;
    const auto subscriber = node.subscribe<sensor_msgs::JointState>(
        feedback_topic, 50,
        [&](const sensor_msgs::JointStateConstPtr& message) {
          Eigen::VectorXd joints;
          if (!orderedJoints(*message, &joints)) return;
          std::lock_guard<std::mutex> lock(mutex);
          measured = joints;
          stamp = ros::WallTime::now();
          ++feedback_count;
        });
    (void)subscriber;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(3.0);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        if (measured.size() == 7) break;
      }
      if (ros::WallTime::now() >= deadline)
        throw std::runtime_error("Gazebo joint feedback timeout");
      ros::WallDuration(.01).sleep();
    }

    rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
    rm75_motion_planning::MoveitCollisionChecker collision(
        RM75_DEFAULT_URDF, RM75_DEFAULT_SRDF);
    if (add_obstacle) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      // Deliberately enclosing box for the negative test: preflight must reject
      // the path before a Gazebo controller publisher is created.
      collision.addBox("shadow_blocking_obstacle",
                       Eigen::Vector3d(3.0, 3.0, 3.0), pose);
    }
    Eigen::VectorXd q0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      q0 = measured;
    }
    Eigen::VectorXd q1 = q0;
    q1[moving_joint] += delta;
    rm75_motion_planning::JointPath path{q0, q1, q0};
    rm75_motion_planning::DynamicLimits dynamics;
    dynamics.max_velocity = Eigen::VectorXd::Constant(7, .30);
    dynamics.max_acceleration = Eigen::VectorXd::Constant(7, .50);
    dynamics.max_jerk = Eigen::VectorXd::Constant(7, 2.0);
    const auto trajectory =
        rm75_motion_planning::TrajectoryParameterizer::parameterize(
            path, dynamics, .02);
    rm75_motion_planning::PoseVector poses{
        model.forward(q0).isometry(), model.forward(q1).isometry(),
        model.forward(q0).isometry()};
    const auto validation = rm75_motion_planning::TrajectoryValidator(
        model, &collision).validate(poses, path, trajectory, dynamics,
                                    .001, .01, .5, true);
    if (!validation.valid)
      throw std::runtime_error("shadow preflight trajectory is invalid");
    std::cout << "shadow_preflight=PASS\n"
              << "samples=" << trajectory.size() << '\n'
              << "duration_sec=" << trajectory.back().time << '\n';
    rm75_motion_planning::CommandSourceManager sources(execute);
    if (!sources.acquire(rm75_motion_planning::CommandSource::AUTONOMOUS))
      throw std::runtime_error("cannot acquire shadow command source");
    if (!sources.commandPublisherAllowed()) {
      std::cout << "shadow_execution=DISARMED\n";
      return 0;
    }

    std::string source_reason;
    if (!rm75_motion_planning::commandTopicUnclaimed(
            command_topic, &source_reason))
      throw std::runtime_error("shadow command ownership check failed: " +
                               source_reason);

    ros::Publisher publisher =
        node.advertise<trajectory_msgs::JointTrajectory>(command_topic, 1);
    const ros::WallTime controller_deadline =
        ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::ok() && publisher.getNumSubscribers() < 1 &&
           ros::WallTime::now() < controller_deadline)
      ros::WallDuration(.01).sleep();
    if (publisher.getNumSubscribers() < 1)
      throw std::runtime_error("Gazebo controller is not subscribed");

    const auto limits = model.limits();
    rm75_motion_planning::RuntimeGuardConfig config;
    config.lower = limits.lower;
    config.upper = limits.upper;
    config.max_tracking_error_rad = .08;
    config.feedback_timeout_sec = .10;
    rm75_motion_planning::RuntimeGuard guard(config, &collision);
    rm75_motion_planning::GuardedCommandGateway gateway(sources, guard);
    Eigen::VectorXd previous_target = q0;
    double max_tracking = 0.0;
    std::size_t warnings = 0;
    std::string stop_reason;
    ros::WallRate rate(50.0);
    const ros::WallTime begin = ros::WallTime::now();
    std::size_t initial_feedback_count;
    {
      std::lock_guard<std::mutex> lock(mutex);
      initial_feedback_count = feedback_count;
    }
    for (const auto& point : trajectory) {
      Eigen::VectorXd current;
      ros::WallTime current_stamp;
      {
        std::lock_guard<std::mutex> lock(mutex);
        current = measured;
        current_stamp = stamp;
      }
      rm75_motion_planning::RuntimeGuardInput input;
      input.current = current;
      input.next_target = point.position;
      input.previous_target = previous_target;
      input.now_sec = ros::WallTime::now().toSec();
      input.feedback_stamp_sec = current_stamp.toSec();
      input.commanded_velocity = point.velocity;
      input.cycle_sec = .02;
      const auto guarded = gateway.evaluate(
          rm75_motion_planning::CommandSource::AUTONOMOUS, input);
      if (!guarded.accepted) {
        stop_reason = guarded.reason;
        publisher.publish(commandFor(current));
        break;
      }
      if (guarded.decision == rm75_motion_planning::GuardDecision::WARN)
        ++warnings;
      max_tracking = std::max(
          max_tracking, guarded.guard.max_tracking_error_rad);
      publisher.publish(commandFor(point.position));
      previous_target = point.position;
      rate.sleep();
    }
    std::size_t final_feedback_count;
    {
      std::lock_guard<std::mutex> lock(mutex);
      final_feedback_count = feedback_count;
    }
    const double elapsed = (ros::WallTime::now() - begin).toSec();
    std::cout << std::fixed << std::setprecision(8)
              << "shadow_execution=" << (stop_reason.empty() ? "PASS" : "STOP") << '\n'
              << "stop_reason=" << stop_reason << '\n'
              << "max_joint_tracking_error_rad=" << max_tracking << '\n'
              << "guard_warn_samples=" << warnings << '\n'
              << "feedback_frequency_hz="
              << ((final_feedback_count - initial_feedback_count) /
                  std::max(elapsed, 1e-9)) << '\n';
    return stop_reason.empty() ? 0 : 1;
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Gazebo shadow execution failed: " << error.what());
    return 2;
  }
}
