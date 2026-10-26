#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <rm_msgs/Err.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Six_Force.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_motion_planning/command_source_manager.h"
#include "rm75_motion_planning/moveit_collision_checker.h"
#include "rm75_motion_planning/runtime_guard.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

namespace {

bool orderedJointVector(const sensor_msgs::JointState& message,
                        Eigen::VectorXd* joints) {
  if (message.name.size() != message.position.size()) return false;
  std::unordered_map<std::string, double> values;
  for (std::size_t i = 0; i < message.name.size(); ++i) {
    if (!std::isfinite(message.position[i]) ||
        !values.emplace(message.name[i], message.position[i]).second)
      return false;
  }
  joints->resize(7);
  for (int index = 0; index < 7; ++index) {
    const auto found = values.find("joint" + std::to_string(index + 1));
    if (found == values.end()) return false;
    (*joints)[index] = found->second;
  }
  return true;
}

struct Feedback {
  Eigen::VectorXd joints;
  ros::WallTime joint_stamp;
  ros::WallTime force_stamp;
  double force_norm{0.0};
  double torque_norm{0.0};
  bool have_joints{false};
  bool have_force{false};
  bool plan_failed{false};
  bool controller_error{false};
  std::string controller_error_detail;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_low_speed_commissioning");
  ros::NodeHandle node("~");
  bool execute = false;
  double joint7_delta = 0.02;
  double max_tracking_error = 0.03;
  double max_force_n = 8.0;
  double max_torque_nm = 0.5;
  double feedback_timeout_sec = 0.10;
  node.param("execute", execute, execute);
  node.param("joint7_delta", joint7_delta, joint7_delta);
  node.param("max_tracking_error_rad", max_tracking_error,
             max_tracking_error);
  node.param("max_force_n", max_force_n, max_force_n);
  node.param("max_torque_nm", max_torque_nm, max_torque_nm);
  node.param("feedback_timeout_sec", feedback_timeout_sec,
             feedback_timeout_sec);
  if (std::abs(joint7_delta) > 0.0200001 || std::abs(joint7_delta) < 0.001 ||
      max_tracking_error <= 0.0 || max_tracking_error > 0.05 ||
      max_force_n <= 0.0 || max_torque_nm <= 0.0 ||
      feedback_timeout_sec <= 0.0 || feedback_timeout_sec > 0.25) {
    ROS_ERROR("Commissioning parameters exceed the hard safety envelope");
    return 2;
  }

  std::mutex feedback_mutex;
  Feedback feedback;
  double peak_force = 0.0;
  double peak_torque = 0.0;
  const auto joint_subscriber = node.subscribe<sensor_msgs::JointState>(
      "/joint_states", 20,
      [&](const sensor_msgs::JointStateConstPtr& message) {
        Eigen::VectorXd joints;
        if (!orderedJointVector(*message, &joints)) return;
        std::lock_guard<std::mutex> lock(feedback_mutex);
        feedback.joints = joints;
        feedback.joint_stamp = ros::WallTime::now();
        feedback.have_joints = true;
      });
  const auto force_subscriber = node.subscribe<rm_msgs::Six_Force>(
      "/rm_driver/UdpSixZeroForce", 20,
      [&](const rm_msgs::Six_ForceConstPtr& message) {
        const double force = std::sqrt(message->force_Fx * message->force_Fx +
                                       message->force_Fy * message->force_Fy +
                                       message->force_Fz * message->force_Fz);
        const double torque = std::sqrt(message->force_Mx * message->force_Mx +
                                        message->force_My * message->force_My +
                                        message->force_Mz * message->force_Mz);
        std::lock_guard<std::mutex> lock(feedback_mutex);
        feedback.force_norm = force;
        feedback.torque_norm = torque;
        feedback.force_stamp = ros::WallTime::now();
        feedback.have_force = true;
        peak_force = std::max(peak_force, force);
        peak_torque = std::max(peak_torque, torque);
      });
  const auto plan_subscriber = node.subscribe<rm_msgs::Plan_State>(
      "/rm_driver/Plan_State", 10,
      [&](const rm_msgs::Plan_StateConstPtr& message) {
        if (!message->state) {
          std::lock_guard<std::mutex> lock(feedback_mutex);
          feedback.plan_failed = true;
        }
      });
  const auto error_subscriber = node.subscribe<rm_msgs::Err>(
      "/rm_driver/Error", 20,
      [&](const rm_msgs::ErrConstPtr& message) {
        for (const auto code : message->err) {
          if (code != 0) {
            std::lock_guard<std::mutex> lock(feedback_mutex);
            feedback.controller_error = true;
            feedback.controller_error_detail =
                "controller error code " + std::to_string(code);
            break;
          }
        }
      });
  (void)joint_subscriber;
  (void)force_subscriber;
  (void)plan_subscriber;
  (void)error_subscriber;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  const ros::WallTime feedback_deadline =
      ros::WallTime::now() + ros::WallDuration(3.0);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      if (feedback.have_joints && feedback.have_force) break;
    }
    if (ros::WallTime::now() >= feedback_deadline) {
      ROS_ERROR("Fresh joint and force feedback were not available");
      return 2;
    }
    ros::WallDuration(0.01).sleep();
  }

  try {
    rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
    rm75_motion_planning::MoveitCollisionChecker collision(
        RM75_DEFAULT_URDF, RM75_DEFAULT_SRDF);
    Feedback initial;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      initial = feedback;
    }
    Eigen::VectorXd q0 = initial.joints;
    Eigen::VectorXd q1 = q0;
    q1[6] += joint7_delta;
    const auto model_limits = model.limits();
    const double target_margin =
        std::min((q1 - model_limits.lower).minCoeff(),
                 (model_limits.upper - q1).minCoeff());
    if (target_margin < 0.05)
      throw std::runtime_error("target joint-limit margin is below 0.05 rad");

    rm75_motion_planning::JointPath path{q0, q1, q0};
    rm75_motion_planning::PoseVector poses{
        model.forward(q0).isometry(), model.forward(q1).isometry(),
        model.forward(q0).isometry()};
    rm75_motion_planning::DynamicLimits limits;
    limits.max_velocity = Eigen::VectorXd::Constant(7, 0.05);
    limits.max_acceleration = Eigen::VectorXd::Constant(7, 0.10);
    limits.max_jerk = Eigen::VectorXd::Constant(7, 0.50);
    const auto trajectory =
        rm75_motion_planning::TrajectoryParameterizer::parameterize(
            path, limits, 0.02);
    const auto report = rm75_motion_planning::TrajectoryValidator(
                            model, &collision)
                            .validate(poses, path, trajectory, limits, 0.0001,
                                      0.0001, 0.03, true);
    if (!report.valid) {
      std::string reason;
      for (const auto& violation : report.violations)
        reason += (reason.empty() ? "" : "; ") + violation;
      throw std::runtime_error("trajectory validation failed: " + reason);
    }

    std::cout << std::fixed << std::setprecision(6)
              << "preflight=PASS\n"
              << "data_source=real_robot\n"
              << "reference_frame=base_link_to_Link7\n"
              << "joint7_start_rad=" << q0[6] << '\n'
              << "joint7_target_rad=" << q1[6] << '\n'
              << "target_limit_margin_rad=" << target_margin << '\n'
              << "trajectory_samples=" << trajectory.size() << '\n'
              << "trajectory_duration_sec=" << trajectory.back().time << '\n';
    rm75_motion_planning::CommandSourceManager sources(execute);
    if (!sources.acquire(rm75_motion_planning::CommandSource::AUTONOMOUS))
      throw std::runtime_error("cannot acquire commissioning command source");
    if (!sources.commandPublisherAllowed()) {
      std::cout << "execution=DISARMED\n"
                << "jointpos_publisher_created=false\n";
      return 0;
    }

    std::string source_reason;
    if (!rm75_motion_planning::commandTopicUnclaimed(
            "/rm_driver/JointPos", &source_reason))
      throw std::runtime_error("JointPos ownership check failed: " +
                               source_reason);

    ros::Publisher command =
        node.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 1);
    ros::Publisher emergency =
        node.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1, true);
    const ros::WallTime subscriber_deadline =
        ros::WallTime::now() + ros::WallDuration(2.0);
    while (ros::ok() && command.getNumSubscribers() < 1 &&
           ros::WallTime::now() < subscriber_deadline)
      ros::WallDuration(0.01).sleep();
    if (command.getNumSubscribers() != 1)
      throw std::runtime_error("expected exactly one JointPos subscriber");

    bool abort = false;
    std::string abort_reason;
    double max_tracking = 0.0;
    double max_joint7_excursion = 0.0;
    Eigen::VectorXd last_command = q0;
    rm75_motion_planning::RuntimeGuardConfig guard_config;
    guard_config.lower = model_limits.lower;
    guard_config.upper = model_limits.upper;
    guard_config.max_tracking_error_rad = max_tracking_error;
    guard_config.feedback_timeout_sec = feedback_timeout_sec;
    rm75_motion_planning::RuntimeGuard guard(guard_config, &collision);
    rm75_motion_planning::GuardedCommandGateway gateway(sources, guard);
    ros::WallRate rate(50.0);
    for (const auto& point : trajectory) {
      Feedback current;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        current = feedback;
      }
      const ros::WallTime now = ros::WallTime::now();
      if (!current.have_joints ||
          (now - current.joint_stamp).toSec() > feedback_timeout_sec ||
          !current.have_force ||
          (now - current.force_stamp).toSec() > feedback_timeout_sec) {
        abort = true;
        abort_reason = "feedback timeout";
      } else if (current.plan_failed) {
        abort = true;
        abort_reason = "controller reported planning failure";
      } else if (current.controller_error) {
        abort = true;
        abort_reason = current.controller_error_detail;
      } else if (current.force_norm > max_force_n ||
                 current.torque_norm > max_torque_nm) {
        abort = true;
        abort_reason = "force/torque safety threshold exceeded";
      } else {
        const double tracking = (current.joints - last_command).norm();
        max_tracking = std::max(max_tracking, tracking);
        max_joint7_excursion = std::max(
            max_joint7_excursion, std::abs(current.joints[6] - q0[6]));
        rm75_motion_planning::RuntimeGuardInput guard_input;
        guard_input.current = current.joints;
        guard_input.next_target = point.position;
        guard_input.previous_target = last_command;
        guard_input.now_sec = now.toSec();
        guard_input.feedback_stamp_sec = current.joint_stamp.toSec();
        guard_input.commanded_velocity = point.velocity;
        guard_input.cycle_sec = .02;
        const auto guarded = gateway.evaluate(
            rm75_motion_planning::CommandSource::AUTONOMOUS, guard_input);
        if (!guarded.accepted) {
          abort = true;
          abort_reason = guarded.reason;
        }
      }
      if (abort) break;
      rm_msgs::JointPos message;
      message.joint.resize(7);
      for (int joint = 0; joint < 7; ++joint)
        message.joint[joint] = static_cast<float>(point.position[joint]);
      message.expand = 0.0F;
      command.publish(message);
      last_command = point.position;
      rate.sleep();
    }

    if (!abort) {
      const ros::WallTime settle_deadline =
          ros::WallTime::now() + ros::WallDuration(2.0);
      while (ros::ok() && ros::WallTime::now() < settle_deadline) {
        Feedback current;
        {
          std::lock_guard<std::mutex> lock(feedback_mutex);
          current = feedback;
        }
        max_tracking = std::max(max_tracking, (current.joints - q0).norm());
        max_joint7_excursion = std::max(
            max_joint7_excursion, std::abs(current.joints[6] - q0[6]));
        if ((current.joints - q0).norm() <= 0.01) break;
        rate.sleep();
      }
      Feedback final;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        final = feedback;
      }
      if ((final.joints - q0).norm() > 0.01) {
        abort = true;
        abort_reason = "robot did not settle at the starting pose";
      }
    }

    if (abort) {
      emergency.publish(std_msgs::Empty());
      ros::WallDuration(0.10).sleep();
      std::cout << "execution=STOP\n"
                << "stop_reason=" << abort_reason << '\n'
                << "estop_count=1\n"
                << "control_failure_count=1\n";
      ROS_ERROR_STREAM("Commissioning aborted: " << abort_reason);
      return 1;
    }
    Feedback final;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      final = feedback;
    }
    std::cout << "execution=PASS\n"
              << "max_joint_tracking_error_rad=" << max_tracking << '\n'
              << "max_joint7_excursion_rad=" << max_joint7_excursion << '\n'
              << "return_joint_error_rad=" << (final.joints - q0).norm() << '\n'
              << "peak_force_n=" << peak_force << '\n'
              << "peak_torque_nm=" << peak_torque << '\n'
              << "fcl_collision_count=0\n"
              << "estop_count=0\n"
              << "control_failure_count=0\n";
    return 0;
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Commissioning preflight failed: " << error.what());
    return 2;
  }
}
