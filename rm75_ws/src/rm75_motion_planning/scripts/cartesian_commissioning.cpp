#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Err.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Six_Force.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_motion_planning/cartesian_path_generator.h"
#include "rm75_motion_planning/command_source_manager.h"
#include "rm75_motion_planning/continuous_ik_solver.h"
#include "rm75_motion_planning/moveit_collision_checker.h"
#include "rm75_motion_planning/real_execution_metrics.h"
#include "rm75_motion_planning/runtime_guard.h"
#include "rm75_motion_planning/se3_interpolator.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

namespace {

using rm75_motion_planning::Pose;

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
  double joint_source_stamp_sec{0.0};
  bool have_joint_source_stamp{false};
  ros::WallTime force_stamp;
  double force_norm{0.0};
  double torque_norm{0.0};
  double max_measured_velocity_rad_s{0.0};
  double max_measured_acceleration_rad_s2{0.0};
  Eigen::VectorXd last_velocity;
  std::size_t joint_feedback_count{0};
  bool have_joints{false};
  bool have_force{false};
  bool plan_failed{false};
  bool controller_error{false};
  std::string controller_error_detail;
};

double orientationDistance(const Pose& a, const Pose& b) {
  return Eigen::Quaterniond(a.rotation()).angularDistance(
      Eigen::Quaterniond(b.rotation()));
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_cartesian_commissioning");
  ros::NodeHandle node("~");
  bool execute = false;
  double dx = 0.0, dy = 0.0, dz = 0.0;
  double droll = 0.0, dpitch = 0.0, dyaw = 0.0;
  double link7_speed = 0.02;
  double angular_speed = 0.0872664626;  // 5 deg/s
  double max_tracking_error = 0.03;
  double feedback_timeout = 0.10;
  double joint_limit_margin = 0.02;
  double warning_limit_margin = 0.05;
  double warning_tracking_ratio = 0.80;
  double stop_observation = 2.0;
  double force_warning = 6.0;
  double force_stop = 8.0;
  double torque_warning = 0.4;
  double torque_stop = 0.5;
  std::string raw_output_csv;
  std::string run_summary_json;
  std::string run_id;
  std::string scenario;
  int repetition = 0;
  node.param("execute", execute, execute);
  node.param("dx", dx, dx); node.param("dy", dy, dy); node.param("dz", dz, dz);
  node.param("droll", droll, droll);
  node.param("dpitch", dpitch, dpitch);
  node.param("dyaw", dyaw, dyaw);
  node.param("link7_speed_mps", link7_speed, link7_speed);
  node.param("angular_speed_rad_s", angular_speed, angular_speed);
  node.param("max_tracking_error_rad", max_tracking_error,
             max_tracking_error);
  node.param("feedback_timeout_sec", feedback_timeout, feedback_timeout);
  node.param("joint_limit_margin_rad", joint_limit_margin,
             joint_limit_margin);
  node.param("warning_limit_margin_rad", warning_limit_margin,
             warning_limit_margin);
  node.param("warning_tracking_ratio", warning_tracking_ratio,
             warning_tracking_ratio);
  node.param("stop_observation_sec", stop_observation, stop_observation);
  node.param("force_warning_n", force_warning, force_warning);
  node.param("force_stop_n", force_stop, force_stop);
  node.param("torque_warning_nm", torque_warning, torque_warning);
  node.param("torque_stop_nm", torque_stop, torque_stop);
  // Compatibility with Stage 3 command lines. The calibrated YAML names above
  // are canonical; these aliases may only tighten/replace the STOP values.
  node.param("max_force_n", force_stop, force_stop);
  node.param("max_torque_nm", torque_stop, torque_stop);
  node.param("raw_output_csv", raw_output_csv, raw_output_csv);
  node.param("run_summary_json", run_summary_json, run_summary_json);
  node.param("run_id", run_id, run_id);
  node.param("scenario", scenario, scenario);
  node.param("repetition", repetition, repetition);

  const bool recording_requested =
      !raw_output_csv.empty() || !run_summary_json.empty();
  const bool valid_scenario = scenario == "x" || scenario == "y" ||
      scenario == "z" || scenario == "rx" || scenario == "ry" ||
      scenario == "rz" || scenario == "6d";

  const Eigen::Vector3d translation_delta(dx, dy, dz);
  const Eigen::Vector3d rotation_delta(droll, dpitch, dyaw);
  if (!translation_delta.allFinite() || !rotation_delta.allFinite() ||
      translation_delta.cwiseAbs().maxCoeff() > 0.0100001 ||
      rotation_delta.cwiseAbs().maxCoeff() > 0.0872666 ||
      (translation_delta.norm() < 1e-9 && rotation_delta.norm() < 1e-9) ||
      link7_speed < 0.01 || link7_speed > 0.03 || angular_speed <= 0.0 ||
      angular_speed > 0.174533 || max_tracking_error <= 0.0 ||
      max_tracking_error > 0.05 || feedback_timeout <= 0.0 ||
      feedback_timeout > 0.25 || joint_limit_margin < 0.0 ||
      warning_limit_margin < joint_limit_margin ||
      warning_tracking_ratio <= 0.0 || warning_tracking_ratio >= 1.0 ||
      stop_observation <= 0.0 || stop_observation > 10.0 ||
      force_warning <= 0.0 || force_stop <= force_warning ||
      torque_warning <= 0.0 || torque_stop <= torque_warning ||
      (recording_requested &&
       (raw_output_csv.empty() || run_summary_json.empty() || run_id.empty() ||
        !valid_scenario || repetition < 1))) {
    ROS_ERROR("Cartesian commissioning parameters exceed the hard envelope");
    return 2;
  }

  try {
    std::mutex feedback_mutex;
    Feedback feedback;
    double peak_force = 0.0;
    double peak_torque = 0.0;
    const auto joint_subscriber = node.subscribe<sensor_msgs::JointState>(
        "/joint_states", 50,
        [&](const sensor_msgs::JointStateConstPtr& message) {
          Eigen::VectorXd joints;
          if (!orderedJointVector(*message, &joints)) return;
          std::lock_guard<std::mutex> lock(feedback_mutex);
          const ros::WallTime receipt = ros::WallTime::now();
          if (feedback.have_joints) {
            const double dt = (receipt - feedback.joint_stamp).toSec();
            if (dt > 1e-6 && dt < .25) {
              const Eigen::VectorXd velocity =
                  (joints - feedback.joints) / dt;
              feedback.max_measured_velocity_rad_s = std::max(
                  feedback.max_measured_velocity_rad_s,
                  velocity.cwiseAbs().maxCoeff());
              if (feedback.last_velocity.size() == velocity.size())
                feedback.max_measured_acceleration_rad_s2 = std::max(
                    feedback.max_measured_acceleration_rad_s2,
                    ((velocity - feedback.last_velocity) / dt)
                        .cwiseAbs().maxCoeff());
              feedback.last_velocity = velocity;
            }
          }
          feedback.joints = joints;
          feedback.joint_stamp = receipt;
          const double source_stamp = message->header.stamp.toSec();
          feedback.have_joint_source_stamp =
              !message->header.stamp.isZero() && std::isfinite(source_stamp) &&
              source_stamp > 0.0;
          feedback.joint_source_stamp_sec = feedback.have_joint_source_stamp
              ? source_stamp : 0.0;
          feedback.have_joints = true;
          ++feedback.joint_feedback_count;
        });
    const auto force_subscriber = node.subscribe<rm_msgs::Six_Force>(
        "/rm_driver/UdpSixZeroForce", 50,
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

    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(3.0);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        if (feedback.have_joints && feedback.have_force) break;
      }
      if (ros::WallTime::now() >= deadline)
        throw std::runtime_error(
            "fresh joint and force feedback unavailable");
      ros::WallDuration(.01).sleep();
    }

    rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
    rm75_motion_planning::MoveitCollisionChecker collision(
        RM75_DEFAULT_URDF, RM75_DEFAULT_SRDF);
    Feedback initial;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      initial = feedback;
    }
    const Eigen::VectorXd q0 = initial.joints;
    const Pose start_link7 = model.forward(q0).isometry();
    Pose goal_link7 = start_link7;
    goal_link7.translation() += translation_delta;
    goal_link7.linear() = goal_link7.rotation() *
        (Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    const auto link7_path = rm75_motion_planning::CartesianPathGenerator::line(
        start_link7, goal_link7, {.002, .0174532925});

    rm75_motion_planning::ContinuousIkOptions ik_options;
    ik_options.max_joint_jump = .20;
    ik_options.enable_ipopt_fallback = false;
    rm75_kinematics::IkOptions dls_options;
    dls_options.position_tolerance = 2e-4;
    dls_options.orientation_tolerance = 2e-3;
    dls_options.timeout_sec = .2;
    const auto solved = rm75_motion_planning::ContinuousIkSolver(model).solve(
        link7_path, q0, ik_options, dls_options);
    if (!solved.success)
      throw std::runtime_error("continuous IK failed: " + solved.failure_reason);

    rm75_motion_planning::DynamicLimits dynamics;
    dynamics.max_velocity = Eigen::VectorXd::Constant(7, .05);
    dynamics.max_acceleration = Eigen::VectorXd::Constant(7, .10);
    dynamics.max_jerk = Eigen::VectorXd::Constant(7, .50);
    const double minimum_duration = std::max(
        translation_delta.norm() / link7_speed,
        orientationDistance(start_link7, goal_link7) / angular_speed);
    const auto trajectory =
        rm75_motion_planning::TrajectoryParameterizer::parameterizeAtLeastDuration(
            solved.joints, dynamics, minimum_duration, .02);
    const auto validation = rm75_motion_planning::TrajectoryValidator(
        model, &collision).validate(solved.solved_poses, solved.joints,
                                    trajectory, dynamics, .001, .01, .20, true);
    if (!validation.valid) {
      std::string reason;
      for (const auto& item : validation.violations)
        reason += (reason.empty() ? "" : "; ") + item;
      throw std::runtime_error("trajectory validation failed: " + reason);
    }

    std::cout << std::fixed << std::setprecision(8)
              << "preflight=PASS\n"
              << "data_source=real_robot\n"
              << "reference_frame=base_link_to_Link7\n"
              << "start_Link7_xyz_m=" << start_link7.translation().transpose() << '\n'
              << "start_Link7_quaternion_xyzw="
              << Eigen::Quaterniond(start_link7.rotation()).coeffs().transpose() << '\n'
              << "target_Link7_xyz_m=" << goal_link7.translation().transpose() << '\n'
              << "target_Link7_quaternion_xyzw="
              << Eigen::Quaterniond(goal_link7.rotation()).coeffs().transpose() << '\n'
              << "ik_status=SUCCESS\n"
              << "ik_dls_calls=" << solved.dls_calls << '\n'
              << "ik_refinements=" << solved.refinements << '\n'
              << "ik_fallback_calls=" << solved.fallback_calls << '\n'
              << "trajectory_samples=" << trajectory.size() << '\n'
              << "trajectory_duration_sec=" << trajectory.back().time << '\n';

    if (recording_requested) {
      std::ifstream raw_existing(raw_output_csv);
      std::ifstream summary_existing(run_summary_json);
      if (raw_existing.good() || summary_existing.good())
        throw std::runtime_error(
            "refusing to overwrite an existing Stage 4 evidence file");
    }

    rm75_motion_planning::CommandSourceManager sources(execute);
    if (!sources.acquire(rm75_motion_planning::CommandSource::AUTONOMOUS))
      throw std::runtime_error("cannot acquire Cartesian command source");
    if (!sources.commandPublisherAllowed()) {
      std::cout << "execution=DISARMED\n"
              << "jointpos_publisher_created=false\n";
      return 0;
    }

    std::unique_ptr<rm75_motion_planning::RealExecutionCsvWriter> raw_writer;

    std::string source_reason;
    if (!sources.accept(rm75_motion_planning::CommandSource::AUTONOMOUS,
                        trajectory.front().position, &source_reason))
      throw std::runtime_error("command source rejected: " + source_reason);
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
      ros::WallDuration(.01).sleep();
    if (command.getNumSubscribers() != 1)
      throw std::runtime_error("expected exactly one JointPos subscriber");
    if (recording_requested)
      raw_writer.reset(new rm75_motion_planning::RealExecutionCsvWriter(
          raw_output_csv));

    const auto joint_limits = model.limits();
    rm75_motion_planning::RuntimeGuardConfig guard_config;
    guard_config.lower = joint_limits.lower;
    guard_config.upper = joint_limits.upper;
    guard_config.joint_limit_margin_rad = joint_limit_margin;
    guard_config.warning_limit_margin_rad = warning_limit_margin;
    guard_config.max_tracking_error_rad = max_tracking_error;
    guard_config.warning_tracking_ratio = warning_tracking_ratio;
    guard_config.feedback_timeout_sec = feedback_timeout;
    rm75_motion_planning::RuntimeGuard guard(guard_config, &collision);
    rm75_motion_planning::GuardedCommandGateway gateway(sources, guard);
    Eigen::VectorXd previous_target = q0;
    double max_joint_error = 0.0;
    double max_joint_change = 0.0;
    double max_link7_position_error = 0.0;
    double max_link7_orientation_error = 0.0;
    double link7_position_error_square_sum = 0.0;
    double link7_orientation_error_square_sum = 0.0;
    double joint_error_square_sum = 0.0;
    std::size_t joint_error_samples = 0;
    double max_feedback_age_ms = 0.0;
    double feedback_age_sum_ms = 0.0;
    double max_command_period_ms = 0.0;
    double command_period_sum_ms = 0.0;
    std::size_t command_period_samples = 0;
    std::size_t runtime_collision_count = 0;
    std::size_t warn_count = 0;
    bool abort = false;
    std::string abort_reason;
    ros::WallRate rate(50.0);
    const ros::WallTime execution_start = ros::WallTime::now();
    ros::WallTime last_command_time;
    rm75_motion_planning::RealExecutionSample stop_sample_template;
    bool have_stop_sample_template = false;
    const std::size_t initial_feedback_count = initial.joint_feedback_count;
    for (const auto& point : trajectory) {
      Feedback current;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        current = feedback;
      }
      const double now = ros::WallTime::now().toSec();
      rm75_motion_planning::RuntimeGuardInput guard_input;
      guard_input.current = current.joints;
      guard_input.next_target = point.position;
      guard_input.previous_target = previous_target;
      guard_input.now_sec = now;
      guard_input.feedback_stamp_sec = current.joint_stamp.toSec();
      guard_input.commanded_velocity = point.velocity;
      guard_input.cycle_sec = .02;
      const auto gateway_result = gateway.evaluate(
          rm75_motion_planning::CommandSource::AUTONOMOUS, guard_input);
      const auto& guarded = gateway_result.guard;
      if (!gateway_result.accepted) {
        abort = true;
        abort_reason = gateway_result.reason;
        if (guarded.collision) ++runtime_collision_count;
      } else if (!current.have_force ||
                 now - current.force_stamp.toSec() > feedback_timeout) {
        abort = true;
        abort_reason = "force feedback timeout";
      } else if (current.force_norm > force_stop ||
                 current.torque_norm > torque_stop) {
        abort = true;
        abort_reason = "force/torque safety threshold exceeded";
      } else if (current.plan_failed) {
        abort = true;
        abort_reason = "controller reported planning failure";
      } else if (current.controller_error) {
        abort = true;
        abort_reason = current.controller_error_detail;
      }
      const Pose measured_link7 = model.forward(current.joints).isometry();
      const Pose commanded_link7 = model.forward(point.position).isometry();
      // Recover the geometric progress of the time-parameterized joint sample
      // on the requested straight SE(3) path. Using wall/time fraction here
      // would incorrectly fold the quintic speed profile into "IK error".
      double trajectory_fraction = 0.0;
      const Eigen::Vector3d cartesian_delta =
          goal_link7.translation() - start_link7.translation();
      if (cartesian_delta.squaredNorm() > 1e-16) {
        // Translation is the numerically better progress observable whenever
        // it exists. Averaging it with a very small rotation would amplify IK
        // residuals in recovery/combined trajectories and corrupt the error
        // decomposition while leaving execution itself unchanged.
        trajectory_fraction =
            (commanded_link7.translation() - start_link7.translation())
                .dot(cartesian_delta) / cartesian_delta.squaredNorm();
      } else {
        const double total_rotation =
            orientationDistance(start_link7, goal_link7);
        trajectory_fraction = total_rotation > 1e-10
            ? orientationDistance(start_link7, commanded_link7) /
                  total_rotation
            : 1.0;
      }
      trajectory_fraction = std::min(1.0, std::max(0.0, trajectory_fraction));
      const Pose ideal_link7 = rm75_motion_planning::Se3Interpolator::interpolate(
          start_link7, goal_link7, trajectory_fraction);
      if (abort) {
        if (raw_writer) {
          rm75_motion_planning::RealExecutionSample sample;
          sample.run_id = run_id;
          sample.scenario = scenario;
          sample.repetition = repetition;
          sample.requested_linear_speed_mps = link7_speed;
          sample.requested_angular_speed_rad_s = angular_speed;
          sample.sample_time_sec = ros::WallTime::now().toSec();
          sample.command_time_sec = last_command_time.isZero()
              ? execution_start.toSec() : last_command_time.toSec();
          sample.feedback_time_sec = current.joint_stamp.toSec();
          sample.feedback_source_time_sec = current.joint_source_stamp_sec;
          sample.feedback_source_time_valid = current.have_joint_source_stamp;
          sample.command_published = false;
          sample.ideal_pose = ideal_link7;
          sample.commanded_pose = commanded_link7;
          sample.actual_pose = measured_link7;
          sample.target_joints = point.position;
          sample.target_velocity = point.velocity;
          sample.target_acceleration = point.acceleration;
          sample.actual_joints = current.joints;
          sample.force_n = current.force_norm;
          sample.torque_nm = current.torque_norm;
          sample.controller_status = current.controller_error
              ? current.controller_error_detail
              : (current.plan_failed ? "PLAN_FAILED" : "OK");
          sample.guard_result = "STOP";
          sample.collision = guarded.collision;
          sample.stop_reason = abort_reason;
          raw_writer->append(sample);
          stop_sample_template = sample;
          have_stop_sample_template = true;
        }
        break;
      }
      const bool force_torque_warning =
          current.force_norm > force_warning ||
          current.torque_norm > torque_warning;
      if (guarded.decision == rm75_motion_planning::GuardDecision::WARN ||
          force_torque_warning)
        ++warn_count;

      const Eigen::VectorXd joint_error = current.joints - previous_target;
      max_joint_error = std::max(
          max_joint_error,
          joint_error.cwiseAbs().maxCoeff());
      max_joint_change = std::max(
          max_joint_change,
          (current.joints - q0).cwiseAbs().maxCoeff());
      joint_error_square_sum += joint_error.squaredNorm() / 7.0;
      ++joint_error_samples;
      const double feedback_age_ms =
          1000.0 * (now - current.joint_stamp.toSec());
      max_feedback_age_ms = std::max(
          max_feedback_age_ms, feedback_age_ms);
      feedback_age_sum_ms += feedback_age_ms;
      const double position_error =
          (measured_link7.translation() - commanded_link7.translation()).norm();
      const double orientation_error =
          orientationDistance(measured_link7, commanded_link7);
      max_link7_position_error = std::max(
          max_link7_position_error, position_error);
      max_link7_orientation_error = std::max(
          max_link7_orientation_error, orientation_error);
      link7_position_error_square_sum += position_error * position_error;
      link7_orientation_error_square_sum +=
          orientation_error * orientation_error;

      rm_msgs::JointPos message;
      message.joint.resize(7);
      for (int joint = 0; joint < 7; ++joint)
        message.joint[joint] = static_cast<float>(point.position[joint]);
      message.expand = 0.0F;
      command.publish(message);
      const ros::WallTime published_at = ros::WallTime::now();
      if (!last_command_time.isZero()) {
        const double period_ms =
            1000.0 * (published_at - last_command_time).toSec();
        max_command_period_ms = std::max(max_command_period_ms, period_ms);
        command_period_sum_ms += period_ms;
        ++command_period_samples;
      }
      last_command_time = published_at;
      if (raw_writer) {
        rm75_motion_planning::RealExecutionSample sample;
        sample.run_id = run_id;
        sample.scenario = scenario;
        sample.repetition = repetition;
        sample.requested_linear_speed_mps = link7_speed;
        sample.requested_angular_speed_rad_s = angular_speed;
        sample.sample_time_sec = ros::WallTime::now().toSec();
        sample.command_time_sec = published_at.toSec();
        sample.feedback_time_sec = current.joint_stamp.toSec();
        sample.feedback_source_time_sec = current.joint_source_stamp_sec;
        sample.feedback_source_time_valid = current.have_joint_source_stamp;
        sample.command_published = true;
        sample.ideal_pose = ideal_link7;
        sample.commanded_pose = commanded_link7;
        sample.actual_pose = measured_link7;
        sample.target_joints = point.position;
        sample.target_velocity = point.velocity;
        sample.target_acceleration = point.acceleration;
        sample.actual_joints = current.joints;
        sample.force_n = current.force_norm;
        sample.torque_nm = current.torque_norm;
        sample.controller_status = "OK";
        sample.guard_result = force_torque_warning
            ? "WARN" : rm75_motion_planning::toString(guarded.decision);
        sample.collision = guarded.collision;
        raw_writer->append(sample);
      }
      previous_target = point.position;
      rate.sleep();
    }

    rm75_motion_planning::StopRecorder stop;
    if (abort) {
      Feedback at_stop;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex);
        at_stop = feedback;
      }
      stop.request(ros::WallTime::now().toSec(), at_stop.joints);
      emergency.publish(std_msgs::Empty());
      const ros::WallTime observation_deadline =
          ros::WallTime::now() + ros::WallDuration(stop_observation);
      while (ros::ok() && ros::WallTime::now() < observation_deadline) {
        Feedback sample;
        {
          std::lock_guard<std::mutex> lock(feedback_mutex);
          sample = feedback;
        }
        if (sample.have_joints) {
          stop.observe(ros::WallTime::now().toSec(), sample.joints);
          if (raw_writer && have_stop_sample_template) {
            auto raw_sample = stop_sample_template;
            raw_sample.sample_time_sec = ros::WallTime::now().toSec();
            raw_sample.feedback_time_sec = sample.joint_stamp.toSec();
            raw_sample.feedback_source_time_sec =
                sample.joint_source_stamp_sec;
            raw_sample.feedback_source_time_valid =
                sample.have_joint_source_stamp;
            raw_sample.actual_joints = sample.joints;
            raw_sample.actual_pose = model.forward(sample.joints).isometry();
            raw_sample.force_n = sample.force_norm;
            raw_sample.torque_nm = sample.torque_norm;
            raw_writer->append(raw_sample);
          }
        }
        rate.sleep();
      }
      std::cout << "execution=STOP\n"
                << "stop_reason=" << abort_reason << '\n'
                << "fcl_collision_count=" << runtime_collision_count << '\n'
                << "estop_count=1\n"
                << "control_failure_count=1\n"
                << "stop_latency_sec=" << stop.stopLatencySec() << '\n'
                << "stop_observation_sec=" << stop.elapsedSec() << '\n'
                << "stop_distance_max_joint_rad="
                << stop.maxJointDistanceRad() << '\n';
      if (raw_writer) {
        Feedback stopped_feedback;
        {
          std::lock_guard<std::mutex> lock(feedback_mutex);
          stopped_feedback = feedback;
        }
        const Pose stopped_pose = model.forward(stopped_feedback.joints).isometry();
        rm75_motion_planning::writeRealExecutionRunJson(
            run_summary_json, run_id, scenario, "STOP",
            raw_writer->rowsWritten(),
            rm75_motion_planning::measurePoseError(goal_link7, stopped_pose),
            abort_reason, stop.stopLatencySec(), stop.maxJointDistanceRad());
      }
      return 1;
    }

    Feedback final;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex);
      final = feedback;
    }
    const Pose final_actual_link7 = model.forward(final.joints).isometry();
    const double elapsed =
        (ros::WallTime::now() - execution_start).toSec();
    const std::size_t received_feedback =
        final.joint_feedback_count - initial_feedback_count;
    const std::size_t expected_feedback = static_cast<std::size_t>(
        std::floor(elapsed * 50.0));
    const std::size_t dropped_frames = expected_feedback > received_feedback
        ? expected_feedback - received_feedback : 0;
    max_joint_change = std::max(
        max_joint_change, (final.joints - q0).cwiseAbs().maxCoeff());
    const double sample_denominator = static_cast<double>(
        std::max<std::size_t>(joint_error_samples, 1));
    std::cout << "execution=PASS\n"
              << "elapsed_sec=" << elapsed << '\n'
              << "actual_Link7_xyz_m="
              << final_actual_link7.translation().transpose() << '\n'
              << "actual_Link7_quaternion_xyzw="
              << Eigen::Quaterniond(final_actual_link7.rotation()).coeffs().transpose()
              << '\n'
              << "final_Link7_position_error_m="
              << (final_actual_link7.translation() -
                  goal_link7.translation()).norm() << '\n'
              << "final_Link7_orientation_error_rad="
              << orientationDistance(final_actual_link7, goal_link7) << '\n'
              << "max_Link7_tracking_position_error_m="
              << max_link7_position_error << '\n'
              << "rms_Link7_tracking_position_error_m="
              << std::sqrt(link7_position_error_square_sum /
                           sample_denominator) << '\n'
              << "max_Link7_tracking_orientation_error_rad="
              << max_link7_orientation_error << '\n'
              << "rms_Link7_tracking_orientation_error_rad="
              << std::sqrt(link7_orientation_error_square_sum /
                           sample_denominator) << '\n'
              << "max_joint_tracking_error_rad=" << max_joint_error << '\n'
              << "max_joint_change_rad=" << max_joint_change << '\n'
              << "rms_joint_tracking_error_rad="
              << std::sqrt(joint_error_square_sum / sample_denominator) << '\n'
              << "actual_control_frequency_hz="
              << (command_period_samples > 0 ?
                  1000.0 * command_period_samples /
                      std::max(command_period_sum_ms, 1e-9) : 0.0) << '\n'
              << "mean_command_period_ms="
              << command_period_sum_ms /
                  std::max<std::size_t>(command_period_samples, 1) << '\n'
              << "max_command_period_ms=" << max_command_period_ms << '\n'
              << "actual_feedback_frequency_hz="
              << received_feedback / std::max(elapsed, 1e-9) << '\n'
              << "mean_feedback_age_ms="
              << feedback_age_sum_ms / sample_denominator << '\n'
              << "max_feedback_age_ms=" << max_feedback_age_ms << '\n'
              << "estimated_dropped_feedback_frames=" << dropped_frames << '\n'
              << "max_measured_velocity_rad_s="
              << final.max_measured_velocity_rad_s << '\n'
              << "max_measured_acceleration_rad_s2="
              << final.max_measured_acceleration_rad_s2 << '\n'
              << "guard_warn_samples=" << warn_count << '\n'
              << "fcl_collision_count=" << runtime_collision_count << '\n'
              << "peak_force_n=" << peak_force << '\n'
              << "peak_torque_nm=" << peak_torque << '\n'
              << "estop_count=0\n"
              << "control_failure_count=0\n";
    if (raw_writer)
      rm75_motion_planning::writeRealExecutionRunJson(
          run_summary_json, run_id, scenario, "PASS",
          raw_writer->rowsWritten(),
          rm75_motion_planning::measurePoseError(goal_link7,
                                                 final_actual_link7), "");
    return 0;
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("Cartesian commissioning failed: " << error.what());
    return 2;
  }
}
