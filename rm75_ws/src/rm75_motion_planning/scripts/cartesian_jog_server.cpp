#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <ros/master.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <rm_msgs/Err.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Plan_State.h>
#include <rm_msgs/Six_Force.h>
#include <moveit_msgs/GetStateValidity.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_motion_planning/cartesian_jog.h"
#include "rm75_motion_planning/cartesian_path_generator.h"
#include "rm75_motion_planning/command_source_manager.h"
#include "rm75_motion_planning/continuous_ik_solver.h"
#include "rm75_motion_planning/moveit_collision_checker.h"
#include "rm75_motion_planning/runtime_guard.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

namespace {

using rm75_motion_planning::CartesianJogDirection;
using rm75_motion_planning::Pose;
using rm75_motion_planning::TrajectoryPoint;

std::string jsonEscape(const std::string& value) {
  std::string out;
  for (char item : value) {
    if (item == '\\' || item == '"') out.push_back('\\');
    if (item == '\n') out += "\\n";
    else if (item != '\r') out.push_back(item);
  }
  return out;
}

bool orderedJoints(const sensor_msgs::JointState& message,
                   Eigen::VectorXd* result) {
  if (!result || message.name.size() != message.position.size()) return false;
  std::unordered_map<std::string, double> values;
  for (std::size_t i = 0; i < message.name.size(); ++i)
    values[message.name[i]] = message.position[i];
  result->resize(7);
  for (int index = 0; index < 7; ++index) {
    const auto found = values.find("joint" + std::to_string(index + 1));
    if (found == values.end() || !std::isfinite(found->second)) return false;
    (*result)[index] = found->second;
  }
  return true;
}

std::vector<std::string> competingNodes() {
  static const std::vector<std::string> conflicts{
      "/joint_control_server", "/trajectory_bridge", "/rm75_touch_teleop",
      "/rm75_vertical_press", "/rm_control", "/rm75_low_speed_commissioning",
      "/rm75_cartesian_commissioning"};
  ros::V_string nodes;
  if (!ros::master::getNodes(nodes))
    throw std::runtime_error("cannot query ROS nodes");
  std::vector<std::string> active;
  for (const auto& candidate : conflicts)
    if (std::find(nodes.begin(), nodes.end(), candidate) != nodes.end())
      active.push_back(candidate);
  return active;
}

std::string join(const std::vector<std::string>& values) {
  std::string result;
  for (const auto& value : values)
    result += (result.empty() ? "" : ", ") + value;
  return result;
}

class CartesianJogServer {
 public:
  CartesianJogServer()
      : node_("~"),
        model_(RM75_DEFAULT_URDF),
        collision_(RM75_DEFAULT_URDF, RM75_DEFAULT_SRDF),
        sources_(read<bool>("authorized", false)) {
    authorized_ = read<bool>("authorized", false);
    linear_speed_ = read<double>("link7_speed_mps", .01);
    angular_speed_ = read<double>("angular_speed_rad_s", .0872664626);
    max_translation_ = read<double>("max_translation_m", .010);
    max_rotation_ = read<double>("max_rotation_rad", .0872664626);
    feedback_timeout_ = read<double>("feedback_timeout_sec", .10);
    heartbeat_timeout_ = read<double>("heartbeat_timeout_sec", .35);
    stop_timeout_ = read<double>("stop_observation_sec", 2.0);
    force_stop_ = read<double>("force_stop_n", 8.0);
    torque_stop_ = read<double>("torque_stop_nm", .5);
    joint_limit_margin_ = read<double>("joint_limit_margin_rad", .02);
    tracking_limit_ = read<double>("max_tracking_error_rad", .03);
    if (linear_speed_ < .01 || linear_speed_ > .03 || angular_speed_ <= 0.0 ||
        angular_speed_ > .174533 || max_translation_ <= 0.0 ||
        max_translation_ > .0100001 || max_rotation_ <= 0.0 ||
        max_rotation_ > .0872666 || feedback_timeout_ <= 0.0 ||
        feedback_timeout_ > .25 || heartbeat_timeout_ <= 0.0 ||
        heartbeat_timeout_ > 1.0 || stop_timeout_ <= 0.0 ||
        joint_limit_margin_ < 0.0 || tracking_limit_ <= 0.0)
      throw std::runtime_error("Cartesian jog parameters exceed hard envelope");

    const auto limits = model_.limits();
    rm75_motion_planning::RuntimeGuardConfig guard_config;
    guard_config.lower = limits.lower;
    guard_config.upper = limits.upper;
    guard_config.joint_limit_margin_rad = joint_limit_margin_;
    guard_config.warning_limit_margin_rad = std::max(.05, joint_limit_margin_);
    guard_config.max_tracking_error_rad = tracking_limit_;
    guard_config.warning_tracking_ratio = .8;
    guard_config.feedback_timeout_sec = feedback_timeout_;
    guard_.reset(new rm75_motion_planning::RuntimeGuard(
        guard_config, &collision_));
    gateway_.reset(new rm75_motion_planning::GuardedCommandGateway(
        sources_, *guard_));

    std::string source_reason;
    if (!sources_.acquire(rm75_motion_planning::CommandSource::AUTONOMOUS,
                          &source_reason))
      throw std::runtime_error("cannot acquire Cartesian jog source: " +
                               source_reason);
    if (authorized_) {
      if (!rm75_motion_planning::commandTopicUnclaimed(
              "/rm_driver/JointPos", &source_reason))
        throw std::runtime_error("JointPos ownership check failed: " +
                                 source_reason);
      command_ = node_.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 1);
    }
    stop_ = node_.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1);
    status_ = node_.advertise<std_msgs::String>(
        "/rm75_cartesian_jog/status", 1, true);
    subscribers_ = {
        node_.subscribe("/joint_states", 20,
                        &CartesianJogServer::jointCallback, this),
        node_.subscribe("/rm_driver/UdpSixZeroForce", 20,
                        &CartesianJogServer::forceCallback, this),
        node_.subscribe("/rm75_sampling_gui/cartesian_jog_heartbeat", 1,
                        &CartesianJogServer::heartbeatCallback, this),
        node_.subscribe("/rm_driver/Set_Arm_Stop_Result", 10,
                        &CartesianJogServer::stopAckCallback, this),
        node_.subscribe("/rm_driver/Plan_State", 10,
                        &CartesianJogServer::planCallback, this),
        node_.subscribe("/rm_driver/Error", 10,
                        &CartesianJogServer::errorCallback, this)};

    addStartService("x_plus"); addStartService("x_minus");
    addStartService("y_plus"); addStartService("y_minus");
    addStartService("z_plus"); addStartService("z_minus");
    addStartService("rx_plus"); addStartService("rx_minus");
    addStartService("ry_plus"); addStartService("ry_minus");
    addStartService("rz_plus"); addStartService("rz_minus");
    stop_service_ = node_.advertiseService(
        "/rm75_cartesian_jog/stop", &CartesianJogServer::stopCallback, this);
    timer_ = node_.createWallTimer(
        ros::WallDuration(.02), &CartesianJogServer::timerCallback, this);
    publishStatus();
    ROS_WARN("Cartesian jog server ready: authorized=%s Link7 max=%.1fmm/%.1fdeg",
             authorized_ ? "true" : "false", max_translation_ * 1000.0,
             max_rotation_ * 180.0 / M_PI);
  }

 private:
  template <typename T>
  T read(const std::string& name, const T& fallback) {
    T value = fallback;
    node_.param(name, value, fallback);
    return value;
  }

  void addStartService(const std::string& direction) {
    start_services_.push_back(node_.advertiseService<std_srvs::Trigger::Request,
        std_srvs::Trigger::Response>(
        "/rm75_cartesian_jog/start/" + direction,
        [this, direction](std_srvs::Trigger::Request&,
                          std_srvs::Trigger::Response& response) {
          return startCallback(direction, response);
        }));
  }

  void jointCallback(const sensor_msgs::JointStateConstPtr& message) {
    Eigen::VectorXd joints;
    if (!orderedJoints(*message, &joints)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    joints_ = joints;
    joint_stamp_ = ros::WallTime::now();
    have_joints_ = true;
  }

  void forceCallback(const rm_msgs::Six_ForceConstPtr& message) {
    const double force = std::sqrt(
        message->force_Fx * message->force_Fx +
        message->force_Fy * message->force_Fy +
        message->force_Fz * message->force_Fz);
    const double torque = std::sqrt(
        message->force_Mx * message->force_Mx +
        message->force_My * message->force_My +
        message->force_Mz * message->force_Mz);
    if (!std::isfinite(force) || !std::isfinite(torque)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    force_norm_ = force;
    torque_norm_ = torque;
    force_stamp_ = ros::WallTime::now();
    have_force_ = true;
  }

  void heartbeatCallback(const std_msgs::EmptyConstPtr&) {
    std::lock_guard<std::mutex> lock(mutex_);
    heartbeat_stamp_ = ros::WallTime::now();
    have_heartbeat_ = true;
  }

  void stopAckCallback(const std_msgs::BoolConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stop_ack_sequence_;
    stop_ack_value_ = message->data;
  }

  void planCallback(const rm_msgs::Plan_StateConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!message->state) plan_failed_ = true;
  }

  void errorCallback(const rm_msgs::ErrConstPtr& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto code : message->err) {
      if (code != 0) {
        driver_fault_ = "driver error code " + std::to_string(code);
        break;
      }
    }
  }

  bool startCallback(const std::string& value,
                     std_srvs::Trigger::Response& response) {
    CartesianJogDirection direction;
    if (!rm75_motion_planning::parseCartesianJogDirection(value, &direction)) {
      response.success = false;
      response.message = "unsupported Cartesian jog direction";
      return true;
    }
    try {
      start(direction);
      response.success = true;
      response.message = "Cartesian jog started";
    } catch (const std::exception& error) {
      response.success = false;
      response.message = error.what();
      std::lock_guard<std::mutex> lock(mutex_);
      stop_reason_ = error.what();
      publishStatusLocked();
    }
    return true;
  }

  void start(CartesianJogDirection direction) {
    if (!authorized_ || !sources_.commandPublisherAllowed())
      throw std::runtime_error("Cartesian jog execution is disarmed");
    if (!ros::service::exists("/check_state_validity", false))
      throw std::runtime_error("MoveIt state validity service unavailable");
    const auto conflicts = competingNodes();
    if (!conflicts.empty())
      throw std::runtime_error("competing control node: " + join(conflicts));

    Eigen::VectorXd current;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == "JOGGING" || state_ == "STOPPING" ||
          state_ == "LIMIT_REACHED")
        throw std::runtime_error(
            "release the active direction before starting another jog");
      const ros::WallTime now = ros::WallTime::now();
      if (!have_joints_ || (now - joint_stamp_).toSec() > feedback_timeout_)
        throw std::runtime_error("fresh joint feedback unavailable");
      if (!have_force_ || (now - force_stamp_).toSec() > feedback_timeout_)
        throw std::runtime_error("fresh force feedback unavailable");
      if (!have_heartbeat_ ||
          (now - heartbeat_stamp_).toSec() > heartbeat_timeout_)
        throw std::runtime_error("web control heartbeat unavailable");
      if (force_norm_ > force_stop_ || torque_norm_ > torque_stop_)
        throw std::runtime_error("force/torque safety threshold exceeded");
      if (!driver_fault_.empty()) throw std::runtime_error(driver_fault_);
      if (plan_failed_)
        throw std::runtime_error("controller reported planning failure");
      current = joints_;
    }

    const Pose start_pose = model_.forward(current).isometry();
    moveit_msgs::GetStateValidity validity;
    validity.request.group_name = "arm";
    validity.request.robot_state.joint_state.name = {
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    validity.request.robot_state.joint_state.position.assign(
        current.data(), current.data() + current.size());
    if (!ros::service::call("/check_state_validity", validity))
      throw std::runtime_error("MoveIt state validity service call failed");
    if (!validity.response.valid)
      throw std::runtime_error("MoveIt reports current-state collision");
    const Pose target_pose = rm75_motion_planning::cartesianJogTarget(
        start_pose, direction, max_translation_, max_rotation_);
    const auto poses = rm75_motion_planning::CartesianPathGenerator::line(
        start_pose, target_pose, {.002, .0174532925});
    rm75_motion_planning::ContinuousIkOptions ik_options;
    ik_options.max_joint_jump = .20;
    ik_options.enable_ipopt_fallback = false;
    rm75_kinematics::IkOptions dls;
    dls.position_tolerance = 2e-4;
    dls.orientation_tolerance = 2e-3;
    dls.timeout_sec = .2;
    const auto solved = rm75_motion_planning::ContinuousIkSolver(model_).solve(
        poses, current, ik_options, dls);
    if (!solved.success)
      throw std::runtime_error("continuous IK failed: " + solved.failure_reason);

    for (std::size_t index = 0; index < solved.joints.size(); ++index) {
      validity.request.robot_state.joint_state.position.assign(
          solved.joints[index].data(),
          solved.joints[index].data() + solved.joints[index].size());
      if (!ros::service::call("/check_state_validity", validity))
        throw std::runtime_error(
            "MoveIt state validity service failed at path sample " +
            std::to_string(index));
      if (!validity.response.valid)
        throw std::runtime_error(
            "MoveIt reports collision at path sample " +
            std::to_string(index));
    }

    rm75_motion_planning::DynamicLimits dynamics;
    dynamics.max_velocity = Eigen::VectorXd::Constant(7, .05);
    dynamics.max_acceleration = Eigen::VectorXd::Constant(7, .10);
    dynamics.max_jerk = Eigen::VectorXd::Constant(7, .50);
    const double duration = rm75_motion_planning::isCartesianJogRotation(direction)
        ? max_rotation_ / angular_speed_
        : max_translation_ / linear_speed_;
    const auto trajectory =
        rm75_motion_planning::TrajectoryParameterizer::parameterizeAtLeastDuration(
            solved.joints, dynamics, duration, .02);
    const auto validation = rm75_motion_planning::TrajectoryValidator(
        model_, &collision_).validate(solved.solved_poses, solved.joints,
                                      trajectory, dynamics, .001, .01, .20, true);
    if (!validation.valid) {
      std::string reason;
      for (const auto& item : validation.violations)
        reason += (reason.empty() ? "" : "; ") + item;
      throw std::runtime_error("trajectory validation failed: " + reason);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime commit_time = ros::WallTime::now();
    if (!have_joints_ ||
        (commit_time - joint_stamp_).toSec() > feedback_timeout_)
      throw std::runtime_error("joint feedback expired during jog preflight");
    if (!have_force_ ||
        (commit_time - force_stamp_).toSec() > feedback_timeout_)
      throw std::runtime_error("force feedback expired during jog preflight");
    if (!have_heartbeat_ ||
        (commit_time - heartbeat_stamp_).toSec() > heartbeat_timeout_)
      throw std::runtime_error("web heartbeat expired during jog preflight");
    if (force_norm_ > force_stop_ || torque_norm_ > torque_stop_)
      throw std::runtime_error(
          "force/torque threshold exceeded during jog preflight");
    if (!driver_fault_.empty()) throw std::runtime_error(driver_fault_);
    if (plan_failed_)
      throw std::runtime_error("controller failed during jog preflight");
    if (joints_.size() != current.size() ||
        (joints_ - current).cwiseAbs().maxCoeff() > tracking_limit_)
      throw std::runtime_error(
          "joint feedback changed beyond limit during jog preflight");
    trajectory_ = trajectory;
    direction_ = rm75_motion_planning::toString(direction);
    start_pose_ = start_pose;
    target_pose_ = target_pose;
    previous_target_ = current;
    started_ = ros::WallTime::now();
    state_ = "JOGGING";
    stop_reason_.clear();
    pending_limit_stop_ = false;
    progress_ = 0.0;
    publishStatusLocked();
  }

  bool stopCallback(std_srvs::Trigger::Request&,
                    std_srvs::Trigger::Response& response) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == "IDLE" || state_ == "STOPPED") {
        response.success = true;
        response.message = "already stopped";
        return true;
      }
      if (state_ == "LIMIT_REACHED") {
        state_ = "STOPPED";
        direction_.clear();
        publishStatusLocked();
        response.success = true;
        response.message = "limit stop confirmed";
        return true;
      }
      pending_limit_stop_ = false;
      requestStopLocked("operator release", false);
    }
    std::unique_lock<std::mutex> lock(mutex_);
    stop_condition_.wait_for(lock, std::chrono::duration<double>(stop_timeout_ + .5),
                             [this] { return state_ != "STOPPING"; });
    response.success = state_ == "STOPPED";
    response.message = response.success ? "stop ACK and stationary feedback confirmed"
                                        : stop_reason_;
    return true;
  }

  void requestStopLocked(const std::string& reason, bool safe_stop) {
    if (state_ == "STOPPING") {
      pending_safe_stop_ = pending_safe_stop_ || safe_stop;
      return;
    }
    if (state_ != "JOGGING" && state_ != "LIMIT_REACHED") return;
    state_ = "STOPPING";
    stop_reason_ = reason;
    pending_safe_stop_ = safe_stop;
    stop_requested_ = ros::WallTime::now();
    stop_ack_baseline_ = stop_ack_sequence_;
    stop_previous_ = joints_;
    stable_samples_ = 0;
    stop_.publish(std_msgs::Empty());
    publishStatusLocked();
  }

  void timerCallback(const ros::WallTimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::WallTime now = ros::WallTime::now();
    if (state_ == "JOGGING") {
      std::string failure;
      if (!have_heartbeat_ ||
          (now - heartbeat_stamp_).toSec() > heartbeat_timeout_)
        failure = "web control heartbeat timeout";
      else if (!have_joints_ ||
               (now - joint_stamp_).toSec() > feedback_timeout_)
        failure = "joint feedback timeout";
      else if (!have_force_ ||
               (now - force_stamp_).toSec() > feedback_timeout_)
        failure = "force feedback timeout";
      else if (force_norm_ > force_stop_ || torque_norm_ > torque_stop_)
        failure = "force/torque safety threshold exceeded";
      else if (!driver_fault_.empty()) failure = driver_fault_;
      else if (plan_failed_) failure = "controller reported planning failure";
      if (!failure.empty()) {
        pending_limit_stop_ = false;
        requestStopLocked(failure, true);
        return;
      }
      const double elapsed = (now - started_).toSec();
      std::size_t index = 0;
      while (index + 1 < trajectory_.size() &&
             trajectory_[index].time < elapsed) ++index;
      const auto& point = trajectory_[index];
      rm75_motion_planning::RuntimeGuardInput input;
      input.current = joints_;
      input.next_target = point.position;
      input.previous_target = previous_target_;
      input.commanded_velocity = point.velocity;
      input.cycle_sec = .02;
      input.now_sec = now.toSec();
      input.feedback_stamp_sec = joint_stamp_.toSec();
      const auto result = gateway_->evaluate(
          rm75_motion_planning::CommandSource::AUTONOMOUS, input);
      if (!result.accepted) {
        pending_limit_stop_ = false;
        requestStopLocked(result.reason, true);
        return;
      }
      rm_msgs::JointPos message;
      message.joint.assign(point.position.data(),
                           point.position.data() + point.position.size());
      message.expand = 0.0;
      command_.publish(message);
      previous_target_ = point.position;
      progress_ = trajectory_.back().time > 0.0
          ? std::min(1.0, elapsed / trajectory_.back().time) : 1.0;
      if (index + 1 == trajectory_.size() &&
          (joints_ - point.position).cwiseAbs().maxCoeff() <= .015) {
        pending_limit_stop_ = true;
        requestStopLocked(
            "maximum jog travel reached; release before restarting", false);
      }
      publishStatusLocked();
    } else if (state_ == "STOPPING") {
      if (have_joints_ && (now - joint_stamp_).toSec() <= feedback_timeout_) {
        const bool stationary = stop_previous_.size() == joints_.size() &&
            (joints_ - stop_previous_).cwiseAbs().maxCoeff() < .001;
        stable_samples_ = stationary ? stable_samples_ + 1 : 0;
        stop_previous_ = joints_;
      } else {
        stable_samples_ = 0;
      }
      const bool acknowledged = stop_ack_sequence_ > stop_ack_baseline_ &&
                                stop_ack_value_;
      if (acknowledged && stable_samples_ >= 5) {
        state_ = pending_safe_stop_ ? "SAFE_STOP" :
                 (pending_limit_stop_ ? "LIMIT_REACHED" : "STOPPED");
        if (!pending_limit_stop_) direction_.clear();
        trajectory_.clear();
        stop_condition_.notify_all();
      } else if ((now - stop_requested_).toSec() > stop_timeout_) {
        state_ = "SAFE_STOP";
        stop_reason_ += "; STOP_UNCONFIRMED";
        direction_.clear();
        trajectory_.clear();
        stop_condition_.notify_all();
      }
      publishStatusLocked();
    } else {
      publishStatusLocked();
    }
  }

  static void appendPose(std::ostringstream& stream, const Pose& pose) {
    const Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);
    stream << "{\"x\":" << pose.translation().x()
           << ",\"y\":" << pose.translation().y()
           << ",\"z\":" << pose.translation().z()
           << ",\"rx\":" << rpy.x()
           << ",\"ry\":" << rpy.y()
           << ",\"rz\":" << rpy.z() << "}";
  }

  void publishStatus() {
    std::lock_guard<std::mutex> lock(mutex_);
    publishStatusLocked();
  }

  void publishStatusLocked() {
    Pose actual = Pose::Identity();
    if (have_joints_) actual = model_.forward(joints_).isometry();
    std::ostringstream stream;
    stream << std::setprecision(10)
           << "{\"state\":\"" << jsonEscape(state_)
           << "\",\"direction\":\"" << jsonEscape(direction_)
           << "\",\"stop_reason\":\"" << jsonEscape(stop_reason_)
           << "\",\"reference_frame\":\"base_link\""
           << ",\"rotation_frame\":\"Link7\""
           << ",\"end_effector\":\"Link7\""
           << ",\"tcp_calibrated\":false"
           << ",\"progress\":" << progress_
           << ",\"limits\":{\"translation_m\":" << max_translation_
           << ",\"rotation_rad\":" << max_rotation_
           << ",\"linear_speed_mps\":" << linear_speed_
           << ",\"angular_speed_rad_s\":" << angular_speed_ << "}"
           << ",\"actual_pose\":";
    appendPose(stream, actual);
    stream << ",\"target_pose\":";
    appendPose(stream, target_pose_);
    stream << "}";
    std_msgs::String message;
    message.data = stream.str();
    status_.publish(message);
  }

  ros::NodeHandle node_;
  rm75_kinematics::KinematicsModel model_;
  rm75_motion_planning::MoveitCollisionChecker collision_;
  rm75_motion_planning::CommandSourceManager sources_;
  std::unique_ptr<rm75_motion_planning::RuntimeGuard> guard_;
  std::unique_ptr<rm75_motion_planning::GuardedCommandGateway> gateway_;
  ros::Publisher command_, stop_, status_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::ServiceServer> start_services_;
  ros::ServiceServer stop_service_;
  ros::WallTimer timer_;
  std::mutex mutex_;
  std::condition_variable stop_condition_;
  Eigen::VectorXd joints_, previous_target_, stop_previous_;
  std::vector<TrajectoryPoint, Eigen::aligned_allocator<TrajectoryPoint>> trajectory_;
  Pose start_pose_{Pose::Identity()}, target_pose_{Pose::Identity()};
  ros::WallTime joint_stamp_, force_stamp_, heartbeat_stamp_, started_,
                stop_requested_;
  bool have_joints_{false}, have_force_{false}, have_heartbeat_{false};
  bool authorized_{false}, plan_failed_{false}, stop_ack_value_{false};
  bool pending_safe_stop_{false}, pending_limit_stop_{false};
  unsigned int stop_ack_sequence_{0}, stop_ack_baseline_{0};
  int stable_samples_{0};
  double force_norm_{0.0}, torque_norm_{0.0}, progress_{0.0};
  double linear_speed_{.01}, angular_speed_{.0872664626};
  double max_translation_{.01}, max_rotation_{.0872664626};
  double feedback_timeout_{.10}, heartbeat_timeout_{.35};
  double stop_timeout_{2.0}, force_stop_{8.0}, torque_stop_{.5};
  double joint_limit_margin_{.02}, tracking_limit_{.03};
  std::string state_{"IDLE"}, direction_, stop_reason_, driver_fault_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_cartesian_jog");
  try {
    CartesianJogServer server;
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
  } catch (const std::exception& error) {
    ROS_FATAL("Cartesian jog server failed: %s", error.what());
    return 1;
  }
}
