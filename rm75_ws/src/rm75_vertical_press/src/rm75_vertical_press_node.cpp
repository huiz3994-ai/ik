#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Six_Force.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#include "rm75_vertical_press/tool5_feed.h"
#include "rm75_touch_teleop/hybrid_force_control.h"
#include "rm75_touch_teleop/mapping.h"

namespace {

bool finitePose(const geometry_msgs::Pose& pose) {
  const double values[] = {pose.position.x, pose.position.y, pose.position.z,
                           pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w};
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  return std::sqrt(pose.orientation.x * pose.orientation.x +
                   pose.orientation.y * pose.orientation.y +
                   pose.orientation.z * pose.orientation.z +
                   pose.orientation.w * pose.orientation.w) > 1e-6;
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
  result.linear() = quaternion.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose result;
  result.position.x = transform.translation().x();
  result.position.y = transform.translation().y();
  result.position.z = transform.translation().z();
  Eigen::Quaterniond quaternion(transform.linear());
  quaternion.normalize();
  result.orientation.x = quaternion.x();
  result.orientation.y = quaternion.y();
  result.orientation.z = quaternion.z();
  result.orientation.w = quaternion.w();
  return result;
}

class VerticalPressController {
 public:
  VerticalPressController()
      : nh_(), private_nh_("~"), model_loader_("robot_description") {
    loadParameters();
    initialiseRobotModel();

    touch_pose_sub_ = nh_.subscribe(touch_pose_topic_, 1,
        &VerticalPressController::touchPoseCallback, this);
    joy_sub_ = nh_.subscribe(touch_joy_topic_, 1,
        &VerticalPressController::joyCallback, this);
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1,
        &VerticalPressController::jointStateCallback, this);
    force_sub_ = nh_.subscribe(force_topic_, 1,
        &VerticalPressController::forceCallback, this);
    coordinate_sub_ = nh_.subscribe("/rm_driver/Udp_Coordinate", 1,
        &VerticalPressController::coordinateCallback, this);
    drill_start_sub_ = private_nh_.subscribe("drill_start_depth_m", 1,
        &VerticalPressController::drillStartCallback, this);
    drill_stop_sub_ = private_nh_.subscribe("drill_stop_feed", 1,
        &VerticalPressController::drillStopCallback, this);

    command_pub_ = nh_.advertise<rm_msgs::JointPos>(command_topic_, 1);
    stop_pub_ = nh_.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1);
    target_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    active_pub_ = private_nh_.advertise<std_msgs::Bool>("active", 1, true);
    mode_pub_ = private_nh_.advertise<std_msgs::String>("control_mode", 1, true);
    reason_pub_ = private_nh_.advertise<std_msgs::String>("stop_reason", 1, true);
    raw_force_pub_ = private_nh_.advertise<std_msgs::Float64>("force_z", 1);
    force_pub_ = private_nh_.advertise<std_msgs::Float64>("filtered_force_z", 1);
    force_target_pub_ =
        private_nh_.advertise<std_msgs::Float64>("force_setpoint", 1, true);
    drill_state_pub_ = private_nh_.advertise<std_msgs::String>("drill_state", 1, true);
    drill_depth_pub_ = private_nh_.advertise<std_msgs::Float64>("drill_actual_depth_m", 1);

    timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0 / std::max(1.0, publish_rate_)),
        &VerticalPressController::update, this);
    publishActive(false);
    publishMode();
    std_msgs::Float64 force_target;
    force_target.data = hybrid_.cfg.desired_force;
    force_target_pub_.publish(force_target);
    publishDrillState("IDLE");

    ROS_INFO("RM75 vertical-press controller ready");
    ROS_INFO("Grey: orientation control; white: position control; grey+white: lock XY/orientation and press along base -Z");
    if (!motion_enabled_) {
      ROS_WARN("RM75 motion is DISARMED. Pass enable_motion:=true only after the workspace is safe.");
    }
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("planning_group", planning_group_, "arm");
    private_nh_.param<std::string>("tip_link", tip_link_, "Link7");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("touch_pose_topic", touch_pose_topic_,
                                   "/Geomagic/pose_raw");
    private_nh_.param<std::string>("touch_joy_topic", touch_joy_topic_,
                                   "/Geomagic/joy");
    private_nh_.param<std::string>("joint_state_topic", joint_state_topic_,
                                   "/joint_states");
    private_nh_.param<std::string>("command_topic", command_topic_,
                                   "/rm_driver/JointPos");
    private_nh_.param<std::string>("force_topic", force_topic_,
                                   "/rm_driver/UdpSixZeroForce");
    private_nh_.param("motion_enabled", motion_enabled_, false);
    private_nh_.param("metres_per_touch_unit", metres_per_touch_unit_, 0.001);
    private_nh_.param("position_scale", position_scale_, 0.4);
    private_nh_.param("publish_rate", publish_rate_, 50.0);
    private_nh_.param("ik_timeout", ik_timeout_, 0.015);
    private_nh_.param("input_timeout", input_timeout_, 0.25);
    private_nh_.param("max_touch_translation", max_touch_translation_, 0.08);
    private_nh_.param("max_joint_velocity", max_joint_velocity_, 0.20);
    private_nh_.param("max_joint_tracking_error", max_joint_tracking_error_, 0.15);
    private_nh_.param("j2_limit_avoidance_enabled",
                      j2_limit_avoidance_enabled_, true);
    private_nh_.param("j2_limit_avoidance_margin",
                      j2_limit_avoidance_margin_, 0.25);
    private_nh_.param("j2_reseed_step", j2_reseed_step_, 0.05);
    private_nh_.param("j2_reseed_timeout", j2_reseed_timeout_, 0.002);
    private_nh_.param("j2_reseed_attempts", j2_reseed_attempts_, 4);
    private_nh_.param("j2_reseed_max_joint_delta",
                      j2_reseed_max_joint_delta_, 0.35);
    private_nh_.param("max_external_force", max_external_force_, 100.0);
    private_nh_.param("max_external_torque", max_external_torque_, 5.0);
    private_nh_.param("force_feedback_timeout", force_feedback_timeout_, 0.25);
    private_nh_.param("force_sign", force_sign_, -1.0);
    private_nh_.param("force_frame_verified", force_frame_verified_, false);
    private_nh_.param("expected_force_coordinate", expected_coordinate_, 1);
    private_nh_.param("direct_press_speed", direct_press_speed_, 0.003);
    private_nh_.param("direct_press_max_travel", direct_press_max_travel_, 0.05);
    private_nh_.param("vertical_alignment_tolerance",
                      vertical_alignment_tolerance_, 0.0523598776);
    private_nh_.param("tool5_drill_enabled", tool5_drill_enabled_, false);
    private_nh_.param("tool5_require_force_feedback",
                      tool5_require_force_feedback_, false);
    private_nh_.param("tool5_drill_max_feed_travel",
                      tool5_drill_max_feed_travel_, 0.05);

    auto& config = hybrid_.cfg;
    private_nh_.param("desired_force", config.desired_force, config.desired_force);
    private_nh_.param("contact_threshold", config.contact_threshold,
                      config.contact_threshold);
    private_nh_.param("release_threshold", config.release_threshold,
                      config.release_threshold);
    private_nh_.param("force_gain", config.force_gain, config.force_gain);
    private_nh_.param("force_limit", config.force_limit, config.force_limit);
    private_nh_.param("max_z_velocity", config.max_z_velocity,
                      config.max_z_velocity);
    private_nh_.param("max_z_step", config.max_z_step, config.max_z_step);
    private_nh_.param("max_force_travel", config.max_force_travel,
                      config.max_force_travel);
    private_nh_.param("max_z_tracking_error", config.max_z_tracking_error,
                      config.max_z_tracking_error);
    private_nh_.param("force_deadband", config.force_deadband,
                      config.force_deadband);
    private_nh_.param("force_filter_tau", config.filter_tau, config.filter_tau);
    private_nh_.param("contact_debounce", config.transition_time,
                      config.transition_time);
    private_nh_.param("force_ramp_rate", config.force_ramp_rate,
                      config.force_ramp_rate);
    private_nh_.param("force_motion_sign", config.motion_sign,
                      config.motion_sign);
    config.validate();

    const double positive[] = {
        metres_per_touch_unit_, position_scale_, publish_rate_, ik_timeout_,
        input_timeout_, max_touch_translation_, max_joint_velocity_,
        max_joint_tracking_error_, max_external_force_, max_external_torque_,
        force_feedback_timeout_, direct_press_speed_, direct_press_max_travel_,
        vertical_alignment_tolerance_, tool5_drill_max_feed_travel_};
    for (double value : positive) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error("vertical-press parameters must be finite and positive");
      }
    }
    if (force_sign_ != 1.0 && force_sign_ != -1.0) {
      throw std::runtime_error("force_sign must be +1 or -1");
    }
    if (expected_coordinate_ < 0 || expected_coordinate_ > 2) {
      throw std::runtime_error("expected_force_coordinate must be in [0,2]");
    }
    if (max_external_force_ <= std::abs(config.desired_force)) {
      throw std::runtime_error("max_external_force must exceed abs(desired_force)");
    }
    if (!std::isfinite(j2_limit_avoidance_margin_) ||
        j2_limit_avoidance_margin_ <= 0.0 ||
        !std::isfinite(j2_reseed_step_) || j2_reseed_step_ <= 0.0 ||
        !std::isfinite(j2_reseed_timeout_) || j2_reseed_timeout_ <= 0.0 ||
        j2_reseed_attempts_ < 1 || j2_reseed_attempts_ > 12 ||
        !std::isfinite(j2_reseed_max_joint_delta_) ||
        j2_reseed_max_joint_delta_ <= 0.0) {
      throw std::runtime_error("Invalid J2 joint-limit avoidance parameters");
    }
  }

  void initialiseRobotModel() {
    robot_model_ = model_loader_.getModel();
    if (!robot_model_) throw std::runtime_error("robot_description could not be loaded");
    joint_group_ = robot_model_->getJointModelGroup(planning_group_);
    if (!joint_group_) {
      throw std::runtime_error("MoveIt group '" + planning_group_ + "' does not exist");
    }
    if (!robot_model_->hasLinkModel(tip_link_) ||
        !robot_model_->hasLinkModel(base_frame_)) {
      throw std::runtime_error("configured base or tip link does not exist");
    }
    joint_names_ = joint_group_->getVariableNames();
    if (joint_names_.size() != 7) {
      throw std::runtime_error("RM75 vertical press requires exactly seven arm joints");
    }
    latest_state_.reset(new moveit::core::RobotState(robot_model_));
    command_seed_.reset(new moveit::core::RobotState(robot_model_));
    latest_state_->setToDefaultValues();
    command_seed_->setToDefaultValues();
  }

  void touchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (!finitePose(message->pose)) return;
    latest_touch_ = poseToEigen(message->pose);
    have_touch_ = true;
    last_touch_time_ = ros::WallTime::now();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& message) {
    if (message->name.size() != message->position.size()) return;
    std::map<std::string, double> positions;
    for (std::size_t index = 0; index < message->name.size(); ++index) {
      if (std::isfinite(message->position[index])) {
        positions[message->name[index]] = message->position[index];
      }
    }
    for (const std::string& name : joint_names_) {
      if (!positions.count(name)) return;
    }
    latest_state_->setVariablePositions(positions);
    latest_state_->update();
    have_joint_state_ = true;
    last_joint_time_ = ros::WallTime::now();
    if (!have_command_seed_) {
      *command_seed_ = *latest_state_;
      latest_state_->copyJointGroupPositions(joint_group_, last_command_);
      have_command_seed_ = true;
    }
  }

  void forceCallback(const rm_msgs::Six_Force::ConstPtr& message) {
    const double values[] = {message->force_Fx, message->force_Fy,
        message->force_Fz, message->force_Mx, message->force_My,
        message->force_Mz};
    for (double value : values) {
      if (!std::isfinite(value)) {
        have_force_ = false;
        if (active_) safetyStop("non-finite force feedback");
        return;
      }
    }
    force_norm_ = std::sqrt(message->force_Fx * message->force_Fx +
                            message->force_Fy * message->force_Fy +
                            message->force_Fz * message->force_Fz);
    torque_norm_ = std::sqrt(message->force_Mx * message->force_Mx +
                             message->force_My * message->force_My +
                             message->force_Mz * message->force_Mz);
    const ros::WallTime now = ros::WallTime::now();
    const double dt = have_force_ ? (now - last_force_time_).toSec() : 1.0;
    if (!have_force_ || dt > force_feedback_timeout_) hybrid_.have_force = false;
    hybrid_.observe(force_sign_ * message->force_Fz, dt);
    have_force_ = true;
    last_force_time_ = now;
    std_msgs::Float64 filtered;
    filtered.data = hybrid_.filtered;
    std_msgs::Float64 raw;
    raw.data = force_sign_ * message->force_Fz;
    raw_force_pub_.publish(raw);
    force_pub_.publish(filtered);
    if (forceLimitExceeded() && active_) {
      safetyStop("six-axis force safety limit exceeded");
    }
  }

  void coordinateCallback(const std_msgs::UInt16::ConstPtr& message) {
    coordinate_ = message->data;
    last_coordinate_time_ = ros::WallTime::now();
    if (active_ && coordinate_ != expected_coordinate_) {
      safetyStop("force coordinate mismatch");
    }
  }

  void drillStartCallback(const std_msgs::Float64::ConstPtr& message) {
    if (!tool5_drill_enabled_) {
      ROS_WARN("Tool5 drill start rejected: dedicated mode is disabled");
      return;
    }
    if (!std::isfinite(message->data) || message->data <= 0.0 ||
        message->data > tool5_drill_max_feed_travel_) {
      safetyStop("invalid Tool5 drill depth or independent feed limit exceeded");
      return;
    }
    if (active_) {
      ROS_WARN("Tool5 drill start rejected: controller is already active");
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    const bool force_ready = force_frame_verified_ && coordinateReady(now) &&
        have_force_ &&
        (now - last_force_time_).toSec() <= force_feedback_timeout_ &&
        !forceLimitExceeded();
    if (!motion_enabled_ || !have_joint_state_ ||
        (now - last_joint_time_).toSec() > input_timeout_ ||
        !latest_state_->satisfiesBounds(joint_group_)) {
      safetyStop("Tool5 drill preflight failed: motion, joint, or limit check");
      return;
    }
    if (tool5_require_force_feedback_ && !force_ready) {
      safetyStop("Tool5 drill preflight failed: required force feedback is unavailable");
      return;
    }
    const Eigen::Isometry3d measured =
        latest_state_->getGlobalLinkTransform(tip_link_);
    robot_anchor_ = target_pose_ = measured;
    robot_orientation_anchor_ = measured.linear();
    *command_seed_ = *latest_state_;
    latest_state_->copyJointGroupPositions(joint_group_, last_command_);
    have_command_seed_ = true;
    drill_start_z_ = measured.translation().z();
    drill_target_depth_ = message->data;
    drill_actual_depth_ = 0.0;
    drill_force_control_active_ = tool5_require_force_feedback_ && force_ready;
    if (drill_force_control_active_) {
      hybrid_.beginForceFeed(drill_start_z_);
      if (hybrid_.mode != rm75_touch_teleop::ControlMode::CONTACT_MODE) {
        safetyStop("Tool5 drill start failed: force feed could not start");
        return;
      }
    } else {
      hybrid_.reset(drill_start_z_);
    }
    last_update_time_ = now;
    stop_latched_ = false;
    drill_active_ = true;
    drill_holding_ = false;
    active_ = true;
    std_msgs::String empty_reason;
    reason_pub_.publish(empty_reason);
    publishActive(true);
    publishMode();
    publishDrillState("DRILLING");
    publishDrillDepth();
    ROS_WARN("Tool5 direct Z drill feed engaged from current pose: target depth %.4f m, mode=%s",
             drill_target_depth_,
             drill_force_control_active_ ? "FORCE" : "BOUNDED_POSITION");
  }

  void drillStopCallback(const std_msgs::Empty::ConstPtr&) {
    if (!active_ || (!drill_active_ && !drill_holding_)) return;
    drill_active_ = false;
    drill_holding_ = true;
    if (have_joint_state_) {
      const double measured_z =
          latest_state_->getGlobalLinkTransform(tip_link_).translation().z();
      drill_actual_depth_ = std::max(0.0, drill_start_z_ - measured_z);
      hybrid_.reset(measured_z);
    }
    publishMode();
    publishDrillState("HOLDING");
    publishDrillDepth();
    ROS_INFO("Tool5 Z feed stopped; holding current pose without retract");
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& message) {
    if (message->buttons.size() < 2) return;
    last_joy_time_ = ros::WallTime::now();
    const bool grey = message->buttons[0] != 0;
    const bool white = message->buttons[1] != 0;
    const bool grey_rising = grey && !grey_pressed_;
    const bool white_rising = white && !white_pressed_;
    const bool entering_direct = grey && white && !(grey_pressed_ && white_pressed_);
    const bool leaving_direct = grey_pressed_ && white_pressed_ && !(grey && white);

    if (!grey && !white) released_since_stop_ = true;
    if (!active_ && (grey_rising || white_rising)) {
      engage();
    }
    if (!active_) {
      grey_pressed_ = grey;
      white_pressed_ = white;
      return;
    }

    if (grey_rising && !white) setOrientationAnchor();
    if (white_rising && !grey) setPositionAnchor();
    if (entering_direct) {
      const Eigen::Matrix3d measured_orientation =
          latest_state_->getGlobalLinkTransform(tip_link_).linear();
      const double alignment_cosine = std::max(-1.0, std::min(1.0,
          measured_orientation.col(2).dot(Eigen::Vector3d(0.0, 0.0, -1.0))));
      const double alignment_error = std::acos(alignment_cosine);
      if (alignment_error > vertical_alignment_tolerance_) {
        safetyStop("tool is not vertical; use the grey button to align before dual-button descent");
        grey_pressed_ = grey;
        white_pressed_ = white;
        return;
      }
      setPositionAnchor();
      setOrientationAnchor();
      direct_press_start_z_ = target_pose_.translation().z();
      direct_descent_started_ = true;
      hybrid_.reset(direct_press_start_z_);
      publishMode();
      ROS_INFO("Grey+white: XY and orientation locked; base -Z press engaged");
    } else if (leaving_direct) {
      if (white) setPositionAnchor();
      if (grey) setOrientationAnchor();
      direct_descent_started_ = false;
      hybrid_.reset(target_pose_.translation().z());
      publishMode();
    }
    if (!white && white_pressed_) {
      hybrid_.reset(target_pose_.translation().z());
      publishMode();
    }

    grey_pressed_ = grey;
    white_pressed_ = white;
    if (!grey && !white) disengage("both Touch buttons released");
  }

  void setPositionAnchor() {
    if (!have_joint_state_) return;
    const Eigen::Isometry3d measured =
        latest_state_->getGlobalLinkTransform(tip_link_);
    robot_anchor_.translation() = measured.translation();
    target_pose_.translation() = measured.translation();
    touch_anchor_ = latest_touch_.translation();
  }

  void setOrientationAnchor() {
    if (!have_joint_state_) return;
    const Eigen::Isometry3d measured =
        latest_state_->getGlobalLinkTransform(tip_link_);
    target_pose_.linear() = measured.linear();
    robot_orientation_anchor_ = measured.linear();
    touch_orientation_anchor_ = latest_touch_.linear();
  }

  void engage() {
    const ros::WallTime now = ros::WallTime::now();
    if (!motion_enabled_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage: RM75 motion is disarmed");
      return;
    }
    if (!released_since_stop_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage: release both Touch buttons after a stop");
      return;
    }
    if (!force_frame_verified_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage: force_frame_verified is false");
      return;
    }
    if (!coordinateReady(now)) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage: force coordinate is stale or mismatched");
      return;
    }
    if (!have_touch_ || !have_joint_state_ || !have_force_ ||
        (now - last_touch_time_).toSec() > input_timeout_ ||
        (now - last_joint_time_).toSec() > input_timeout_ ||
        (now - last_force_time_).toSec() > force_feedback_timeout_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage: Touch, joints, or force feedback is missing/stale");
      return;
    }
    if (forceLimitExceeded() || !latest_state_->satisfiesBounds(joint_group_)) {
      ROS_WARN("Cannot engage: force or joint state is outside its safety limit");
      return;
    }

    robot_anchor_ = latest_state_->getGlobalLinkTransform(tip_link_);
    target_pose_ = robot_anchor_;
    touch_anchor_ = latest_touch_.translation();
    touch_orientation_anchor_ = latest_touch_.linear();
    robot_orientation_anchor_ = robot_anchor_.linear();
    *command_seed_ = *latest_state_;
    latest_state_->copyJointGroupPositions(joint_group_, last_command_);
    have_command_seed_ = true;
    hybrid_.reset(robot_anchor_.translation().z());
    direct_press_start_z_ = robot_anchor_.translation().z();
    direct_descent_started_ = false;
    last_update_time_ = now;
    stop_latched_ = false;
    active_ = true;
    std_msgs::String empty_reason;
    reason_pub_.publish(empty_reason);
    publishActive(true);
    publishMode();
    ROS_INFO("Vertical-press control engaged at current RM75 pose");
  }

  void update(const ros::WallTimerEvent&) {
    if (!active_) return;
    const ros::WallTime now = ros::WallTime::now();
    const double elapsed = (now - last_update_time_).toSec();
    last_update_time_ = now;
    const double dt = std::min(1.0 / publish_rate_, std::max(0.0, elapsed));
    const bool drill = drill_active_ || drill_holding_;
    const bool direct = !drill && grey_pressed_ && white_pressed_;
    const bool force_feedback_needed = !drill || drill_force_control_active_;
    if ((!drill && ((now - last_touch_time_).toSec() > input_timeout_ ||
                    (now - last_joy_time_).toSec() > input_timeout_)) ||
        (now - last_joint_time_).toSec() > input_timeout_ ||
        (force_feedback_needed && (!have_force_ ||
         (now - last_force_time_).toSec() > force_feedback_timeout_))) {
      safetyStop("Touch, joint, increment, button, or force feedback timed out");
      return;
    }
    if ((force_feedback_needed && !coordinateReady(now)) ||
        (have_force_ && forceLimitExceeded())) {
      safetyStop("force coordinate or wrench safety check failed");
      return;
    }
    std::vector<double> actual_joints;
    latest_state_->copyJointGroupPositions(joint_group_, actual_joints);
    for (std::size_t index = 0; index < actual_joints.size(); ++index) {
      if (std::abs(actual_joints[index] - last_command_[index]) >
          max_joint_tracking_error_) {
        ROS_ERROR("Joint tracking error: joint=%zu actual=%.6f command=%.6f diff=%.6f limit=%.6f",
                  index + 1, actual_joints[index], last_command_[index],
                  actual_joints[index] - last_command_[index],
                  max_joint_tracking_error_);
        safetyStop("joint tracking error exceeded");
        return;
      }
    }

    const Eigen::Isometry3d previous_target = target_pose_;
    const double measured_z =
        latest_state_->getGlobalLinkTransform(tip_link_).translation().z();
    if (drill) {
      drill_actual_depth_ = std::max(0.0, drill_start_z_ - measured_z);
      publishDrillDepth();
      if (drill_actual_depth_ >= drill_target_depth_) {
        drill_active_ = false;
        drill_holding_ = true;
        hybrid_.reset(measured_z);
        publishMode();
        publishDrillState("DEPTH_REACHED");
      }
      if (drill_holding_) {
        publishMeasuredHold();
        return;
      }
      if (drill_force_control_active_) {
        if (hybrid_.mode != rm75_touch_teleop::ControlMode::CONTACT_MODE) {
          safetyStop("Tool5 drill force loop left force-feed mode");
          return;
        }
        target_pose_.translation().z() = hybrid_.nextZ(measured_z, dt);
      } else {
        // Tool5 has its own reviewed Z velocity supplied as max_z_velocity.
        // Do not reuse the much faster manual dual-button direct-press speed.
        target_pose_.translation().z() -=
            rm75_vertical_press::boundedTool5FeedStep(
                hybrid_.cfg.max_z_step, hybrid_.cfg.max_z_velocity, dt);
      }
      const double proposed_depth =
          drill_start_z_ - target_pose_.translation().z();
      if (proposed_depth >= drill_target_depth_) {
        target_pose_.translation().z() = drill_start_z_ - drill_target_depth_;
      }
      if (proposed_depth > tool5_drill_max_feed_travel_ ||
          drill_actual_depth_ > tool5_drill_max_feed_travel_ ||
          hybrid_.travelExceeded(target_pose_.translation().z(), measured_z)) {
        safetyStop("Tool5 drill feed travel limit exceeded");
        return;
      }
    } else if (direct) {
      // X/Y and orientation were captured on dual-button entry. Only Z changes.
      if (hybrid_.transition(measured_z, dt)) publishMode();
      if (hybrid_.mode == rm75_touch_teleop::ControlMode::POSITION_MODE) {
        const double proposed = target_pose_.translation().z() -
                                direct_press_speed_ * dt;
        target_pose_.translation().z() =
            std::max(proposed, measured_z - hybrid_.cfg.max_z_tracking_error);
        if (direct_press_start_z_ - target_pose_.translation().z() >
            direct_press_max_travel_) {
          safetyStop("direct-press free-space travel limit exceeded");
          return;
        }
      } else if (hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
        target_pose_.translation().z() = hybrid_.nextZ(measured_z, dt);
        if (hybrid_.travelExceeded(target_pose_.translation().z(), measured_z)) {
          safetyStop("force-control Z travel exceeded");
          return;
        }
      } else {
        return;
      }
    } else if (white_pressed_) {
      const Eigen::Vector3d touch_delta =
          latest_touch_.translation() - touch_anchor_;
      Eigen::Vector3d robot_delta = rm75_touch_teleop::mapTranslation(
          touch_delta, metres_per_touch_unit_ * position_scale_);
      if (hybrid_.transition(measured_z, dt)) {
        if (hybrid_.mode == rm75_touch_teleop::ControlMode::POSITION_MODE) {
          robot_anchor_.translation().z() = measured_z;
          touch_anchor_.y() = latest_touch_.translation().y();
          robot_delta.z() = 0.0;
        }
        publishMode();
      }
      Eigen::Vector3d checked_delta = robot_delta;
      if (hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
        checked_delta.z() = 0.0;
      }
      if (!robot_delta.allFinite() || checked_delta.norm() > max_touch_translation_) {
        ROS_WARN_THROTTLE(0.5, "Touch displacement exceeds safety limit; command withheld");
        return;
      }
      target_pose_.translation() = robot_anchor_.translation() + robot_delta;
      if (hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
        target_pose_.translation().z() = hybrid_.nextZ(measured_z, dt);
        if (hybrid_.travelExceeded(target_pose_.translation().z(), measured_z)) {
          safetyStop("force-control Z travel exceeded");
          return;
        }
      }
    } else if (grey_pressed_) {
      const Eigen::Matrix3d touch_delta_rotation =
          latest_touch_.linear() * touch_orientation_anchor_.transpose();
      target_pose_.linear() = rm75_touch_teleop::mapRelativeRotation(
                                  touch_delta_rotation) *
                              robot_orientation_anchor_;
    }

    moveit::core::RobotState candidate(*command_seed_);
    if (!solveIKWithJ2LimitAvoidance(target_pose_, candidate)) {
      target_pose_ = previous_target;
      ROS_WARN_THROTTLE(0.5, "RM75 IK failed or violated joint bounds; command withheld");
      return;
    }
    candidate.harmonizePositions(joint_group_);
    std::vector<double> desired;
    candidate.copyJointGroupPositions(joint_group_, desired);
    if (desired.size() != 7 || last_command_.size() != 7) return;
    const double max_step = max_joint_velocity_ / publish_rate_;
    double fraction = 1.0;
    for (std::size_t index = 0; index < desired.size(); ++index) {
      const double delta = std::abs(desired[index] - last_command_[index]);
      if (delta > max_step) fraction = std::min(fraction, max_step / delta);
    }
    for (std::size_t index = 0; index < desired.size(); ++index) {
      if (!std::isfinite(desired[index])) {
        safetyStop("non-finite IK solution");
        return;
      }
      desired[index] = last_command_[index] +
          fraction * (desired[index] - last_command_[index]);
    }

    const bool bounded_drill_position =
        drill_active_ && !drill_force_control_active_;
    if (((white_pressed_ || drill_active_) &&
         hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) ||
        bounded_drill_position) {
      const double old_z =
          command_seed_->getGlobalLinkTransform(tip_link_).translation().z();
      const double z_bound = bounded_drill_position ?
          rm75_vertical_press::boundedTool5FeedStep(
              hybrid_.cfg.max_z_step, hybrid_.cfg.max_z_velocity, dt) :
          std::min(hybrid_.cfg.max_z_step, hybrid_.cfg.max_z_velocity * dt);
      bool accepted = false;
      for (int attempt = 0; attempt < 16; ++attempt) {
        candidate.setJointGroupPositions(joint_group_, desired);
        candidate.update();
        const double new_z =
            candidate.getGlobalLinkTransform(tip_link_).translation().z();
        if (std::abs(new_z - old_z) <= z_bound &&
            std::abs(new_z - measured_z) <= hybrid_.cfg.max_z_tracking_error &&
            !hybrid_.travelExceeded(new_z, measured_z)) {
          accepted = true;
          break;
        }
        for (std::size_t index = 0; index < desired.size(); ++index) {
          desired[index] = last_command_[index] +
              0.5 * (desired[index] - last_command_[index]);
        }
      }
      if (!accepted) {
        safetyStop("cannot satisfy Cartesian Z command limits");
        return;
      }
    }

    command_seed_->setJointGroupPositions(joint_group_, desired);
    // Keep continuous joints in the same unwrapped revolution as the real
    // controller.  enforceBounds() folds (for example) -5.66 rad to +0.62 rad;
    // those angles are geometrically equal but the RM driver and tracking
    // watchdog correctly treat the numeric 2*pi jump as a new command.
    command_seed_->copyJointGroupPositions(joint_group_, desired);
    command_seed_->update();
    last_command_ = desired;
    if ((white_pressed_ || (drill_active_ && drill_force_control_active_)) &&
        hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
      hybrid_.commit(
          command_seed_->getGlobalLinkTransform(tip_link_).translation().z(), dt);
    }

    rm_msgs::JointPos command;
    command.joint.resize(desired.size());
    for (std::size_t index = 0; index < desired.size(); ++index) {
      command.joint[index] = static_cast<float>(desired[index]);
    }
    command.expand = 0.0F;
    command_pub_.publish(command);

    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = base_frame_;
    target.pose = eigenToPose(target_pose_);
    target_pub_.publish(target);
  }

  void publishMeasuredHold() {
    std::vector<double> actual;
    latest_state_->copyJointGroupPositions(joint_group_, actual);
    if (actual.size() != 7) {
      safetyStop("Tool5 hold failed: incomplete joint feedback");
      return;
    }
    for (double value : actual) {
      if (!std::isfinite(value)) {
        safetyStop("Tool5 hold failed: non-finite joint feedback");
        return;
      }
    }
    *command_seed_ = *latest_state_;
    target_pose_ = latest_state_->getGlobalLinkTransform(tip_link_);
    last_command_ = actual;
    rm_msgs::JointPos command;
    command.joint.resize(actual.size());
    for (std::size_t index = 0; index < actual.size(); ++index) {
      command.joint[index] = static_cast<float>(actual[index]);
    }
    command.expand = 0.0F;
    command_pub_.publish(command);
  }

  double j2LimitMargin(const moveit::core::RobotState& state) const {
    const moveit::core::VariableBounds& bounds =
        robot_model_->getVariableBounds(joint_names_[1]);
    if (!bounds.position_bounded_) return std::numeric_limits<double>::infinity();
    const double position = state.getVariablePosition(joint_names_[1]);
    return std::min(position - bounds.min_position_,
                    bounds.max_position_ - position);
  }

  bool solveIKWithJ2LimitAvoidance(
      const Eigen::Isometry3d& target,
      moveit::core::RobotState& result) {
    moveit::core::RobotState primary(*command_seed_);
    const bool primary_valid =
        primary.setFromIK(joint_group_, target, tip_link_, ik_timeout_) &&
        primary.satisfiesBounds(joint_group_);
    if (!j2_limit_avoidance_enabled_) {
      if (primary_valid) result = primary;
      return primary_valid;
    }

    const moveit::core::VariableBounds& bounds =
        robot_model_->getVariableBounds(joint_names_[1]);
    if (!bounds.position_bounded_) {
      if (primary_valid) result = primary;
      return primary_valid;
    }

    double best_margin = -std::numeric_limits<double>::infinity();
    if (primary_valid) {
      result = primary;
      best_margin = j2LimitMargin(primary);
      if (best_margin >= j2_limit_avoidance_margin_) return true;
    }

    std::vector<double> base_positions;
    command_seed_->copyJointGroupPositions(joint_group_, base_positions);
    if (base_positions.size() != joint_names_.size()) return primary_valid;
    const double midpoint =
        0.5 * (bounds.min_position_ + bounds.max_position_);
    const double direction = base_positions[1] <= midpoint ? 1.0 : -1.0;
    bool used_reseeded_solution = false;

    for (int attempt = 1; attempt <= j2_reseed_attempts_; ++attempt) {
      std::vector<double> seed_positions = base_positions;
      seed_positions[1] = std::max(
          bounds.min_position_ + 1e-6,
          std::min(bounds.max_position_ - 1e-6,
                   base_positions[1] + direction * j2_reseed_step_ * attempt));
      moveit::core::RobotState alternative(*command_seed_);
      alternative.setJointGroupPositions(joint_group_, seed_positions);
      alternative.update();
      if (!alternative.setFromIK(joint_group_, target, tip_link_,
                                 j2_reseed_timeout_) ||
          !alternative.satisfiesBounds(joint_group_)) {
        continue;
      }

      std::vector<double> alternative_positions;
      alternative.copyJointGroupPositions(joint_group_, alternative_positions);
      double max_delta = 0.0;
      for (std::size_t index = 0; index < alternative_positions.size(); ++index) {
        max_delta = std::max(
            max_delta,
            std::abs(alternative_positions[index] - base_positions[index]));
      }
      if (max_delta > j2_reseed_max_joint_delta_) continue;

      const double margin = j2LimitMargin(alternative);
      if (margin > best_margin + 1e-4) {
        result = alternative;
        best_margin = margin;
        used_reseeded_solution = true;
      }
    }

    if (used_reseeded_solution) {
      ROS_INFO_THROTTLE(
          1.0,
          "J2 limit avoidance selected a continuous IK branch (margin %.1f deg)",
          best_margin * 180.0 / 3.14159265358979323846);
    }
    return primary_valid || used_reseeded_solution;
  }

  bool forceLimitExceeded() const {
    return force_norm_ > max_external_force_ ||
           torque_norm_ > max_external_torque_ ||
           std::abs(hybrid_.raw) > hybrid_.cfg.force_limit;
  }

  bool coordinateReady(const ros::WallTime& now) const {
    return coordinate_ == expected_coordinate_ &&
        (now - last_coordinate_time_).toSec() <= force_feedback_timeout_;
  }

  void disengage(const std::string& reason) {
    if (!active_) return;
    stop_pub_.publish(std_msgs::Empty());
    active_ = false;
    hybrid_.reset(target_pose_.translation().z());
    publishActive(false);
    publishMode();
    if (drill_active_ || drill_holding_) publishDrillState("IDLE");
    drill_active_ = drill_holding_ = drill_force_control_active_ = false;
    ROS_INFO_STREAM("Vertical-press control disengaged: " << reason);
  }

  void safetyStop(const std::string& reason) {
    if (stop_latched_) return;
    stop_latched_ = true;
    if (active_ && motion_enabled_) stop_pub_.publish(std_msgs::Empty());
    active_ = false;
    released_since_stop_ = false;
    hybrid_.stop();
    publishActive(false);
    publishMode();
    std_msgs::String message;
    message.data = reason;
    reason_pub_.publish(message);
    if (tool5_drill_enabled_) publishDrillState("SAFE_STOP");
    drill_active_ = drill_holding_ = drill_force_control_active_ = false;
    ROS_ERROR_STREAM("RM75 VERTICAL PRESS SAFE_STOP: " << reason);
  }

  void publishActive(bool value) {
    std_msgs::Bool message;
    message.data = value;
    active_pub_.publish(message);
  }

  void publishMode() {
    std_msgs::String message;
    message.data = rm75_touch_teleop::modeName(hybrid_.mode);
    mode_pub_.publish(message);
  }

  void publishDrillState(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    drill_state_pub_.publish(message);
  }

  void publishDrillDepth() {
    std_msgs::Float64 message;
    message.data = drill_actual_depth_;
    drill_depth_pub_.publish(message);
  }

  ros::NodeHandle nh_, private_nh_;
  robot_model_loader::RobotModelLoader model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  std::unique_ptr<moveit::core::RobotState> latest_state_, command_seed_;
  rm75_touch_teleop::HybridForceControl hybrid_;

  ros::Subscriber touch_pose_sub_, joy_sub_, joint_state_sub_;
  ros::Subscriber force_sub_, coordinate_sub_, drill_start_sub_, drill_stop_sub_;
  ros::Publisher command_pub_, stop_pub_, target_pub_, active_pub_, mode_pub_;
  ros::Publisher reason_pub_, raw_force_pub_, force_pub_, force_target_pub_;
  ros::Publisher drill_state_pub_, drill_depth_pub_;
  ros::WallTimer timer_;

  std::string planning_group_, tip_link_, base_frame_, touch_pose_topic_;
  std::string touch_joy_topic_, joint_state_topic_, command_topic_, force_topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> last_command_;
  Eigen::Isometry3d latest_touch_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d robot_anchor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d touch_anchor_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d touch_orientation_anchor_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d robot_orientation_anchor_ = Eigen::Matrix3d::Identity();

  ros::WallTime last_touch_time_, last_joint_time_;
  ros::WallTime last_force_time_, last_coordinate_time_, last_joy_time_;
  ros::WallTime last_update_time_;
  bool motion_enabled_ = false, force_frame_verified_ = false;
  bool have_touch_ = false, have_joint_state_ = false;
  bool have_force_ = false, have_command_seed_ = false;
  bool grey_pressed_ = false, white_pressed_ = false, active_ = false;
  bool released_since_stop_ = true, stop_latched_ = false;
  bool direct_descent_started_ = false;
  bool tool5_drill_enabled_ = false, tool5_require_force_feedback_ = false;
  bool drill_active_ = false, drill_holding_ = false;
  bool drill_force_control_active_ = false;
  int coordinate_ = -1, expected_coordinate_ = 1;
  double metres_per_touch_unit_ = 0.001, position_scale_ = 0.4;
  double publish_rate_ = 50.0, ik_timeout_ = 0.015, input_timeout_ = 0.25;
  double max_touch_translation_ = 0.08, max_joint_velocity_ = 0.20;
  double max_joint_tracking_error_ = 0.15;
  bool j2_limit_avoidance_enabled_ = true;
  double j2_limit_avoidance_margin_ = 0.25;
  double j2_reseed_step_ = 0.05;
  double j2_reseed_timeout_ = 0.002;
  int j2_reseed_attempts_ = 4;
  double j2_reseed_max_joint_delta_ = 0.35;
  double max_external_force_ = 100.0, max_external_torque_ = 5.0;
  double force_feedback_timeout_ = 0.25, force_sign_ = -1.0;
  double direct_press_speed_ = 0.003, direct_press_max_travel_ = 0.05;
  double vertical_alignment_tolerance_ = 0.0523598776;
  double direct_press_start_z_ = 0.0, force_norm_ = 0.0, torque_norm_ = 0.0;
  double tool5_drill_max_feed_travel_ = 0.05;
  double drill_start_z_ = 0.0, drill_target_depth_ = 0.0;
  double drill_actual_depth_ = 0.0;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_vertical_press");
  try {
    VerticalPressController controller;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("RM75 vertical-press controller failed to start: " << error.what());
    return 1;
  }
  return 0;
}
