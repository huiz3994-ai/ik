#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rm_msgs/JointPos.h>
#include <rm_msgs/Six_Force.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "rm75_touch_teleop/mapping.h"
#include "rm75_touch_teleop/hybrid_force_control.h"
#include "rm75_touch_teleop/haptic_renderer.h"
#include "rm75_touch_teleop/constrained_differential_ik.h"

namespace {

bool finitePose(const geometry_msgs::Pose& pose) {
  const double values[] = {pose.position.x, pose.position.y, pose.position.z,
                           pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w};
  for (double value : values) {
    if (!std::isfinite(value)) return false;
  }
  const double q_norm = std::sqrt(
      pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w);
  return q_norm > 1e-6;
}

Eigen::Isometry3d poseToEigen(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  result.linear() = q.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::Pose eigenToPose(const Eigen::Isometry3d& transform) {
  geometry_msgs::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

class Rm75TouchTeleop {
 public:
  Rm75TouchTeleop()
      : nh_(), private_nh_("~"), model_loader_("robot_description"),
        startup_wall_time_(ros::WallTime::now()) {
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
                                   "/arm/arm_joint_controller/command");
    private_nh_.param<std::string>("output_mode", output_mode_, "trajectory");
    private_nh_.param("motion_enabled", motion_enabled_, true);
    private_nh_.param("metres_per_touch_unit", metres_per_touch_unit_, 0.001);
    private_nh_.param("position_scale", position_scale_, 1.0);
    private_nh_.param("translation_x_enabled", translation_x_enabled_, true);
    private_nh_.param("translation_y_enabled", translation_y_enabled_, true);
    private_nh_.param("translation_z_enabled", translation_z_enabled_, true);
    private_nh_.param("orientation_enabled", orientation_enabled_, true);
    private_nh_.param("max_z_position_travel", max_z_position_travel_, 0.0);
    private_nh_.param("use_touch_increment", use_touch_increment_, true);
    private_nh_.param("publish_rate", publish_rate_, 100.0);
    private_nh_.param("command_duration", command_duration_, 0.08);
    private_nh_.param("max_touch_translation", max_touch_translation_, 0.25);
    private_nh_.param("max_joint_velocity", max_joint_velocity_, 0.8);
    private_nh_.param("input_timeout", input_timeout_, 0.15);
    private_nh_.param<std::string>("force_topic", force_topic_,
                                   "/rm_driver/UdpSixZeroForce");
    private_nh_.param("force_safety_enabled", force_safety_enabled_, false);
    private_nh_.param("require_force_feedback", require_force_feedback_, false);
    private_nh_.param("force_feedback_timeout", force_feedback_timeout_, 0.25);
    private_nh_.param("force_startup_grace_period",
                      force_startup_grace_period_, 0.0);
    private_nh_.param("max_external_force", max_external_force_, 35.0);
    private_nh_.param("max_external_torque", max_external_torque_, 5.0);

    private_nh_.param("haptic_feedback_enabled", haptic_enabled_, false);
    private_nh_.param("haptic_only_in_contact", haptic_only_in_contact_, true);
    private_nh_.param("haptic_require_teleop_active", haptic_require_active_, true);
    private_nh_.param("haptic_output_x_enabled", haptic_output_x_enabled_, true);
    private_nh_.param("haptic_output_y_enabled", haptic_output_y_enabled_, true);
    private_nh_.param("haptic_output_z_enabled", haptic_output_z_enabled_, true);
    private_nh_.param<std::string>("touch_twist_topic", touch_twist_topic_,
                                   "/Geomagic/twist");
    private_nh_.param("haptic_force_timeout", haptic_force_timeout_, 0.1);
    auto& h = haptic_.cfg;
    private_nh_.param("haptic_force_scale", h.force_scale, h.force_scale);
    private_nh_.param("haptic_force_filter_tau", h.filter_tau, h.filter_tau);
    private_nh_.param("haptic_force_deadband", h.deadband, h.deadband);
    private_nh_.param("haptic_damping_x", h.damping.x(), h.damping.x());
    private_nh_.param("haptic_damping_y", h.damping.y(), h.damping.y());
    private_nh_.param("haptic_damping_z", h.damping.z(), h.damping.z());
    private_nh_.param("haptic_max_force_x", h.max_force.x(), h.max_force.x());
    private_nh_.param("haptic_max_force_y", h.max_force.y(), h.max_force.y());
    private_nh_.param("haptic_max_force_z", h.max_force.z(), h.max_force.z());
    private_nh_.param("haptic_max_force_norm", h.max_force_norm, h.max_force_norm);
    private_nh_.param("haptic_force_ramp_rate", h.ramp_rate, h.ramp_rate);
    private_nh_.param("haptic_max_dt", h.max_dt, h.max_dt);
    private_nh_.param("haptic_position_stiffness_x", h.position_stiffness.x(),
                      h.position_stiffness.x());
    private_nh_.param("haptic_position_stiffness_y", h.position_stiffness.y(),
                      h.position_stiffness.y());
    private_nh_.param("haptic_position_stiffness_z", h.position_stiffness.z(),
                      h.position_stiffness.z());
    private_nh_.param("haptic_position_deadband", h.position_deadband,
                      h.position_deadband);
    private_nh_.param("haptic_max_position_error", h.max_position_error,
                      h.max_position_error);
    std::vector<double> haptic_sign, haptic_rpy;
    if (!private_nh_.getParam("haptic_force_sign", haptic_sign)) haptic_sign = {1, 1, 1};
    if (!private_nh_.getParam("haptic_force_rotation_rpy", haptic_rpy)) haptic_rpy = {0, 0, 0};
    if (haptic_sign.size() != 3 || haptic_rpy.size() != 3)
      throw std::runtime_error("haptic force sign/rotation parameters require three elements");
    h.force_sign = Eigen::Vector3d(haptic_sign[0], haptic_sign[1], haptic_sign[2]);
    h.robot_force_rotation =
        (Eigen::AngleAxisd(haptic_rpy[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(haptic_rpy[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(haptic_rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    h.validate();
    if (!std::isfinite(haptic_force_timeout_) || haptic_force_timeout_ <= 0.0)
      throw std::runtime_error("haptic_force_timeout must be positive");

    private_nh_.param("hybrid_control_enabled", hybrid_enabled_, false);
    private_nh_.param("force_frame_verified", force_frame_verified_, false);
    private_nh_.param<std::string>("force_frame", force_frame_, "base_link");
    private_nh_.param("force_sign", force_sign_, 1.0);
    private_nh_.param("expected_force_coordinate", expected_coordinate_, -1);
    private_nh_.param("require_touch_increment", require_increment_, false);
    private_nh_.param("max_joint_tracking_error", max_joint_tracking_error_, 0.15);
    private_nh_.param("max_joint_acceleration", max_joint_acceleration_, 1.5);
    private_nh_.param("min_control_dt", min_control_dt_, 0.001);
    private_nh_.param("max_control_dt", max_control_dt_, 0.04);
    private_nh_.param("posture_reference_update_rate",
                      posture_reference_update_rate_, 0.30);
    private_nh_.param("qp_translation_gain", qp_config_.translation_gain,
                      qp_config_.translation_gain);
    private_nh_.param("qp_rotation_gain", qp_config_.rotation_gain,
                      qp_config_.rotation_gain);
    private_nh_.param("max_ee_linear_velocity", qp_config_.max_linear_velocity,
                      qp_config_.max_linear_velocity);
    private_nh_.param("max_ee_angular_velocity", qp_config_.max_angular_velocity,
                      qp_config_.max_angular_velocity);
    private_nh_.param("qp_translation_weight", qp_config_.translation_weight,
                      qp_config_.translation_weight);
    private_nh_.param("qp_rotation_weight", qp_config_.rotation_weight,
                      qp_config_.rotation_weight);
    private_nh_.param("qp_regularization", qp_config_.regularization,
                      qp_config_.regularization);
    private_nh_.param("qp_velocity_smoothing_weight",
                      qp_config_.velocity_smoothing_weight,
                      qp_config_.velocity_smoothing_weight);
    private_nh_.param("qp_posture_weight", qp_config_.posture_weight,
                      qp_config_.posture_weight);
    private_nh_.param("qp_posture_gain", qp_config_.posture_gain,
                      qp_config_.posture_gain);
    private_nh_.param("qp_soft_limit_gain", qp_config_.soft_limit_gain,
                      qp_config_.soft_limit_gain);
    private_nh_.param("qp_soft_limit_margin", qp_config_.soft_limit_margin,
                      qp_config_.soft_limit_margin);
    private_nh_.param("qp_singular_value_threshold",
                      qp_config_.singular_value_threshold,
                      qp_config_.singular_value_threshold);
    private_nh_.param("qp_singular_regularization",
                      qp_config_.singular_regularization,
                      qp_config_.singular_regularization);
    private_nh_.param("qp_max_iterations", qp_config_.solver.max_iterations,
                      qp_config_.solver.max_iterations);
    private_nh_.param("qp_timeout", qp_config_.solver.time_limit_sec,
                      qp_config_.solver.time_limit_sec);
    auto& c = hybrid_.cfg;
    private_nh_.param("desired_force", c.desired_force, c.desired_force);
    private_nh_.param("contact_threshold", c.contact_threshold, c.contact_threshold);
    private_nh_.param("release_threshold", c.release_threshold, c.release_threshold);
    private_nh_.param("force_gain", c.force_gain, c.force_gain);
    private_nh_.param("force_limit", c.force_limit, c.force_limit);
    private_nh_.param("max_z_velocity", c.max_z_velocity, c.max_z_velocity);
    private_nh_.param("max_z_step", c.max_z_step, c.max_z_step);
    private_nh_.param("max_force_travel", c.max_force_travel, c.max_force_travel);
    private_nh_.param("max_z_tracking_error", c.max_z_tracking_error, c.max_z_tracking_error);
    private_nh_.param("force_deadband", c.force_deadband, c.force_deadband);
    private_nh_.param("force_filter_tau", c.filter_tau, c.filter_tau);
    private_nh_.param("contact_debounce", c.transition_time, c.transition_time);
    private_nh_.param("force_ramp_rate", c.force_ramp_rate, c.force_ramp_rate);
    private_nh_.param("force_motion_sign", c.motion_sign, c.motion_sign);
    if (hybrid_enabled_) {
      if (!translation_z_enabled_)
        throw std::runtime_error("hybrid force control requires translation_z_enabled=true");
      c.validate();
      force_safety_enabled_ = require_force_feedback_ = true;
      if (force_sign_ != 1.0 && force_sign_ != -1.0)
        throw std::runtime_error("force_sign must be +1 or -1");
      if (max_external_force_ <= std::abs(c.desired_force))
        throw std::runtime_error("max_external_force must exceed abs(desired_force)");
      if (expected_coordinate_ < -1 || expected_coordinate_ > 2 ||
          (output_mode_ == "rm_joint_pos" && expected_coordinate_ < 0))
        throw std::runtime_error("Hardware sampling requires expected_force_coordinate in [0,2]");
    }
    const double positive[] = {publish_rate_, command_duration_, input_timeout_,
        max_touch_translation_, max_joint_velocity_, force_feedback_timeout_,
        max_external_force_, max_external_torque_, max_joint_tracking_error_,
        max_joint_acceleration_, min_control_dt_, max_control_dt_};
    for (double v : positive)
      if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid positive teleop parameter");
    if (!std::isfinite(position_scale_) || position_scale_ <= 0 ||
        !std::isfinite(metres_per_touch_unit_) || metres_per_touch_unit_ <= 0)
      throw std::runtime_error("Invalid Touch scale");
    if (!std::isfinite(force_startup_grace_period_) ||
        force_startup_grace_period_ < 0.0)
      throw std::runtime_error("force_startup_grace_period must be nonnegative");
    if (!std::isfinite(max_z_position_travel_) || max_z_position_travel_ < 0.0)
      throw std::runtime_error("max_z_position_travel must be nonnegative");
    if (min_control_dt_ > max_control_dt_ ||
        !std::isfinite(posture_reference_update_rate_) ||
        posture_reference_update_rate_ <= 0.0)
      throw std::runtime_error("Invalid control timing or posture reference parameters");
    std::vector<double> rpy;
    if (!private_nh_.getParam("force_frame_rotation_rpy", rpy)) rpy = {0, 0, 0};
    if (rpy.size() != 3 || !std::isfinite(rpy[0]) || !std::isfinite(rpy[1]) || !std::isfinite(rpy[2]))
      throw std::runtime_error("force_frame_rotation_rpy must contain three finite radians");
    force_rotation_ = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();

    robot_model_ = model_loader_.getModel();
    if (!robot_model_) {
      throw std::runtime_error("robot_description could not be loaded");
    }
    joint_group_ = robot_model_->getJointModelGroup(planning_group_);
    if (!joint_group_) {
      throw std::runtime_error("MoveIt group '" + planning_group_ + "' does not exist");
    }
    if (!robot_model_->hasLinkModel(tip_link_)) {
      throw std::runtime_error("tip link '" + tip_link_ + "' does not exist");
    }
    if ((hybrid_enabled_ || haptic_enabled_) && (!robot_model_->hasLinkModel(base_frame_) ||
        !robot_model_->hasLinkModel(force_frame_)))
      throw std::runtime_error("Force-processing base_frame and force_frame must be robot model links");
    joint_names_ = joint_group_->getVariableNames();
    if (joint_names_.size() != 7) {
      throw std::runtime_error("RM75 teleop requires exactly 7 arm joints; group '" +
                               planning_group_ + "' has " +
                               std::to_string(joint_names_.size()));
    }
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const moveit::core::VariableBounds& bounds =
          robot_model_->getVariableBounds(joint_names_[i]);
      if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) ||
          !std::isfinite(bounds.max_position_))
        throw std::runtime_error("All RM75 joints require finite position bounds");
      position_lower_[i] = bounds.min_position_;
      position_upper_[i] = bounds.max_position_;
      double limit = max_joint_velocity_;
      if (bounds.velocity_bounded_)
        limit = std::min(limit, std::max(std::abs(bounds.min_velocity_),
                                         std::abs(bounds.max_velocity_)));
      if (!std::isfinite(limit) || limit <= 0.0)
        throw std::runtime_error("All RM75 joints require positive velocity limits");
      velocity_limit_[i] = limit;
      acceleration_limit_[i] = max_joint_acceleration_;
    }

    latest_state_.reset(new moveit::core::RobotState(robot_model_));
    latest_state_->setToDefaultValues();
    command_seed_.reset(new moveit::core::RobotState(robot_model_));
    command_seed_->setToDefaultValues();
    if (hybrid_enabled_ || haptic_enabled_) {
      const auto* link = robot_model_->getLinkModel(base_frame_);
      while (link && link->getParentLinkModel()) {
        if (link->getParentJointModel()->getType() != moveit::core::JointModel::FIXED)
          throw std::runtime_error("Sampling base must be fixed to the model frame");
        link = link->getParentLinkModel();
      }
      if (!latest_state_->getGlobalLinkTransform(base_frame_).matrix().isIdentity(1e-9))
        throw std::runtime_error("Sampling base axes/origin must coincide with the model frame");
    }

    touch_pose_sub_ = nh_.subscribe(touch_pose_topic_, 1,
                                    &Rm75TouchTeleop::touchPoseCallback, this);
    touch_twist_sub_ = nh_.subscribe(touch_twist_topic_, 1,
                                    &Rm75TouchTeleop::touchTwistCallback, this);
    joy_sub_ = nh_.subscribe(touch_joy_topic_, 1,
                             &Rm75TouchTeleop::joyCallback, this);
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1,
                                    &Rm75TouchTeleop::jointStateCallback, this);
    touch_increment_sub_ = nh_.subscribe("/Touch_Pose_increment", 1,
                                        &Rm75TouchTeleop::touchIncrementCallback, this);
    touch_anchor_sub_ = nh_.subscribe("/Touch_Anchor_updated", 1,
                                     &Rm75TouchTeleop::touchAnchorCallback, this);
    if (force_safety_enabled_ || haptic_enabled_) {
      force_sub_ = nh_.subscribe(force_topic_, 1,
                                 &Rm75TouchTeleop::forceCallback, this);
    }
    if (output_mode_ == "trajectory") {
      command_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic_, 1);
    } else if (output_mode_ == "rm_joint_pos") {
      rm_joint_pos_pub_ = nh_.advertise<rm_msgs::JointPos>(command_topic_, 1);
    } else {
      throw std::runtime_error("unsupported output_mode '" + output_mode_ +
                               "' (expected trajectory or rm_joint_pos)");
    }
    if (hybrid_enabled_ && expected_coordinate_ >= 0)
      coordinate_sub_ = nh_.subscribe("/rm_driver/Udp_Coordinate", 1,
          &Rm75TouchTeleop::coordinateCallback, this);
    stop_pub_ = nh_.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1);
    mode_pub_ = private_nh_.advertise<std_msgs::String>("control_mode", 1, true);
    reason_pub_ = private_nh_.advertise<std_msgs::String>("stop_reason", 1, true);
    fz_pub_ = private_nh_.advertise<std_msgs::Float64>("force_z", 1);
    filtered_fz_pub_ = private_nh_.advertise<std_msgs::Float64>("filtered_force_z", 1);
    force_target_pub_ = private_nh_.advertise<std_msgs::Float64>("force_setpoint", 1);
    target_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
    active_pub_ = private_nh_.advertise<std_msgs::Bool>("active", 1, true);
    haptic_enabled_pub_ = private_nh_.advertise<std_msgs::Bool>("haptic_enabled", 1, true);
    haptic_raw_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_raw_force", 1);
    haptic_filtered_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_filtered_force", 1);
    haptic_command_diag_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_command", 1);
    haptic_position_force_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_position_force", 1);
    haptic_tracking_error_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_tracking_error", 1);
    haptic_fresh_pub_ = private_nh_.advertise<std_msgs::Bool>("haptic_force_fresh", 1, true);
    haptic_device_command_pub_ = private_nh_.advertise<geometry_msgs::Vector3Stamped>("haptic_device_command", 1);
    qp_status_pub_ = private_nh_.advertise<std_msgs::String>("qp_status", 1, true);
    qp_solve_time_pub_ = private_nh_.advertise<std_msgs::Float64>("qp_solve_time_ms", 1);
    control_dt_pub_ = private_nh_.advertise<std_msgs::Float64>("control_dt", 1);
    target_joint_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("target_joint", 1);
    actual_joint_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("actual_joint", 1);
    joint_velocity_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("joint_velocity", 1);
    joint_acceleration_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("joint_acceleration", 1);
    tracking_error_pub_ = private_nh_.advertise<std_msgs::Float64MultiArray>("joint_tracking_error", 1);
    timer_ = nh_.createSteadyTimer(
        ros::WallDuration(1.0 / std::max(1.0, publish_rate_)),
        &Rm75TouchTeleop::update, this);
    publishActive(false);
    publishMode();
    std_msgs::Bool haptic_enabled_msg; haptic_enabled_msg.data = haptic_enabled_;
    haptic_enabled_pub_.publish(haptic_enabled_msg);
    std_msgs::Bool haptic_fresh_msg; haptic_fresh_msg.data = false;
    haptic_fresh_pub_.publish(haptic_fresh_msg);

    ROS_INFO_STREAM("RM75 Touch teleop ready: " << joint_names_.size()
                    << " joints, pose=" << touch_pose_topic_
                    << ", output=" << output_mode_
                    << ", command=" << command_topic_);
    if (!motion_enabled_) {
      ROS_WARN("RM75 motion is DISARMED. Set ~motion_enabled:=true only after the real robot workspace is safe.");
    }
    if (force_safety_enabled_) {
      ROS_INFO_STREAM("Six-axis force safety enabled: topic=" << force_topic_
                      << ", force_limit=" << max_external_force_
                      << " N, torque_limit=" << max_external_torque_ << " Nm");
    }
    if (orientation_enabled_)
      ROS_INFO("Hold grey to translate; hold white independently to rotate.");
    else
      ROS_INFO("Orientation input is locked; the white Touch button is ignored.");
  }

  ~Rm75TouchTeleop() {
    if (!haptic_enabled_) return;
    geometry_msgs::Vector3Stamped zero;
    zero.header.stamp = ros::Time::now();
    zero.header.frame_id = "touch_base";
    for (int i = 0; i < 3; ++i) haptic_device_command_pub_.publish(zero);
  }

 private:
  void touchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!finitePose(msg->pose)) {
      ROS_WARN_THROTTLE(1.0, "Ignoring non-finite Touch pose");
      return;
    }
    latest_touch_ = poseToEigen(msg->pose);
    have_touch_ = true;
    last_touch_wall_time_ = ros::WallTime::now();
  }

  void touchTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    const Eigen::Vector3d velocity(msg->linear.x, msg->linear.y, msg->linear.z);
    if (!velocity.allFinite()) return;
    // geomagic_control converts its native mm/s estimate to m/s before
    // publishing /Geomagic/twist.
    latest_touch_velocity_mps_ = velocity;
    have_touch_velocity_ = true;
    last_touch_twist_wall_time_ = ros::WallTime::now();
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() != msg->position.size()) return;
    std::map<std::string, double> positions;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (std::isfinite(msg->position[i])) positions[msg->name[i]] = msg->position[i];
    }
    for (const std::string& name : joint_names_) {
      if (positions.count(name) == 0) return;
    }
    latest_state_->setVariablePositions(positions);
    latest_state_->update();
    have_joint_state_ = true;
    last_joint_wall_time_ = ros::WallTime::now();
    if (!have_command_seed_) {
      *command_seed_ = *latest_state_;
      latest_state_->copyJointGroupPositions(joint_group_, last_command_);
      have_command_seed_ = true;
    }
  }

  void touchIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 3) return;
    const Eigen::Vector3d value(msg->data[0], msg->data[1], msg->data[2]);
    if (!value.allFinite()) return;
    latest_touch_increment_ = value;
    have_touch_increment_ = true;
    last_increment_wall_time_ = ros::WallTime::now();
  }

  void touchAnchorCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data || !active_ || !grey_pressed_ || !have_joint_state_) return;
    const double saved_z = target_pose_.translation().z();
    const Eigen::Isometry3d measured =
        latest_state_->getGlobalLinkTransform(tip_link_);
    robot_anchor_.translation() = measured.translation();
    target_pose_.translation() = measured.translation();
    if (hybrid_enabled_ && hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE)
      target_pose_.translation().z() = saved_z;
    touch_anchor_.translation() = latest_touch_.translation();
    touch_increment_anchor_ = latest_touch_increment_;
    ROS_INFO("Grey-button position anchor synchronized with the current RM75 pose");
  }

  void forceCallback(const rm_msgs::Six_Force::ConstPtr& msg) {
    const double values[] = {msg->force_Fx, msg->force_Fy, msg->force_Fz,
                             msg->force_Mx, msg->force_My, msg->force_Mz};
    for (double value : values) {
      if (!std::isfinite(value)) {
        have_force_feedback_ = false;
        haptic_force_valid_ = false;
        if (((force_safety_enabled_ && active_) || hybrid_enabled_) &&
            (active_ || forceStartupGraceElapsed()))
          safetyStop("non-finite force feedback");
        ROS_WARN_THROTTLE(1.0, "Ignoring non-finite RM75 force feedback");
        return;
      }
    }
    latest_force_norm_ = std::sqrt(msg->force_Fx * msg->force_Fx +
                                   msg->force_Fy * msg->force_Fy +
                                   msg->force_Fz * msg->force_Fz);
    latest_torque_norm_ = std::sqrt(msg->force_Mx * msg->force_Mx +
                                    msg->force_My * msg->force_My +
                                    msg->force_Mz * msg->force_Mz);
    const ros::WallTime now = ros::WallTime::now();
    if (hybrid_enabled_ || haptic_enabled_) {
      if (!have_joint_state_ || (now - last_joint_wall_time_).toSec() > input_timeout_) {
        haptic_force_valid_ = false;
        if (hybrid_enabled_) {
          have_force_feedback_ = false;
          if (active_) safetyStop("joint feedback unavailable for wrench transform");
        }
        return;
      }
      const Eigen::Matrix3d rotation = latest_state_->getGlobalLinkTransform(base_frame_).linear().transpose() *
          latest_state_->getGlobalLinkTransform(force_frame_).linear() * force_rotation_;
      const Eigen::Vector3d base_force = rotation *
          Eigen::Vector3d(msg->force_Fx, msg->force_Fy, msg->force_Fz);
      if (haptic_enabled_) {
        latest_haptic_robot_force_ = base_force;
        haptic_force_valid_ = base_force.allFinite();
      }
      if (hybrid_enabled_) {
        const Eigen::Vector3d force = force_sign_ * base_force;
        const double dt = have_force_feedback_ ? (now - last_force_wall_time_).toSec() : 1.0;
        if (!have_force_feedback_ || dt > force_feedback_timeout_) hybrid_.have_force = false;
        hybrid_.observe(force.z(), dt);
        std_msgs::Float64 value;
        value.data = hybrid_.raw; fz_pub_.publish(value);
        value.data = hybrid_.filtered; filtered_fz_pub_.publish(value);
      }
    }
    have_force_feedback_ = true;
    last_force_wall_time_ = now;
    if (forceLimitExceeded() && (active_ || hybrid_enabled_) &&
        (active_ || forceStartupGraceElapsed()))
      safetyStop("six-axis force safety limit exceeded");
  }

  bool forceStartupGraceElapsed() const {
    return (ros::WallTime::now() - startup_wall_time_).toSec() >=
           force_startup_grace_period_;
  }

  bool forceLimitExceeded() const {
    return latest_force_norm_ > max_external_force_ ||
           latest_torque_norm_ > max_external_torque_ ||
           (hybrid_enabled_ && std::abs(hybrid_.raw) > hybrid_.cfg.force_limit);
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    if (msg->buttons.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "Touch Joy message has fewer than two buttons");
      return;
    }
    last_joy_wall_time_ = ros::WallTime::now();
    const bool grey = msg->buttons[0] != 0;
    const bool white = orientation_enabled_ && msg->buttons[1] != 0;
    const bool grey_rising = grey && !grey_pressed_;
    const bool white_rising = white && !white_pressed_;

    if (!grey) released_since_stop_ = true;
    const bool was_active = active_;
    if (!active_ && (grey_rising || white_rising)) engage(grey);

    // engage() already establishes the initial anchor.  This branch is only
    // for adding translation while an orientation-only session is active.
    if (was_active && active_ && grey_rising) {
      const Eigen::Isometry3d measured =
          latest_state_->getGlobalLinkTransform(tip_link_);
      robot_anchor_.translation().z() = measured.translation().z();
      target_pose_.translation().z() = measured.translation().z();
      if (!zOnlyLockedMode()) {
        robot_anchor_.translation() = measured.translation();
        target_pose_.translation() = measured.translation();
      }
      touch_anchor_.translation() = latest_touch_.translation();
      touch_increment_anchor_ = latest_touch_increment_;
      ROS_INFO("Grey-button position anchor set");
    }
    if (active_ && white_rising) {
      touch_orientation_anchor_ = latest_touch_.linear();
      robot_orientation_anchor_ = target_pose_.linear();
      ROS_INFO("White-button orientation anchor set");
    }
    if (active_ && !grey && grey_pressed_ && hybrid_enabled_) {
      hybrid_.reset(target_pose_.translation().z());
      publishMode();
    }
    grey_pressed_ = grey;
    white_pressed_ = white;
    if (active_ && !grey && !white) disengage("both Touch buttons released");
  }

  void engage(bool needs_position_input) {
    if (!motion_enabled_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage Touch: RM75 motion is disarmed");
      return;
    }
    if (hybrid_enabled_ && !released_since_stop_) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage sampling: release grey after a stop");
      return;
    }
    if (hybrid_enabled_ && !coordinateReady(ros::WallTime::now())) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage sampling: force coordinate actual=%d expected=%d age=%.3f", coordinate_, expected_coordinate_, (ros::WallTime::now() - last_coordinate_wall_time_).toSec());
      return;
    }
    if (needs_position_input && hybrid_enabled_ && require_increment_ &&
        (!have_touch_increment_ || (ros::WallTime::now() - last_increment_wall_time_).toSec() > input_timeout_)) {
      ROS_WARN_THROTTLE(1.0, "Cannot engage sampling: Touch increment missing/stale");
      return;
    }
    if (force_safety_enabled_ && require_force_feedback_) {
      const bool force_stale = !have_force_feedback_ ||
          (ros::WallTime::now() - last_force_wall_time_).toSec() >
              force_feedback_timeout_;
      if (force_stale) {
        ROS_WARN("Cannot engage Touch: waiting for fresh six-axis force feedback");
        return;
      }
      if (forceLimitExceeded()) {
        ROS_WARN("Cannot engage Touch: external wrench is already above the safety limit");
        return;
      }
    }
    if (!have_touch_ || !have_joint_state_) {
      ROS_WARN("Cannot engage Touch: waiting for both Touch pose and RM75 joint states");
      return;
    }
    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_touch_wall_time_).toSec() > input_timeout_ ||
        (now - last_joint_wall_time_).toSec() > input_timeout_) {
      ROS_WARN("Cannot engage Touch: input is stale");
      return;
    }
    if (force_safety_enabled_ && require_force_feedback_ &&
        (!have_force_feedback_ ||
         (now - last_force_wall_time_).toSec() > force_feedback_timeout_)) {
      disengage("six-axis force feedback timed out");
      return;
    }
    if (force_safety_enabled_ && have_force_feedback_ && forceLimitExceeded()) {
      disengage("six-axis force safety limit exceeded");
      return;
    }
    if (!latest_state_->satisfiesBounds(joint_group_)) {
      ROS_WARN("Cannot engage: measured joints violate model bounds"); return;
    }
    touch_anchor_ = latest_touch_;
    touch_increment_anchor_ = latest_touch_increment_;
    const Eigen::Isometry3d measured =
        latest_state_->getGlobalLinkTransform(tip_link_);
    robot_anchor_ = measured;
    if (zOnlyLockedMode()) {
      if (!z_only_session_initialized_) {
        z_only_session_pose_ = measured;
        z_only_session_initialized_ = true;
      } else {
        if (max_z_position_travel_ > 0.0 &&
            std::abs(measured.translation().z() -
                     z_only_session_pose_.translation().z()) >
                max_z_position_travel_ + hybrid_.cfg.max_z_tracking_error) {
          ROS_WARN("Cannot engage Z-only control: session Z travel already exceeded");
          return;
        }
        robot_anchor_.translation().x() = z_only_session_pose_.translation().x();
        robot_anchor_.translation().y() = z_only_session_pose_.translation().y();
        robot_anchor_.linear() = z_only_session_pose_.linear();
      }
    }
    target_pose_ = robot_anchor_;
    touch_orientation_anchor_ = latest_touch_.linear();
    robot_orientation_anchor_ = robot_anchor_.linear();
    *command_seed_ = *latest_state_;
    latest_state_->copyJointGroupPositions(joint_group_, last_command_);
    for (int i = 0; i < rm75_touch_teleop::kRm75Dof; ++i)
      posture_reference_[i] = last_command_[static_cast<std::size_t>(i)];
    last_valid_velocity_.setZero();
    have_command_seed_ = true;
    hybrid_.reset(robot_anchor_.translation().z());
    last_control_steady_time_ = ros::SteadyTime::now();
    stop_latched_ = false;
    active_ = true;
    std_msgs::String reason; reason_pub_.publish(reason);
    publishMode();
    publishActive(true);
    ROS_INFO("Touch control engaged at the current RM75 pose");
    if (hybrid_enabled_ && !force_frame_verified_)
      ROS_WARN("Force frame/sign is unverified: free-space POSITION_MODE only; contact will SAFE_STOP");
  }

  void disengage(const std::string& reason) {
    if (!active_) return;
    if (hybrid_enabled_ && output_mode_ == "rm_joint_pos") stop_pub_.publish(std_msgs::Empty());
    active_ = false;
    if (hybrid_.mode != rm75_touch_teleop::ControlMode::SAFE_STOP)
      hybrid_.mode = rm75_touch_teleop::ControlMode::POSITION_MODE;
    publishMode();
    publishActive(false);
    ROS_INFO_STREAM("Touch disengaged: " << reason);
  }

  bool zOnlyLockedMode() const {
    return translation_z_enabled_ && !translation_x_enabled_ &&
           !translation_y_enabled_ && !orientation_enabled_;
  }

  void coordinateCallback(const std_msgs::UInt16::ConstPtr& msg) {
    coordinate_ = msg->data;
    last_coordinate_wall_time_ = ros::WallTime::now();
    if (coordinate_ != expected_coordinate_) safetyStop("force coordinate mismatch");
  }

  bool coordinateReady(const ros::WallTime& now) const {
    return expected_coordinate_ < 0 || (coordinate_ == expected_coordinate_ &&
        (now - last_coordinate_wall_time_).toSec() <= force_feedback_timeout_);
  }

  void publishMode() {
    std_msgs::String msg;
    msg.data = rm75_touch_teleop::modeName(hybrid_.mode);
    mode_pub_.publish(msg);
  }

  void safetyStop(const std::string& reason) {
    if (stop_latched_) return;
    stop_latched_ = true;
    hybrid_.stop();
    released_since_stop_ = false;
    // Only request a hardware stop if this node was actively commanding motion.
    // A disarmed commissioning node must remain read-only even when an operator
    // intentionally loads the sensor to identify its sign and axes.
    if (active_ && motion_enabled_ && output_mode_ == "rm_joint_pos")
      stop_pub_.publish(std_msgs::Empty());
    active_ = false;
    publishActive(false);
    publishMode();
    std_msgs::String msg; msg.data = reason; reason_pub_.publish(msg);
    ROS_ERROR_STREAM("RM75 SAFE_STOP: " << reason);
  }

  void update(const ros::SteadyTimerEvent&) {
    const ros::WallTime now = ros::WallTime::now();
    updateHaptic(now);
    if (!active_) return;
    const ros::SteadyTime control_now = ros::SteadyTime::now();
    const double elapsed =
        (control_now - last_control_steady_time_).toSec();
    last_control_steady_time_ = control_now;
    if (!std::isfinite(elapsed) || elapsed <= 0.0) {
      safetyStop("invalid monotonic control period");
      return;
    }
    if (elapsed < min_control_dt_) {
      ROS_DEBUG_THROTTLE(1.0,
                         "Control period %.6f s is below %.6f s; cycle skipped",
                         elapsed, min_control_dt_);
      return;
    }
    const double dt = std::min(max_control_dt_, elapsed);
    if (elapsed > max_control_dt_)
      ROS_WARN_THROTTLE(1.0, "Control period %.4f s capped to %.4f s for constraints",
                        elapsed, dt);
    if ((now - last_touch_wall_time_).toSec() > input_timeout_ ||
        (now - last_joint_wall_time_).toSec() > input_timeout_ ||
        (now - last_joy_wall_time_).toSec() > input_timeout_ ||
        (grey_pressed_ && hybrid_enabled_ && use_touch_increment_ &&
         (have_touch_increment_ || require_increment_) &&
         (!have_touch_increment_ || (now - last_increment_wall_time_).toSec() > input_timeout_))) {
      safetyStop("Touch pose/buttons/increment or joint state timed out");
      return;
    }
    if (force_safety_enabled_ && require_force_feedback_ &&
        (!have_force_feedback_ || (now - last_force_wall_time_).toSec() > force_feedback_timeout_)) {
      safetyStop("six-axis force feedback timed out");
      return;
    }
    if (force_safety_enabled_ && have_force_feedback_ && forceLimitExceeded()) {
      safetyStop("six-axis force safety limit exceeded");
      return;
    }
    if (hybrid_enabled_ && !coordinateReady(now)) {
      safetyStop("force coordinate feedback timed out or mismatched");
      return;
    }
    // A blocked slave must first create bounded bilateral position feedback,
    // but it must never be allowed to accumulate an unbounded queued motion.
    // Keep this final joint-space guard active for both position-only haptics
    // and hybrid force control.
    std::vector<double> actual;
    latest_state_->copyJointGroupPositions(joint_group_, actual);
    for (std::size_t i = 0; i < actual.size(); ++i)
      if (std::abs(actual[i] - last_command_[i]) > max_joint_tracking_error_) {
        safetyStop("joint tracking error exceeded"); return;
      }

    const Eigen::Isometry3d previous_target = target_pose_;
    const double measured_z = latest_state_->getGlobalLinkTransform(tip_link_).translation().z();
    if (grey_pressed_) {
      Eigen::Vector3d touch_delta;
      if (use_touch_increment_ && have_touch_increment_) {
        touch_delta = latest_touch_increment_ - touch_increment_anchor_;
      } else {
        touch_delta = latest_touch_.translation() - touch_anchor_.translation();
      }
      Eigen::Vector3d robot_delta = rm75_touch_teleop::mapTranslation(
          touch_delta, metres_per_touch_unit_ * position_scale_);
      // Axis locks are applied after the Touch-to-robot basis mapping.  This
      // lets a deployment expose a deliberately smaller Cartesian subspace
      // without changing the calibrated device mapping.  Locked axes stay at
      // the pose captured on engagement and do not contribute to the travel
      // guard below.
      robot_delta = rm75_touch_teleop::applyTranslationAxisMask(
          robot_delta, translation_x_enabled_, translation_y_enabled_,
          translation_z_enabled_);
      if (hybrid_enabled_ && !force_frame_verified_ &&
          hybrid_.mode == rm75_touch_teleop::ControlMode::POSITION_MODE &&
          std::abs(hybrid_.filtered) > hybrid_.cfg.contact_threshold) {
        safetyStop("contact detected before force frame/sign verification");
        return;
      }
      if (hybrid_enabled_ && hybrid_.transition(measured_z, dt)) {
        if (hybrid_.mode == rm75_touch_teleop::ControlMode::POSITION_MODE) {
          // Only rebase Z: preserve operator X/Y and orientation continuity.
          robot_anchor_.translation().z() = measured_z;
          // Touch Y maps to robot Z. Rebase both supported input sources,
          // so ignored hand travel cannot trip the position displacement guard.
          touch_anchor_.translation().y() = latest_touch_.translation().y();
          touch_increment_anchor_.y() = latest_touch_increment_.y();
          robot_delta.z() = 0.0;
        }
        publishMode();
      }
      Eigen::Vector3d checked_delta = robot_delta;
      if (hybrid_enabled_ && hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE)
        checked_delta.z() = 0;  // Ignored robot-Z/Touch-Y motion must not block XY control.
      if (!robot_delta.allFinite() || checked_delta.norm() > max_touch_translation_) {
        ROS_WARN_THROTTLE(0.5, "Touch displacement exceeds safety limit; command withheld");
        return;
      }
      target_pose_.translation() = robot_anchor_.translation() + robot_delta;
      if (hybrid_enabled_ && hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
        target_pose_.translation().z() = hybrid_.nextZ(measured_z, dt);
        if (hybrid_.travelExceeded(target_pose_.translation().z(), measured_z)) {
          safetyStop("force-control Z travel exceeded"); return;
        }
      }
      if (zOnlyLockedMode()) {
        // Keep the lock frame for the complete node lifetime, not merely one
        // grey-button clutch.  This prevents repeated release/re-engage cycles
        // from ratcheting measured X/Y/orientation drift into the target.
        target_pose_.translation().x() = z_only_session_pose_.translation().x();
        target_pose_.translation().y() = z_only_session_pose_.translation().y();
        target_pose_.linear() = z_only_session_pose_.linear();

        const bool contact = hybrid_enabled_ &&
            hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE;
        if (!contact) {
          target_pose_.translation().z() = rm75_touch_teleop::limitAxisStep(
              previous_target.translation().z(), target_pose_.translation().z(),
              hybrid_.cfg.max_z_velocity, hybrid_.cfg.max_z_step, dt);
        }
        const double bounded_z = rm75_touch_teleop::clampAxisTravel(
            target_pose_.translation().z(), z_only_session_pose_.translation().z(),
            max_z_position_travel_);
        if (contact && bounded_z != target_pose_.translation().z()) {
          safetyStop("Z-only session travel exceeded in contact mode"); return;
        }
        if (bounded_z != target_pose_.translation().z())
          ROS_WARN_THROTTLE(0.5, "Z-only session travel limit reached; command clamped");
        target_pose_.translation().z() = bounded_z;
      }
    }
    if (orientation_enabled_ && white_pressed_) {
      const Eigen::Matrix3d touch_delta_rotation =
          latest_touch_.linear() * touch_orientation_anchor_.transpose();
      target_pose_.linear() = rm75_touch_teleop::mapRelativeRotation(
                                  touch_delta_rotation) *
                              robot_orientation_anchor_;
    }
    target_pose_ = rm75_touch_teleop::limitPoseStep(
        previous_target, target_pose_, qp_config_.max_linear_velocity,
        qp_config_.max_angular_velocity, dt);

    const Eigen::Isometry3d measured_pose =
        latest_state_->getGlobalLinkTransform(tip_link_);
    const moveit::core::LinkModel* tip = robot_model_->getLinkModel(tip_link_);
    Eigen::MatrixXd dynamic_jacobian;
    if (!tip || !latest_state_->getJacobian(joint_group_, tip,
                                             Eigen::Vector3d::Zero(),
                                             dynamic_jacobian) ||
        dynamic_jacobian.rows() != 6 || dynamic_jacobian.cols() != 7 ||
        !dynamic_jacobian.allFinite()) {
      target_pose_ = previous_target;
      safetyStop("invalid 6x7 Jacobian from measured joint feedback");
      return;
    }

    rm75_touch_teleop::DifferentialIkInput qp_input;
    qp_input.jacobian = dynamic_jacobian;
    qp_input.pose_error.head<3>() =
        target_pose_.translation() - measured_pose.translation();
    const Eigen::AngleAxisd rotation_error(
        target_pose_.linear() * measured_pose.linear().transpose());
    qp_input.pose_error.tail<3>() =
        rotation_error.axis() * rotation_error.angle();
    qp_input.position_lower = position_lower_;
    qp_input.position_upper = position_upper_;
    qp_input.velocity_limit = velocity_limit_;
    qp_input.acceleration_limit = acceleration_limit_;
    qp_input.previous_velocity = last_valid_velocity_;
    qp_input.posture_reference = posture_reference_;
    qp_input.dt = dt;
    for (int i = 0; i < rm75_touch_teleop::kRm75Dof; ++i) {
      qp_input.measured_position[i] = actual[static_cast<std::size_t>(i)];
      qp_input.previous_command[i] = last_command_[static_cast<std::size_t>(i)];
    }

    const rm75_touch_teleop::DifferentialIkResult qp =
        rm75_touch_teleop::solveConstrainedDifferentialIk(qp_input, qp_config_);
    publishQpStatus(qp, dt);
    if (!qp.success()) {
      target_pose_ = previous_target;
      safetyStop(std::string("constrained QP failed: ") + qp.reason);
      return;
    }

    std::vector<double> desired(7);
    rm75_touch_teleop::JointVector acceleration;
    for (int i = 0; i < rm75_touch_teleop::kRm75Dof; ++i) {
      desired[static_cast<std::size_t>(i)] =
          last_command_[static_cast<std::size_t>(i)] + qp.velocity[i] * dt;
      acceleration[i] = (qp.velocity[i] - last_valid_velocity_[i]) / dt;
      if (!std::isfinite(desired[static_cast<std::size_t>(i)]) ||
          desired[static_cast<std::size_t>(i)] < position_lower_[i] - 1e-9 ||
          desired[static_cast<std::size_t>(i)] > position_upper_[i] + 1e-9 ||
          qp.velocity[i] < qp.lower_bound[i] - 1e-8 ||
          qp.velocity[i] > qp.upper_bound[i] + 1e-8) {
        target_pose_ = previous_target;
        safetyStop("QP solution failed post-solve safety validation");
        return;
      }
    }
    moveit::core::RobotState candidate(*latest_state_);
    candidate.setJointGroupPositions(joint_group_, desired);
    candidate.update();
    if (!candidate.satisfiesBounds(joint_group_)) {
      target_pose_ = previous_target;
      safetyStop("QP command violates MoveIt joint bounds");
      return;
    }
    if (hybrid_enabled_ && hybrid_.mode == rm75_touch_teleop::ControlMode::CONTACT_MODE) {
      const double old_z = command_seed_->getGlobalLinkTransform(tip_link_).translation().z();
      const double z_bound = std::min(hybrid_.cfg.max_z_step, hybrid_.cfg.max_z_velocity * dt);
      const double new_z = candidate.getGlobalLinkTransform(tip_link_).translation().z();
      if (std::abs(new_z - old_z) > z_bound + 1e-9 ||
          std::abs(new_z - measured_z) > hybrid_.cfg.max_z_tracking_error ||
          hybrid_.travelExceeded(new_z, measured_z)) {
        target_pose_ = previous_target;
        safetyStop("QP command cannot satisfy Cartesian Z safety limits");
        return;
      }
    }
    command_seed_->setJointGroupPositions(joint_group_, desired);
    command_seed_->enforceBounds(joint_group_);
    command_seed_->copyJointGroupPositions(joint_group_, desired);
    command_seed_->update();
    last_command_ = desired;
    last_valid_velocity_ = qp.velocity;
    posture_reference_ = rm75_touch_teleop::updatePostureReference(
        posture_reference_, qp_input.measured_position,
        posture_reference_update_rate_, dt);
    publishJointDiagnostics(qp_input.measured_position, qp.velocity,
                            acceleration, desired);
    if (hybrid_enabled_) {
      hybrid_.commit(command_seed_->getGlobalLinkTransform(tip_link_).translation().z(), dt);
      std_msgs::Float64 setpoint; setpoint.data = hybrid_.setpoint;
      force_target_pub_.publish(setpoint);
    }

    const ros::Time command_stamp = ros::Time::now();
    if (output_mode_ == "trajectory") {
      trajectory_msgs::JointTrajectory trajectory;
      trajectory.header.stamp = command_stamp;
      trajectory.joint_names = joint_names_;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = desired;
      point.time_from_start = ros::Duration(command_duration_);
      trajectory.points.push_back(point);
      command_pub_.publish(trajectory);
    } else {
      rm_msgs::JointPos command;
      command.joint.resize(desired.size());
      for (std::size_t i = 0; i < desired.size(); ++i) {
        command.joint[i] = static_cast<float>(desired[i]);
      }
      command.expand = 0.0F;
      rm_joint_pos_pub_.publish(command);
    }

    geometry_msgs::PoseStamped target;
    target.header.stamp = command_stamp;
    target.header.frame_id = base_frame_;
    target.pose = eigenToPose(target_pose_);
    target_pub_.publish(target);
  }

  void publishQpStatus(
      const rm75_touch_teleop::DifferentialIkResult& qp,
      double measured_dt) {
    std_msgs::String status;
    status.data = std::string(rm75_touch_teleop::boxQpStatusName(qp.status)) +
        ";reason=" + qp.reason +
        ";iterations=" + std::to_string(qp.iterations) +
        ";min_singular=" + std::to_string(qp.minimum_singular_value) +
        ";kkt=" + std::to_string(qp.kkt_residual);
    qp_status_pub_.publish(status);
    std_msgs::Float64 value;
    value.data = qp.solve_time_sec * 1000.0;
    qp_solve_time_pub_.publish(value);
    value.data = measured_dt;
    control_dt_pub_.publish(value);
  }

  void publishJointDiagnostics(
      const rm75_touch_teleop::JointVector& actual,
      const rm75_touch_teleop::JointVector& velocity,
      const rm75_touch_teleop::JointVector& acceleration,
      const std::vector<double>& target) {
    std_msgs::Float64MultiArray target_msg, actual_msg, velocity_msg;
    std_msgs::Float64MultiArray acceleration_msg, error_msg;
    target_msg.data.resize(7); actual_msg.data.resize(7);
    velocity_msg.data.resize(7); acceleration_msg.data.resize(7);
    error_msg.data.resize(7);
    for (int i = 0; i < rm75_touch_teleop::kRm75Dof; ++i) {
      target_msg.data[i] = target[static_cast<std::size_t>(i)];
      actual_msg.data[i] = actual[i];
      velocity_msg.data[i] = velocity[i];
      acceleration_msg.data[i] = acceleration[i];
      error_msg.data[i] = target_msg.data[i] - actual_msg.data[i];
    }
    target_joint_pub_.publish(target_msg);
    actual_joint_pub_.publish(actual_msg);
    joint_velocity_pub_.publish(velocity_msg);
    joint_acceleration_pub_.publish(acceleration_msg);
    tracking_error_pub_.publish(error_msg);
  }

  void updateHaptic(const ros::WallTime& now) {
    if (!haptic_enabled_) return;
    const double elapsed = last_haptic_update_wall_time_.isZero() ?
        1.0 / publish_rate_ : (now - last_haptic_update_wall_time_).toSec();
    last_haptic_update_wall_time_ = now;
    const bool fresh = haptic_force_valid_ && have_force_feedback_ &&
        (now - last_force_wall_time_).toSec() <= haptic_force_timeout_;
    const bool safe_stop = hybrid_.mode == rm75_touch_teleop::ControlMode::SAFE_STOP;
    const bool allowed = rm75_touch_teleop::hapticOutputAllowed(
        haptic_enabled_, haptic_only_in_contact_, haptic_require_active_, active_,
        hybrid_.mode, fresh, force_frame_verified_);
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    if (have_touch_velocity_ &&
        (now - last_touch_twist_wall_time_).toSec() <= input_timeout_)
      velocity = latest_touch_velocity_mps_;
    Eigen::Vector3d tracking_error = Eigen::Vector3d::Zero();
    if (active_ && have_joint_state_)
      tracking_error = latest_state_->getGlobalLinkTransform(tip_link_).translation() -
                       target_pose_.translation();
    const auto output = haptic_.step(latest_haptic_robot_force_, velocity,
                                     elapsed, allowed, safe_stop,
                                     tracking_error);
    // Axis masking is applied to the command sent to the device, while the raw
    // and filtered diagnostics retain the complete measured wrench.  The
    // dedicated Z-only path therefore renders robot Z solely on Touch Y and
    // cannot turn lateral sensor bias into unintended handle forces.
    Eigen::Vector3d device_command = output.command;
    if (!haptic_output_x_enabled_) device_command.x() = 0.0;
    if (!haptic_output_y_enabled_) device_command.y() = 0.0;
    if (!haptic_output_z_enabled_) device_command.z() = 0.0;
    publishHapticVector(haptic_raw_pub_, output.raw_touch_force, now);
    publishHapticVector(haptic_filtered_pub_, output.filtered_touch_force, now);
    publishHapticVector(haptic_command_diag_pub_, device_command, now);
    publishHapticVector(haptic_position_force_pub_,
                        output.position_touch_force, now);
    publishHapticVector(haptic_tracking_error_pub_, tracking_error, now);
    publishHapticVector(haptic_device_command_pub_, device_command, now);
    std_msgs::Bool fresh_msg; fresh_msg.data = fresh;
    haptic_fresh_pub_.publish(fresh_msg);
  }

  static void publishHapticVector(ros::Publisher& publisher,
                                  const Eigen::Vector3d& value,
                                  const ros::WallTime&) {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "touch_base";
    msg.vector.x = value.x(); msg.vector.y = value.y(); msg.vector.z = value.z();
    publisher.publish(msg);
  }

  void publishActive(bool active) {
    std_msgs::Bool msg;
    msg.data = active;
    active_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  robot_model_loader::RobotModelLoader model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  std::unique_ptr<moveit::core::RobotState> latest_state_;
  std::unique_ptr<moveit::core::RobotState> command_seed_;
  ros::Subscriber touch_pose_sub_, touch_twist_sub_, joy_sub_, joint_state_sub_;
  ros::Subscriber touch_increment_sub_, touch_anchor_sub_, force_sub_;
  ros::Publisher command_pub_, rm_joint_pos_pub_, target_pub_, active_pub_;
  ros::SteadyTimer timer_;
  ros::Subscriber coordinate_sub_;
  ros::Publisher stop_pub_, mode_pub_, reason_pub_, fz_pub_, filtered_fz_pub_, force_target_pub_;
  ros::Publisher haptic_enabled_pub_, haptic_raw_pub_, haptic_filtered_pub_;
  ros::Publisher haptic_command_diag_pub_, haptic_fresh_pub_, haptic_device_command_pub_;
  ros::Publisher haptic_position_force_pub_, haptic_tracking_error_pub_;
  ros::Publisher qp_status_pub_, qp_solve_time_pub_, control_dt_pub_;
  ros::Publisher target_joint_pub_, actual_joint_pub_, joint_velocity_pub_;
  ros::Publisher joint_acceleration_pub_, tracking_error_pub_;
  rm75_touch_teleop::HybridForceControl hybrid_;
  rm75_touch_teleop::HapticRenderer haptic_;
  bool hybrid_enabled_ = false, force_frame_verified_ = false, require_increment_ = false;
  bool released_since_stop_ = true, stop_latched_ = false;
  std::string force_frame_;
  Eigen::Matrix3d force_rotation_ = Eigen::Matrix3d::Identity();
  double force_sign_ = 1.0, max_joint_tracking_error_ = 0.15;
  double force_startup_grace_period_ = 0.0;
  int expected_coordinate_ = -1, coordinate_ = -1;
  ros::WallTime last_joy_wall_time_, last_increment_wall_time_;
  ros::SteadyTime last_control_steady_time_;
  ros::WallTime last_coordinate_wall_time_, startup_wall_time_;

  std::string planning_group_, tip_link_, base_frame_, output_mode_;
  std::string touch_pose_topic_, touch_joy_topic_, joint_state_topic_, command_topic_;
  std::string force_topic_, touch_twist_topic_;
  std::vector<std::string> joint_names_;
  std::vector<double> last_command_;
  rm75_touch_teleop::DifferentialIkConfig qp_config_;
  rm75_touch_teleop::JointVector position_lower_ =
      rm75_touch_teleop::JointVector::Zero();
  rm75_touch_teleop::JointVector position_upper_ =
      rm75_touch_teleop::JointVector::Zero();
  rm75_touch_teleop::JointVector velocity_limit_ =
      rm75_touch_teleop::JointVector::Zero();
  rm75_touch_teleop::JointVector acceleration_limit_ =
      rm75_touch_teleop::JointVector::Zero();
  rm75_touch_teleop::JointVector last_valid_velocity_ =
      rm75_touch_teleop::JointVector::Zero();
  rm75_touch_teleop::JointVector posture_reference_ =
      rm75_touch_teleop::JointVector::Zero();
  Eigen::Isometry3d latest_touch_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d touch_anchor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d robot_anchor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d z_only_session_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d touch_orientation_anchor_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d robot_orientation_anchor_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d latest_touch_increment_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d touch_increment_anchor_ = Eigen::Vector3d::Zero();
  ros::WallTime last_touch_wall_time_, last_joint_wall_time_, last_force_wall_time_;
  ros::WallTime last_touch_twist_wall_time_, last_haptic_update_wall_time_;
  bool have_touch_ = false, have_joint_state_ = false, have_command_seed_ = false;
  bool grey_pressed_ = false, white_pressed_ = false, active_ = false;
  bool translation_x_enabled_ = true, translation_y_enabled_ = true;
  bool translation_z_enabled_ = true, orientation_enabled_ = true;
  bool z_only_session_initialized_ = false;
  bool motion_enabled_ = true;
  bool haptic_output_x_enabled_ = true;
  bool haptic_output_y_enabled_ = true;
  bool haptic_output_z_enabled_ = true;
  bool force_safety_enabled_ = false, require_force_feedback_ = false;
  bool have_force_feedback_ = false;
  bool haptic_enabled_ = false, haptic_only_in_contact_ = true;
  bool haptic_require_active_ = true, haptic_force_valid_ = false;
  bool have_touch_velocity_ = false;
  bool use_touch_increment_ = true, have_touch_increment_ = false;
  double metres_per_touch_unit_ = 0.001, position_scale_ = 1.0;
  double publish_rate_ = 100.0, command_duration_ = 0.08;
  double max_touch_translation_ = 0.25, max_joint_velocity_ = 0.8;
  double max_joint_acceleration_ = 1.5;
  double min_control_dt_ = 0.001, max_control_dt_ = 0.04;
  double posture_reference_update_rate_ = 0.30;
  double max_z_position_travel_ = 0.0;
  double input_timeout_ = 0.15;
  double force_feedback_timeout_ = 0.25;
  double haptic_force_timeout_ = 0.1;
  double max_external_force_ = 35.0, max_external_torque_ = 5.0;
  double latest_force_norm_ = 0.0, latest_torque_norm_ = 0.0;
  Eigen::Vector3d latest_haptic_robot_force_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_touch_velocity_mps_ = Eigen::Vector3d::Zero();
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_touch_teleop");
  try {
    Rm75TouchTeleop node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("RM75 Touch teleop failed to start: " << error.what());
    return 1;
  }
  return 0;
}
