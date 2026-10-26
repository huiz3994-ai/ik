#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

#include "rm75_touch_teleop/mapping.h"
#include "rm75_touch_teleop/hybrid_force_control.h"

namespace rm75_touch_teleop {

struct HapticConfig {
  double force_scale = 0.02;
  double filter_tau = 0.03;
  double deadband = 0.5;
  Eigen::Vector3d damping = Eigen::Vector3d::Zero();  // N s/m
  Eigen::Vector3d max_force = Eigen::Vector3d::Constant(0.5);  // N
  double max_force_norm = 0.5;  // N
  double ramp_rate = 2.0;       // N/s, vector-norm slew limit
  double max_dt = 0.05;         // s
  Eigen::Vector3d force_sign = Eigen::Vector3d::Ones();
  Eigen::Matrix3d robot_force_rotation = Eigen::Matrix3d::Identity();
  // Bilateral virtual coupling.  The input error is actual minus requested
  // robot position, so the resulting force pulls the Touch back toward the
  // pose that the slave has actually reached.
  Eigen::Vector3d position_stiffness = Eigen::Vector3d::Zero();  // N/m
  double position_deadband = 0.001;  // m, radial and continuous
  double max_position_error = 0.02;  // m before stiffness calculation

  void validate() const {
    const double nonnegative[] = {force_scale, filter_tau, deadband,
                                  position_deadband};
    for (double value : nonnegative) {
      if (!std::isfinite(value) || value < 0.0)
        throw std::runtime_error("Invalid nonnegative haptic parameter");
    }
    const double positive[] = {max_force_norm, ramp_rate, max_dt,
                               max_position_error};
    for (double value : positive) {
      if (!std::isfinite(value) || value <= 0.0)
        throw std::runtime_error("Invalid positive haptic parameter");
    }
    if (!damping.allFinite() || (damping.array() < 0.0).any() ||
        !max_force.allFinite() || (max_force.array() <= 0.0).any() ||
        !position_stiffness.allFinite() ||
        (position_stiffness.array() < 0.0).any() ||
        !force_sign.allFinite() || !robot_force_rotation.allFinite())
      throw std::runtime_error("Invalid haptic vector or rotation parameter");
    for (int axis = 0; axis < 3; ++axis) {
      if (force_sign[axis] != 1.0 && force_sign[axis] != -1.0)
        throw std::runtime_error("Haptic force signs must be +1 or -1");
    }
  }
};

struct HapticOutput {
  Eigen::Vector3d raw_touch_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d filtered_touch_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d command = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_touch_force = Eigen::Vector3d::Zero();
};

inline bool hapticOutputAllowed(bool enabled, bool only_in_contact,
                                bool require_active, bool active,
                                ControlMode mode, bool force_fresh,
                                bool force_frame_verified) {
  return enabled && (!require_active || active) &&
         (!only_in_contact || mode == ControlMode::CONTACT_MODE) &&
         mode != ControlMode::SAFE_STOP && force_fresh && force_frame_verified;
}

// Pure haptic rendering algorithm. It has no ROS I/O and cannot affect the
// robot motion path. Robot/base force is mapped to Touch axes by S^T (virtual
// work consistency with v_robot=S*v_touch), after an independently configured
// robot-force rotation and before per-Touch-axis sign correction.
class HapticRenderer {
 public:
  explicit HapticRenderer(const HapticConfig& config = HapticConfig())
      : cfg(config) {
    cfg.validate();
  }

  HapticConfig cfg;

  HapticOutput step(const Eigen::Vector3d& robot_base_force,
                    const Eigen::Vector3d& touch_velocity_mps,
                    double dt, bool output_allowed,
                    bool immediate_zero = false,
                    const Eigen::Vector3d& robot_position_error =
                        Eigen::Vector3d::Zero()) {
    if (!robot_base_force.allFinite() || !touch_velocity_mps.allFinite() ||
        !robot_position_error.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
      reset();
      return output_;
    }

    const double bounded_dt = std::min(dt, cfg.max_dt);
    output_.raw_touch_force = cfg.force_sign.asDiagonal() *
        touchToRobotBasis().transpose() * cfg.robot_force_rotation *
        robot_base_force;
    if (!output_.raw_touch_force.allFinite()) {
      reset();
      return output_;
    }

    const double alpha = cfg.filter_tau == 0.0 ? 1.0 :
        bounded_dt / (cfg.filter_tau + bounded_dt);
    filtered_ += alpha * (output_.raw_touch_force - filtered_);
    output_.filtered_touch_force = filtered_;

    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    output_.position_touch_force.setZero();
    if (output_allowed) {
      for (int axis = 0; axis < 3; ++axis) {
        const double value = filtered_[axis];
        if (std::abs(value) > cfg.deadband)
          target[axis] = std::copysign(std::abs(value) - cfg.deadband, value);
      }
      target *= cfg.force_scale;

      Eigen::Vector3d bounded_error = robot_position_error;
      double error_norm = bounded_error.norm();
      if (error_norm > cfg.max_position_error) {
        bounded_error *= cfg.max_position_error / error_norm;
        error_norm = cfg.max_position_error;
      }
      if (error_norm > cfg.position_deadband) {
        const Eigen::Vector3d effective_error =
            bounded_error * ((error_norm - cfg.position_deadband) / error_norm);
        const Eigen::Vector3d robot_coupling_force =
            cfg.position_stiffness.cwiseProduct(effective_error);
        output_.position_touch_force =
            touchToRobotBasis().transpose() * robot_coupling_force;
        target += output_.position_touch_force;
      }
      target -= cfg.damping.cwiseProduct(touch_velocity_mps);
      for (int axis = 0; axis < 3; ++axis)
        target[axis] = std::max(-cfg.max_force[axis],
                                std::min(cfg.max_force[axis], target[axis]));
      const double norm = target.norm();
      if (norm > cfg.max_force_norm) target *= cfg.max_force_norm / norm;
    }

    if (!target.allFinite() || immediate_zero) {
      command_.setZero();
    } else {
      Eigen::Vector3d delta = target - command_;
      const double max_delta = cfg.ramp_rate * bounded_dt;
      if (delta.norm() > max_delta) delta *= max_delta / delta.norm();
      command_ += delta;
    }
    if (!command_.allFinite()) command_.setZero();
    output_.command = command_;
    return output_;
  }

  void reset() {
    filtered_.setZero();
    command_.setZero();
    output_ = HapticOutput();
  }

  const HapticOutput& output() const { return output_; }

 private:
  Eigen::Vector3d filtered_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
  HapticOutput output_;
};

}  // namespace rm75_touch_teleop
