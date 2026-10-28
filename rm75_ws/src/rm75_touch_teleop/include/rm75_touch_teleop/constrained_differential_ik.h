#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "rm75_touch_teleop/box_qp_solver.h"

namespace rm75_touch_teleop {

constexpr int kRm75Dof = 7;
using JointVector = Eigen::Matrix<double, kRm75Dof, 1>;
using TaskVector = Eigen::Matrix<double, 6, 1>;
using GeometricJacobian = Eigen::Matrix<double, 6, kRm75Dof>;

struct DifferentialIkConfig {
  double translation_gain = 4.0;
  double rotation_gain = 3.0;
  double max_linear_velocity = 0.12;
  double max_angular_velocity = 0.60;
  double translation_weight = 1.0;
  double rotation_weight = 0.35;
  double regularization = 1e-3;
  double velocity_smoothing_weight = 0.02;
  double posture_weight = 0.08;
  double posture_gain = 0.7;
  double soft_limit_gain = 0.35;
  double soft_limit_margin = 0.20;
  double singular_value_threshold = 0.05;
  double singular_regularization = 0.05;
  BoxQpOptions solver;
};

struct DifferentialIkInput {
  GeometricJacobian jacobian = GeometricJacobian::Zero();
  TaskVector pose_error = TaskVector::Zero();
  JointVector measured_position = JointVector::Zero();
  JointVector previous_command = JointVector::Zero();
  JointVector previous_velocity = JointVector::Zero();
  JointVector posture_reference = JointVector::Zero();
  JointVector position_lower = JointVector::Constant(-M_PI);
  JointVector position_upper = JointVector::Constant(M_PI);
  JointVector velocity_limit = JointVector::Constant(0.5);
  JointVector acceleration_limit = JointVector::Constant(1.0);
  double dt = 0.01;
};

struct DifferentialIkResult {
  BoxQpStatus status = BoxQpStatus::INVALID_INPUT;
  JointVector velocity = JointVector::Zero();
  JointVector lower_bound = JointVector::Zero();
  JointVector upper_bound = JointVector::Zero();
  JointVector posture_velocity = JointVector::Zero();
  TaskVector desired_twist = TaskVector::Zero();
  double minimum_singular_value = 0.0;
  double adaptive_regularization = 0.0;
  double solve_time_sec = 0.0;
  double kkt_residual = std::numeric_limits<double>::infinity();
  int iterations = 0;
  std::string reason;

  bool success() const { return status == BoxQpStatus::SUCCESS; }
};

inline bool validDifferentialIkInput(const DifferentialIkInput& input,
                                     const DifferentialIkConfig& config) {
  return input.jacobian.allFinite() && input.pose_error.allFinite() &&
      input.measured_position.allFinite() && input.previous_command.allFinite() &&
      input.previous_velocity.allFinite() && input.posture_reference.allFinite() &&
      input.position_lower.allFinite() && input.position_upper.allFinite() &&
      input.velocity_limit.allFinite() && input.acceleration_limit.allFinite() &&
      std::isfinite(input.dt) && input.dt > 0.0 &&
      config.translation_gain >= 0.0 && config.rotation_gain >= 0.0 &&
      config.max_linear_velocity > 0.0 && config.max_angular_velocity > 0.0 &&
      config.translation_weight > 0.0 && config.rotation_weight > 0.0 &&
      config.regularization > 0.0 &&
      config.velocity_smoothing_weight >= 0.0 &&
      config.posture_weight >= 0.0 &&
      config.posture_gain >= 0.0 && config.soft_limit_gain >= 0.0 &&
      config.soft_limit_margin > 0.0 && config.singular_value_threshold > 0.0 &&
      config.singular_regularization >= 0.0;
}

inline Eigen::Vector3d clampNorm(const Eigen::Vector3d& value, double limit) {
  const double norm = value.norm();
  if (norm <= limit || norm < 1e-15) return value;
  return value * (limit / norm);
}

inline JointVector updatePostureReference(
    const JointVector& reference, const JointVector& measured_position,
    double maximum_rate, double dt) {
  if (!reference.allFinite() || !measured_position.allFinite() ||
      !std::isfinite(maximum_rate) || maximum_rate < 0.0 ||
      !std::isfinite(dt) || dt <= 0.0) {
    return reference;
  }
  const double maximum_step = maximum_rate * dt;
  return reference +
      (measured_position - reference)
          .cwiseMax(JointVector::Constant(-maximum_step))
          .cwiseMin(JointVector::Constant(maximum_step));
}

inline DifferentialIkResult solveConstrainedDifferentialIk(
    const DifferentialIkInput& input, const DifferentialIkConfig& config) {
  DifferentialIkResult output;
  if (!validDifferentialIkInput(input, config)) {
    output.reason = "invalid or non-finite QP input";
    return output;
  }

  output.desired_twist.head<3>() = clampNorm(
      config.translation_gain * input.pose_error.head<3>(),
      config.max_linear_velocity);
  output.desired_twist.tail<3>() = clampNorm(
      config.rotation_gain * input.pose_error.tail<3>(),
      config.max_angular_velocity);

  JointVector raw_posture_velocity = JointVector::Zero();
  for (int i = 0; i < kRm75Dof; ++i) {
    if (!(input.position_lower[i] < input.position_upper[i]) ||
        input.velocity_limit[i] <= 0.0 || input.acceleration_limit[i] <= 0.0 ||
        input.measured_position[i] < input.position_lower[i] - 1e-9 ||
        input.measured_position[i] > input.position_upper[i] + 1e-9) {
      output.status = BoxQpStatus::INVALID_INPUT;
      output.reason = "invalid joint limit or measured joint outside hard bounds";
      return output;
    }
    raw_posture_velocity[i] = config.posture_gain *
        (input.posture_reference[i] - input.measured_position[i]);
    const double lower_distance =
        input.measured_position[i] - input.position_lower[i];
    const double upper_distance =
        input.position_upper[i] - input.measured_position[i];
    if (lower_distance < config.soft_limit_margin) {
      raw_posture_velocity[i] += config.soft_limit_gain *
          (config.soft_limit_margin - lower_distance) /
          config.soft_limit_margin;
    }
    if (upper_distance < config.soft_limit_margin) {
      raw_posture_velocity[i] -= config.soft_limit_gain *
          (config.soft_limit_margin - upper_distance) /
          config.soft_limit_margin;
    }

    double lower = -input.velocity_limit[i];
    double upper = input.velocity_limit[i];
    lower = std::max(lower, input.previous_velocity[i] -
                              input.acceleration_limit[i] * input.dt);
    upper = std::min(upper, input.previous_velocity[i] +
                              input.acceleration_limit[i] * input.dt);
    // The command is integrated from the last safe command, while the
    // Jacobian is evaluated at the latest measured configuration.
    lower = std::max(lower, (input.position_lower[i] -
                             input.previous_command[i]) / input.dt);
    upper = std::min(upper, (input.position_upper[i] -
                             input.previous_command[i]) / input.dt);

    // Velocity damper: inside the soft margin, motion toward the nearer hard
    // stop decreases linearly and reaches zero at that stop.
    if (lower_distance < config.soft_limit_margin) {
      const double scale = std::max(0.0, lower_distance /
                                             config.soft_limit_margin);
      lower = std::max(lower, -input.velocity_limit[i] * scale);
    }
    if (upper_distance < config.soft_limit_margin) {
      const double scale = std::max(0.0, upper_distance /
                                             config.soft_limit_margin);
      upper = std::min(upper, input.velocity_limit[i] * scale);
    }
    output.lower_bound[i] = lower;
    output.upper_bound[i] = upper;
    if (lower > upper + 1e-12) {
      output.status = BoxQpStatus::INFEASIBLE_BOUNDS;
      output.reason = "position/velocity/acceleration bounds are infeasible";
      return output;
    }
  }

  Eigen::JacobiSVD<GeometricJacobian> svd(input.jacobian,
                                           Eigen::ComputeFullV);
  output.minimum_singular_value = svd.singularValues().minCoeff();
  const double singular_ratio = std::max(
      0.0, (config.singular_value_threshold - output.minimum_singular_value) /
               config.singular_value_threshold);
  output.adaptive_regularization = config.regularization +
      config.singular_regularization * singular_ratio * singular_ratio;

  // Selective, continuous nullspace projection. The seventh singular-vector
  // direction is always the true redundant direction. A task direction is
  // opened gradually only as its singular value approaches zero. This avoids
  // the posture jump caused by changing the numerical rank at a hard epsilon.
  JointVector nullspace_gains = JointVector::Ones();
  for (int i = 0; i < 6; ++i) {
    const double normalized = std::min(
        1.0, svd.singularValues()[i] / config.singular_value_threshold);
    const double remaining = 1.0 - normalized * normalized;
    nullspace_gains[i] = remaining * remaining;
  }
  output.posture_velocity = svd.matrixV() * nullspace_gains.asDiagonal() *
      svd.matrixV().transpose() * raw_posture_velocity;

  TaskVector weights;
  weights.head<3>().setConstant(config.translation_weight);
  weights.tail<3>().setConstant(config.rotation_weight);
  const TaskVector weighted_twist = weights.array() * output.desired_twist.array();
  const GeometricJacobian weighted_jacobian = weights.asDiagonal() * input.jacobian;
  Eigen::Matrix<double, kRm75Dof, kRm75Dof> hessian =
      weighted_jacobian.transpose() * weighted_jacobian;
  hessian.diagonal().array() += output.adaptive_regularization +
                                config.velocity_smoothing_weight +
                                config.posture_weight;
  const JointVector gradient =
      -weighted_jacobian.transpose() * weighted_twist -
      config.velocity_smoothing_weight * input.previous_velocity -
      config.posture_weight * output.posture_velocity;

  const BoxQpResult qp = solveBoxQp(
      hessian, gradient, output.lower_bound, output.upper_bound,
      input.previous_velocity.cwiseMax(output.lower_bound)
                             .cwiseMin(output.upper_bound), config.solver);
  output.status = qp.status;
  output.solve_time_sec = qp.solve_time_sec;
  output.kkt_residual = qp.kkt_residual;
  output.iterations = qp.iterations;
  output.reason = boxQpStatusName(qp.status);
  if (qp.status == BoxQpStatus::SUCCESS && qp.solution.size() == kRm75Dof &&
      qp.solution.allFinite()) {
    output.velocity = qp.solution;
  } else if (qp.status == BoxQpStatus::SUCCESS) {
    output.status = BoxQpStatus::NUMERICAL_ERROR;
    output.reason = "QP returned invalid solution dimension or values";
  }
  return output;
}

}  // namespace rm75_touch_teleop
