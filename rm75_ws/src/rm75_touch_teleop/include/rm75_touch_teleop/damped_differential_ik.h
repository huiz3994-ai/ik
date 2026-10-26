#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace rm75_touch_teleop {

// One bounded damped-least-squares step.  This is used only as a local
// fallback when the plugin IK solver rejects a small Cartesian update near a
// singular posture.  It intentionally does not attempt a global IK branch
// change.
inline Eigen::VectorXd dampedDifferentialIkStep(
    const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& task_error,
    double damping, double max_joint_step) {
  if (jacobian.rows() != task_error.size() || jacobian.cols() == 0 ||
      !jacobian.allFinite() || !task_error.allFinite() ||
      !std::isfinite(damping) || damping <= 0.0 ||
      !std::isfinite(max_joint_step) || max_joint_step <= 0.0) {
    throw std::runtime_error("Invalid damped differential IK input");
  }

  Eigen::MatrixXd normal = jacobian * jacobian.transpose();
  normal.diagonal().array() += damping * damping;
  Eigen::VectorXd delta = jacobian.transpose() * normal.ldlt().solve(task_error);
  if (!delta.allFinite()) return Eigen::VectorXd::Zero(jacobian.cols());

  const double largest = delta.cwiseAbs().maxCoeff();
  if (largest > max_joint_step) delta *= max_joint_step / largest;
  return delta;
}

}  // namespace rm75_touch_teleop
