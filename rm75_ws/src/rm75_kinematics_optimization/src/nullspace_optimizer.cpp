#include "rm75_kinematics_optimization/nullspace_optimizer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <Eigen/SVD>

namespace rm75_kinematics {

double NullspaceOptimizer::minimumLimitMargin(const Eigen::VectorXd& q,
                                               const JointLimits& limits) {
  if (q.size() != limits.lower.size() || q.size() != limits.upper.size())
    throw std::invalid_argument("Joint limit size mismatch");
  return (q - limits.lower).cwiseMin(limits.upper - q).minCoeff();
}

Eigen::VectorXd NullspaceOptimizer::jointCenteringGradient(
    const Eigen::VectorXd& q, const JointLimits& limits, double soft_margin,
    bool* active) {
  if (q.size() != limits.lower.size() || q.size() != limits.upper.size() ||
      !q.allFinite() || soft_margin <= 0.0)
    throw std::invalid_argument("Invalid joint-centering input");
  Eigen::VectorXd gradient(q.size());
  bool triggered = false;
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const double range = limits.upper[i] - limits.lower[i];
    const double mid = 0.5 * (limits.upper[i] + limits.lower[i]);
    const double half = 0.5 * range;
    const double normalized = (q[i] - mid) / half;
    // Negative gradient of 0.5*((q-mid)/half)^2, continuous everywhere.
    gradient[i] = -(q[i] - mid) / (half * half);
    const double margin = std::min(q[i] - limits.lower[i], limits.upper[i] - q[i]);
    if (margin < soft_margin * range) {
      triggered = true;
      // Smooth reciprocal barrier derivative, bounded away from the hard limit.
      const double dl = std::max(q[i] - limits.lower[i], 1e-4);
      const double du = std::max(limits.upper[i] - q[i], 1e-4);
      gradient[i] += std::min(50.0, 0.002 / (dl * dl))
                   - std::min(50.0, 0.002 / (du * du));
    }
    (void)normalized;
  }
  if (active) *active = triggered;
  return gradient;
}

Eigen::VectorXd NullspaceOptimizer::project(
    const Eigen::MatrixXd& j, const Eigen::MatrixXd& pinv,
    const Eigen::VectorXd& secondary, NullspaceDebug* debug) {
  if (j.cols() != secondary.size() || pinv.rows() != j.cols() ||
      pinv.cols() != j.rows() || !j.allFinite() || !pinv.allFinite() ||
      !secondary.allFinite()) throw std::invalid_argument("Invalid nullspace input");
  const Eigen::MatrixXd n = Eigen::MatrixXd::Identity(j.cols(), j.cols()) - pinv * j;
  const Eigen::VectorXd projected = n * secondary;
  if (debug) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
    const auto s = svd.singularValues();
    debug->secondary_velocity = secondary;
    debug->projected_velocity = projected;
    debug->min_singular_value = s.size() ? s.minCoeff() : 0.0;
    debug->manipulability = s.size() ? s.array().max(0.0).prod() : 0.0;
    debug->primary_leakage_norm = (j * projected).norm();
  }
  return projected;
}

}  // namespace rm75_kinematics
