#pragma once

#include <Eigen/Core>
#include "rm75_kinematics_optimization/types.h"

namespace rm75_kinematics {

struct NullspaceDebug {
  Eigen::VectorXd secondary_velocity;
  Eigen::VectorXd projected_velocity;
  double min_limit_margin{0.0};
  double min_singular_value{0.0};
  double manipulability{0.0};
  double primary_leakage_norm{0.0};
  bool soft_limit_active{false};
};

class NullspaceOptimizer {
 public:
  static Eigen::VectorXd jointCenteringGradient(const Eigen::VectorXd& q,
                                                 const JointLimits& limits,
                                                 double soft_margin,
                                                 bool* soft_limit_active = nullptr);
  static Eigen::VectorXd project(const Eigen::MatrixXd& jacobian,
                                 const Eigen::MatrixXd& damped_pseudoinverse,
                                 const Eigen::VectorXd& secondary,
                                 NullspaceDebug* debug = nullptr);
  static double minimumLimitMargin(const Eigen::VectorXd& q,
                                   const JointLimits& limits);
};

}  // namespace rm75_kinematics
