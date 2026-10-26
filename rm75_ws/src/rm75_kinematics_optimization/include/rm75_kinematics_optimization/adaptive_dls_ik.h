#pragma once

#include "rm75_kinematics_optimization/kinematics_model.h"
#include "rm75_kinematics_optimization/nullspace_optimizer.h"

namespace rm75_kinematics {

class AdaptiveDlsIk {
 public:
  explicit AdaptiveDlsIk(const KinematicsModel& model) : model_(model) {}
  IkResult solve(const Pose& target, const Eigen::VectorXd& seed,
                 const IkOptions& options = IkOptions()) const;
  static double adaptiveDamping(double sigma_min, const IkOptions& options);
 private:
  const KinematicsModel& model_;
};

}  // namespace rm75_kinematics
