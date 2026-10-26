#pragma once

#include "rm75_motion_planning/types.h"
#include "rm75_kinematics_optimization/kinematics_model.h"

namespace rm75_motion_planning {

class CollisionChecker {
 public:
  virtual ~CollisionChecker() = default;
  virtual bool inCollision(const Eigen::VectorXd& q,
                           std::string* detail = nullptr) const = 0;
};

class TrajectoryValidator {
 public:
  explicit TrajectoryValidator(const rm75_kinematics::KinematicsModel& model,
                               const CollisionChecker* collision_checker = nullptr)
      : model_(model), collision_checker_(collision_checker) {}
  ValidationReport validate(const PoseVector& reference_poses,
                            const JointPath& joint_path,
                            const TimedTrajectory& trajectory,
                            const DynamicLimits& dynamic_limits,
                            double position_tolerance_m = 0.002,
                            double orientation_tolerance_rad = 0.034906585,
                            double jump_limit_rad = 0.35,
                            bool require_collision_check = false) const;

 private:
  const rm75_kinematics::KinematicsModel& model_;
  const CollisionChecker* collision_checker_;
};

}  // namespace rm75_motion_planning
