#pragma once

#include <functional>
#include "rm75_motion_planning/types.h"
#include "rm75_kinematics_optimization/adaptive_dls_ik.h"

namespace rm75_motion_planning {

class ContinuousIkSolver {
 public:
  using Fallback = std::function<rm75_kinematics::IkResult(
      const rm75_kinematics::Pose&, const Eigen::VectorXd&)>;

  explicit ContinuousIkSolver(const rm75_kinematics::KinematicsModel& model,
                              Fallback ipopt_fallback = Fallback());
  ContinuousIkResult solve(
      const PoseVector& poses, const Eigen::VectorXd& initial,
      const ContinuousIkOptions& options = ContinuousIkOptions(),
      const rm75_kinematics::IkOptions& ik_options = rm75_kinematics::IkOptions()) const;

 private:
  bool solveSegment(const Pose& from_pose, const Eigen::VectorXd& from_q,
                    const Pose& target, int depth,
                    const ContinuousIkOptions& options,
                    const rm75_kinematics::IkOptions& ik_options,
                    ContinuousIkResult* result) const;
  static rm75_kinematics::Pose convert(const Pose& pose);
  const rm75_kinematics::KinematicsModel& model_;
  rm75_kinematics::AdaptiveDlsIk dls_;
  Fallback fallback_;
};

}  // namespace rm75_motion_planning
