#pragma once

#include "rm75_motion_planning/types.h"

namespace rm75_motion_planning {

class TrajectoryParameterizer {
 public:
  // Path-preserving quintic S-curve fallback. Each input waypoint is reached
  // exactly with zero velocity/acceleration; samples are emitted at control_period.
  static TimedTrajectory parameterize(const JointPath& path,
                                      const DynamicLimits& limits,
                                      double control_period = 0.02);
  // Slow a path to at least minimum_duration by lowering its dynamic limits
  // and regenerating fixed-period samples. This preserves the 20 ms execution
  // contract; changing timestamps alone would not slow a fixed-rate publisher.
  static TimedTrajectory parameterizeAtLeastDuration(
      const JointPath& path, const DynamicLimits& limits,
      double minimum_duration, double control_period = 0.02);
  static double minimumSegmentDuration(const Eigen::VectorXd& delta,
                                       const DynamicLimits& limits);
};

}  // namespace rm75_motion_planning
