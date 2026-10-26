#pragma once

#include "rm75_motion_planning/se3_interpolator.h"

namespace rm75_motion_planning {

class CartesianPathGenerator {
 public:
  static PoseVector line(const Pose& start, const Pose& goal,
                         const SamplingOptions& options = SamplingOptions());
  static PoseVector circularArc(const Pose& start, const Pose& via,
                                const Pose& goal,
                                const SamplingOptions& options = SamplingOptions());
  static PoseVector multiWaypoint(const PoseVector& waypoints,
                                  const SamplingOptions& options = SamplingOptions());
};

}  // namespace rm75_motion_planning
