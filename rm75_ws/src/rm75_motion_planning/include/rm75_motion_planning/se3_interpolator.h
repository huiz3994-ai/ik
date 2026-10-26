#pragma once

#include "rm75_motion_planning/types.h"

namespace rm75_motion_planning {

class Se3Interpolator {
 public:
  static Pose interpolate(const Pose& start, const Pose& goal, double alpha);
  static PoseVector sample(const Pose& start, const Pose& goal,
                           const SamplingOptions& options = SamplingOptions());
  static Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation);
  static Eigen::Matrix3d so3Exp(const Eigen::Vector3d& rotation_vector);
  static double rotationDistance(const Eigen::Matrix3d& a,
                                 const Eigen::Matrix3d& b);
};

// Convert a target expressed in a user frame and at a TCP into the flange pose
// expected by the bare RM75 kinematic model.
inline Pose targetTcpToBaseFlange(const Pose& base_T_user,
                                  const Pose& user_T_tcp,
                                  const Pose& flange_T_tcp) {
  return base_T_user * user_T_tcp * flange_T_tcp.inverse();
}

}  // namespace rm75_motion_planning
