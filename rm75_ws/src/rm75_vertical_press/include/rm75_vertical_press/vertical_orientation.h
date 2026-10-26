#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace rm75_vertical_press {

// Point Link7's local +Z axis down along base -Z.  The projected local +X
// direction is retained, so pressing the white button removes tool tilt without
// introducing an unnecessary yaw jump.
inline Eigen::Matrix3d verticalDownOrientation(const Eigen::Matrix3d& current) {
  const Eigen::Vector3d down(0.0, 0.0, -1.0);
  Eigen::Vector3d x(current(0, 0), current(1, 0), 0.0);

  if (!x.allFinite() || x.head<2>().norm() < 1e-8) {
    // If local X is vertical, recover a horizontal heading from local Y.
    const Eigen::Vector3d y_projection(current(0, 1), current(1, 1), 0.0);
    x = y_projection.cross(down);
  }
  if (!x.allFinite() || x.head<2>().norm() < 1e-8) {
    x = Eigen::Vector3d::UnitX();
  }
  x.normalize();
  const Eigen::Vector3d y = down.cross(x).normalized();

  Eigen::Matrix3d result;
  result.col(0) = x;
  result.col(1) = y;
  result.col(2) = down;
  return result;
}

// Advance along the shortest quaternion arc without asking IK to solve a large
// orientation jump in one cycle.
inline Eigen::Matrix3d stepTowardOrientation(const Eigen::Matrix3d& current,
                                             const Eigen::Matrix3d& goal,
                                             double max_angle) {
  Eigen::Quaterniond from(current);
  Eigen::Quaterniond to(goal);
  from.normalize();
  to.normalize();
  const double angle = from.angularDistance(to);
  if (!std::isfinite(angle) || angle < 1e-10) return goal;
  const double fraction = std::max(0.0, std::min(1.0, max_angle / angle));
  return from.slerp(fraction, to).normalized().toRotationMatrix();
}

}  // namespace rm75_vertical_press
