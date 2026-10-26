#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace rm75_touch_teleop {

// Geomagic Touch -> robot base coordinates, matching the working Unitree Z1
// teleoperation implementation: robot X=-Touch Z, Y=-Touch X, Z=Touch Y.
inline Eigen::Matrix3d touchToRobotBasis() {
  Eigen::Matrix3d basis;
  basis <<  0.0, 0.0, -1.0,
           -1.0, 0.0,  0.0,
            0.0, 1.0,  0.0;
  return basis;
}

inline Eigen::Vector3d mapTranslation(const Eigen::Vector3d& touch_delta,
                                      double metres_per_touch_unit) {
  return metres_per_touch_unit * touchToRobotBasis() * touch_delta;
}

inline Eigen::Vector3d applyTranslationAxisMask(
    Eigen::Vector3d robot_delta, bool enable_x, bool enable_y, bool enable_z) {
  if (!enable_x) robot_delta.x() = 0.0;
  if (!enable_y) robot_delta.y() = 0.0;
  if (!enable_z) robot_delta.z() = 0.0;
  return robot_delta;
}

// Bound an operator-requested Cartesian axis by both a velocity-derived step
// and an absolute per-cycle step.  HybridForceControl already applies these
// limits in CONTACT_MODE; the Z-only Web path also needs them while approaching
// in POSITION_MODE so a large or fast Touch movement cannot become a jump.
inline double limitAxisStep(double current, double requested,
                            double max_velocity, double max_step, double dt) {
  const double allowed = std::min(max_step, max_velocity * std::max(0.0, dt));
  return current + std::max(-allowed, std::min(allowed, requested - current));
}

inline double clampAxisTravel(double requested, double origin,
                              double max_travel) {
  if (max_travel <= 0.0) return requested;
  return std::max(origin - max_travel,
                  std::min(origin + max_travel, requested));
}

inline Eigen::Matrix3d mapRelativeRotation(const Eigen::Matrix3d& touch_delta) {
  const Eigen::Matrix3d basis = touchToRobotBasis();
  return basis * touch_delta * basis.transpose();
}

}  // namespace rm75_touch_teleop
