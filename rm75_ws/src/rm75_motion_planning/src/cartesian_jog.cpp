#include "rm75_motion_planning/cartesian_jog.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace rm75_motion_planning {

bool parseCartesianJogDirection(const std::string& value,
                                CartesianJogDirection* direction) {
  if (!direction) return false;
#define RM75_JOG_CASE(text, item) \
  if (value == text) { *direction = CartesianJogDirection::item; return true; }
  RM75_JOG_CASE("x_plus", X_PLUS)
  RM75_JOG_CASE("x_minus", X_MINUS)
  RM75_JOG_CASE("y_plus", Y_PLUS)
  RM75_JOG_CASE("y_minus", Y_MINUS)
  RM75_JOG_CASE("z_plus", Z_PLUS)
  RM75_JOG_CASE("z_minus", Z_MINUS)
  RM75_JOG_CASE("rx_plus", RX_PLUS)
  RM75_JOG_CASE("rx_minus", RX_MINUS)
  RM75_JOG_CASE("ry_plus", RY_PLUS)
  RM75_JOG_CASE("ry_minus", RY_MINUS)
  RM75_JOG_CASE("rz_plus", RZ_PLUS)
  RM75_JOG_CASE("rz_minus", RZ_MINUS)
#undef RM75_JOG_CASE
  return false;
}

const char* toString(CartesianJogDirection direction) {
  switch (direction) {
    case CartesianJogDirection::X_PLUS: return "x_plus";
    case CartesianJogDirection::X_MINUS: return "x_minus";
    case CartesianJogDirection::Y_PLUS: return "y_plus";
    case CartesianJogDirection::Y_MINUS: return "y_minus";
    case CartesianJogDirection::Z_PLUS: return "z_plus";
    case CartesianJogDirection::Z_MINUS: return "z_minus";
    case CartesianJogDirection::RX_PLUS: return "rx_plus";
    case CartesianJogDirection::RX_MINUS: return "rx_minus";
    case CartesianJogDirection::RY_PLUS: return "ry_plus";
    case CartesianJogDirection::RY_MINUS: return "ry_minus";
    case CartesianJogDirection::RZ_PLUS: return "rz_plus";
    case CartesianJogDirection::RZ_MINUS: return "rz_minus";
  }
  return "unknown";
}

bool isCartesianJogRotation(CartesianJogDirection direction) {
  return direction == CartesianJogDirection::RX_PLUS ||
         direction == CartesianJogDirection::RX_MINUS ||
         direction == CartesianJogDirection::RY_PLUS ||
         direction == CartesianJogDirection::RY_MINUS ||
         direction == CartesianJogDirection::RZ_PLUS ||
         direction == CartesianJogDirection::RZ_MINUS;
}

Pose cartesianJogTarget(const Pose& start, CartesianJogDirection direction,
                        double translation_distance_m,
                        double rotation_angle_rad) {
  if (!start.matrix().allFinite() || !std::isfinite(translation_distance_m) ||
      !std::isfinite(rotation_angle_rad) || translation_distance_m < 0.0 ||
      rotation_angle_rad < 0.0)
    throw std::invalid_argument("invalid Cartesian jog target input");
  Pose target = start;
  Eigen::Vector3d axis = Eigen::Vector3d::Zero();
  double sign = 1.0;
  switch (direction) {
    case CartesianJogDirection::X_PLUS: axis.x() = 1.0; break;
    case CartesianJogDirection::X_MINUS: axis.x() = 1.0; sign = -1.0; break;
    case CartesianJogDirection::Y_PLUS: axis.y() = 1.0; break;
    case CartesianJogDirection::Y_MINUS: axis.y() = 1.0; sign = -1.0; break;
    case CartesianJogDirection::Z_PLUS: axis.z() = 1.0; break;
    case CartesianJogDirection::Z_MINUS: axis.z() = 1.0; sign = -1.0; break;
    case CartesianJogDirection::RX_PLUS: axis.x() = 1.0; break;
    case CartesianJogDirection::RX_MINUS: axis.x() = 1.0; sign = -1.0; break;
    case CartesianJogDirection::RY_PLUS: axis.y() = 1.0; break;
    case CartesianJogDirection::RY_MINUS: axis.y() = 1.0; sign = -1.0; break;
    case CartesianJogDirection::RZ_PLUS: axis.z() = 1.0; break;
    case CartesianJogDirection::RZ_MINUS: axis.z() = 1.0; sign = -1.0; break;
  }
  if (isCartesianJogRotation(direction)) {
    target.linear() = start.rotation() *
        Eigen::AngleAxisd(sign * rotation_angle_rad, axis).toRotationMatrix();
  } else {
    target.translation() += sign * translation_distance_m * axis;
  }
  return target;
}

}  // namespace rm75_motion_planning
