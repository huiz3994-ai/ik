#include "rm75_motion_planning/se3_interpolator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm75_motion_planning {

namespace {
void validatePose(const Pose& pose) {
  if (!pose.matrix().allFinite()) throw std::invalid_argument("pose contains NaN/Inf");
  const Eigen::Matrix3d should_be_identity =
      pose.rotation().transpose() * pose.rotation();
  if (!should_be_identity.isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
      std::abs(pose.rotation().determinant() - 1.0) > 1e-8)
    throw std::invalid_argument("pose rotation is not in SO(3)");
}
}  // namespace

Eigen::Vector3d Se3Interpolator::so3Log(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) throw std::invalid_argument("rotation contains NaN/Inf");
  Eigen::Quaterniond q(rotation);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sin_half = q.vec().norm();
  if (sin_half < 1e-12) return 2.0 * q.vec();
  return (2.0 * std::atan2(sin_half, q.w()) / sin_half) * q.vec();
}

Eigen::Matrix3d Se3Interpolator::so3Exp(const Eigen::Vector3d& vector) {
  if (!vector.allFinite()) throw std::invalid_argument("rotation vector contains NaN/Inf");
  const double angle = vector.norm();
  if (angle < 1e-12) {
    Eigen::Matrix3d skew;
    skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
        -vector.y(), vector.x(), 0.0;
    return Eigen::Matrix3d::Identity() + skew;
  }
  return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

double Se3Interpolator::rotationDistance(const Eigen::Matrix3d& a,
                                         const Eigen::Matrix3d& b) {
  return so3Log(a.transpose() * b).norm();
}

Pose Se3Interpolator::interpolate(const Pose& start, const Pose& goal,
                                  double alpha) {
  validatePose(start);
  validatePose(goal);
  if (!std::isfinite(alpha)) throw std::invalid_argument("alpha is not finite");
  alpha = std::max(0.0, std::min(1.0, alpha));
  Pose result = Pose::Identity();
  result.translation() =
      (1.0 - alpha) * start.translation() + alpha * goal.translation();
  Eigen::Quaterniond q0(start.rotation()), q1(goal.rotation());
  q0.normalize();
  q1.normalize();
  result.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  return result;
}

PoseVector Se3Interpolator::sample(const Pose& start, const Pose& goal,
                                   const SamplingOptions& options) {
  if (!(options.max_translation_step > 0.0) ||
      !(options.max_rotation_step > 0.0))
    throw std::invalid_argument("sampling steps must be positive");
  validatePose(start);
  validatePose(goal);
  const double distance = (goal.translation() - start.translation()).norm();
  const double angle = rotationDistance(start.rotation(), goal.rotation());
  const std::size_t translation_segments = static_cast<std::size_t>(
      std::ceil(distance / options.max_translation_step));
  const std::size_t rotation_segments = static_cast<std::size_t>(
      std::ceil(angle / options.max_rotation_step));
  const std::size_t segments =
      std::max<std::size_t>(1, std::max(translation_segments, rotation_segments));
  PoseVector result;
  result.reserve(segments + 1);
  for (std::size_t i = 0; i <= segments; ++i)
    result.push_back(interpolate(start, goal,
                                 static_cast<double>(i) / segments));
  return result;
}

}  // namespace rm75_motion_planning
