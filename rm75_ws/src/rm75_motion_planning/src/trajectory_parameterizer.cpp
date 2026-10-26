#include "rm75_motion_planning/trajectory_parameterizer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm75_motion_planning {

namespace {
void validateLimits(const DynamicLimits& limits, Eigen::Index dof) {
  if (limits.max_velocity.size() != dof ||
      limits.max_acceleration.size() != dof ||
      limits.max_jerk.size() != dof || !limits.max_velocity.allFinite() ||
      !limits.max_acceleration.allFinite() || !limits.max_jerk.allFinite() ||
      (limits.max_velocity.array() <= 0.0).any() ||
      (limits.max_acceleration.array() <= 0.0).any() ||
      (limits.max_jerk.array() <= 0.0).any())
    throw std::invalid_argument("dynamic limits must be finite, positive and match path DOF");
}
}  // namespace

double TrajectoryParameterizer::minimumSegmentDuration(
    const Eigen::VectorXd& delta, const DynamicLimits& limits) {
  validateLimits(limits, delta.size());
  if (!delta.allFinite()) throw std::invalid_argument("joint delta contains NaN/Inf");
  double duration = 0.0;
  // q(u)=q0+d*(10u^3-15u^4+6u^5). Exact maxima:
  // max s'=15/8, max |s''|=10/sqrt(3), max |s'''|=60.
  for (Eigen::Index i = 0; i < delta.size(); ++i) {
    const double distance = std::abs(delta[i]);
    duration = std::max(duration, 1.875 * distance / limits.max_velocity[i]);
    duration = std::max(duration, std::sqrt((10.0 / std::sqrt(3.0)) *
                                            distance / limits.max_acceleration[i]));
    duration = std::max(duration, std::cbrt(60.0 * distance /
                                           limits.max_jerk[i]));
  }
  return duration;
}

TimedTrajectory TrajectoryParameterizer::parameterize(
    const JointPath& path, const DynamicLimits& limits, double control_period) {
  if (path.empty()) throw std::invalid_argument("joint path is empty");
  if (!(control_period > 0.0) || !std::isfinite(control_period))
    throw std::invalid_argument("control period must be finite and positive");
  const Eigen::Index dof = path.front().size();
  validateLimits(limits, dof);
  for (const auto& q : path)
    if (q.size() != dof || !q.allFinite())
      throw std::invalid_argument("joint path dimensions/values are invalid");

  TimedTrajectory output;
  TrajectoryPoint first;
  first.position = path.front();
  first.velocity = Eigen::VectorXd::Zero(dof);
  first.acceleration = Eigen::VectorXd::Zero(dof);
  first.jerk = Eigen::VectorXd::Zero(dof);
  output.push_back(first);
  double elapsed = 0.0;
  for (std::size_t segment = 1; segment < path.size(); ++segment) {
    const Eigen::VectorXd delta = path[segment] - path[segment - 1];
    double duration = minimumSegmentDuration(delta, limits);
    if (duration < 1e-12) continue;
    // Round duration up to an integer control tick. This can only reduce dynamics.
    const std::size_t ticks = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(duration / control_period)));
    duration = ticks * control_period;
    for (std::size_t tick = 1; tick <= ticks; ++tick) {
      const double u = static_cast<double>(tick) / ticks;
      const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
      const double s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
      const double ds = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
      const double dds = (60.0 * u - 180.0 * u2 + 120.0 * u3) /
                         (duration * duration);
      const double ddds = (60.0 - 360.0 * u + 360.0 * u2) /
                          (duration * duration * duration);
      TrajectoryPoint point;
      point.time = elapsed + tick * control_period;
      point.position = path[segment - 1] + s * delta;
      point.velocity = ds * delta;
      point.acceleration = dds * delta;
      point.jerk = ddds * delta;
      output.push_back(std::move(point));
    }
    elapsed += duration;
  }
  if (output.size() == 1 && path.size() > 1) output.back().position = path.back();
  return output;
}

TimedTrajectory TrajectoryParameterizer::parameterizeAtLeastDuration(
    const JointPath& path, const DynamicLimits& limits,
    double minimum_duration, double control_period) {
  if (!std::isfinite(minimum_duration) || minimum_duration < 0.0)
    throw std::invalid_argument("minimum duration must be finite and nonnegative");
  DynamicLimits slowed = limits;
  for (int iteration = 0; iteration < 8; ++iteration) {
    TimedTrajectory trajectory = parameterize(path, slowed, control_period);
    if (trajectory.empty() || trajectory.back().time >= minimum_duration)
      return trajectory;
    if (trajectory.back().time <= 0.0)
      throw std::invalid_argument(
          "nonzero minimum duration requires a moving path");
    // Include one tick of margin because the current duration was itself
    // rounded upward. Scaling from that rounded value can otherwise finish one
    // tick before the requested wall-clock duration.
    const double scale =
        (minimum_duration + control_period) / trajectory.back().time;
    slowed.max_velocity /= scale;
    slowed.max_acceleration /= scale * scale;
    slowed.max_jerk /= scale * scale * scale;
  }
  throw std::runtime_error("failed to satisfy minimum trajectory duration");
}

}  // namespace rm75_motion_planning
