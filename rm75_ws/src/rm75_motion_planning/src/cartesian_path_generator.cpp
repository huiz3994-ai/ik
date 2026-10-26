#include "rm75_motion_planning/cartesian_path_generator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm75_motion_planning {

PoseVector CartesianPathGenerator::line(const Pose& start, const Pose& goal,
                                        const SamplingOptions& options) {
  return Se3Interpolator::sample(start, goal, options);
}

PoseVector CartesianPathGenerator::circularArc(
    const Pose& start, const Pose& via, const Pose& goal,
    const SamplingOptions& options) {
  if (!(options.max_translation_step > 0.0) ||
      !(options.max_rotation_step > 0.0))
    throw std::invalid_argument("sampling steps must be positive");
  const Eigen::Vector3d a = via.translation() - start.translation();
  const Eigen::Vector3d b = goal.translation() - start.translation();
  const Eigen::Vector3d cross = a.cross(b);
  const double cross_squared = cross.squaredNorm();
  if (cross_squared < 1e-14)
    throw std::invalid_argument("arc points are collinear or coincident");

  const Eigen::Vector3d center = start.translation() +
      (a.squaredNorm() * b.cross(cross) +
       b.squaredNorm() * cross.cross(a)) / (2.0 * cross_squared);
  const double radius = (start.translation() - center).norm();
  const Eigen::Vector3d e1 = (start.translation() - center) / radius;
  const Eigen::Vector3d normal = cross.normalized();
  const Eigen::Vector3d e2 = normal.cross(e1);
  auto angle_of = [&](const Eigen::Vector3d& point) {
    const Eigen::Vector3d r = (point - center) / radius;
    double angle = std::atan2(r.dot(e2), r.dot(e1));
    if (angle < 0.0) angle += 2.0 * M_PI;
    return angle;
  };
  double via_angle = angle_of(via.translation());
  double goal_angle = angle_of(goal.translation());
  double total_angle;
  if (via_angle <= goal_angle) {
    total_angle = goal_angle;
  } else {
    total_angle = goal_angle - 2.0 * M_PI;
    via_angle -= 2.0 * M_PI;
  }
  const auto segment_count = [&](double arc_angle,
                                 const Eigen::Matrix3d& r0,
                                 const Eigen::Matrix3d& r1) {
    return std::max<std::size_t>(
        1, std::max(static_cast<std::size_t>(std::ceil(
                        radius * std::abs(arc_angle) /
                        options.max_translation_step)),
                    static_cast<std::size_t>(std::ceil(
                        Se3Interpolator::rotationDistance(r0, r1) /
                        options.max_rotation_step))));
  };
  const std::size_t first_segments =
      segment_count(via_angle, start.rotation(), via.rotation());
  const std::size_t second_segments = segment_count(
      total_angle - via_angle, via.rotation(), goal.rotation());
  PoseVector result;
  result.reserve(first_segments + second_segments + 1);
  auto append = [&](const Pose& orientation_start, const Pose& orientation_goal,
                    double angle_start, double angle_delta,
                    std::size_t segments, std::size_t first_index) {
   for (std::size_t i = first_index; i <= segments; ++i) {
    const double alpha = static_cast<double>(i) / segments;
    const double theta = angle_start + alpha * angle_delta;
    Pose pose = Pose::Identity();
    pose.translation() = center + radius * (std::cos(theta) * e1 +
                                             std::sin(theta) * e2);
    pose.linear() = Se3Interpolator::interpolate(
                        orientation_start, orientation_goal, alpha).rotation();
    result.push_back(pose);
   }
  };
  append(start, via, 0.0, via_angle, first_segments, 0);
  append(via, goal, via_angle, total_angle - via_angle, second_segments, 1);
  result.front() = start;
  result[first_segments] = via;
  result.back() = goal;
  return result;
}

PoseVector CartesianPathGenerator::multiWaypoint(
    const PoseVector& waypoints, const SamplingOptions& options) {
  if (waypoints.size() < 2)
    throw std::invalid_argument("at least two waypoints are required");
  PoseVector result;
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    PoseVector segment = line(waypoints[i - 1], waypoints[i], options);
    if (!result.empty()) segment.erase(segment.begin());
    result.insert(result.end(), segment.begin(), segment.end());
  }
  return result;
}

}  // namespace rm75_motion_planning
