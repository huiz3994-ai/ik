#include "rm75_motion_planning/trajectory_validator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace rm75_motion_planning {

ValidationReport TrajectoryValidator::validate(
    const PoseVector& reference_poses, const JointPath& joint_path,
    const TimedTrajectory& trajectory, const DynamicLimits& dynamic_limits,
    double position_tolerance_m, double orientation_tolerance_rad,
    double jump_limit_rad, bool require_collision_check) const {
  if (reference_poses.empty() || reference_poses.size() != joint_path.size())
    throw std::invalid_argument("reference poses must match joint path");
  if (trajectory.empty()) throw std::invalid_argument("timed trajectory is empty");
  const Eigen::Index dof = model_.dof();
  ValidationReport report;
  report.max_velocity = Eigen::VectorXd::Zero(dof);
  report.max_acceleration = Eigen::VectorXd::Zero(dof);
  report.max_jerk = Eigen::VectorXd::Zero(dof);
  report.min_joint_limit_margin_rad = std::numeric_limits<double>::infinity();
  const auto limits = model_.limits();

  for (std::size_t i = 0; i < joint_path.size(); ++i) {
    if (joint_path[i].size() != dof || !joint_path[i].allFinite()) {
      report.violations.push_back("non-finite or wrong-size joint waypoint");
      continue;
    }
    const auto actual = model_.forward(joint_path[i]);
    report.max_position_error_m = std::max(
        report.max_position_error_m,
        (actual.position - reference_poses[i].translation()).norm());
    Eigen::Quaterniond qa(actual.rotation), qr(reference_poses[i].rotation());
    qa.normalize(); qr.normalize();
    report.max_orientation_error_rad = std::max(
        report.max_orientation_error_rad, qa.angularDistance(qr));
    report.min_joint_limit_margin_rad = std::min(
        report.min_joint_limit_margin_rad,
        std::min((joint_path[i] - limits.lower).minCoeff(),
                 (limits.upper - joint_path[i]).minCoeff()));
    if (i > 0)
      report.max_joint_jump_rad = std::max(
          report.max_joint_jump_rad, (joint_path[i] - joint_path[i - 1]).norm());
  }

  double previous_time = -1.0;
  for (std::size_t sample = 0; sample < trajectory.size(); ++sample) {
    const auto& point = trajectory[sample];
    if (point.position.size() != dof || point.velocity.size() != dof ||
        point.acceleration.size() != dof || point.jerk.size() != dof ||
        !point.position.allFinite() || !point.velocity.allFinite() ||
        !point.acceleration.allFinite() || !point.jerk.allFinite() ||
        point.time <= previous_time) {
      report.violations.push_back("invalid timed trajectory sample");
      continue;
    }
    previous_time = point.time;
    report.max_velocity = report.max_velocity.cwiseMax(point.velocity.cwiseAbs());
    report.max_acceleration =
        report.max_acceleration.cwiseMax(point.acceleration.cwiseAbs());
    report.max_jerk = report.max_jerk.cwiseMax(point.jerk.cwiseAbs());
    report.min_joint_limit_margin_rad = std::min(
        report.min_joint_limit_margin_rad,
        std::min((point.position - limits.lower).minCoeff(),
                 (limits.upper - point.position).minCoeff()));
    if (collision_checker_) {
      report.collision_checked = true;
      ++report.collision_checks;
      std::string detail;
      if (collision_checker_->inCollision(point.position, &detail)) {
        if (report.first_collision.empty())
          report.first_collision = "sample " + std::to_string(sample) +
                                   ": " + detail;
      }
    }
  }

  if (report.max_position_error_m > position_tolerance_m)
    report.violations.push_back("FK position tolerance exceeded");
  if (report.max_orientation_error_rad > orientation_tolerance_rad)
    report.violations.push_back("FK orientation tolerance exceeded");
  if ((report.max_velocity.array() >
       dynamic_limits.max_velocity.array() + 1e-10).any())
    report.violations.push_back("velocity limit exceeded");
  if ((report.max_acceleration.array() >
       dynamic_limits.max_acceleration.array() + 1e-10).any())
    report.violations.push_back("acceleration limit exceeded");
  if ((report.max_jerk.array() >
       dynamic_limits.max_jerk.array() + 1e-10).any())
    report.violations.push_back("jerk limit exceeded");
  if (report.min_joint_limit_margin_rad < -1e-10)
    report.violations.push_back("joint position limit exceeded");
  if (report.max_joint_jump_rad > jump_limit_rad)
    report.violations.push_back("joint jump limit exceeded");
  report.collision_free = report.collision_checked && report.first_collision.empty();
  if (!report.first_collision.empty())
    report.violations.push_back("collision: " + report.first_collision);
  if (require_collision_check && !report.collision_checked)
    report.violations.push_back("collision checker required but unavailable");
  report.valid = report.violations.empty();
  return report;
}

}  // namespace rm75_motion_planning
