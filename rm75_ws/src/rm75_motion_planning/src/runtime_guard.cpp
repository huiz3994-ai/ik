#include "rm75_motion_planning/runtime_guard.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm75_motion_planning {

RuntimeGuard::RuntimeGuard(RuntimeGuardConfig config,
                           const CollisionChecker* collision_checker)
    : config_(std::move(config)), collision_checker_(collision_checker) {
  const int dof = config_.lower.size();
  if (dof <= 0 || config_.upper.size() != dof ||
      !config_.lower.allFinite() || !config_.upper.allFinite() ||
      (config_.lower.array() >= config_.upper.array()).any() ||
      config_.joint_limit_margin_rad < 0.0 ||
      config_.warning_limit_margin_rad < config_.joint_limit_margin_rad ||
      config_.max_tracking_error_rad <= 0.0 ||
      config_.warning_tracking_ratio <= 0.0 ||
      config_.warning_tracking_ratio >= 1.0 ||
      config_.feedback_timeout_sec <= 0.0)
    throw std::invalid_argument("invalid runtime guard configuration");
}

RuntimeGuardResult RuntimeGuard::evaluate(
    const RuntimeGuardInput& input) const {
  RuntimeGuardResult result;
  const int dof = config_.lower.size();
  if (input.current.size() != dof || input.next_target.size() != dof ||
      input.previous_target.size() != dof || !input.current.allFinite() ||
      !input.next_target.allFinite() || !input.previous_target.allFinite() ||
      (input.commanded_velocity.size() != 0 &&
       input.commanded_velocity.size() != dof) ||
      (input.commanded_velocity.size() != 0 &&
       !input.commanded_velocity.allFinite()) ||
      !std::isfinite(input.cycle_sec) || input.cycle_sec <= 0.0 ||
      !std::isfinite(input.now_sec) || !std::isfinite(input.feedback_stamp_sec)) {
    result.invalid_input = true;
    result.reason = "invalid or non-finite guard input";
    return result;
  }
  const double age = input.now_sec - input.feedback_stamp_sec;
  if (age < 0.0 || age > config_.feedback_timeout_sec) {
    result.timeout = true;
    result.reason = "feedback timeout";
    return result;
  }

  result.max_tracking_error_rad =
      (input.current - input.previous_target).cwiseAbs().maxCoeff();
  result.max_joint_error = result.max_tracking_error_rad;
  const Eigen::VectorXd predicted = input.commanded_velocity.size() == dof
      ? input.current + input.commanded_velocity * input.cycle_sec
      : input.next_target;
  const double current_margin = std::min(
      (input.current - config_.lower).minCoeff(),
      (config_.upper - input.current).minCoeff());
  const double next_margin = std::min(
      (input.next_target - config_.lower).minCoeff(),
      (config_.upper - input.next_target).minCoeff());
  const double predicted_margin = std::min(
      (predicted - config_.lower).minCoeff(),
      (config_.upper - predicted).minCoeff());
  result.min_limit_margin_rad =
      std::min(current_margin, std::min(next_margin, predicted_margin));
  if (result.min_limit_margin_rad < config_.joint_limit_margin_rad) {
    result.joint_limit = true;
    result.reason = "joint limit margin violated";
    return result;
  }
  if (result.max_tracking_error_rad > config_.max_tracking_error_rad) {
    result.tracking_error = true;
    result.reason = "joint tracking error exceeded";
    return result;
  }
  if (!collision_checker_) {
    result.reason = "collision checker unavailable";
    return result;
  }
  result.collision_checked = true;
  std::string detail;
  try {
    if (collision_checker_->inCollision(input.current, &detail)) {
      result.collision = true;
      result.reason = "current-state collision: " + detail;
      return result;
    }
    if (collision_checker_->inCollision(input.next_target, &detail)) {
      result.collision = true;
      result.reason = "next-target collision: " + detail;
      return result;
    }
  } catch (const std::exception& error) {
    result.collision_checked = false;
    result.reason = std::string("collision checker failure: ") + error.what();
    return result;
  }

  const bool tracking_warning =
      result.max_tracking_error_rad >=
      config_.warning_tracking_ratio * config_.max_tracking_error_rad;
  const bool limit_warning =
      result.min_limit_margin_rad < config_.warning_limit_margin_rad;
  result.decision = (tracking_warning || limit_warning)
                        ? GuardDecision::WARN
                        : GuardDecision::ALLOW;
  result.allow = true;
  result.reason = tracking_warning ? "tracking error warning"
                                   : (limit_warning ? "joint limit warning"
                                                    : "checks passed");
  return result;
}

const char* toString(GuardDecision decision) {
  switch (decision) {
    case GuardDecision::ALLOW: return "ALLOW";
    case GuardDecision::WARN: return "WARN";
    case GuardDecision::STOP: return "STOP";
  }
  return "STOP";
}

void StopRecorder::request(double time_sec, const Eigen::VectorXd& joints) {
  if (!std::isfinite(time_sec) || joints.size() == 0 || !joints.allFinite())
    throw std::invalid_argument("invalid stop request sample");
  active_ = true;
  start_sec_ = time_sec;
  elapsed_sec_ = 0.0;
  stop_latency_sec_ = 0.0;
  max_distance_rad_ = 0.0;
  start_joints_ = joints;
  previous_joints_ = joints;
}

void StopRecorder::observe(double time_sec, const Eigen::VectorXd& joints) {
  if (!active_) throw std::logic_error("stop was not requested");
  if (!std::isfinite(time_sec) || time_sec < start_sec_ ||
      joints.size() != start_joints_.size() || !joints.allFinite())
    throw std::invalid_argument("invalid stop observation sample");
  elapsed_sec_ = time_sec - start_sec_;
  if ((joints - previous_joints_).cwiseAbs().maxCoeff() > 1e-5)
    stop_latency_sec_ = elapsed_sec_;
  max_distance_rad_ = std::max(
      max_distance_rad_, (joints - start_joints_).cwiseAbs().maxCoeff());
  previous_joints_ = joints;
}

}  // namespace rm75_motion_planning
