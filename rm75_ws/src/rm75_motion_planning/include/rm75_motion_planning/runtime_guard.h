#pragma once

#include <string>

#include "rm75_motion_planning/trajectory_validator.h"

namespace rm75_motion_planning {

enum class GuardDecision { ALLOW, WARN, STOP };

struct RuntimeGuardConfig {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  double joint_limit_margin_rad{0.02};
  double warning_limit_margin_rad{0.05};
  double max_tracking_error_rad{0.03};
  double warning_tracking_ratio{0.8};
  double feedback_timeout_sec{0.10};
};

struct RuntimeGuardInput {
  Eigen::VectorXd current;
  Eigen::VectorXd next_target;
  Eigen::VectorXd previous_target;
  double now_sec{0.0};
  double feedback_stamp_sec{0.0};
  // When supplied, the guard also checks q_current + dq_command * cycle_sec.
  // An empty vector keeps backwards compatibility and treats next_target as
  // the already-predicted next-cycle state.
  Eigen::VectorXd commanded_velocity;
  double cycle_sec{0.02};
};

struct RuntimeGuardResult {
  GuardDecision decision{GuardDecision::STOP};
  bool allow{false};
  bool collision{false};
  bool timeout{false};
  bool tracking_error{false};
  bool joint_limit{false};
  bool invalid_input{false};
  std::string reason;
  double max_joint_error{0.0};
  double max_tracking_error_rad{0.0};
  double min_limit_margin_rad{0.0};
  bool collision_checked{false};
};

class RuntimeGuard {
 public:
  RuntimeGuard(RuntimeGuardConfig config,
               const CollisionChecker* collision_checker);
  RuntimeGuardResult evaluate(const RuntimeGuardInput& input) const;
  const RuntimeGuardConfig& config() const { return config_; }

 private:
  RuntimeGuardConfig config_;
  const CollisionChecker* collision_checker_;
};

const char* toString(GuardDecision decision);

// Pure bookkeeping used after a stop request.  Distance is the maximum joint
// displacement from the measured state at which stop was requested.
class StopRecorder {
 public:
  void request(double time_sec, const Eigen::VectorXd& joints);
  void observe(double time_sec, const Eigen::VectorXd& joints);
  bool active() const { return active_; }
  double elapsedSec() const { return elapsed_sec_; }
  // Time of the last observed inter-sample motion above 1e-5 rad. This is an
  // estimate bounded by the feedback period, unlike elapsedSec(), which is the
  // complete observation window.
  double stopLatencySec() const { return stop_latency_sec_; }
  double maxJointDistanceRad() const { return max_distance_rad_; }

 private:
  bool active_{false};
  double start_sec_{0.0};
  double elapsed_sec_{0.0};
  double stop_latency_sec_{0.0};
  double max_distance_rad_{0.0};
  Eigen::VectorXd start_joints_;
  Eigen::VectorXd previous_joints_;
};

}  // namespace rm75_motion_planning
