#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace rm75_motion_planning {

using Pose = Eigen::Isometry3d;
using PoseVector = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
using JointPath = std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>>;

struct SamplingOptions {
  double max_translation_step{0.005};
  double max_rotation_step{0.05};
};

struct ContinuousIkOptions {
  double max_joint_jump{0.35};
  int max_refinement_depth{4};
  bool enable_ipopt_fallback{true};
};

struct ContinuousIkResult {
  bool success{false};
  JointPath joints;
  PoseVector solved_poses;
  int dls_calls{0};
  int refinements{0};
  int fallback_calls{0};
  std::string failure_reason;
};

struct DynamicLimits {
  Eigen::VectorXd max_velocity;
  Eigen::VectorXd max_acceleration;
  Eigen::VectorXd max_jerk;
};

struct TrajectoryPoint {
  double time{0.0};
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

using TimedTrajectory =
    std::vector<TrajectoryPoint, Eigen::aligned_allocator<TrajectoryPoint>>;

struct ValidationReport {
  bool valid{false};
  double max_position_error_m{0.0};
  double max_orientation_error_rad{0.0};
  Eigen::VectorXd max_velocity;
  Eigen::VectorXd max_acceleration;
  Eigen::VectorXd max_jerk;
  double min_joint_limit_margin_rad{0.0};
  double max_joint_jump_rad{0.0};
  bool collision_checked{false};
  bool collision_free{false};
  std::size_t collision_checks{0};
  std::string first_collision;
  std::vector<std::string> violations;
};

}  // namespace rm75_motion_planning
