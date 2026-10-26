#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <string>

namespace rm75_kinematics {

using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class IkStatus {
  SUCCESS, MAX_ITERATIONS, TIMEOUT, JOINT_LIMIT, SINGULARITY,
  NUMERICAL_ERROR, UNREACHABLE_OR_STALLED
};

inline const char* toString(IkStatus status) {
  switch (status) {
    case IkStatus::SUCCESS: return "SUCCESS";
    case IkStatus::MAX_ITERATIONS: return "MAX_ITERATIONS";
    case IkStatus::TIMEOUT: return "TIMEOUT";
    case IkStatus::JOINT_LIMIT: return "JOINT_LIMIT";
    case IkStatus::SINGULARITY: return "SINGULARITY";
    case IkStatus::NUMERICAL_ERROR: return "NUMERICAL_ERROR";
    default: return "UNREACHABLE_OR_STALLED";
  }
}

struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Isometry3d isometry() const {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = rotation; t.translation() = position; return t;
  }
};

struct JointLimits {
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
};

struct IkOptions {
  int max_iterations{150};
  double timeout_sec{0.05};
  double position_tolerance{0.002};
  double orientation_tolerance{0.034906585};
  double position_weight{1.0};
  double orientation_weight{0.25};
  bool orientation_enabled{true};
  double damping_min{1e-4};
  double damping_max{0.12};
  double singular_value_threshold{0.08};
  double singularity_failure_threshold{1e-7};
  double max_joint_step{0.12};
  double min_line_search_scale{1.0 / 128.0};
  int stall_iterations{12};
  bool enable_joint_centering{false};
  double joint_centering_gain{0.08};
  bool enable_joint_reference{false};
  double joint_reference_gain{0.005};
  bool enable_singularity_avoidance{false};
  double manipulability_gain{0.00005};
  double manipulability_fd_step{1e-4};
  double soft_limit_margin{0.12};
};

struct IkResult {
  IkStatus status{IkStatus::NUMERICAL_ERROR};
  Eigen::VectorXd q;
  int iterations{0};
  double solve_time_ms{0.0};
  double position_error{0.0};
  double orientation_error{0.0};
  double min_singular_value{0.0};
  double damping{0.0};
  double min_limit_margin{0.0};
  std::string detail;
};

}  // namespace rm75_kinematics
