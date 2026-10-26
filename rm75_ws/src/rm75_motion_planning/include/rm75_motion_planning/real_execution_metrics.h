#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "rm75_motion_planning/types.h"

namespace rm75_motion_planning {

struct DistributionSummary {
  std::size_t count{0};
  double mean{0.0};
  double rms{0.0};
  double maximum{0.0};
  double p95{0.0};
  double p99{0.0};
  double standard_deviation{0.0};
};

struct PoseError {
  double position_m{0.0};
  double orientation_rad{0.0};
};

// Pure helpers used by both the real-execution recorder and its GTests.
DistributionSummary summarizeDistribution(const std::vector<double>& values);
PoseError measurePoseError(const Pose& desired, const Pose& actual);
bool timestampsAreConsistent(const std::vector<double>& sample_times,
                             const std::vector<double>& command_times,
                             const std::vector<double>& feedback_times,
                             std::string* reason = nullptr);

struct RealExecutionSample {
  std::string run_id;
  std::string scenario;
  int repetition{0};
  double requested_linear_speed_mps{0.0};
  double requested_angular_speed_rad_s{0.0};
  double sample_time_sec{0.0};
  double command_time_sec{0.0};
  // feedback_time_sec is the local WallTime receipt timestamp used by the
  // runtime guard and response-alignment analysis. The source timestamp is
  // kept separately because ROS and WallTime are not assumed synchronized.
  double feedback_time_sec{0.0};
  double feedback_source_time_sec{0.0};
  bool feedback_source_time_valid{false};
  bool command_published{false};
  Pose ideal_pose{Pose::Identity()};
  Pose commanded_pose{Pose::Identity()};
  Pose actual_pose{Pose::Identity()};
  Eigen::VectorXd target_joints;
  Eigen::VectorXd target_velocity;
  Eigen::VectorXd target_acceleration;
  Eigen::VectorXd actual_joints;
  double force_n{0.0};
  double torque_nm{0.0};
  std::string controller_status{"OK"};
  std::string guard_result{"ALLOW"};
  bool collision{false};
  std::string stop_reason;
};

// Creates a raw, one-row-per-control-cycle CSV. It never talks to ROS and
// cannot command hardware. The caller owns run lifecycle and supervision.
class RealExecutionCsvWriter {
 public:
  explicit RealExecutionCsvWriter(const std::string& path);
  ~RealExecutionCsvWriter();
  RealExecutionCsvWriter(const RealExecutionCsvWriter&) = delete;
  RealExecutionCsvWriter& operator=(const RealExecutionCsvWriter&) = delete;

  void append(const RealExecutionSample& sample);
  std::size_t rowsWritten() const { return rows_written_; }

 private:
  class Impl;
  Impl* impl_;
  std::size_t rows_written_{0};
};

// Small dependency-free JSON writer for an individual run sidecar. Aggregate
// reports and calibration remain the responsibility of the offline analyzer.
void writeRealExecutionRunJson(const std::string& path,
                               const std::string& run_id,
                               const std::string& scenario,
                               const std::string& status,
                               std::size_t samples,
                               const PoseError& final_error,
                               const std::string& stop_reason,
                               double stop_latency_sec = -1.0,
                               double stop_distance_max_joint_rad = -1.0);

}  // namespace rm75_motion_planning
