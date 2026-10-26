#include "rm75_motion_planning/real_execution_metrics.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rm75_motion_planning {
namespace {

double percentile(const std::vector<double>& sorted, double probability) {
  if (sorted.empty()) return 0.0;
  const double index = probability * static_cast<double>(sorted.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

std::string csvEscape(const std::string& value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
  std::string escaped = "\"";
  for (const char character : value) {
    escaped += character;
    if (character == '\"') escaped += '\"';
  }
  escaped += '\"';
  return escaped;
}

std::string jsonEscape(const std::string& value) {
  std::ostringstream output;
  for (const unsigned char character : value) {
    switch (character) {
      case '\"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  return output.str();
}

void requireVector7(const Eigen::VectorXd& value, const char* name) {
  if (value.size() != 7 || !value.allFinite())
    throw std::invalid_argument(std::string(name) + " must contain 7 finite values");
}

void writeVector(std::ostream& output, const Eigen::VectorXd& vector) {
  for (int index = 0; index < vector.size(); ++index)
    output << ',' << vector[index];
}

void writePose(std::ostream& output, const Pose& pose) {
  const Eigen::Quaterniond quaternion(pose.rotation());
  output << ',' << pose.translation().x()
         << ',' << pose.translation().y()
         << ',' << pose.translation().z()
         << ',' << quaternion.x()
         << ',' << quaternion.y()
         << ',' << quaternion.z()
         << ',' << quaternion.w();
}

bool finitePose(const Pose& pose) { return pose.matrix().allFinite(); }

}  // namespace

DistributionSummary summarizeDistribution(const std::vector<double>& values) {
  if (values.empty()) throw std::invalid_argument("cannot summarize an empty sample set");
  std::vector<double> sorted = values;
  double sum = 0.0;
  double square_sum = 0.0;
  for (const double value : sorted) {
    if (!std::isfinite(value))
      throw std::invalid_argument("distribution contains a non-finite value");
    sum += value;
    square_sum += value * value;
  }
  std::sort(sorted.begin(), sorted.end());
  DistributionSummary result;
  result.count = sorted.size();
  result.mean = sum / static_cast<double>(result.count);
  result.rms = std::sqrt(square_sum / static_cast<double>(result.count));
  result.maximum = sorted.back();
  result.p95 = percentile(sorted, .95);
  result.p99 = percentile(sorted, .99);
  double variance_sum = 0.0;
  for (const double value : sorted) {
    const double centered = value - result.mean;
    variance_sum += centered * centered;
  }
  result.standard_deviation =
      std::sqrt(variance_sum / static_cast<double>(result.count));
  return result;
}

PoseError measurePoseError(const Pose& desired, const Pose& actual) {
  if (!finitePose(desired) || !finitePose(actual))
    throw std::invalid_argument("pose contains a non-finite value");
  PoseError result;
  result.position_m = (desired.translation() - actual.translation()).norm();
  result.orientation_rad = Eigen::Quaterniond(desired.rotation()).angularDistance(
      Eigen::Quaterniond(actual.rotation()));
  return result;
}

bool timestampsAreConsistent(const std::vector<double>& sample_times,
                             const std::vector<double>& command_times,
                             const std::vector<double>& feedback_times,
                             std::string* reason) {
  if (sample_times.empty() || sample_times.size() != command_times.size() ||
      sample_times.size() != feedback_times.size()) {
    if (reason) *reason = "timestamp vectors must be non-empty and equally sized";
    return false;
  }
  for (std::size_t index = 0; index < sample_times.size(); ++index) {
    if (!std::isfinite(sample_times[index]) ||
        !std::isfinite(command_times[index]) ||
        !std::isfinite(feedback_times[index])) {
      if (reason) *reason = "timestamp contains a non-finite value";
      return false;
    }
    if (index > 0 && (sample_times[index] < sample_times[index - 1] ||
                      command_times[index] < command_times[index - 1] ||
                      feedback_times[index] < feedback_times[index - 1])) {
      if (reason) *reason = "timestamps are not monotonic";
      return false;
    }
    if (command_times[index] > sample_times[index] + 1e-9 ||
        feedback_times[index] > sample_times[index] + 1e-9) {
      if (reason) *reason = "receipt or command timestamp is in the future";
      return false;
    }
  }
  if (reason) reason->clear();
  return true;
}

class RealExecutionCsvWriter::Impl {
 public:
  explicit Impl(const std::string& path) {
    std::ifstream existing(path);
    if (existing.good())
      throw std::runtime_error("refusing to overwrite raw benchmark CSV: " + path);
    stream.open(path, std::ios::out | std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot open raw benchmark CSV: " + path);
    stream << "data_source,reference_frame,run_id,scenario,repetition,"
              "requested_linear_speed_mps,requested_angular_speed_rad_s,"
              "sample_time_sec,command_time_sec,feedback_time_sec,"
              "feedback_source_time_sec,feedback_source_time_valid,"
              "command_published";
    for (const char* pose : {"ideal", "command", "actual"}) {
      for (const char* field : {"x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"})
        stream << ',' << pose << '_' << field;
    }
    for (const char* vector : {"target_q", "target_dq", "target_ddq", "actual_q"})
      for (int joint = 1; joint <= 7; ++joint)
        stream << ',' << vector << joint;
    stream << ",force_n,torque_nm,controller_status,guard_result,collision,stop_reason\n";
  }

  std::ofstream stream;
};

RealExecutionCsvWriter::RealExecutionCsvWriter(const std::string& path)
    : impl_(new Impl(path)) {}

RealExecutionCsvWriter::~RealExecutionCsvWriter() { delete impl_; }

void RealExecutionCsvWriter::append(const RealExecutionSample& sample) {
  requireVector7(sample.target_joints, "target_joints");
  requireVector7(sample.target_velocity, "target_velocity");
  requireVector7(sample.target_acceleration, "target_acceleration");
  requireVector7(sample.actual_joints, "actual_joints");
  if (sample.run_id.empty() || sample.scenario.empty() || sample.repetition < 1 ||
      !std::isfinite(sample.requested_linear_speed_mps) ||
      !std::isfinite(sample.requested_angular_speed_rad_s) ||
      sample.requested_linear_speed_mps < 0.0 ||
      sample.requested_angular_speed_rad_s < 0.0 ||
      !std::isfinite(sample.sample_time_sec) ||
      !std::isfinite(sample.command_time_sec) ||
      !std::isfinite(sample.feedback_time_sec) ||
      !std::isfinite(sample.feedback_source_time_sec) ||
      (sample.feedback_source_time_valid &&
       sample.feedback_source_time_sec <= 0.0) ||
      !std::isfinite(sample.force_n) || !std::isfinite(sample.torque_nm) ||
      !finitePose(sample.ideal_pose) || !finitePose(sample.commanded_pose) ||
      !finitePose(sample.actual_pose))
    throw std::invalid_argument("invalid real-execution sample");
  impl_->stream << std::setprecision(17)
      << "real_robot,base_link_to_Link7," << csvEscape(sample.run_id) << ','
      << csvEscape(sample.scenario) << ',' << sample.repetition << ','
      << sample.requested_linear_speed_mps << ','
      << sample.requested_angular_speed_rad_s << ','
      << sample.sample_time_sec << ',' << sample.command_time_sec << ','
      << sample.feedback_time_sec << ',' << sample.feedback_source_time_sec
      << ',' << (sample.feedback_source_time_valid ? 1 : 0) << ','
      << (sample.command_published ? 1 : 0);
  writePose(impl_->stream, sample.ideal_pose);
  writePose(impl_->stream, sample.commanded_pose);
  writePose(impl_->stream, sample.actual_pose);
  writeVector(impl_->stream, sample.target_joints);
  writeVector(impl_->stream, sample.target_velocity);
  writeVector(impl_->stream, sample.target_acceleration);
  writeVector(impl_->stream, sample.actual_joints);
  impl_->stream << ',' << sample.force_n << ',' << sample.torque_nm << ','
      << csvEscape(sample.controller_status) << ','
      << csvEscape(sample.guard_result) << ',' << (sample.collision ? 1 : 0)
      << ',' << csvEscape(sample.stop_reason) << '\n';
  impl_->stream.flush();
  if (!impl_->stream) throw std::runtime_error("failed writing raw benchmark CSV");
  ++rows_written_;
}

void writeRealExecutionRunJson(const std::string& path,
                               const std::string& run_id,
                               const std::string& scenario,
                               const std::string& status,
                               std::size_t samples,
                               const PoseError& final_error,
                               const std::string& stop_reason,
                               double stop_latency_sec,
                               double stop_distance_max_joint_rad) {
  if (run_id.empty() || scenario.empty() ||
      (status != "PASS" && status != "STOP" && status != "FAIL") ||
      !std::isfinite(final_error.position_m) ||
      !std::isfinite(final_error.orientation_rad))
    throw std::invalid_argument("invalid run summary");
  std::ofstream output;
  {
    std::ifstream existing(path);
    if (existing.good())
      throw std::runtime_error("refusing to overwrite run JSON: " + path);
  }
  output.open(path, std::ios::out | std::ios::trunc);
  if (!output) throw std::runtime_error("cannot open run JSON: " + path);
  output << std::setprecision(17)
         << "{\n"
         << "  \"data_source\": \"real_robot\",\n"
         << "  \"reference_frame\": \"base_link_to_Link7\",\n"
         << "  \"run_id\": \"" << jsonEscape(run_id) << "\",\n"
         << "  \"scenario\": \"" << jsonEscape(scenario) << "\",\n"
         << "  \"status\": \"" << status << "\",\n"
         << "  \"samples\": " << samples << ",\n"
         << "  \"final_position_error_m\": " << final_error.position_m << ",\n"
         << "  \"final_orientation_error_rad\": "
         << final_error.orientation_rad << ",\n"
         << "  \"stop_latency_sec\": ";
  if (stop_latency_sec < 0.0) output << "null";
  else output << stop_latency_sec;
  output << ",\n  \"stop_distance_max_joint_rad\": ";
  if (stop_distance_max_joint_rad < 0.0) output << "null";
  else output << stop_distance_max_joint_rad;
  output << ",\n"
         << "  \"stop_reason\": \"" << jsonEscape(stop_reason) << "\"\n"
         << "}\n";
  if (!output) throw std::runtime_error("failed writing run JSON");
}

}  // namespace rm75_motion_planning
