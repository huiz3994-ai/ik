#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "rm75_motion_planning/cartesian_path_generator.h"
#include "rm75_motion_planning/continuous_ik_solver.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

using namespace rm75_motion_planning;

namespace {
struct Row {
  std::string solver;
  std::string scenario;
  int sample{0};
  bool success{false};
  double planning_ms{0.0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double max_jerk{0.0};
  double max_jump{0.0};
  double min_margin{0.0};
  int pose_count{0};
  int refinements{0};
  std::string failure;
  Eigen::VectorXd start_q;
  Eigen::VectorXd goal_q;
};

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(std::floor(p * (values.size() - 1)))];
}

std::string csvEscape(std::string value) {
  std::replace(value.begin(), value.end(), '"', '\'');
  return '"' + value + '"';
}

Eigen::VectorXd sampleQ(std::mt19937& rng,
                        const rm75_kinematics::JointLimits& limits,
                        double fraction) {
  std::uniform_real_distribution<double> unit(-fraction, fraction);
  Eigen::VectorXd q(limits.lower.size());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    const double center = .5 * (limits.lower[i] + limits.upper[i]);
    q[i] = center + unit(rng) * .5 * (limits.upper[i] - limits.lower[i]);
  }
  return q;
}

DynamicLimits benchmarkLimits() {
  return {Eigen::VectorXd::Constant(7, 1.0),
          Eigen::VectorXd::Constant(7, .5),
          Eigen::VectorXd::Constant(7, 2.0)};
}

class KdlContinuousSolver {
 public:
  KdlContinuousSolver(const std::string& urdf,
                      const rm75_kinematics::JointLimits& limits,
                      const rm75_kinematics::KinematicsModel& model)
      : model_(model) {
    std::ifstream stream(urdf);
    const std::string xml((std::istreambuf_iterator<char>(stream)), {});
    KDL::Tree tree;
    if (xml.empty() || !kdl_parser::treeFromString(xml, tree) ||
        !tree.getChain("base_link", "Link7", chain_))
      throw std::runtime_error("failed to construct RM75 KDL chain");
    lower_.resize(7); upper_.resize(7);
    for (int i = 0; i < 7; ++i) {
      lower_(i) = limits.lower[i]; upper_(i) = limits.upper[i];
    }
  }

  ContinuousIkResult solve(const PoseVector& poses,
                           const Eigen::VectorXd& initial,
                           double max_jump) const {
    ContinuousIkResult result;
    result.joints.push_back(initial);
    result.solved_poses.push_back(poses.front());
    KDL::ChainFkSolverPos_recursive fk(chain_);
    KDL::ChainIkSolverVel_pinv velocity(chain_, 1e-5, 150);
    KDL::ChainIkSolverPos_NR_JL ik(chain_, lower_, upper_, fk, velocity,
                                  150, 1e-6);
    Eigen::VectorXd seed = initial;
    for (std::size_t index = 1; index < poses.size(); ++index) {
      KDL::JntArray q0(7), q(7);
      for (int joint = 0; joint < 7; ++joint) q0(joint) = seed[joint];
      const auto& pose = poses[index];
      const auto& r = pose.rotation();
      KDL::Rotation rotation(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1),
                             r(1, 2), r(2, 0), r(2, 1), r(2, 2));
      const int code = ik.CartToJnt(q0, KDL::Frame(
          rotation, KDL::Vector(pose.translation().x(), pose.translation().y(),
                                pose.translation().z())), q);
      Eigen::VectorXd solved(7);
      for (int joint = 0; joint < 7; ++joint) solved[joint] = q(joint);
      const auto actual = model_.forward(solved);
      Eigen::Quaterniond qa(actual.rotation), qr(pose.rotation());
      if (code < 0 || !solved.allFinite() ||
          (actual.position - pose.translation()).norm() > .002 ||
          qa.angularDistance(qr) > .034906585 ||
          (solved - seed).norm() > max_jump) {
        result.failure_reason = "KDL failure code " + std::to_string(code) +
                                " at pose index " + std::to_string(index);
        return result;
      }
      result.joints.push_back(solved);
      result.solved_poses.push_back(pose);
      seed = solved;
    }
    result.success = true;
    return result;
  }

 private:
  const rm75_kinematics::KinematicsModel& model_;
  KDL::Chain chain_;
  KDL::JntArray lower_, upper_;
};

std::string jsonEscape(const std::string& input) {
  std::string out;
  for (const char c : input) {
    if (c == '\\' || c == '"') out += '\\';
    if (c == '\n' || c == '\r') out += ' ';
    else out += c;
  }
  return out;
}

std::string classifyFailure(const std::string& detail) {
  if (detail.find("KDL failure code -5") != std::string::npos)
    return "KDL_MAX_ITERATIONS";
  if (detail.find("KDL failure code") != std::string::npos)
    return "KDL_VALIDATION_OR_JUMP";
  if (detail.find("line search") != std::string::npos)
    return "DLS_LINE_SEARCH";
  if (detail.find("iteration budget") != std::string::npos)
    return "DLS_MAX_ITERATIONS";
  if (detail.find("insufficient progress") != std::string::npos)
    return "DLS_STALLED";
  if (detail.find("converged") != std::string::npos)
    return "JOINT_JUMP_AFTER_DLS";
  if (detail.empty()) return "UNKNOWN";
  return detail;
}
}  // namespace

int main(int argc, char** argv) {
  int count_per_scenario = 1000;
  unsigned seed = 7506;
  std::string output = "/tmp/rm75_motion_benchmark";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--count-per-scenario" && i + 1 < argc)
      count_per_scenario = std::stoi(argv[++i]);
    else if (arg == "--seed" && i + 1 < argc)
      seed = static_cast<unsigned>(std::stoul(argv[++i]));
    else if (arg == "--output" && i + 1 < argc)
      output = argv[++i];
    else {
      std::cerr << "Usage: rm75_motion_benchmark [--count-per-scenario N] "
                   "[--seed N] [--output PREFIX]\n";
      return 2;
    }
  }
  if (count_per_scenario <= 0) return 2;

  rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
  const auto joint_limits = model.limits();
  const KdlContinuousSolver kdl(RM75_DEFAULT_URDF, joint_limits, model);
  const DynamicLimits dynamic_limits = benchmarkLimits();
  const SamplingOptions sampling{.005, .05};
  const std::vector<std::string> scenarios{
      "short_line", "large_cartesian", "random_6d", "near_singular",
      "near_joint_limit"};
  const std::vector<std::string> solvers{
      "kdl", "fixed_dls", "adaptive_dls", "adaptive_dls_nullspace"};
  std::mt19937 rng(seed);
  std::normal_distribution<double> small(0.0, .01);
  std::vector<Row> rows;
  rows.reserve(static_cast<std::size_t>(count_per_scenario) *
               scenarios.size() * solvers.size());

  for (const auto& scenario : scenarios) {
    for (int sample = 0; sample < count_per_scenario; ++sample) {
      Eigen::VectorXd start = sampleQ(rng, joint_limits, .45);
      Eigen::VectorXd goal = start;
      if (scenario == "short_line") {
        for (int j = 0; j < 7; ++j) goal[j] += small(rng);
      } else if (scenario == "large_cartesian") {
        goal = sampleQ(rng, joint_limits, .55);
      } else if (scenario == "random_6d") {
        start = sampleQ(rng, joint_limits, .6);
        goal = sampleQ(rng, joint_limits, .6);
      } else if (scenario == "near_singular") {
        start.setZero();
        goal.setZero();
        for (int j = 0; j < 7; ++j) {
          start[j] = .01 * small(rng);
          goal[j] = start[j] + small(rng);
        }
      } else {
        const int joint = sample % 7;
        start[joint] = joint_limits.upper[joint] - .06;
        goal = start;
        goal[joint] -= .03;
        for (int j = 0; j < 7; ++j) goal[j] += .3 * small(rng);
      }
      goal = goal.cwiseMax((joint_limits.lower.array() + 1e-5).matrix())
                 .cwiseMin((joint_limits.upper.array() - 1e-5).matrix());
      PoseVector poses = CartesianPathGenerator::line(
          model.forward(start).isometry(), model.forward(goal).isometry(),
          sampling);

      for (const auto& solver_name : solvers) {
        rm75_kinematics::IkOptions ik;
        ik.timeout_sec = .05;
        ik.max_iterations = 120;
        ik.position_tolerance = .001;
        ik.orientation_tolerance = .01;
        if (solver_name == "fixed_dls") {
          ik.damping_min = .025;
          ik.damping_max = .025;
        }
        if (solver_name == "adaptive_dls_nullspace")
          ik.enable_joint_centering = true;
        ContinuousIkOptions continuous;
        continuous.enable_ipopt_fallback = false;
        continuous.max_joint_jump = .35;
        continuous.max_refinement_depth = 3;

        const auto begin = std::chrono::steady_clock::now();
        const auto solved = solver_name == "kdl"
            ? kdl.solve(poses, start, continuous.max_joint_jump)
            : ContinuousIkSolver(model).solve(poses, start, continuous, ik);
        Row row;
        row.solver = solver_name;
        row.scenario = scenario;
        row.sample = sample;
        row.pose_count = static_cast<int>(poses.size());
        row.refinements = solved.refinements;
        row.failure = solved.failure_reason;
        row.start_q = start;
        row.goal_q = goal;
        if (solved.success) {
          const auto timed = TrajectoryParameterizer::parameterize(
              solved.joints, dynamic_limits, .02);
          const auto report = TrajectoryValidator(model).validate(
              solved.solved_poses, solved.joints, timed, dynamic_limits,
              .002, .034906585, continuous.max_joint_jump);
          row.success = report.valid;
          row.position_error_m = report.max_position_error_m;
          row.orientation_error_rad = report.max_orientation_error_rad;
          row.max_velocity = report.max_velocity.maxCoeff();
          row.max_acceleration = report.max_acceleration.maxCoeff();
          row.max_jerk = report.max_jerk.maxCoeff();
          row.max_jump = report.max_joint_jump_rad;
          row.min_margin = report.min_joint_limit_margin_rad;
          if (!report.valid) {
            std::ostringstream reason;
            for (const auto& violation : report.violations)
              reason << violation << "; ";
            row.failure = reason.str();
          }
        }
        row.planning_ms = 1e3 * std::chrono::duration<double>(
            std::chrono::steady_clock::now() - begin).count();
        rows.push_back(std::move(row));
      }
    }
    std::cout << "Completed " << scenario << " (" << count_per_scenario
              << " paths x " << solvers.size() << " solvers)\n";
  }

  std::ofstream csv(output + ".csv");
  csv << "solver,scenario,sample,success,planning_ms,pose_count,refinements,"
         "max_position_error_m,max_orientation_error_rad,max_velocity_rad_s,"
         "max_acceleration_rad_s2,max_jerk_rad_s3,max_joint_jump_rad,"
         "min_joint_margin_rad,failure";
  for (int joint = 1; joint <= 7; ++joint) csv << ",start_q" << joint;
  for (int joint = 1; joint <= 7; ++joint) csv << ",goal_q" << joint;
  csv << "\n" << std::setprecision(12);
  for (const auto& row : rows) {
    csv << row.solver << ',' << row.scenario << ',' << row.sample << ','
        << (row.success ? 1 : 0) << ',' << row.planning_ms << ','
        << row.pose_count << ',' << row.refinements << ','
        << row.position_error_m << ',' << row.orientation_error_rad << ','
        << row.max_velocity << ',' << row.max_acceleration << ','
        << row.max_jerk << ',' << row.max_jump << ',' << row.min_margin << ','
        << csvEscape(row.failure);
    for (int joint = 0; joint < 7; ++joint) csv << ',' << row.start_q[joint];
    for (int joint = 0; joint < 7; ++joint) csv << ',' << row.goal_q[joint];
    csv << '\n';
  }

  std::ofstream json(output + ".json");
  json << "{\n  \"model\": \"RM75-6FB\",\n  \"base_frame\": \"base_link\",\n"
          "  \"flange_frame\": \"Link7 (Link7_6fb xacro branch)\",\n"
       << "  \"seed\": " << seed << ",\n  \"count_per_scenario\": "
       << count_per_scenario << ",\n  \"control_rate_hz\": 50,\n"
          "  \"solvers\": {\n";
  for (std::size_t solver_index = 0; solver_index < solvers.size(); ++solver_index) {
    const auto& solver = solvers[solver_index];
    json << "    \"" << solver << "\": {\n";
    for (std::size_t scenario_index = 0; scenario_index < scenarios.size();
         ++scenario_index) {
      const auto& scenario = scenarios[scenario_index];
      int successes = 0;
      double max_position = 0.0, max_orientation = 0.0, max_velocity = 0.0;
      double max_acceleration = 0.0, max_jerk = 0.0, max_jump = 0.0;
      double min_margin = std::numeric_limits<double>::infinity();
      std::vector<double> times;
      std::map<std::string, int> failures;
      for (const auto& row : rows) {
        if (row.solver != solver || row.scenario != scenario) continue;
        times.push_back(row.planning_ms);
        if (row.success) {
          ++successes;
          max_position = std::max(max_position, row.position_error_m);
          max_orientation = std::max(max_orientation, row.orientation_error_rad);
          max_velocity = std::max(max_velocity, row.max_velocity);
          max_acceleration = std::max(max_acceleration, row.max_acceleration);
          max_jerk = std::max(max_jerk, row.max_jerk);
          max_jump = std::max(max_jump, row.max_jump);
          min_margin = std::min(min_margin, row.min_margin);
        } else {
          ++failures[classifyFailure(row.failure)];
        }
      }
      const double mean = std::accumulate(times.begin(), times.end(), 0.0) /
                          std::max<std::size_t>(1, times.size());
      json << "      \"" << scenario << "\": {\"success_rate\": "
           << static_cast<double>(successes) / count_per_scenario
           << ", \"mean_planning_ms\": " << mean
           << ", \"p95_planning_ms\": " << percentile(times, .95)
           << ", \"p99_planning_ms\": " << percentile(times, .99)
           << ", \"max_position_error_m\": " << max_position
           << ", \"max_orientation_error_rad\": " << max_orientation
           << ", \"max_velocity_rad_s\": " << max_velocity
           << ", \"max_acceleration_rad_s2\": " << max_acceleration
           << ", \"max_jerk_rad_s3\": " << max_jerk
           << ", \"max_joint_jump_rad\": " << max_jump
           << ", \"min_joint_margin_rad\": "
           << (successes ? min_margin : 0.0) << ", \"failures\": {";
      bool first = true;
      for (const auto& failure : failures) {
        if (!first) json << ',';
        first = false;
        json << '"' << jsonEscape(failure.first) << "\":" << failure.second;
      }
      json << "}}" << (scenario_index + 1 < scenarios.size() ? "," : "")
           << '\n';
    }
    json << "    }" << (solver_index + 1 < solvers.size() ? "," : "") << '\n';
  }
  json << "  }\n}\n";
  std::cout << "Wrote " << output << ".csv and " << output << ".json ("
            << rows.size() << " complete pipeline runs)\n";
  return 0;
}
