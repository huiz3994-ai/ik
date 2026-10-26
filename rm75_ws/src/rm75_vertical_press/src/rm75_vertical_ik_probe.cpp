#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/JointState.h>

#include "rm75_vertical_press/vertical_orientation.h"

namespace {

double degrees(double radians) { return radians * 180.0 / 3.14159265358979323846; }

struct Result {
  bool solved = false;
  double max_joint_delta = 0.0;
};

struct SearchCandidate {
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  double yaw_delta = 0.0;
  double max_joint_delta = 0.0;
  double score = 0.0;
};

Result solve(const moveit::core::RobotState& seed,
             const moveit::core::JointModelGroup* group,
             const Eigen::Isometry3d& target,
             const std::string& tip,
             double timeout) {
  moveit::core::RobotState candidate(seed);
  Result result;
  result.solved = candidate.setFromIK(group, target, tip, timeout) &&
                  candidate.satisfiesBounds(group);
  if (!result.solved) return result;
  std::vector<double> before, after;
  seed.copyJointGroupPositions(group, before);
  candidate.copyJointGroupPositions(group, after);
  for (std::size_t index = 0; index < before.size(); ++index) {
    result.max_joint_delta = std::max(
        result.max_joint_delta, std::abs(after[index] - before[index]));
  }
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "rm75_vertical_ik_probe",
            ros::init_options::AnonymousName);
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  robot_model_loader::RobotModelLoader loader("robot_description");
  const moveit::core::RobotModelPtr model = loader.getModel();
  if (!model) {
    std::cerr << "robot_description unavailable\n";
    return 1;
  }
  const auto* group = model->getJointModelGroup("arm");
  if (!group) {
    std::cerr << "arm group unavailable\n";
    return 1;
  }
  const auto message = ros::topic::waitForMessage<sensor_msgs::JointState>(
      "/joint_states", node, ros::Duration(3.0));
  if (!message) {
    std::cerr << "joint_states unavailable\n";
    return 1;
  }
  std::map<std::string, double> positions;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    positions[message->name[index]] = message->position[index];
  }
  moveit::core::RobotState seed(model);
  seed.setToDefaultValues();
  seed.setVariablePositions(positions);
  seed.update();
  const std::string tip = "Link7";
  const Eigen::Isometry3d current = seed.getGlobalLinkTransform(tip);
  const Eigen::Matrix3d down_goal =
      rm75_vertical_press::verticalDownOrientation(current.linear());
  const Eigen::Matrix3d up_goal = down_goal *
      Eigen::AngleAxisd(3.14159265358979323846,
                        Eigen::Vector3d::UnitX()).toRotationMatrix();

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "position " << current.translation().transpose() << "\n";
  std::cout << "tool +Z " << current.linear().col(2).transpose() << "\n";
  std::cout << "angle(+Z,-baseZ) deg "
            << degrees(std::acos(std::max(-1.0, std::min(1.0,
               current.linear().col(2).dot(Eigen::Vector3d(0, 0, -1))))))
            << "\n";

  auto report = [&](const std::string& label, const Eigen::Isometry3d& target) {
    const Result result = solve(seed, group, target, tip, 0.2);
    std::cout << label << " solved=" << (result.solved ? "yes" : "no")
              << " max_joint_delta=" << result.max_joint_delta << "\n";
    return result.solved;
  };

  report("current", current);
  Eigen::Isometry3d target = current;
  for (double angle : {0.001, 0.003, 0.01, 0.03, 0.10}) {
    target = current;
    target.linear() = rm75_vertical_press::stepTowardOrientation(
        current.linear(), down_goal, angle);
    report("down_step_" + std::to_string(angle), target);
  }
  for (double timeout : {0.005, 0.015, 0.03, 0.05, 0.10, 0.20}) {
    target = current;
    target.linear() = rm75_vertical_press::stepTowardOrientation(
        current.linear(), down_goal, 0.003);
    const Result result = solve(seed, group, target, tip, timeout);
    std::cout << "timeout_probe timeout=" << timeout
              << " solved=" << (result.solved ? "yes" : "no")
              << " max_joint_delta=" << result.max_joint_delta << "\n";
  }
  target = current;
  target.linear() = down_goal;
  report("down_final_same_position", target);
  target.linear() = up_goal;
  report("opposite_axis_same_position", target);

  const double current_heading = std::atan2(down_goal(1, 0), down_goal(0, 0));
  bool have_yaw_solution = false;
  double best_yaw_delta = 0.0;
  double best_yaw_joint_delta = 1e9;
  for (int step = -12; step < 12; ++step) {
    const double yaw_delta = step * 3.14159265358979323846 / 12.0;
    target = current;
    target.linear() =
        (Eigen::AngleAxisd(current_heading + yaw_delta,
                           Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(3.14159265358979323846,
                           Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Result result = solve(seed, group, target, tip, 0.1);
    if (result.solved) {
      std::cout << "same_position_yaw_solution delta_deg="
                << degrees(yaw_delta) << " max_joint_delta="
                << result.max_joint_delta << "\n";
      if (result.max_joint_delta < best_yaw_joint_delta) {
        have_yaw_solution = true;
        best_yaw_delta = yaw_delta;
        best_yaw_joint_delta = result.max_joint_delta;
      }
    }
  }
  std::cout << "best_same_position_yaw solved="
            << (have_yaw_solution ? "yes" : "no")
            << " delta_deg=" << degrees(best_yaw_delta)
            << " max_joint_delta=" << best_yaw_joint_delta << "\n";

  bool quick = false;
  private_node.param("quick", quick, false);
  if (quick) {
    int continuous_paths = 0;
    for (double depth : {0.24, 0.27, 0.30, 0.33, 0.36}) {
      for (int yaw_index = 0; yaw_index < 8; ++yaw_index) {
        const double yaw = yaw_index * 2.0 * 3.14159265358979323846 / 8.0;
        const Eigen::Matrix3d quick_orientation = down_goal *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        moveit::core::RobotState path_state(seed);
        bool path_ok = true;
        double largest_step = 0.0;
        // First lower without rotating, then rotate at the lower pose.  This
        // avoids the wrist singularity seen on the diagonal interpolation.
        for (int phase = 0; phase < 2 && path_ok; ++phase) {
          const int steps = phase == 0 ? 90 : 180;
          const Eigen::Isometry3d phase_start =
              path_state.getGlobalLinkTransform(tip);
          Eigen::Isometry3d phase_goal = phase_start;
          if (phase == 0) phase_goal.translation().z() = current.translation().z() - depth;
          else phase_goal.linear() = quick_orientation;
          const double phase_angle =
              Eigen::Quaterniond(phase_start.linear()).angularDistance(
                  Eigen::Quaterniond(phase_goal.linear()));
          for (int step = 1; step <= steps; ++step) {
            const double fraction = static_cast<double>(step) / steps;
            target = phase_start;
            target.translation() = phase_start.translation() + fraction *
                (phase_goal.translation() - phase_start.translation());
            target.linear() = rm75_vertical_press::stepTowardOrientation(
                phase_start.linear(), phase_goal.linear(),
                fraction * phase_angle);
            std::vector<double> before, after;
            path_state.copyJointGroupPositions(group, before);
            moveit::core::RobotState next(path_state);
            if (!next.setFromIK(group, target, tip, 0.01) ||
                !next.satisfiesBounds(group)) {
              path_ok = false;
              break;
            }
            next.harmonizePositions(group);
            next.copyJointGroupPositions(group, after);
            for (std::size_t index = 0; index < before.size(); ++index) {
              largest_step = std::max(largest_step,
                                      std::abs(after[index] - before[index]));
            }
            if (largest_step > 0.35) {
              path_ok = false;
              break;
            }
            path_state = next;
            path_state.update();
          }
        }
        if (path_ok) {
          ++continuous_paths;
          std::cout << "quick_two_stage_ok depth=" << depth
                    << " yaw_deg=" << degrees(yaw)
                    << " largest_joint_step=" << largest_step << "\n";
        }
      }
    }
    std::cout << "quick_continuous_path_count " << continuous_paths << "\n";
    return continuous_paths > 0 ? 0 : 2;
  }

  int nearby_solutions = 0;
  double best_offset_norm = 1e9;
  Eigen::Vector3d best_offset = Eigen::Vector3d::Zero();
  double best_offset_joint_delta = 1e9;
  for (double dx : {0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06}) {
    for (double dy : {0.0, -0.01, -0.02, -0.03, -0.04, -0.05, -0.06}) {
      for (double dz : {0.0, -0.01, -0.02, -0.03, -0.04, -0.05}) {
        target = current;
        target.translation() += Eigen::Vector3d(dx, dy, dz);
        target.linear() = down_goal;
        const Result result = solve(seed, group, target, tip, 0.05);
        if (result.solved) {
          ++nearby_solutions;
          std::cout << "nearby_down_solution offset=" << dx << " " << dy
                    << " " << dz << " max_joint_delta="
                    << result.max_joint_delta << "\n";
          const double norm = Eigen::Vector3d(dx, dy, dz).norm();
          if (norm < best_offset_norm ||
              (std::abs(norm - best_offset_norm) < 1e-9 &&
               result.max_joint_delta < best_offset_joint_delta)) {
            best_offset_norm = norm;
            best_offset = Eigen::Vector3d(dx, dy, dz);
            best_offset_joint_delta = result.max_joint_delta;
          }
        }
      }
    }
  }
  std::cout << "nearby_solution_count " << nearby_solutions << "\n";
  std::cout << "best_nearby_offset " << best_offset.transpose()
            << " norm=" << best_offset_norm
            << " max_joint_delta=" << best_offset_joint_delta << "\n";

  // Search all base-frame directions and all tool yaw angles.  This is
  // deliberately read-only: setFromIK only updates local RobotState copies.
  std::vector<SearchCandidate> broad_candidates;
  const std::vector<double> broad_xy = {
      -0.48, -0.36, -0.24, -0.12, 0.0, 0.12, 0.24, 0.36};
  const std::vector<double> broad_z = {
      -0.54, -0.45, -0.36, -0.27, -0.18, -0.09, 0.0, 0.09};
  for (double dx : broad_xy) {
    for (double dy : broad_xy) {
      for (double dz : broad_z) {
        const Eigen::Vector3d offset(dx, dy, dz);
        for (int yaw_step = 0; yaw_step < 8; ++yaw_step) {
          const double yaw_delta = yaw_step * 2.0 * 3.14159265358979323846 / 8.0;
          target = current;
          target.translation() += offset;
          target.linear() = down_goal *
              Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ())
                  .toRotationMatrix();
          const Result result = solve(seed, group, target, tip, 0.005);
          if (!result.solved) continue;
          SearchCandidate candidate;
          candidate.offset = offset;
          candidate.orientation = target.linear();
          candidate.yaw_delta = yaw_delta;
          candidate.max_joint_delta = result.max_joint_delta;
          candidate.score = offset.norm() + 0.01 * result.max_joint_delta;
          broad_candidates.push_back(candidate);
        }
      }
    }
  }
  std::sort(broad_candidates.begin(), broad_candidates.end(),
            [](const SearchCandidate& a, const SearchCandidate& b) {
              return a.score < b.score;
            });
  std::cout << "broad_solution_count " << broad_candidates.size() << "\n";
  for (std::size_t index = 0;
       index < std::min<std::size_t>(20, broad_candidates.size()); ++index) {
    const SearchCandidate& candidate = broad_candidates[index];
    std::cout << "broad_candidate rank=" << index
              << " offset=" << candidate.offset.transpose()
              << " yaw_deg=" << degrees(candidate.yaw_delta)
              << " max_joint_delta=" << candidate.max_joint_delta
              << " score=" << candidate.score << "\n";
  }

  // A final IK solution is not enough: verify that the whole simultaneous
  // translation/orientation interpolation remains solvable from this pose.
  int broad_paths_ok = 0;
  for (std::size_t index = 0;
       index < std::min<std::size_t>(30, broad_candidates.size()); ++index) {
    const SearchCandidate& search = broad_candidates[index];
    moveit::core::RobotState path_state(seed);
    bool path_ok = true;
    int failed_step = 0;
    const double full_angle =
        Eigen::Quaterniond(current.linear()).angularDistance(
            Eigen::Quaterniond(search.orientation));
    for (int step = 1; step <= 120; ++step) {
      const double fraction = step / 120.0;
      target = current;
      target.translation() = current.translation() + fraction * search.offset;
      target.linear() = rm75_vertical_press::stepTowardOrientation(
          current.linear(), search.orientation, fraction * full_angle);
      if (!path_state.setFromIK(group, target, tip, 0.01) ||
          !path_state.satisfiesBounds(group)) {
        path_ok = false;
        failed_step = step;
        break;
      }
      path_state.update();
    }
    if (path_ok) {
      ++broad_paths_ok;
      std::cout << "broad_path_ok rank=" << index
                << " offset=" << search.offset.transpose()
                << " yaw_deg=" << degrees(search.yaw_delta)
                << " max_joint_delta=" << search.max_joint_delta << "\n";
      if (broad_paths_ok >= 10) break;
    } else if (index < 10) {
      std::cout << "broad_path_fail rank=" << index
                << " failed_step=" << failed_step << "\n";
    }
  }
  std::cout << "broad_path_count " << broad_paths_ok << "\n";

  for (const Eigen::Vector3d offset : {
           Eigen::Vector3d(0.03, -0.03, -0.04),
           Eigen::Vector3d(0.00, -0.04, -0.05),
           Eigen::Vector3d(0.04, -0.02, -0.04)}) {
    moveit::core::RobotState path_state(seed);
    bool path_ok = true;
    int failed_step = 0;
    for (int step = 1; step <= 100; ++step) {
      const double fraction = step / 100.0;
      target = current;
      target.translation() = current.translation() + fraction * offset;
      target.linear() = rm75_vertical_press::stepTowardOrientation(
          current.linear(), down_goal,
          fraction * Eigen::Quaterniond(current.linear()).angularDistance(
                         Eigen::Quaterniond(down_goal)));
      if (!path_state.setFromIK(group, target, tip, 0.05) ||
          !path_state.satisfiesBounds(group)) {
        path_ok = false;
        failed_step = step;
        break;
      }
      path_state.update();
    }
    bool down_ok = path_ok;
    if (path_ok) {
      const Eigen::Isometry3d vertical_pose =
          path_state.getGlobalLinkTransform(tip);
      for (int step = 1; step <= 10; ++step) {
        target = vertical_pose;
        target.translation().z() -= step * 0.0005;
        if (!path_state.setFromIK(group, target, tip, 0.05) ||
            !path_state.satisfiesBounds(group)) {
          down_ok = false;
          break;
        }
        path_state.update();
      }
    }
    std::cout << "path offset=" << offset.transpose()
              << " align_ok=" << (path_ok ? "yes" : "no")
              << " failed_step=" << failed_step
              << " down_5mm_ok=" << (down_ok ? "yes" : "no") << "\n";
  }

  // Mirror the controller's 50 Hz goal stepping and 0.2 rad/s coupled joint
  // limiter without publishing anything to the robot.
  moveit::core::RobotState simulated(seed);
  Eigen::Isometry3d simulated_goal = current;
  simulated_goal.translation() += Eigen::Vector3d(0.03, -0.03, -0.04);
  simulated_goal.linear() = down_goal;
  const double simulated_full_angle =
      Eigen::Quaterniond(current.linear()).angularDistance(
          Eigen::Quaterniond(simulated_goal.linear()));
  const double simulated_duration = std::max(
      simulated_full_angle / 0.15,
      (simulated_goal.translation() - current.translation()).norm() / 0.015);
  double simulated_progress = 0.0;
  bool controller_path_ok = true;
  int controller_cycles = 0;
  for (; controller_cycles < 5000; ++controller_cycles) {
    const Eigen::Isometry3d commanded = simulated.getGlobalLinkTransform(tip);
    const double angle_error = Eigen::Quaterniond(commanded.linear()).angularDistance(
        Eigen::Quaterniond(simulated_goal.linear()));
    const Eigen::Vector3d remaining =
        simulated_goal.translation() - commanded.translation();
    if (angle_error <= 0.0523598776 && remaining.norm() <= 0.003) break;
    const double requested_progress =
        std::min(1.0, simulated_progress + 0.02 / simulated_duration);
    target = current;
    target.linear() = rm75_vertical_press::stepTowardOrientation(
        current.linear(), simulated_goal.linear(),
        requested_progress * simulated_full_angle);
    target.translation() = current.translation() + requested_progress *
        (simulated_goal.translation() - current.translation());
    moveit::core::RobotState candidate(simulated);
    if (!candidate.setFromIK(group, target, tip, 0.015) ||
        !candidate.satisfiesBounds(group)) {
      controller_path_ok = false;
      break;
    }
    std::vector<double> before, after;
    simulated.copyJointGroupPositions(group, before);
    candidate.copyJointGroupPositions(group, after);
    double fraction = 1.0;
    for (std::size_t index = 0; index < before.size(); ++index) {
      const double delta = std::abs(after[index] - before[index]);
      if (delta > 0.004) fraction = std::min(fraction, 0.004 / delta);
    }
    for (std::size_t index = 0; index < before.size(); ++index) {
      after[index] = before[index] + fraction * (after[index] - before[index]);
    }
    simulated.setJointGroupPositions(group, after);
    simulated.update();
    simulated_progress += fraction *
        (requested_progress - simulated_progress);
  }
  std::cout << "controller_exact_simulation ok="
            << (controller_path_ok && controller_cycles < 5000 ? "yes" : "no")
            << " cycles=" << controller_cycles << "\n";
  return 0;
}
