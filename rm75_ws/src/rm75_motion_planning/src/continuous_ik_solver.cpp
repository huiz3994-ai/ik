#include "rm75_motion_planning/continuous_ik_solver.h"

#include <cmath>
#include <stdexcept>
#include "rm75_motion_planning/se3_interpolator.h"

namespace rm75_motion_planning {

ContinuousIkSolver::ContinuousIkSolver(
    const rm75_kinematics::KinematicsModel& model, Fallback ipopt_fallback)
    : model_(model), dls_(model), fallback_(std::move(ipopt_fallback)) {}

rm75_kinematics::Pose ContinuousIkSolver::convert(const Pose& pose) {
  rm75_kinematics::Pose out;
  out.position = pose.translation();
  out.rotation = pose.rotation();
  return out;
}

bool ContinuousIkSolver::solveSegment(
    const Pose& from_pose, const Eigen::VectorXd& from_q, const Pose& target,
    int depth, const ContinuousIkOptions& options,
    const rm75_kinematics::IkOptions& ik_options,
    ContinuousIkResult* result) const {
  ++result->dls_calls;
  rm75_kinematics::IkResult solved = dls_.solve(convert(target), from_q, ik_options);
  const bool dls_ok = solved.status == rm75_kinematics::IkStatus::SUCCESS &&
      solved.q.allFinite() && (solved.q - from_q).norm() <= options.max_joint_jump;
  if (dls_ok) {
    result->joints.push_back(solved.q);
    result->solved_poses.push_back(target);
    return true;
  }

  if (depth < options.max_refinement_depth) {
    ++result->refinements;
    const Pose midpoint = Se3Interpolator::interpolate(from_pose, target, 0.5);
    if (!solveSegment(from_pose, from_q, midpoint, depth + 1, options,
                      ik_options, result))
      return false;
    const Eigen::VectorXd midpoint_q = result->joints.back();
    return solveSegment(midpoint, midpoint_q, target, depth + 1, options,
                        ik_options, result);
  }

  if (options.enable_ipopt_fallback && fallback_) {
    ++result->fallback_calls;
    solved = fallback_(convert(target), from_q);
    if (solved.status == rm75_kinematics::IkStatus::SUCCESS &&
        solved.q.allFinite() &&
        (solved.q - from_q).norm() <= options.max_joint_jump) {
      result->joints.push_back(solved.q);
      result->solved_poses.push_back(target);
      return true;
    }
  }
  result->failure_reason = std::string("IK failed at refinement depth ") +
      std::to_string(depth) + ": " + solved.detail;
  return false;
}

ContinuousIkResult ContinuousIkSolver::solve(
    const PoseVector& poses, const Eigen::VectorXd& initial,
    const ContinuousIkOptions& options,
    const rm75_kinematics::IkOptions& ik_options) const {
  if (poses.empty()) throw std::invalid_argument("pose path is empty");
  if (initial.size() != model_.dof() || !initial.allFinite())
    throw std::invalid_argument("invalid initial joint vector");
  if (!(options.max_joint_jump > 0.0) || options.max_refinement_depth < 0)
    throw std::invalid_argument("invalid continuous IK options");

  ContinuousIkResult result;
  ++result.dls_calls;
  rm75_kinematics::IkResult first =
      dls_.solve(convert(poses.front()), initial, ik_options);
  bool first_ok = first.status == rm75_kinematics::IkStatus::SUCCESS &&
                  first.q.allFinite() &&
                  (first.q - initial).norm() <= options.max_joint_jump;
  if (!first_ok && options.enable_ipopt_fallback && fallback_) {
    ++result.fallback_calls;
    first = fallback_(convert(poses.front()), initial);
    first_ok = first.status == rm75_kinematics::IkStatus::SUCCESS &&
               first.q.allFinite() &&
               (first.q - initial).norm() <= options.max_joint_jump;
  }
  if (!first_ok) {
    result.failure_reason = "initial pose IK failed: " + first.detail;
    return result;
  }
  result.joints.push_back(first.q);
  result.solved_poses.push_back(poses.front());
  Pose from_pose = poses.front();
  Eigen::VectorXd from_q = first.q;
  for (std::size_t i = 1; i < poses.size(); ++i) {
    if (!solveSegment(from_pose, from_q, poses[i], 0, options, ik_options,
                      &result)) {
      result.failure_reason = "pose index " + std::to_string(i) + ": " +
                              result.failure_reason;
      return result;
    }
    from_pose = poses[i];
    from_q = result.joints.back();
  }
  result.success = true;
  return result;
}

}  // namespace rm75_motion_planning
