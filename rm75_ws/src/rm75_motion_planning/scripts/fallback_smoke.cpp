#include <iostream>
#include "rm75_motion_planning/continuous_ik_solver.h"
#include "rm75_motion_planning/persistent_ipopt_fallback.h"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: rm75_fallback_smoke PYTHON SERVER_PY OPTIMIZATION_IK_PY\n";
    return 2;
  }
  rm75_kinematics::KinematicsModel model(RM75_DEFAULT_URDF);
  rm75_motion_planning::PersistentIpoptFallback fallback(
      argv[1], argv[2], argv[3], RM75_DEFAULT_URDF, 10000);
  Eigen::VectorXd seed(7), goal(7);
  seed << -.3, -.15, -.2, -.35, -.25, .2, .4;
  goal << .2, -.5, .25, -.7, .3, .6, -.2;
  rm75_motion_planning::PoseVector poses{model.forward(goal).isometry()};
  rm75_motion_planning::ContinuousIkOptions continuous;
  continuous.max_refinement_depth = 0;
  continuous.max_joint_jump = 4.0;
  rm75_kinematics::IkOptions ik;
  ik.max_iterations = 1;
  ik.timeout_sec = .001;
  const auto result = rm75_motion_planning::ContinuousIkSolver(
      model, fallback.callback()).solve(poses, seed, continuous, ik);
  if (!result.success) {
    std::cerr << result.failure_reason << '\n';
    return 1;
  }
  const auto actual = model.forward(result.joints.back());
  const auto target = model.forward(goal);
  const double position_error = (actual.position - target.position).norm();
  std::cout << "fallback_calls=" << result.fallback_calls
            << " position_error_m=" << position_error << '\n';
  return result.fallback_calls == 1 && position_error <= .002 ? 0 : 1;
}
