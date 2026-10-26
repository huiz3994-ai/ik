#pragma once

#include <memory>
#include <string>
#include "rm75_motion_planning/continuous_ik_solver.h"

namespace rm75_motion_planning {

// Keeps one CasADi/Ipopt Python process alive, so the symbolic NLP is built once
// and warm-start state is retained across fallback calls.
class PersistentIpoptFallback {
 public:
  PersistentIpoptFallback(const std::string& python_executable,
                          const std::string& server_script,
                          const std::string& optimization_ik_script,
                          const std::string& urdf_path,
                          int response_timeout_ms = 5000);
  ~PersistentIpoptFallback();
  PersistentIpoptFallback(const PersistentIpoptFallback&) = delete;
  PersistentIpoptFallback& operator=(const PersistentIpoptFallback&) = delete;

  rm75_kinematics::IkResult solve(const rm75_kinematics::Pose& target,
                                  const Eigen::VectorXd& seed);
  ContinuousIkSolver::Fallback callback();
  bool healthy() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm75_motion_planning
