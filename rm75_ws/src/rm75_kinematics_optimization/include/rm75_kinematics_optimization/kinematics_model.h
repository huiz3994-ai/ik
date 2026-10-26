#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Core>
#include "rm75_kinematics_optimization/types.h"

namespace rm75_kinematics {

class KinematicsModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit KinematicsModel(const std::string& urdf_path,
                           const std::string& tip_link = "Link7");
  ~KinematicsModel();
  KinematicsModel(KinematicsModel&&) noexcept;
  KinematicsModel& operator=(KinematicsModel&&) noexcept;
  KinematicsModel(const KinematicsModel&) = delete;
  KinematicsModel& operator=(const KinematicsModel&) = delete;

  int dof() const;
  Pose forward(const Eigen::VectorXd& q) const;
  // 6x7, LOCAL_WORLD_ALIGNED at the tip origin, rows [linear; angular].
  Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;
  JointLimits limits() const;
  const std::vector<std::string>& jointNames() const;
  static Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation);
  // Error expressed in the base/world-aligned frame, [translation; rotation].
  Vector6d poseError(const Pose& current, const Pose& target) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm75_kinematics
