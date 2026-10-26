#include "rm75_kinematics_optimization/kinematics_model.h"

#include <cmath>
#include <stdexcept>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace rm75_kinematics {

struct KinematicsModel::Impl {
  pinocchio::Model model;
  mutable pinocchio::Data data;
  pinocchio::FrameIndex tip_frame;
  std::vector<std::string> joint_names;
  explicit Impl(const std::string& path, const std::string& tip)
      : model(), data(model), tip_frame(0) {
    pinocchio::urdf::buildModel(path, model);
    data = pinocchio::Data(model);
    if (!model.existFrame(tip)) throw std::runtime_error("Tip frame not found: " + tip);
    tip_frame = model.getFrameId(tip);
    if (model.nq != 7 || model.nv != 7)
      throw std::runtime_error("RM75-6FB model must have exactly seven 1-DoF joints");
    for (int i = 1; i < model.njoints; ++i)
      if (model.joints[i].nv() == 1) joint_names.push_back(model.names[i]);
  }
};

KinematicsModel::KinematicsModel(const std::string& path, const std::string& tip)
    : impl_(new Impl(path, tip)) {}
KinematicsModel::~KinematicsModel() = default;
KinematicsModel::KinematicsModel(KinematicsModel&&) noexcept = default;
KinematicsModel& KinematicsModel::operator=(KinematicsModel&&) noexcept = default;
int KinematicsModel::dof() const { return impl_->model.nv; }

static void validateQ(const Eigen::VectorXd& q, int n) {
  if (q.size() != n || !q.allFinite()) throw std::invalid_argument("Invalid joint vector");
}

Pose KinematicsModel::forward(const Eigen::VectorXd& q) const {
  validateQ(q, dof());
  pinocchio::forwardKinematics(impl_->model, impl_->data, q);
  pinocchio::updateFramePlacements(impl_->model, impl_->data);
  const auto& transform = impl_->data.oMf[impl_->tip_frame];
  Pose result; result.position = transform.translation(); result.rotation = transform.rotation();
  return result;
}

Eigen::MatrixXd KinematicsModel::jacobian(const Eigen::VectorXd& q) const {
  validateQ(q, dof());
  Eigen::MatrixXd j(6, impl_->model.nv); j.setZero();
  pinocchio::computeJointJacobians(impl_->model, impl_->data, q);
  pinocchio::updateFramePlacements(impl_->model, impl_->data);
  pinocchio::getFrameJacobian(impl_->model, impl_->data, impl_->tip_frame,
                             pinocchio::LOCAL_WORLD_ALIGNED, j);
  return j;  // Pinocchio Motion ordering is [linear; angular].
}

JointLimits KinematicsModel::limits() const {
  return {impl_->model.lowerPositionLimit, impl_->model.upperPositionLimit};
}
const std::vector<std::string>& KinematicsModel::jointNames() const { return impl_->joint_names; }

Eigen::Vector3d KinematicsModel::so3Log(const Eigen::Matrix3d& r) {
  if (!r.allFinite()) throw std::invalid_argument("Non-finite rotation");
  Eigen::Quaterniond q(r); q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double s = q.vec().norm();
  if (s < 1e-12) return 2.0 * q.vec();
  return (2.0 * std::atan2(s, q.w()) / s) * q.vec();
}

Vector6d KinematicsModel::poseError(const Pose& current, const Pose& target) const {
  Vector6d e;
  e.head<3>() = target.position - current.position;
  e.tail<3>() = so3Log(target.rotation * current.rotation.transpose());
  return e;
}

}  // namespace rm75_kinematics
