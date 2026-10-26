#pragma once

#include <memory>
#include <string>
#include "rm75_motion_planning/trajectory_validator.h"

namespace rm75_motion_planning {

class MoveitCollisionChecker : public CollisionChecker {
 public:
  MoveitCollisionChecker(const std::string& urdf_path,
                         const std::string& srdf_path,
                         const std::string& group_name = "arm");
  ~MoveitCollisionChecker() override;
  MoveitCollisionChecker(MoveitCollisionChecker&&) noexcept;
  MoveitCollisionChecker& operator=(MoveitCollisionChecker&&) noexcept;
  MoveitCollisionChecker(const MoveitCollisionChecker&) = delete;
  MoveitCollisionChecker& operator=(const MoveitCollisionChecker&) = delete;

  bool inCollision(const Eigen::VectorXd& q,
                   std::string* detail = nullptr) const override;
  void addBox(const std::string& id, const Eigen::Vector3d& size,
              const Eigen::Isometry3d& base_pose);
  void removeObject(const std::string& id);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm75_motion_planning
