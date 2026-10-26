#include "rm75_motion_planning/moveit_collision_checker.h"

#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace rm75_motion_planning {

struct MoveitCollisionChecker::Impl {
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  const moveit::core::JointModelGroup* group{nullptr};
  std::string group_name;
  std::vector<std::string> variables;
  mutable std::mutex mutex;

  Impl(const std::string& urdf_path, const std::string& srdf_path,
       const std::string& requested_group) : group_name(requested_group) {
    std::ifstream input(urdf_path);
    if (!input) throw std::runtime_error("cannot open URDF: " + urdf_path);
    const std::string xml((std::istreambuf_iterator<char>(input)), {});
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(xml);
    if (!urdf_model) throw std::runtime_error("cannot parse URDF: " + urdf_path);
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initFile(*urdf_model, srdf_path))
      throw std::runtime_error("cannot parse SRDF: " + srdf_path);
    model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    group = model->getJointModelGroup(group_name);
    if (!group) throw std::runtime_error("MoveIt group not found: " + group_name);
    variables = group->getVariableNames();
    if (variables.size() != 7)
      throw std::runtime_error("RM75 collision group must have seven variables");
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setActiveCollisionDetector(
        collision_detection::CollisionDetectorAllocatorFCL::create(), true);
  }
};

MoveitCollisionChecker::MoveitCollisionChecker(const std::string& urdf_path,
                                               const std::string& srdf_path,
                                               const std::string& group_name)
    : impl_(new Impl(urdf_path, srdf_path, group_name)) {}
MoveitCollisionChecker::~MoveitCollisionChecker() = default;
MoveitCollisionChecker::MoveitCollisionChecker(MoveitCollisionChecker&&) noexcept = default;
MoveitCollisionChecker& MoveitCollisionChecker::operator=(MoveitCollisionChecker&&) noexcept = default;

bool MoveitCollisionChecker::inCollision(const Eigen::VectorXd& q,
                                         std::string* detail) const {
  if (q.size() != 7 || !q.allFinite())
    throw std::invalid_argument("collision check requires a finite 7-DOF vector");
  std::lock_guard<std::mutex> lock(impl_->mutex);
  moveit::core::RobotState state(impl_->model);
  state.setToDefaultValues();
  state.setJointGroupPositions(impl_->group,
      std::vector<double>(q.data(), q.data() + q.size()));
  state.update();
  collision_detection::CollisionRequest request;
  request.group_name = impl_->group_name;
  request.contacts = true;
  request.max_contacts = 1;
  request.max_contacts_per_pair = 1;
  collision_detection::CollisionResult result;
  impl_->scene->checkCollision(request, result, state);
  if (detail) {
    detail->clear();
    if (result.collision) {
      if (!result.contacts.empty()) {
        const auto& pair = result.contacts.begin()->first;
        *detail = pair.first + " <-> " + pair.second;
      } else {
        *detail = "collision (contact names unavailable)";
      }
    }
  }
  return result.collision;
}

void MoveitCollisionChecker::addBox(const std::string& id,
                                    const Eigen::Vector3d& size,
                                    const Eigen::Isometry3d& base_pose) {
  if (id.empty() || !size.allFinite() || (size.array() <= 0.0).any() ||
      !base_pose.matrix().allFinite())
    throw std::invalid_argument("invalid collision box");
  moveit_msgs::CollisionObject object;
  object.id = id;
  object.header.frame_id = impl_->model->getModelFrame();
  object.operation = moveit_msgs::CollisionObject::ADD;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions = {size.x(), size.y(), size.z()};
  object.primitives.push_back(primitive);
  geometry_msgs::Pose pose;
  pose.position.x = base_pose.translation().x();
  pose.position.y = base_pose.translation().y();
  pose.position.z = base_pose.translation().z();
  Eigen::Quaterniond quaternion(base_pose.rotation());
  quaternion.normalize();
  pose.orientation.x = quaternion.x(); pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z(); pose.orientation.w = quaternion.w();
  object.primitive_poses.push_back(pose);
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->scene->processCollisionObjectMsg(object))
    throw std::runtime_error("MoveIt rejected collision object: " + id);
}

void MoveitCollisionChecker::removeObject(const std::string& id) {
  moveit_msgs::CollisionObject object;
  object.id = id;
  object.header.frame_id = impl_->model->getModelFrame();
  object.operation = moveit_msgs::CollisionObject::REMOVE;
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->scene->processCollisionObjectMsg(object))
    throw std::runtime_error("MoveIt rejected collision object removal: " + id);
}

}  // namespace rm75_motion_planning
