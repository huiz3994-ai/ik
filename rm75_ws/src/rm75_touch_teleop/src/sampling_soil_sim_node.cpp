#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/Six_Force.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

// Analytic spring/damper soil model driven by Gazebo joint feedback, not target_pose.
// This models the measured load signal; it does NOT apply physical Gazebo contact forces.
class SamplingSoil {
 public:
  SamplingSoil() : pnh_("~"), loader_("robot_description") {
    bool sim = false; nh_.param("/use_sim_time", sim, false);
    if (!sim) throw std::runtime_error("Soil model requires /use_sim_time=true; never run with a real driver");
    model_ = loader_.getModel();
    if (!model_) throw std::runtime_error("robot_description missing");
    state_.reset(new moveit::core::RobotState(model_)); state_->setToDefaultValues();
    group_ = model_->getJointModelGroup("arm");
    pnh_.param<std::string>("tip_link", tip_, "Link7");
    pnh_.param("surface_z", surface_, 0.30);
    pnh_.param("surface_enabled", surface_enabled_, false);
    pnh_.param("stiffness", stiffness_, 5000.0);
    pnh_.param("damping", damping_, 20.0);
    pnh_.param("surface_offset", offset_, 0.01);
    if (!group_ || !model_->hasLinkModel(tip_) || !std::isfinite(surface_) ||
        !std::isfinite(stiffness_) || stiffness_ <= 0 || !std::isfinite(damping_) || damping_ < 0 ||
        !std::isfinite(offset_) || offset_ < 0) throw std::runtime_error("Invalid soil parameters/model");
    sub_ = nh_.subscribe("/joint_states", 1, &SamplingSoil::joints, this);
    pub_ = nh_.advertise<rm_msgs::Six_Force>("/sampling_sim/wrench", 1);
    surface_pub_ = pnh_.advertise<std_msgs::Float64>("surface_z", 1, true);
    service_ = pnh_.advertiseService("set_surface", &SamplingSoil::setSurface, this);
    ROS_INFO("Analytic soil sensor: /sampling_sim/wrench. Set surface after initial robot pose settles.");
    publishSurface();
  }
 private:
  void publishSurface() { std_msgs::Float64 m; m.data = surface_; surface_pub_.publish(m); }
  bool setSurface(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& r) {
    r.success = have_ && (ros::WallTime::now() - last_wall_).toSec() < 0.25;
    if (r.success) { surface_ = last_z_ - offset_; surface_enabled_ = true; publishSurface(); }
    r.message = r.success ? "Surface placed below current Link7 by surface_offset" : "No fresh joint feedback";
    return true;
  }
  void joints(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() != msg->position.size()) return;
    std::map<std::string, double> q;
    for (std::size_t i = 0; i < msg->name.size(); ++i)
      if (std::isfinite(msg->position[i])) q[msg->name[i]] = msg->position[i];
    for (const auto& name : group_->getVariableNames()) if (!q.count(name)) return;
    state_->setVariablePositions(q); state_->update();
    const double z = state_->getGlobalLinkTransform(tip_).translation().z();
    const ros::Time now = ros::Time::now();
    const double dt = (now - last_stamp_).toSec();
    // Do not manufacture sensor freshness if the physics clock is paused.
    if (now.isZero() || (have_ && dt <= 0)) return;
    const double vz = have_ && dt < 0.25 ? (z - last_z_) / dt : 0.0;
    const double depth = std::max(0.0, surface_ - z);
    rm_msgs::Six_Force force;
    force.force_Fz = surface_enabled_ && depth > 0 ? -std::max(0.0, stiffness_ * depth - damping_ * vz) : 0.0;
    pub_.publish(force);
    last_z_ = z; last_stamp_ = now; last_wall_ = ros::WallTime::now(); have_ = true;
  }
  ros::NodeHandle nh_, pnh_;
  robot_model_loader::RobotModelLoader loader_;
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<moveit::core::RobotState> state_;
  const moveit::core::JointModelGroup* group_;
  ros::Subscriber sub_; ros::Publisher pub_, surface_pub_; ros::ServiceServer service_;
  std::string tip_; double surface_, stiffness_, damping_, offset_, last_z_ = 0;
  ros::Time last_stamp_; ros::WallTime last_wall_; bool have_ = false, surface_enabled_ = false;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "sampling_soil");
  try { SamplingSoil soil; ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL("%s", e.what()); return 1; }
  return 0;
}
