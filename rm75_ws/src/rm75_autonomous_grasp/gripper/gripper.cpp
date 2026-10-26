#include <rm75_autonomous_grasp/core.h>
#include <rm_msgs/Gripper_Pick.h>
#include <rm_msgs/Gripper_Set.h>
namespace rm75 {
Gripper::Gripper(ros::NodeHandle &p) : p_(p) {
  p.param("simulation", simulation, true);
  p.param("gripper_timeout", timeout, 5.0);
  p.param("gripper_speed", speed, 200);
  p.param("gripper_force", force, 200);
  p.param("gripper_feedback_verified", verified, false);
  sim_ = nh_.serviceClient<std_srvs::SetBool>("/sim_gripper/close");
  open_ = nh_.advertise<rm_msgs::Gripper_Set>("/rm_driver/Gripper_Set", 1);
  close_ = nh_.advertise<rm_msgs::Gripper_Pick>("/rm_driver/Gripper_Pick", 1);
  ack_ = nh_.subscribe<std_msgs::Bool>("/rm_driver/Set_Gripper_Result", 1,
                                       [this](const std_msgs::BoolConstPtr &m) {
                                         std::lock_guard<std::mutex> l(mutex_);
                                         ack = true;
                                         ok = m->data;
                                       });
  holding_ = nh_.subscribe<std_msgs::Bool>(
      "/gripper/object_detected", 1, [this](const std_msgs::BoolConstPtr &m) {
        std::lock_guard<std::mutex> l(mutex_);
        held = m->data;
        held_time = ros::WallTime::now();
      });
}
bool Gripper::holding(std::string &error) {
  std::lock_guard<std::mutex> l(mutex_);
  if ((!simulation && !verified) || !held || (ros::WallTime::now() - held_time).toSec() > 0.5) {
    error = "GRASP_UNCONFIRMED: configure physical object feedback";
    return false;
  }
  return true;
}
bool Gripper::command(bool close, std::string &error) {
  if (simulation) {
    std_srvs::SetBool s;
    s.request.data = close;
    if (!sim_.call(s) || !s.response.success) {
      error = "GRIPPER_FAIL: " + s.response.message;
      return false;
    }
    return true;
  }
  if (!verified) {
    error = "GRIPPER_NOT_CONFIGURED";
    return false;
  }
  {
    std::lock_guard<std::mutex> l(mutex_);
    ack = false;
  }
  if (close) {
    rm_msgs::Gripper_Pick m;
    m.speed = speed;
    m.force = force;
    close_.publish(m);
  } else {
    rm_msgs::Gripper_Set m;
    m.position = 1000;
    open_.publish(m);
  }
  auto t = ros::WallTime::now();
  while (ros::ok() && (ros::WallTime::now() - t).toSec() < timeout) {
    {
      std::lock_guard<std::mutex> l(mutex_);
      if (ack) {
        if (!ok) {
          error = "GRIPPER_COMMAND_REJECTED";
          return false;
        }
        if (!close)
          return true;
        if (held && (ros::WallTime::now() - held_time).toSec() < 0.5)
          return true;
      }
    }
    ros::WallDuration(0.05).sleep();
  }
  error = "TIMEOUT: gripper acknowledgement/object feedback";
  return false;
}
} // namespace rm75
