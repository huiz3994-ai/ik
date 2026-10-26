#include <condition_variable>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
namespace gazebo {
// Idealized attachment model, not a friction/contact validation of a physical
// gripper.
class RM75SimGripper : public ModelPlugin {
  physics::ModelPtr model;
  physics::JointPtr attachment;
  event::ConnectionPtr update;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::ServiceServer service;
  ros::Publisher holding;
  std::mutex mutex;
  std::condition_variable done;
  int request = -1;
  bool result = false;
  std::string message;
  ros::WallTime published;
  bool command(std_srvs::SetBool::Request &q, std_srvs::SetBool::Response &r) {
    std::unique_lock<std::mutex> l(mutex);
    request = q.data ? 1 : 0;
    if (!done.wait_for(l, std::chrono::seconds(2),
                       [this] { return request == -1; })) {
      r.success = false;
      r.message = "Gazebo update timeout";
      request = -1;
      return true;
    }
    r.success = result;
    r.message = message;
    return true;
  }
  void tick() {
    std::lock_guard<std::mutex> l(mutex);
    if (request >= 0) {
      result = false;
      auto tcp = model->GetLink("grasp_tcp");
      auto target = model->GetWorld()->ModelByName("target");
      if (request == 0) {
        if (attachment) {
          attachment->Detach();
          attachment.reset();
        }
        result = true;
        message = "opened";
      } else if (tcp && target && target->GetLink("link")) {
        auto object = target->GetLink("link");
        double d =
            (tcp->WorldPose().Pos() - object->WorldPose().Pos()).Length();
        if (d < 0.035) {
          if (!attachment) {
            attachment =
                model->GetWorld()->Physics()->CreateJoint("fixed", model);
            attachment->Load(tcp, object, ignition::math::Pose3d());
            attachment->Init();
          }
          result = true;
          message = "idealized fixed attachment";
        } else
          message = "target outside 35 mm grasp capture radius";
      } else
        message = "TCP or target missing";
      request = -1;
      done.notify_all();
    }
    if ((ros::WallTime::now() - published).toSec() > .1) {
      std_msgs::Bool h;
      h.data = bool(attachment);
      holding.publish(h);
      published = ros::WallTime::now();
    }
  }

public:
  void Load(physics::ModelPtr m, sdf::ElementPtr) override {
    model = m;
    if (!ros::isInitialized()) {
      ROS_FATAL("gazebo_ros_api_plugin must initialize ROS");
      return;
    }
    nh.reset(new ros::NodeHandle);
    service = nh->advertiseService("/sim_gripper/close",
                                   &RM75SimGripper::command, this);
    holding = nh->advertise<std_msgs::Bool>("/gripper/object_detected", 1);
    update = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RM75SimGripper::tick, this));
  }
};
GZ_REGISTER_MODEL_PLUGIN(RM75SimGripper)
} // namespace gazebo
