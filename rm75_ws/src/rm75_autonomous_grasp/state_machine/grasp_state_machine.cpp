#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <rm75_autonomous_grasp/GraspAction.h>
#include <rm75_autonomous_grasp/core.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
class GraspAvoidStateMachine {
  ros::NodeHandle nh, p{"~"};
  rm75::MoveItPlanner planner;
  rm75::Gripper gripper;
  rm75::GraspPlanner grasps;
  actionlib::SimpleActionServer<rm75_autonomous_grasp::GraspAction> server;
  tf2_ros::Buffer tf;
  tf2_ros::TransformListener listener{tf};
  ros::Subscriber sub;
  ros::Publisher status;
  std::mutex mutex;
  geometry_msgs::PoseStamped observation;
  ros::WallTime received;
  double object_timeout, approach, lift, offset, size, replan_wait;
  int retries;
  bool attached = false;
  void state(const std::string &value, const std::string &detail = "") {
    std_msgs::String m;
    m.data = value + ": " + detail;
    status.publish(m);
    ROS_INFO("%s", m.data.c_str());
    rm75_autonomous_grasp::GraspFeedback f;
    f.state = value;
    f.detail = detail;
    if (server.isActive())
      server.publishFeedback(f);
  }
  bool cancelled() { return server.isPreemptRequested() || !ros::ok(); }
  bool move(const geometry_msgs::PoseStamped &pose, std::string &error) {
    for (int i = 0; i <= retries && !cancelled(); ++i) {
      if(i) {
        const auto until=ros::WallTime::now()+ros::WallDuration(replan_wait);
        while(ros::ok() && !cancelled() && ros::WallTime::now()<until)
          ros::WallDuration(0.05).sleep();
        if(cancelled()) break;
      }
      state("PLAN_PATH", i ? "Replan from measured stopped state" : "");
      rm75::Plan path;
      error.clear();
      if (!planner.plan(pose, path, error)) {
        if ((i==0 && error.find("IK_FAIL") == 0) || error.find("MAP_") == 0)
          return false;
        continue;
      }
      state("EXECUTE_TRAJECTORY");
      if (planner.run(path, [this] { return cancelled(); }, error))
        return true;
      if (error.find("COLLISION") != 0)
        return false;
      state("COLLISION", error);
    }
    if (cancelled())
      error = "CANCELLED";
    return false;
  }
  bool allowTouch(bool allow) {
    moveit_msgs::GetPlanningScene get;
    get.request.components.components =
        moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    if (!ros::service::call("/get_planning_scene", get))
      return false;
    collision_detection::AllowedCollisionMatrix acm(
        get.response.scene.allowed_collision_matrix);
    for (const std::string link :
         {"gripper_body", "left_finger", "right_finger", "grasp_tcp"})
      acm.setEntry("grasp_target", link, allow);
    moveit_msgs::PlanningScene diff;
    diff.is_diff = true;
    acm.getMessage(diff.allowed_collision_matrix);
    if (!planner.scene.applyPlanningScene(diff)) return false;
    return planner.monitor->requestPlanningSceneState("/get_planning_scene");
  }
  void finish(bool success, const std::string &error) {
    rm75_autonomous_grasp::GraspResult r;
    r.success = success;
    r.error = error;
    if (success) {
      state("SUCCESS");
      server.setSucceeded(r);
    } else {
      std::string label = error.substr(0, error.find(':'));
      if (label != "TF_FAIL" && label != "IK_FAIL" && label != "COLLISION" &&
          label != "TIMEOUT" && label != "VISION_FAIL" && label != "CANCELLED")
        label = "PLAN_FAIL";
      state(label, error);
      if (cancelled())
        server.setPreempted(r);
      else
        server.setAborted(r);
    }
    if (!attached)
      allowTouch(false);
  }

public:
  GraspAvoidStateMachine()
      : planner(p), gripper(p),
        server(nh, "/grasp",
               boost::bind(&GraspAvoidStateMachine::run, this, _1), false) {
    p.param("object_timeout", object_timeout, 10.0);
    p.param("approach_distance", approach, 0.10);
    p.param("lift_distance", lift, 0.12);
    p.param("surface_to_tcp_offset", offset, -0.02);
    p.param("object_size", size, 0.04);
    p.param("max_replans", retries, 3);
    p.param("replan_map_wait", replan_wait, 1.0);
    if(object_timeout<=0 || approach<=0 || lift<=0 || size<=0 || retries<0 ||
       !std::isfinite(offset) || !std::isfinite(replan_wait) || replan_wait<0 || replan_wait>10) throw std::runtime_error("Invalid task configuration");
    status = nh.advertise<std_msgs::String>("/grasp/state", 1, true);
    sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/perception/object_pose", 1,
        [this](const geometry_msgs::PoseStampedConstPtr &m) {
          std::lock_guard<std::mutex> l(mutex);
          observation = *m;
          received = ros::WallTime::now();
        });
    server.start();
    state("IDLE");
  }
  void run(const rm75_autonomous_grasp::GraspGoalConstPtr &goal) {
    std::string error;
    try {
      if (attached) {
        finish(false, "OBJECT_ALREADY_HELD: release/reset before next task");
        return;
      }
      if (!planner.enabled()) {
        finish(false, "EXECUTION_DISABLED: use RViz or planning services to "
                      "inspect plans");
        return;
      }
      bool simulation = true;
      p.param("simulation", simulation, true);
      if (simulation && !nh.param<bool>("/observation_ready", false)) {
        finish(false, "TIMEOUT: observation pose initialization incomplete");
        return;
      }
      state("WAIT_OBJECT");
      auto start = ros::WallTime::now();
      geometry_msgs::PoseStamped object;
      while (ros::ok() && !cancelled() &&
             (ros::WallTime::now() - start).toSec() < object_timeout) {
        {
          std::lock_guard<std::mutex> l(mutex);
          if (!received.isZero() &&
              (ros::WallTime::now() - received).toSec() < 0.5 &&
              (ros::Time::now() - observation.header.stamp).toSec() >= 0 &&
              (ros::Time::now() - observation.header.stamp).toSec() < 0.5) {
            object = observation;
            break;
          }
        }
        ros::WallDuration(0.05).sleep();
      }
      if (cancelled()) {
        finish(false, "CANCELLED");
        return;
      }
      if (object.header.frame_id.empty()) {
        finish(false, "VISION_FAIL: no fresh object");
        return;
      }
      if(!std::isfinite(object.pose.position.x) || !std::isfinite(object.pose.position.y) ||
         !std::isfinite(object.pose.position.z)) { finish(false,"VISION_FAIL: nonfinite position"); return; }
      state("VISION_DETECT");
      for (const std::string frame : {"world", "camera_link"})
        tf.lookupTransform("base_link", frame, object.header.stamp,
                           ros::Duration(0.5));
      if (object.header.frame_id != "base_link")
        throw tf2::TransformException("object pose frame must be base_link");
      state("POSE_ESTIMATION");
      state("UPDATE_OBSTACLE_MAP");
      if (!planner.ready(error)) {
        finish(false, error);
        return;
      }
      moveit_msgs::CollisionObject target;
      target.id = "grasp_target";
      target.header = object.header;
      shape_msgs::SolidPrimitive box;
      box.type = box.BOX;
      box.dimensions = {size, size, size};
      target.primitives.push_back(box);
      auto center = object.pose;
      center.orientation.x = center.orientation.y = center.orientation.z = 0;
      center.orientation.w = 1;
      center.position.z -= size / 2;
      target.primitive_poses.push_back(center);
      target.operation = target.ADD;
      if (!planner.scene.applyCollisionObject(target)) {
        finish(false, "PLAN_FAIL: scene update rejected");
        return;
      }
      ros::WallDuration(0.6).sleep(); // allow world-shape exclusion to reach the octomap updater
      planner.monitor->requestPlanningSceneState("/get_planning_scene");
      auto candidates = grasps.generate(
          object, goal->grasp_mode.empty() ? "top" : goal->grasp_mode, approach,
          lift, offset);
      rm75::Candidate chosen;
      rm75::Plan path;
      bool found = false;
      if (!gripper.command(false, error)) {
        finish(false, error);
        return;
      }
      state("PLAN_PATH", "select feasible pregrasp");
      for (auto &c : candidates) {
        error.clear();
        if (planner.plan(c.pre, path, error)) {
          chosen = c;
          found = true;
          break;
        }
      }
      if (!found) {
        finish(false, error);
        return;
      }
      state("EXECUTE_TRAJECTORY", "pregrasp");
      if (!planner.run(
              path, [this] { return cancelled(); }, error) &&
          !((error.find("COLLISION") == 0) && move(chosen.pre, error))) {
        finish(false, error);
        return;
      }
      if (!allowTouch(true)) {
        finish(false, "PLAN_FAIL: touch ACM update");
        return;
      }
      if (!move(chosen.grasp, error)) {
        finish(false, error);
        return;
      }
      state("GRASP");
      if (!gripper.command(true, error)) {
        finish(false, error);
        return;
      }
      attached = true; // physical closure succeeded; retain held-object state even if scene update fails
      moveit_msgs::AttachedCollisionObject a;
      a.link_name = "grasp_tcp";
      a.object = target;
      a.touch_links = {"grasp_tcp", "gripper_body", "left_finger",
                       "right_finger"};
      if (!planner.scene.applyAttachedCollisionObject(a)) {
        finish(false, "PLAN_FAIL: attachment rejected; object may be held");
        return;
      }
      attached = true;
      state("LIFT");
      if (!move(chosen.lift, error) || !gripper.holding(error)) {
        finish(false, error);
        return;
      }
      finish(true, "");
    } catch (const tf2::TransformException &e) {
      finish(false, std::string("TF_FAIL: ") + e.what());
    } catch (const std::exception &e) {
      std::string stop;
      planner.stopAndWait(stop);
      finish(false, std::string("PLAN_FAIL: ") + e.what());
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_state_machine");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  try {
    GraspAvoidStateMachine n;
    ros::waitForShutdown();
  } catch (const std::exception &e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
