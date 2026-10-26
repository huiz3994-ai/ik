#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <cmath>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <map>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <mutex>
#include <rm_msgs/JointPos.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
// Replaces rm_control only in the autonomous real launch. Never launch both.
class Bridge {
  ros::NodeHandle nh, p{"~"};
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
      server;
  ros::Publisher command, stop;
  ros::Subscriber joint, stop_ack;
  std::mutex mutex;
  std::map<std::string, double> position;
  ros::WallTime last;
  bool ack = false, enabled = false;
  double tolerance, period, max_velocity;
  moveit::core::RobotModelPtr model;
  ros::Time stamp;
  std::vector<std::string> names;
  bool sample(std::vector<double> &q) {
    std::lock_guard<std::mutex> l(mutex);
    if (last.isZero() || (ros::WallTime::now() - last).toSec() > 0.25 ||
        stamp.isZero() || (ros::Time::now()-stamp).toSec()>0.25 ||
        (stamp-ros::Time::now()).toSec()>0.05)
      return false;
    for (auto &n : names) {
      if (!position.count(n))
        return false;
      q.push_back(position[n]);
    }
    return true;
  }
  bool halt() {
    {
      std::lock_guard<std::mutex> l(mutex);
      ack = false;
    }
    stop.publish(std_msgs::Empty());
    auto start = ros::WallTime::now();
    std::vector<double> previous;
    int stable = 0;
    while (ros::ok() && (ros::WallTime::now() - start).toSec() < 3) {
      std::vector<double> q;
      bool good = sample(q) && !previous.empty();
      for (size_t i = 0; i < q.size() && i < previous.size(); ++i)
        good = good && std::abs(q[i] - previous[i]) < 0.001;
      bool acknowledged;
      {
        std::lock_guard<std::mutex> l(mutex);
        acknowledged = ack;
      }
      stable = good ? stable + 1 : 0;
      if (stable >= 5 && acknowledged)
        return true;
      previous = q;
      ros::WallDuration(.1).sleep();
    }
    return false;
  }
  void
  abort(const std::string &msg,
        int code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL) {
    bool stopped = halt();
    control_msgs::FollowJointTrajectoryResult r;
    r.error_code = code;
    r.error_string = msg + (stopped ? "" : "; STOP_UNCONFIRMED");
    server.setAborted(r);
  }

public:
  Bridge()
      : server(nh, "/rm_group/follow_joint_trajectory",
               boost::bind(&Bridge::run, this, _1), false) {
    p.param("execute", enabled, false);
    bool calibrated = false, tool = false, stop_test = false, grip = false;
    bool require_gripper_feedback = true;
    p.param("hand_eye_calibrated", calibrated, false);
    p.param("tool_verified", tool, false);
    p.param("stop_verified", stop_test, false);
    p.param("gripper_feedback_verified", grip, false);
    p.param("require_gripper_feedback", require_gripper_feedback, true);
    enabled = enabled && calibrated && tool && stop_test &&
              (!require_gripper_feedback || grip);
    // Expose the derived gate, not merely the requested execute flag, so task
    // layers can fail closed before sending an action goal.
    p.setParam("enabled", enabled);
    p.param("goal_tolerance", tolerance, 0.015);
    p.param("period", period, 0.01);
    p.param("max_joint_velocity", max_velocity, 0.3);
    if(period<0.002 || period>0.1 || tolerance<=0 || max_velocity<=0)
      throw std::runtime_error("Invalid bridge limits");
    robot_model_loader::RobotModelLoader loader("robot_description");
    model=loader.getModel();
    if(!model) enabled=false;
    for (int i = 1; i <= 7; ++i)
      names.push_back("joint" + std::to_string(i));
    command = nh.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 1);
    stop = nh.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1);
    joint = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 1, [this](const sensor_msgs::JointStateConstPtr &m) {
          if (m->name.size() != m->position.size()) return;
          std::map<std::string,double> next;
          for(size_t i=0;i<m->name.size();++i) {
            if(!std::isfinite(m->position[i])) return;
            next[m->name[i]]=m->position[i];
          }
          for(const auto& n:names) if(!next.count(n)) return;
          std::lock_guard<std::mutex> l(mutex);
          position=next;
          stamp=m->header.stamp;
          last = ros::WallTime::now();
        });
    stop_ack =
        nh.subscribe<std_msgs::Bool>("/rm_driver/Set_Arm_Stop_Result", 1,
                                     [this](const std_msgs::BoolConstPtr &m) {
                                       std::lock_guard<std::mutex> l(mutex);
                                       ack = m->data;
                                     });
    server.start();
  }
  void run(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {
    const auto &t = goal->trajectory;
    if (!enabled) {
      control_msgs::FollowJointTrajectoryResult r;
      r.error_code = r.INVALID_GOAL;
      r.error_string = "Real execution disabled";
      server.setAborted(r);
      return;
    }
    if (t.joint_names.size() != 7 || t.points.size() < 2) {
      abort("Need seven joints and >=2 points");
      return;
    }
    std::vector<size_t> order;
    for (auto &n : names) {
      auto it = std::find(t.joint_names.begin(), t.joint_names.end(), n);
      if (it == t.joint_names.end()) {
        abort("Unknown/duplicate joint");
        return;
      }
      order.push_back(it - t.joint_names.begin());
    }
    double previous = -1;
    for (auto &point : t.points) {
      double ts = point.time_from_start.toSec();
      if (point.positions.size() != 7 || ts < 0 || ts <= previous) {
        abort("Invalid dimensions/time");
        return;
      }
      for (double q : point.positions)
        if (!std::isfinite(q)) {
          abort("Nonfinite joint target");
          return;
        }
      previous = ts;
    }
    for(size_t i=0;i<t.points.size();++i) {
      for(size_t j=0;j<names.size();++j) {
        const auto& bounds=model->getVariableBounds(names[j]);
        double q=t.points[i].positions[order[j]];
        if(bounds.position_bounded_ && (q<bounds.min_position_ || q>bounds.max_position_)) {
          abort("Joint limit violation"); return;
        }
        if(i>0) {
          double speed=std::abs(q-t.points[i-1].positions[order[j]])/
            (t.points[i].time_from_start-t.points[i-1].time_from_start).toSec();
          double limit=bounds.velocity_bounded_?std::min(max_velocity,bounds.max_velocity_):max_velocity;
          if(speed>limit+1e-6) { abort("Segment velocity limit violation"); return; }
        }
      }
    }
    std::vector<double> current;
    if (!sample(current)) {
      abort("Joint state stale");
      return;
    }
    for (size_t j = 0; j < 7; ++j)
      if (std::abs(current[j] - t.points.front().positions[order[j]]) > 0.05) {
        abort("Start state mismatch");
        return;
      }
    if (!t.header.stamp.isZero() &&
        std::abs((t.header.stamp - ros::Time::now()).toSec()) > 0.1) {
      abort("Scheduled trajectory headers unsupported");
      return;
    }
    auto start = ros::WallTime::now();
    size_t segment = 1;
    ros::WallRate rate(1 / period);
    while (ros::ok()) {
      if (server.isPreemptRequested()) {
        if (!halt()) {
          abort("STOP_UNCONFIRMED");
          return;
        }
        control_msgs::FollowJointTrajectoryResult r;
        server.setPreempted(r, "Stopped and acknowledged");
        return;
      }
      current.clear();
      if (!sample(current)) {
        abort("Joint feedback lost");
        return;
      }
      double elapsed = (ros::WallTime::now() - start).toSec();
      while (segment + 1 < t.points.size() &&
             t.points[segment].time_from_start.toSec() < elapsed)
        ++segment;
      auto &a = t.points[segment - 1];
      auto &b = t.points[segment];
      double u = std::max(
          0.0,
          std::min(1.0, (elapsed - a.time_from_start.toSec()) /
                            (b.time_from_start - a.time_from_start).toSec()));
      rm_msgs::JointPos cmd;
      cmd.joint.resize(7);
      control_msgs::FollowJointTrajectoryFeedback f;
      f.header.stamp = ros::Time::now();
      f.joint_names = names;
      f.actual.positions = current;
      f.desired.positions.resize(7);
      f.error.positions.resize(7);
      for (size_t j = 0; j < 7; ++j) {
        cmd.joint[j] = a.positions[order[j]] +
                       (b.positions[order[j]] - a.positions[order[j]]) * u;
        f.desired.positions[j] = cmd.joint[j];
        f.error.positions[j] = cmd.joint[j] - current[j];
        if (std::abs(f.error.positions[j]) > 0.2) {
          abort("Path tolerance exceeded",
                control_msgs::FollowJointTrajectoryResult::
                    PATH_TOLERANCE_VIOLATED);
          return;
        }
      }
      command.publish(cmd);
      server.publishFeedback(f);
      if (elapsed >= t.points.back().time_from_start.toSec()) {
        bool reached = true;
        for (double e : f.error.positions)
          reached = reached && std::abs(e) < tolerance;
        if (reached) {
          control_msgs::FollowJointTrajectoryResult r;
          r.error_code = r.SUCCESSFUL;
          server.setSucceeded(r);
          return;
        }
        if (elapsed > t.points.back().time_from_start.toSec() + 3) {
          abort("Goal tolerance timeout",
                control_msgs::FollowJointTrajectoryResult::
                    GOAL_TOLERANCE_VIOLATED);
          return;
        }
      }
      rate.sleep();
    }
    halt();
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "rm75_trajectory_bridge");
  Bridge b;
  ros::AsyncSpinner spin(3);
  spin.start();
  ros::waitForShutdown();
}
