#include <cmath>
#include <moveit/robot_state/conversions.h>
#include <rm75_autonomous_grasp/core.h>
#include <sstream>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
namespace rm75 {
bool collisionFree(const planning_scene::PlanningScene &scene,
                   const moveit_msgs::RobotTrajectory &trajectory, size_t first,
                   double step, std::string &why) {
  const auto &t = trajectory.joint_trajectory;
  if (t.points.empty() || t.joint_names.empty() || step <= 0) {
    why = "EMPTY_TRAJECTORY";
    return false;
  }
  moveit::core::RobotState state = scene.getCurrentState();
  std::vector<double> from;
  state.copyJointGroupPositions("arm", from);
  // Use names, not assumed storage order.
  from.clear();
  for (const auto &n : t.joint_names) {
    if (!state.getRobotModel()->hasJointModel(n)) {
      why = "UNKNOWN_JOINT";
      return false;
    }
    from.push_back(state.getVariablePosition(n));
  }
  collision_detection::CollisionRequest req;
  req.group_name = "";
  req.contacts = true;
  req.max_contacts = 1;
  for (size_t i = first; i < t.points.size(); ++i) {
    const auto &to = t.points[i].positions;
    if (to.size() != from.size()) {
      why = "INVALID_TRAJECTORY";
      return false;
    }
    double delta = 0;
    for (size_t j = 0; j < to.size(); ++j) {
      if (!std::isfinite(to[j])) {
        why = "NONFINITE_TRAJECTORY";
        return false;
      }
      delta = std::max(delta, std::abs(to[j] - from[j]));
    }
    int n = std::max(1, int(std::ceil(delta / step)));
    double dt=i>first?(t.points[i].time_from_start-t.points[i-1].time_from_start).toSec():0;
    bool spline=i>first && dt>0 && t.points[i-1].velocities.size()==to.size() && t.points[i].velocities.size()==to.size();
    if(spline) n=std::max(n,int(std::ceil(dt/0.02)));
    for (int k = 0; k <= n; ++k) {
      for (size_t j = 0; j < to.size(); ++j) {
        double u=double(k)/n;
        double value=from[j]+(to[j]-from[j])*u;
        if(spline) {
          const auto& previous=t.points[i-1];const auto& next=t.points[i];
          double v0=previous.velocities[j],v1=next.velocities[j];
          if(previous.accelerations.size()==to.size() && next.accelerations.size()==to.size()) {
            double a0=previous.accelerations[j],a1=next.accelerations[j];
            double d=to[j]-from[j], time=u*dt;
            double c3=(20*d-(8*v1+12*v0)*dt-(3*a0-a1)*dt*dt)/(2*dt*dt*dt);
            double c4=(-30*d+(14*v1+16*v0)*dt+(3*a0-2*a1)*dt*dt)/(2*std::pow(dt,4));
            double c5=(12*d-(6*v1+6*v0)*dt-(a0-a1)*dt*dt)/(2*std::pow(dt,5));
            value=from[j]+v0*time+a0*time*time/2+c3*std::pow(time,3)+c4*std::pow(time,4)+c5*std::pow(time,5);
          } else {
            value=(2*u*u*u-3*u*u+1)*from[j]+(u*u*u-2*u*u+u)*dt*v0+
                  (-2*u*u*u+3*u*u)*to[j]+(u*u*u-u*u)*dt*v1;
          }
        }
        if(!std::isfinite(value)){why="NONFINITE_TRAJECTORY";return false;}
        state.setVariablePosition(t.joint_names[j],value);
      }
      state.update();
      collision_detection::CollisionResult res;
      scene.checkCollision(req, res, state);
      if (res.collision) {
        why = "COLLISION";
        if (!res.contacts.empty())
          why += ": " + res.contacts.begin()->first.first + " / " +
                 res.contacts.begin()->first.second;
        return false;
      }
      if (!state.satisfiesBounds()) {
        why = "JOINT_LIMIT";
        return false;
      }
    }
    from = to;
  }
  return true;
}
MoveItPlanner::MoveItPlanner(ros::NodeHandle &p) : p_(p) {
  p_.param("simulation", simulation_, true);
  p_.param("execute", execute_, false);
  p_.param("require_map", require_map_, true);
  p_.param("map_timeout", map_timeout_, 2.0);
  p_.param("collision_joint_step", step_, 0.035);
  p_.param("velocity_scaling", velocity_, 0.15);
  p_.param("acceleration_scaling", acceleration_, 0.1);
  p_.param<std::string>("optimizer", optimizer_, "time_parameterization");
  p_.param<std::string>("planner_type", planner_, "RRTConnect");
  const std::set<std::string> allowed = {"RRTConnect", "RRTstar", "PRM",
                                         "KPIECE", "EST"};
  if (!allowed.count(planner_))
    throw std::runtime_error("Unknown planner_type");
  if (velocity_ <= 0 || velocity_ > 1 || acceleration_ <= 0 ||
      acceleration_ > 1 || step_ <= 0)
    throw std::runtime_error("Invalid motion limits");
  if (!simulation_ && execute_) {
    bool calibration = false, tool = false, stop = false;
    p_.param("hand_eye_calibrated", calibration, false);
    p_.param("tool_verified", tool, false);
    p_.param("stop_verified", stop, false);
    if (!calibration || !tool || !stop)
      throw std::runtime_error("Real execution requires hand_eye_calibrated, "
                               "tool_verified and stop_verified");
  }
  group.setEndEffectorLink("grasp_tcp");
  group.setPoseReferenceFrame("base_link");
  group.setPlannerId(planner_);
  double planning_time=5.0; int planning_attempts=3;
  p_.param("planning_time",planning_time,5.0);
  p_.param("planning_attempts",planning_attempts,3);
  if(planning_time<=0 || planning_attempts<1) throw std::runtime_error("Invalid planning budget");
  group.setPlanningTime(planning_time);
  group.setNumPlanningAttempts(planning_attempts);
  group.setMaxVelocityScalingFactor(velocity_);
  group.setMaxAccelerationScalingFactor(acceleration_);
  group.setGoalPositionTolerance(0.003);
  group.setGoalOrientationTolerance(0.03);
  double padding=0.005;
  p_.param("robot_padding",padding,0.005);
  if(padding<0 || padding>0.05) throw std::runtime_error("Invalid robot padding");
  moveit_msgs::PlanningScene padding_scene; padding_scene.is_diff=true;
  for(const auto* link:group.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    moveit_msgs::LinkPadding item;item.link_name=link->getName();item.padding=padding;
    padding_scene.link_padding.push_back(item);
  }
  if(!scene.applyPlanningScene(padding_scene)) throw std::runtime_error("Cannot apply robot collision padding");
  monitor.reset(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  if (!monitor->getPlanningScene())
    throw std::runtime_error("Robot model missing");
  monitor->startSceneMonitor("/move_group/monitored_planning_scene");
  monitor->startStateMonitor("/joint_states");
  monitor->requestPlanningSceneState("/get_planning_scene");
  {
    planning_scene_monitor::LockedPlanningSceneRW s(monitor);
    s->setActiveCollisionDetector(
        collision_detection::CollisionDetectorAllocatorFCL::create(), true);
  }
  cloud_sub_ = nh_.subscribe<std_msgs::Header>(
      "/mapping/processed", 1, [this](const std_msgs::HeaderConstPtr &h) {
        std::lock_guard<std::mutex> l(mutex_);
        cloud_received_ = ros::WallTime::now();
        cloud_stamp_ = h->stamp;
      });
  scene_sub_ = nh_.subscribe<moveit_msgs::PlanningScene>(
      "/move_group/monitored_planning_scene", 1,
      [this](const moveit_msgs::PlanningSceneConstPtr& scene) {
        if(!scene->world.octomap.octomap.data.empty()) {
          std::lock_guard<std::mutex> lock(mutex_);
          scene_received_=ros::WallTime::now();
        }
      });
  // An unchanged occupied map need not publish a geometry diff. The updater's
  // filtered-cloud output provides a heartbeat even after map convergence.
  filtered_sub_=nh_.subscribe<sensor_msgs::PointCloud2>(
      "/move_group/d435i/self_filtered",1,
      [this](const sensor_msgs::PointCloud2ConstPtr& cloud) {
        double age=(ros::Time::now()-cloud->header.stamp).toSec();
        if(age>=0 && age<map_timeout_) {
          std::lock_guard<std::mutex> lock(mutex_);
          scene_received_=ros::WallTime::now();
        }
      });
  stop_ack_sub_ = nh_.subscribe<std_msgs::Bool>(
      "/rm_driver/Set_Arm_Stop_Result", 1,
      [this](const std_msgs::BoolConstPtr& ack) { stop_acknowledged_=ack->data; });
  stop_pub_ = nh_.advertise<std_msgs::Empty>("/rm_driver/Emergency_Stop", 1);
  metrics_ = nh_.advertise<std_msgs::String>("/grasp/metrics", 10, true);
}
bool MoveItPlanner::mapReady(std::string &error) {
  if (!require_map_)
    return true;
  std::lock_guard<std::mutex> l(mutex_);
  double age = (ros::Time::now() - cloud_stamp_).toSec();
  if (cloud_received_.isZero() ||
      (ros::WallTime::now() - cloud_received_).toSec() > map_timeout_ ||
      age < 0 || age > map_timeout_) {
    error = "TIMEOUT: obstacle cloud stale";
    return false;
  }
  if(scene_received_.isZero() ||
     (ros::WallTime::now()-scene_received_).toSec()>map_timeout_) {
    error="TIMEOUT: monitored OctoMap update stale";
    return false;
  }
  planning_scene_monitor::LockedPlanningSceneRO s(monitor);
  moveit_msgs::PlanningScene msg;
  s->getPlanningSceneMsg(msg);
  if (msg.world.octomap.octomap.data.empty()) {
    error = "MAP_EMPTY: wait for OctoMap integration";
    return false;
  }
  return true;
}
bool MoveItPlanner::ready(std::string &error) {
  if (!group.getCurrentState(2.0)) {
    error = "TIMEOUT: no joint state";
    return false;
  }
  auto c = monitor->getStateMonitor();
  if (!c || !c->haveCompleteState() ||
      (ros::Time::now() - c->getCurrentStateTime()).toSec() > 0.5) {
    error = "TIMEOUT: incomplete/stale joint state";
    return false;
  }
  return mapReady(error);
}
bool MoveItPlanner::plan(const geometry_msgs::PoseStamped &goal, Plan &result,
                         std::string &error) {
  if (!ready(error))
    return false;
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  group.setPoseTarget(goal, "grasp_tcp");
  auto state = group.getCurrentState();
  double margin=0.02;
  p_.param("joint_limit_margin",margin,0.02);
  if(!state || !std::isfinite(margin) || margin<0) {
    error="IK_FAIL: invalid state or joint margin";return false;
  }
  const auto* joints=state->getJointModelGroup("arm");
  bool ik_ok=false;
  for(int attempt=0;attempt<20 && !ik_ok;++attempt) {
    if(attempt) state->setToRandomPositions(joints);
    if(!state->setFromIK(joints,goal.pose,"grasp_tcp",0.05)) continue;
    ik_ok=true;
    for(const auto& name:joints->getVariableNames()) {
      const auto& bound=state->getRobotModel()->getVariableBounds(name);
      double q=state->getVariablePosition(name);
      if(bound.position_bounded_ && (q<bound.min_position_+margin || q>bound.max_position_-margin)) ik_ok=false;
    }
    if(ik_ok) {
      planning_scene_monitor::LockedPlanningSceneRO scene_lock(monitor);
      ik_ok=!scene_lock->isStateColliding(*state,"arm");
    }
  }
  if(!ik_ok) {error="IK_FAIL: no collision-free goal within joint margin";return false;}
  // Plan to the checked IK solution instead of resampling a different goal
  // at a mechanical end stop inside the pose goal sampler.
  group.setJointValueTarget(*state);
  auto begin = ros::WallTime::now();
  ++attempts_;
  bool ok = group.plan(result) == moveit::core::MoveItErrorCode::SUCCESS;
  if (ok)
    ok = TrajectoryOptimizer().optimize(optimizer_, result,
                                        group.getRobotModel(), velocity_,
                                        acceleration_, error);
  if(ok) {
    const auto& trajectory=result.trajectory_.joint_trajectory;
    for(const auto& point:trajectory.points) {
      for(size_t j=0;j<trajectory.joint_names.size();++j) {
        const auto& bound=state->getRobotModel()->getVariableBounds(trajectory.joint_names[j]);
        if(bound.position_bounded_ && (point.positions[j]<bound.min_position_+margin || point.positions[j]>bound.max_position_-margin)) {
          ok=false;error="PLAN_FAIL: trajectory violates joint limit margin";
        }
      }
    }
  }
  if (ok) {
    planning_scene_monitor::LockedPlanningSceneRO s(monitor);
    ok = collisionFree(*s.operator->(), result.trajectory_, 0, step_, error);
  }
  if (ok)
    ++successes_;
  else if (error.empty())
    error = "PLAN_FAIL";
  double length = 0;
  const auto &pts = result.trajectory_.joint_trajectory.points;
  for (size_t i = 1; i < pts.size(); ++i) {
    double n = 0;
    for (size_t j = 0; j < pts[i].positions.size(); ++j)
      n += std::pow(pts[i].positions[j] - pts[i - 1].positions[j], 2);
    length += std::sqrt(n);
  }
  std::ostringstream os;
  os << "{\"planner\":\"" << planner_
     << "\",\"success\":" << (ok ? "true" : "false")
     << ",\"success_rate\":" << double(successes_) / attempts_
     << ",\"wall_seconds\":" << (ros::WallTime::now() - begin).toSec()
     << ",\"joint_path_length_rad\":" << length << "}";
  std_msgs::String m;
  m.data = os.str();
  metrics_.publish(m);
  return ok;
}
bool MoveItPlanner::stopAndWait(std::string &error) {
  stop_acknowledged_=false;
  execution_.cancelAllGoals();
  group.stop();
  if (!simulation_)
    stop_pub_.publish(std_msgs::Empty());
  auto start = ros::WallTime::now();
  std::vector<double> prev;
  int stable = 0;
  while (ros::ok() && (ros::WallTime::now() - start).toSec() < 3) {
    auto st = group.getCurrentState(0.1);
    if (!st)
      break;
    std::vector<double> q;
    st->copyJointGroupPositions("arm", q);
    bool still = !prev.empty();
    for (size_t i = 0; i < q.size() && i < prev.size(); ++i)
      still = still && std::abs(q[i] - prev[i]) < 0.001;
    auto cs = monitor->getStateMonitor();
    still = still && cs &&
            (ros::Time::now() - cs->getCurrentStateTime()).toSec() < 0.25;
    stable = still ? stable + 1 : 0;
    if (stable >= 5 && execution_.getState().isDone() &&
        (simulation_ || stop_acknowledged_))
      return true;
    prev = q;
    ros::WallDuration(0.1).sleep();
  }
  error = "STOP_UNCONFIRMED";
  return false;
}
bool MoveItPlanner::run(const Plan &plan, const std::function<bool()> &cancel,
                        std::string &error) {
  if (!execute_) {
    error = "EXECUTION_DISABLED: planning-only mode";
    return false;
  }
  if (!ready(error))
    return false;
  {
    planning_scene_monitor::LockedPlanningSceneRO s(monitor);
    if (!collisionFree(*s.operator->(), plan.trajectory_, 0, step_, error))
      return false;
  }
  if (!execution_.waitForServer(ros::Duration(3))) {
    error = "TIMEOUT: execute_trajectory unavailable";
    return false;
  }
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = plan.trajectory_;
  execution_.sendGoal(goal);
  auto start = ros::WallTime::now();
  const auto &t = plan.trajectory_.joint_trajectory;
  double limit = t.points.back().time_from_start.toSec() * 2 + 5;
  const auto sim_start = ros::Time::now();
  size_t progress = 0;
  while (ros::ok() && !execution_.getState().isDone()) {
    bool fail = cancel();
    if (fail)
      error = "CANCELLED";
    if (!fail && !ready(error))
      fail = true;
    if (!fail && ((ros::Time::now() - sim_start).toSec() > limit ||
                  (ros::WallTime::now() - start).toSec() > 4 * limit + 10)) {
      fail = true;
      error = "TIMEOUT: execution";
    }
    if (!fail) {
      planning_scene_monitor::LockedPlanningSceneRO s(monitor);
      const auto &state = s->getCurrentState();
      double best = 1e100;
      size_t nearest = progress;
      for (size_t i = progress; i < t.points.size(); ++i) {
        double d = 0;
        for (size_t j = 0; j < t.joint_names.size(); ++j)
          d += std::pow(state.getVariablePosition(t.joint_names[j]) -
                            t.points[i].positions[j],
                        2);
        if (d < best) {
          best = d;
          nearest = i;
        } else {
          break; // never jump across a returning/looping trajectory segment
        }
      }
      progress = nearest;
      if (!collisionFree(*s.operator->(), plan.trajectory_, progress, step_,
                         error))
        fail = true;
    }
    if (fail) {
      std::string stop_error;
      if (!stopAndWait(stop_error))
        error = stop_error;
      return false;
    }
    ros::WallDuration(0.1).sleep();
  }
  if (!ros::ok()) {
    std::string ignored;
    stopAndWait(ignored);
    error = "SHUTDOWN";
    return false;
  }
  auto r = execution_.getResult();
  if (execution_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED ||
      !r || r->error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    error = "EXECUTION_FAILED";
    return false;
  }
  return true;
}
} // namespace rm75
