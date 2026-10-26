#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <rm75_autonomous_grasp/core.h>
namespace rm75 {
bool TrajectoryOptimizer::optimize(
    const std::string &name, Plan &plan,
    const moveit::core::RobotModelConstPtr &model, double v, double a,
    std::string &error) const {
  if (name != "none" && name != "time_parameterization") {
    error = "OPTIMIZER_UNAVAILABLE: " + name +
            " adapter is reserved; it must return a validated RobotTrajectory. "
            "No silent fallback.";
    return false;
  }
  moveit::core::RobotState start(model);
  moveit::core::robotStateMsgToRobotState(plan.start_state_, start);
  robot_trajectory::RobotTrajectory t(model, "arm");
  t.setRobotTrajectoryMsg(start, plan.trajectory_);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(t, v, a)) {
    error = "TIME_PARAMETERIZATION_FAILED";
    return false;
  }
  t.getRobotTrajectoryMsg(plan.trajectory_);
  return true;
}
} // namespace rm75
