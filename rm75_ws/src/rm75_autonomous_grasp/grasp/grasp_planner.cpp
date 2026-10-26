#include <rm75_autonomous_grasp/core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace rm75 {
std::vector<Candidate>
GraspPlanner::generate(const geometry_msgs::PoseStamped &o,
                       const std::string &mode, double approach, double lift,
                       double offset) const {
  if (mode != "top" && mode != "side")
    throw std::invalid_argument("grasp_mode must be top or side");
  std::vector<Candidate> result;
  for (double yaw : {0.0, 1.57079632679, 3.14159265359, -1.57079632679}) {
    Candidate c;
    c.grasp = o;
    tf2::Quaternion q;
    // TCP +Z points toward the object. Top grasp faces down; side faces
    // horizontally.
    q.setRPY(0, mode == "top" ? 3.14159265359 : 1.57079632679, yaw);
    c.grasp.pose.orientation = tf2::toMsg(q);
    c.grasp.pose.position.z += offset;
    c.pre = c.grasp;
    tf2::Vector3 direction = tf2::quatRotate(q, {0, 0, 1});
    c.pre.pose.position.x -= approach * direction.x();
    c.pre.pose.position.y -= approach * direction.y();
    c.pre.pose.position.z -= approach * direction.z();
    c.lift = c.grasp;
    c.lift.pose.position.z += lift;
    result.push_back(c);
  }
  return result;
}
} // namespace rm75
