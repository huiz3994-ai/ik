#include "rm75_motion_planning/command_source_manager.h"

#include <ros/master.h>
#include <ros/this_node.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace rm75_motion_planning {

CommandSourceManager::CommandSourceManager(bool execution_enabled)
    : execution_enabled_(execution_enabled) {}

bool CommandSourceManager::acquire(CommandSource source, std::string* reason) {
  if (source == CommandSource::NONE) {
    if (reason) *reason = "NONE cannot own the controller";
    return false;
  }
  if (owner_ != CommandSource::NONE && owner_ != source) {
    if (reason) *reason = std::string("controller already owned by ") +
                          toString(owner_);
    return false;
  }
  owner_ = source;
  if (reason) reason->clear();
  return true;
}

void CommandSourceManager::release(CommandSource source) {
  if (owner_ == source) owner_ = CommandSource::NONE;
}

bool CommandSourceManager::accept(CommandSource source,
                                  const Eigen::VectorXd& command,
                                  std::string* reason) const {
  std::string problem;
  if (!execution_enabled_) problem = "execution is disarmed";
  else if (source == CommandSource::NONE || source != owner_)
    problem = "command source does not own the controller";
  else if (command.size() != 7 || !command.allFinite())
    problem = "command must be a finite 7-DOF vector";
  if (reason) *reason = problem;
  return problem.empty();
}

GuardedCommandResult GuardedCommandGateway::evaluate(
    CommandSource source, const RuntimeGuardInput& input) const {
  GuardedCommandResult result;
  if (!sources_.accept(source, input.next_target, &result.reason)) return result;
  result.guard = guard_.evaluate(input);
  result.decision = result.guard.decision;
  result.accepted = result.guard.allow;
  result.reason = result.guard.reason;
  return result;
}

const char* toString(CommandSource source) {
  switch (source) {
    case CommandSource::NONE: return "NONE";
    case CommandSource::TOUCH: return "TOUCH";
    case CommandSource::MOVEIT: return "MOVEIT";
    case CommandSource::AUTONOMOUS: return "AUTONOMOUS";
  }
  return "NONE";
}

bool commandTopicUnclaimed(const std::string& topic, std::string* reason) {
  if (topic.empty()) {
    if (reason) *reason = "command topic is empty";
    return false;
  }
  XmlRpc::XmlRpcValue request;
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  request.setSize(1);
  request[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", request, response, payload,
                            false)) {
    if (reason) *reason = "cannot query ROS master system state";
    return false;
  }
  if (payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      payload.size() < 1 ||
      payload[0].getType() != XmlRpc::XmlRpcValue::TypeArray) {
    if (reason) *reason = "invalid ROS master system-state response";
    return false;
  }
  const XmlRpc::XmlRpcValue& publishers = payload[0];
  for (int index = 0; index < publishers.size(); ++index) {
    const XmlRpc::XmlRpcValue& entry = publishers[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        entry.size() != 2 ||
        entry[0].getType() != XmlRpc::XmlRpcValue::TypeString ||
        entry[1].getType() != XmlRpc::XmlRpcValue::TypeArray)
      continue;
    if (static_cast<std::string>(entry[0]) == topic && entry[1].size() > 0) {
      if (reason) {
        *reason = "command topic already has publisher";
        for (int node = 0; node < entry[1].size(); ++node)
          *reason += std::string(node == 0 ? ": " : ", ") +
              static_cast<std::string>(entry[1][node]);
      }
      return false;
    }
  }
  if (reason) reason->clear();
  return true;
}

}  // namespace rm75_motion_planning
