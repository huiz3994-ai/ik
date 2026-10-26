#pragma once

#include <string>

#include <Eigen/Core>

#include "rm75_motion_planning/runtime_guard.h"

namespace rm75_motion_planning {

enum class CommandSource { NONE, TOUCH, MOVEIT, AUTONOMOUS };

class CommandSourceManager {
 public:
  explicit CommandSourceManager(bool execution_enabled = false);
  bool acquire(CommandSource source, std::string* reason = nullptr);
  void release(CommandSource source);
  bool accept(CommandSource source, const Eigen::VectorXd& command,
              std::string* reason = nullptr) const;
  CommandSource owner() const { return owner_; }
  bool executionEnabled() const { return execution_enabled_; }
  // Publishers for physical controller topics may only be constructed after
  // this gate returns true.  Keeping this query on the same object that
  // validates commands makes the execute=false contract unit-testable.
  bool commandPublisherAllowed() const {
    return execution_enabled_ && owner_ != CommandSource::NONE;
  }

 private:
  bool execution_enabled_{false};
  CommandSource owner_{CommandSource::NONE};
};

struct GuardedCommandResult {
  bool accepted{false};
  GuardDecision decision{GuardDecision::STOP};
  RuntimeGuardResult guard;
  std::string reason;
};

// Common command entry for autonomous, MoveIt and future Touch adapters.
// Ownership/execution authorization and RuntimeGuard are intentionally one
// operation so a caller cannot publish a command accepted by only one layer.
class GuardedCommandGateway {
 public:
  GuardedCommandGateway(const CommandSourceManager& sources,
                        const RuntimeGuard& guard)
      : sources_(sources), guard_(guard) {}
  GuardedCommandResult evaluate(CommandSource source,
                                const RuntimeGuardInput& input) const;

 private:
  const CommandSourceManager& sources_;
  const RuntimeGuard& guard_;
};

const char* toString(CommandSource source);

// Query ROS master before advertising a controller command topic. Returns
// false on master/protocol failure or when any publisher already owns it.
bool commandTopicUnclaimed(const std::string& topic,
                           std::string* reason = nullptr);

}  // namespace rm75_motion_planning
