#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rm75_touch_teleop {

enum class ControlMode { POSITION_MODE, CONTACT_MODE, SAFE_STOP };
inline const char* modeName(ControlMode mode) {
  switch (mode) {
    case ControlMode::POSITION_MODE: return "POSITION_MODE";
    case ControlMode::CONTACT_MODE: return "CONTACT_MODE";
    default: return "SAFE_STOP";
  }
}
inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

struct ForceControlConfig {
  double desired_force = -50.0, contact_threshold = 10.0, release_threshold = 5.0;
  double force_gain = 0.002, force_limit = 100.0;
  double max_z_velocity = 0.005, max_z_step = 0.0002, max_force_travel = 0.05;
  double max_z_tracking_error = 0.005, force_deadband = 1.0;
  double filter_tau = 0.03, transition_time = 0.06, force_ramp_rate = 20.0;
  double motion_sign = 1.0;
  void validate() const {
    const double positive[] = {contact_threshold, release_threshold, force_gain,
      force_limit, max_z_velocity, max_z_step, max_force_travel,
      max_z_tracking_error, force_ramp_rate};
    for (double v : positive)
      if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid positive force-control parameter");
    if (!std::isfinite(desired_force) || desired_force >= -contact_threshold ||
        std::abs(desired_force) >= force_limit || release_threshold >= contact_threshold ||
        !std::isfinite(filter_tau) || filter_tau < 0 ||
        !std::isfinite(transition_time) || transition_time < 0 ||
        !std::isfinite(force_deadband) || force_deadband < 0 ||
        (motion_sign != 1.0 && motion_sign != -1.0))
      throw std::runtime_error("Invalid force-control thresholds, sign, or desired force (must be negative)");
  }
};

// No ROS I/O: proposed integration is committed only after a command is accepted.
class HybridForceControl {
 public:
  explicit HybridForceControl(const ForceControlConfig& config = ForceControlConfig()) : cfg(config) {
    cfg.validate();
  }
  ForceControlConfig cfg;
  ControlMode mode = ControlMode::POSITION_MODE;
  double raw = 0, filtered = 0, setpoint = 0, z = 0, contact_z = 0;
  bool have_force = false;
  void observe(double fz, double dt) {
    if (!std::isfinite(fz)) { stop(); have_force = false; return; }
    raw = fz;
    const double alpha = !have_force || cfg.filter_tau == 0 ? 1.0 :
        clamp(dt / (cfg.filter_tau + dt), 0.0, 1.0);
    filtered += alpha * (fz - filtered);
    have_force = true;
    if (std::abs(raw) > cfg.force_limit) stop();
  }
  void stop() { mode = ControlMode::SAFE_STOP; dwell_ = 0; }
  void reset(double measured_z) {
    mode = ControlMode::POSITION_MODE;
    z = contact_z = measured_z;
    setpoint = filtered;
    dwell_ = 0;
  }
  // Enter the force loop at an already-established contact.  This deliberately
  // skips the free-space/contact search transition; callers must perform their
  // own freshness, frame and wrench-limit checks before invoking it.
  void beginContact(double measured_z) {
    if (!have_force || !std::isfinite(measured_z) ||
        std::abs(raw) > cfg.force_limit) {
      stop();
      return;
    }
    mode = ControlMode::CONTACT_MODE;
    z = contact_z = measured_z;
    setpoint = filtered;
    dwell_ = 0;
  }
  bool transition(double measured_z, double dt) {
    if (mode == ControlMode::SAFE_STOP || !have_force) return false;
    const bool condition = mode == ControlMode::POSITION_MODE ?
        std::abs(filtered) > cfg.contact_threshold : std::abs(filtered) < cfg.release_threshold;
    dwell_ = condition ? dwell_ + dt : 0;
    if (!condition || dwell_ < cfg.transition_time) return false;
    dwell_ = 0;
    if (mode == ControlMode::POSITION_MODE) {
      mode = ControlMode::CONTACT_MODE;
      z = contact_z = measured_z;
      setpoint = filtered;
    } else {
      mode = ControlMode::POSITION_MODE;
      z = measured_z;
    }
    return true;
  }
  double nextSetpoint(double dt) const {
    return setpoint + clamp(cfg.desired_force - setpoint,
        -cfg.force_ramp_rate * dt, cfg.force_ramp_rate * dt);
  }
  double nextZ(double measured_z, double dt) const {
    if (mode != ControlMode::CONTACT_MODE) return z;
    const double error = nextSetpoint(dt) - filtered;
    const double velocity = std::abs(error) <= cfg.force_deadband ? 0 :
        clamp(cfg.motion_sign * cfg.force_gain * error, -cfg.max_z_velocity, cfg.max_z_velocity);
    const double step = clamp(velocity * dt, -cfg.max_z_step, cfg.max_z_step);
    // Freeze further accumulation if the plant has not followed the last command.
    if (std::abs(z + step - measured_z) > cfg.max_z_tracking_error) return z;
    return z + step;
  }
  bool travelExceeded(double proposed_z, double measured_z) const {
    return std::abs(proposed_z - contact_z) > cfg.max_force_travel ||
           std::abs(measured_z - contact_z) > cfg.max_force_travel;
  }
  void commit(double commanded_z, double dt) {
    if (mode == ControlMode::CONTACT_MODE) {
      z = commanded_z;
      setpoint = nextSetpoint(dt);
    }
  }
 private:
  double dwell_ = 0;
};
}  // namespace rm75_touch_teleop
