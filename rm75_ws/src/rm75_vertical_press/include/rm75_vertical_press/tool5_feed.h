#pragma once

#include <algorithm>

namespace rm75_vertical_press {

inline double boundedTool5FeedStep(double max_z_step,
                                   double max_z_velocity,
                                   double dt) {
  return std::min(max_z_step, max_z_velocity * std::max(0.0, dt));
}

}  // namespace rm75_vertical_press
