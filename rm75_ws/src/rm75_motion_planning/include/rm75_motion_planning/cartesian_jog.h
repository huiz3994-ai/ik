#pragma once

#include <string>

#include "rm75_motion_planning/types.h"

namespace rm75_motion_planning {

// Browser/API strings are deliberately mapped to a closed enum before any
// pose is constructed.  Translation is expressed in base_link; rotation is
// post-multiplied and therefore expressed in the local Link7 frame.
enum class CartesianJogDirection {
  X_PLUS, X_MINUS, Y_PLUS, Y_MINUS, Z_PLUS, Z_MINUS,
  RX_PLUS, RX_MINUS, RY_PLUS, RY_MINUS, RZ_PLUS, RZ_MINUS
};

bool parseCartesianJogDirection(const std::string& value,
                                CartesianJogDirection* direction);
const char* toString(CartesianJogDirection direction);
bool isCartesianJogRotation(CartesianJogDirection direction);
Pose cartesianJogTarget(const Pose& start, CartesianJogDirection direction,
                        double translation_distance_m,
                        double rotation_angle_rad);

}  // namespace rm75_motion_planning
