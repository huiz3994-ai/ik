#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "rm75_touch_teleop/mapping.h"

TEST(Rm75TouchMapping, MatchesUnitreeZ1AxisConvention) {
  const Eigen::Vector3d mapped = rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d(10.0, 20.0, 30.0), 0.001);
  EXPECT_NEAR(mapped.x(), -0.030, 1e-12);
  EXPECT_NEAR(mapped.y(), -0.010, 1e-12);
  EXPECT_NEAR(mapped.z(),  0.020, 1e-12);
}

TEST(Rm75TouchMapping, RotationMappingRemainsProper) {
  const Eigen::Matrix3d basis = rm75_touch_teleop::touchToRobotBasis();
  EXPECT_NEAR(basis.determinant(), 1.0, 1e-12);
  const Eigen::Matrix3d touch_rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d mapped =
      rm75_touch_teleop::mapRelativeRotation(touch_rotation);
  EXPECT_NEAR(mapped.determinant(), 1.0, 1e-12);
  EXPECT_NEAR((mapped * mapped.transpose() - Eigen::Matrix3d::Identity()).norm(),
              0.0, 1e-12);
}

TEST(Rm75TouchMapping, ZOnlyMaskLocksRobotXY) {
  const Eigen::Vector3d mapped = rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d(10.0, 20.0, 30.0), 0.001);
  const Eigen::Vector3d masked =
      rm75_touch_teleop::applyTranslationAxisMask(mapped, false, false, true);
  EXPECT_DOUBLE_EQ(masked.x(), 0.0);
  EXPECT_DOUBLE_EQ(masked.y(), 0.0);
  EXPECT_NEAR(masked.z(), 0.020, 1e-12);
}

TEST(Rm75TouchMapping, ZOnlyRateAndSessionTravelAreBounded) {
  EXPECT_DOUBLE_EQ(rm75_touch_teleop::limitAxisStep(
      0.0, 0.1, 0.002, 0.00004, 0.02), 0.00004);
  EXPECT_DOUBLE_EQ(rm75_touch_teleop::limitAxisStep(
      0.0, -0.1, 0.002, 0.00004, 0.02), -0.00004);
  EXPECT_DOUBLE_EQ(rm75_touch_teleop::clampAxisTravel(
      0.2, 0.1, 0.01), 0.11);
  EXPECT_DOUBLE_EQ(rm75_touch_teleop::clampAxisTravel(
      0.0, 0.1, 0.01), 0.09);
}

TEST(Rm75TouchMapping, ReverseForceMappingPreservesVirtualWork) {
  const Eigen::Matrix3d basis = rm75_touch_teleop::touchToRobotBasis();
  const Eigen::Vector3d touch_velocity(0.2, -0.4, 0.7);
  const Eigen::Vector3d robot_force(3.0, -2.0, 5.0);
  const Eigen::Vector3d robot_velocity = basis * touch_velocity;
  const Eigen::Vector3d touch_force = basis.transpose() * robot_force;
  EXPECT_NEAR(robot_force.dot(robot_velocity),
              touch_force.dot(touch_velocity), 1e-12);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
