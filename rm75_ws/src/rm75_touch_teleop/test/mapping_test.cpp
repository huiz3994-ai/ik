#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "rm75_touch_teleop/mapping.h"

TEST(Rm75TouchMapping, MapsCalibratedTouchAxesToRm75Base) {
  const Eigen::Vector3d mapped = rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d(10.0, 20.0, 30.0), 0.001);
  EXPECT_NEAR(mapped.x(), -0.030, 1e-12);
  EXPECT_NEAR(mapped.y(), -0.010, 1e-12);
  EXPECT_NEAR(mapped.z(),  0.020, 1e-12);
}

TEST(Rm75TouchMapping, MapsEachTranslationAxisIndependently) {
  EXPECT_TRUE(rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d::UnitX(), 1.0).isApprox(-Eigen::Vector3d::UnitY()));
  EXPECT_TRUE(rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d::UnitY(), 1.0).isApprox(Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(rm75_touch_teleop::mapTranslation(
      Eigen::Vector3d::UnitZ(), 1.0).isApprox(-Eigen::Vector3d::UnitX()));
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

TEST(Rm75TouchMapping, SingleAxisRotationsFollowMappedAxes) {
  const double angle = 0.2;
  const Eigen::Matrix3d basis = rm75_touch_teleop::touchToRobotBasis();
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d touch_axis = Eigen::Vector3d::Unit(axis);
    const Eigen::Matrix3d touch =
        Eigen::AngleAxisd(angle, touch_axis).toRotationMatrix();
    const Eigen::AngleAxisd mapped(
        rm75_touch_teleop::mapRelativeRotation(touch));
    EXPECT_NEAR(mapped.angle(), angle, 1e-12);
    EXPECT_NEAR(std::abs(mapped.axis().dot(basis * touch_axis)), 1.0, 1e-12);
  }
}

TEST(Rm75TouchMapping, RelativeRotationComposesWithRobotAnchor) {
  const Eigen::Matrix3d robot_anchor =
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d touch_delta =
      Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d target =
      rm75_touch_teleop::mapRelativeRotation(touch_delta) * robot_anchor;
  const Eigen::Quaterniond target_q(target);
  EXPECT_NEAR(target_q.norm(), 1.0, 1e-12);
  EXPECT_NEAR(target.determinant(), 1.0, 1e-12);
  EXPECT_NEAR((target * target.transpose() - Eigen::Matrix3d::Identity()).norm(),
              0.0, 1e-12);
}

TEST(Rm75TouchMapping, PoseRateLimitPreservesStillPoseAndBoundsJumps) {
  Eigen::Isometry3d current = Eigen::Isometry3d::Identity();
  current.translation() = Eigen::Vector3d(0.2, -0.1, 0.5);
  const Eigen::Isometry3d still = rm75_touch_teleop::limitPoseStep(
      current, current, 0.1, 0.5, 0.02);
  EXPECT_NEAR((still.matrix() - current.matrix()).norm(), 0.0, 1e-14);

  Eigen::Isometry3d requested = current;
  requested.translation().x() += 1.0;
  requested.linear() = Eigen::AngleAxisd(
      1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix() * current.linear();
  const Eigen::Isometry3d limited = rm75_touch_teleop::limitPoseStep(
      current, requested, 0.1, 0.5, 0.02);
  EXPECT_NEAR((limited.translation() - current.translation()).norm(), 0.002, 1e-12);
  EXPECT_NEAR(Eigen::Quaterniond(current.linear()).angularDistance(
                  Eigen::Quaterniond(limited.linear())), 0.01, 1e-12);
  EXPECT_NEAR(limited.linear().determinant(), 1.0, 1e-12);
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
