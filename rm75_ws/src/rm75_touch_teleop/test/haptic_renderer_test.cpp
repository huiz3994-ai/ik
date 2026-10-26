#include <gtest/gtest.h>
#include <limits>

#include "rm75_touch_teleop/haptic_renderer.h"

using rm75_touch_teleop::HapticConfig;
using rm75_touch_teleop::HapticRenderer;

namespace {
HapticConfig directConfig() {
  HapticConfig c;
  // S maps Touch to robot, so S maps this chosen robot vector back to +Touch X.
  c.force_scale = 1.0;
  c.filter_tau = 0.0;
  c.deadband = 0.0;
  c.max_force = Eigen::Vector3d::Constant(1000.0);
  c.max_force_norm = 1000.0;
  c.ramp_rate = 10000.0;
  return c;
}
Eigen::Vector3d robotForTouch(const Eigen::Vector3d& touch) {
  return rm75_touch_teleop::touchToRobotBasis() * touch;
}
}  // namespace

TEST(HapticRenderer, ZeroInputIsZero) {
  HapticRenderer r(directConfig());
  EXPECT_DOUBLE_EQ(r.step(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          0.01, true).command.norm(), 0.0);
}

TEST(HapticRenderer, MapsAndScalesForceByTranspose) {
  HapticConfig c = directConfig(); c.force_scale = 0.02;
  HapticRenderer r(c);
  const auto out = r.step(robotForTouch(Eigen::Vector3d(10, 0, 0)),
                          Eigen::Vector3d::Zero(), 0.01, true);
  EXPECT_NEAR(out.command.x(), 0.2, 1e-12);
  EXPECT_NEAR(out.command.y(), 0.0, 1e-12);
  EXPECT_NEAR(out.command.z(), 0.0, 1e-12);
}

TEST(HapticRenderer, DeadbandIsContinuous) {
  HapticConfig c = directConfig(); c.deadband = 0.5;
  HapticRenderer r(c);
  EXPECT_DOUBLE_EQ(r.step(robotForTouch(Eigen::Vector3d(0.4, 0, 0)),
                          Eigen::Vector3d::Zero(), .01, true).command.x(), 0.0);
  EXPECT_NEAR(r.step(robotForTouch(Eigen::Vector3d(0.6, 0, 0)),
                     Eigen::Vector3d::Zero(), .01, true).command.x(), .1, 1e-12);
}

TEST(HapticRenderer, LowPassDoesNotJump) {
  HapticConfig c = directConfig(); c.filter_tau = 0.09;
  HapticRenderer r(c);
  const auto out = r.step(robotForTouch(Eigen::Vector3d(10, 0, 0)),
                          Eigen::Vector3d::Zero(), .01, true);
  EXPECT_NEAR(out.filtered_touch_force.x(), 1.0, 1e-12);
  EXPECT_LT(out.filtered_touch_force.x(), 10.0);
}

TEST(HapticRenderer, ComponentThenNormSaturation) {
  HapticConfig c = directConfig();
  c.max_force = Eigen::Vector3d(1, 2, 3); c.max_force_norm = 100;
  HapticRenderer component(c);
  auto out = component.step(robotForTouch(Eigen::Vector3d(10, -10, 10)),
                            Eigen::Vector3d::Zero(), .01, true);
  EXPECT_NEAR(out.command.x(), 1, 1e-12);
  EXPECT_NEAR(out.command.y(), -2, 1e-12);
  EXPECT_NEAR(out.command.z(), 3, 1e-12);
  c.max_force = Eigen::Vector3d::Constant(10); c.max_force_norm = 2;
  HapticRenderer norm(c);
  out = norm.step(robotForTouch(Eigen::Vector3d(3, 4, 0)),
                  Eigen::Vector3d::Zero(), .01, true);
  EXPECT_NEAR(out.command.norm(), 2, 1e-12);
}

TEST(HapticRenderer, SlewRateLimitsVectorChangeAndRampsToZero) {
  HapticConfig c = directConfig(); c.ramp_rate = 2.0;
  HapticRenderer r(c);
  auto out = r.step(robotForTouch(Eigen::Vector3d(100, 0, 0)),
                    Eigen::Vector3d::Zero(), .01, true);
  EXPECT_NEAR(out.command.norm(), .02, 1e-12);
  out = r.step(robotForTouch(Eigen::Vector3d(100, 0, 0)),
               Eigen::Vector3d::Zero(), .01, false);
  EXPECT_NEAR(out.command.norm(), 0.0, 1e-12);
}

TEST(HapticRenderer, DampingAlwaysOpposesVelocity) {
  HapticConfig c = directConfig(); c.damping = Eigen::Vector3d(1, 2, 3);
  HapticRenderer r(c);
  const Eigen::Vector3d velocity(0.1, -0.2, 0.3);
  const auto out = r.step(Eigen::Vector3d::Zero(), velocity, .01, true);
  EXPECT_LE(out.command.dot(velocity), 1e-12);
}

TEST(HapticRenderer, PositionCouplingOpposesRequestedMotion) {
  HapticConfig c = directConfig();
  c.position_stiffness = Eigen::Vector3d::Constant(100.0);
  c.position_deadband = 0.0;
  HapticRenderer r(c);
  // The requested Touch displacement is +X, therefore actual-target in robot
  // coordinates is -S*[10 mm, 0, 0].  The returned Touch force must be -X.
  const Eigen::Vector3d error =
      -robotForTouch(Eigen::Vector3d(0.01, 0.0, 0.0));
  const auto out = r.step(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          0.01, true, false, error);
  EXPECT_NEAR(out.position_touch_force.x(), -1.0, 1e-12);
  EXPECT_NEAR(out.command.x(), -1.0, 1e-12);
}

TEST(HapticRenderer, PositionCouplingHasContinuousDeadbandAndErrorClamp) {
  HapticConfig c = directConfig();
  c.position_stiffness = Eigen::Vector3d::Constant(100.0);
  c.position_deadband = 0.002;
  c.max_position_error = 0.01;
  HapticRenderer r(c);
  auto out = r.step(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                    0.01, true, false,
                    robotForTouch(Eigen::Vector3d(0.001, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(out.position_touch_force.norm(), 0.0);
  out = r.step(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
               0.01, true, false,
               robotForTouch(Eigen::Vector3d(0.02, 0.0, 0.0)));
  EXPECT_NEAR(out.position_touch_force.norm(), 0.8, 1e-12);
}

TEST(HapticRenderer, NonFiniteAndInvalidTimeImmediatelyZero) {
  HapticRenderer r(directConfig());
  r.step(robotForTouch(Eigen::Vector3d(1, 0, 0)), Eigen::Vector3d::Zero(), .01, true);
  Eigen::Vector3d bad = Eigen::Vector3d::Zero();
  bad.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(r.step(bad, Eigen::Vector3d::Zero(), .01, true).command.norm(), 0.0);
  EXPECT_DOUBLE_EQ(r.step(Eigen::Vector3d::Ones(), Eigen::Vector3d::Zero(), 0.0, true).command.norm(), 0.0);
}

TEST(HapticRenderer, GatingAndImmediateSafeStop) {
  HapticRenderer r(directConfig());
  const Eigen::Vector3d force = robotForTouch(Eigen::Vector3d(1, 0, 0));
  EXPECT_DOUBLE_EQ(r.step(force, Eigen::Vector3d::Zero(), .01, false).command.norm(), 0.0);
  EXPECT_GT(r.step(force, Eigen::Vector3d::Zero(), .01, true).command.norm(), 0.0);
  EXPECT_DOUBLE_EQ(r.step(force, Eigen::Vector3d::Zero(), .01, true, true).command.norm(), 0.0);
}

TEST(HapticRenderer, ContactTimeoutAndDisableGates) {
  using rm75_touch_teleop::ControlMode;
  using rm75_touch_teleop::hapticOutputAllowed;
  EXPECT_FALSE(hapticOutputAllowed(false, true, true, true,
                                   ControlMode::CONTACT_MODE, true, true));
  EXPECT_FALSE(hapticOutputAllowed(true, true, true, true,
                                   ControlMode::POSITION_MODE, true, true));
  EXPECT_TRUE(hapticOutputAllowed(true, true, true, true,
                                  ControlMode::CONTACT_MODE, true, true));
  EXPECT_FALSE(hapticOutputAllowed(true, true, true, true,
                                   ControlMode::CONTACT_MODE, false, true));
  EXPECT_FALSE(hapticOutputAllowed(true, false, false, false,
                                   ControlMode::SAFE_STOP, true, true));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
