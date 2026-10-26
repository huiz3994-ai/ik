#include <gtest/gtest.h>
#include <limits>
#include "rm75_touch_teleop/hybrid_force_control.h"
using namespace rm75_touch_teleop;

TEST(Hybrid, HysteresisDebounceAndLatch) {
  HybridForceControl h;
  h.cfg.filter_tau = 0;
  h.observe(-11, .02);
  EXPECT_FALSE(h.transition(.4, .02));
  EXPECT_FALSE(h.transition(.4, .02));
  EXPECT_TRUE(h.transition(.4, .02));
  EXPECT_EQ(h.mode, ControlMode::CONTACT_MODE);
  h.observe(-7, .02);
  for (int i=0; i<10; ++i) EXPECT_FALSE(h.transition(.4,.02));
  h.observe(-4, .02);
  for (int i=0; i<3; ++i) h.transition(.4,.02);
  EXPECT_EQ(h.mode, ControlMode::POSITION_MODE);
  h.observe(-101, .02);
  h.observe(0, .02);
  EXPECT_EQ(h.mode, ControlMode::SAFE_STOP);
  EXPECT_FALSE(h.transition(.4,1));
}
TEST(Hybrid, RawOverforceBypassesFilterAndNanStops) {
  HybridForceControl h;
  h.observe(0,.01); h.observe(101,.001);
  EXPECT_LT(h.filtered, 10);
  EXPECT_EQ(h.mode, ControlMode::SAFE_STOP);
  h.reset(.4); h.observe(std::numeric_limits<double>::quiet_NaN(),.01);
  EXPECT_EQ(h.mode, ControlMode::SAFE_STOP);
  EXPECT_FALSE(h.have_force);
}
TEST(Hybrid, AlreadyContactedEntrySkipsSurfaceSearch) {
  HybridForceControl h;
  h.observe(-12.0, .02);
  h.beginContact(.4);
  EXPECT_EQ(h.mode, ControlMode::CONTACT_MODE);
  EXPECT_DOUBLE_EQ(h.contact_z, .4);
  EXPECT_DOUBLE_EQ(h.setpoint, h.filtered);
  HybridForceControl missing;
  missing.beginContact(.4);
  EXPECT_EQ(missing.mode, ControlMode::SAFE_STOP);
}
TEST(Hybrid, LimitsAndNoCommitOnFailure) {
  HybridForceControl h;
  h.observe(-11,.01); h.transition(.4,1);
  double next = h.nextZ(.4,.02);
  EXPECT_LE(std::abs(next-.4),h.cfg.max_z_velocity*.02);
  EXPECT_DOUBLE_EQ(h.z,.4); // rejected IK must not advance state
  EXPECT_DOUBLE_EQ(h.nextZ(.38,.02),.4); // plant has not followed
  EXPECT_TRUE(h.travelExceeded(.451,.4));
  h.commit(next,.02);
  EXPECT_DOUBLE_EQ(h.z,next);
}
TEST(Hybrid, SpringPlantConvergesAtDifferentLoopRates) {
  for (double dt : {.01,.02}) {
    HybridForceControl h;
    double z=-.0022;
    h.observe(5000*z,dt); h.transition(z,1);
    for (int i=0; i<int(10/dt); ++i) {
      h.observe(5000*z,dt);
      const double proposed=h.nextZ(z,dt);
      EXPECT_LE(std::abs(proposed-z),h.cfg.max_z_velocity*dt+1e-12);
      EXPECT_FALSE(h.travelExceeded(proposed,z));
      h.commit(proposed,dt); z=proposed;
    }
    EXPECT_NEAR(5000*z,-50,1.5);
    EXPECT_EQ(h.mode,ControlMode::CONTACT_MODE);
  }
}
TEST(Hybrid, InvalidConfigurationRejected) {
  ForceControlConfig c;
  c.release_threshold=12; EXPECT_THROW(HybridForceControl h(c),std::runtime_error);
  c=ForceControlConfig(); c.force_gain=std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(HybridForceControl h(c),std::runtime_error);
  c=ForceControlConfig(); c.desired_force=-101;
  EXPECT_THROW(HybridForceControl h(c),std::runtime_error);
}
int main(int argc,char** argv) { testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
