#include <gtest/gtest.h>

#include "rm75_touch_teleop/damped_differential_ik.h"

TEST(DampedDifferentialIk, TracksWellConditionedTask) {
  const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd error(6);
  error << 0.01, -0.02, 0.03, 0.0, 0.0, 0.0;
  const Eigen::VectorXd step =
      rm75_touch_teleop::dampedDifferentialIkStep(jacobian, error, 1e-3, 1.0);
  EXPECT_NEAR(step[0], error[0], 1e-6);
  EXPECT_NEAR(step[1], error[1], 1e-6);
  EXPECT_NEAR(step[2], error[2], 1e-6);
}

TEST(DampedDifferentialIk, RemainsFiniteAtSingularity) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 7);
  jacobian.diagonal().head(5).setOnes();
  const Eigen::VectorXd error = Eigen::VectorXd::Ones(6);
  const Eigen::VectorXd step =
      rm75_touch_teleop::dampedDifferentialIkStep(jacobian, error, 0.05, 0.1);
  EXPECT_TRUE(step.allFinite());
  EXPECT_LE(step.cwiseAbs().maxCoeff(), 0.1 + 1e-12);
}

TEST(DampedDifferentialIk, LimitsLargestJointStep) {
  const Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd error = Eigen::VectorXd::Constant(6, 10.0);
  const Eigen::VectorXd step =
      rm75_touch_teleop::dampedDifferentialIkStep(jacobian, error, 0.01, 0.02);
  EXPECT_NEAR(step.cwiseAbs().maxCoeff(), 0.02, 1e-12);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
