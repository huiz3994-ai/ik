#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "rm75_touch_teleop/constrained_differential_ik.h"
#include "rm75_touch_teleop/damped_differential_ik.h"

namespace {

using rm75_touch_teleop::DifferentialIkConfig;
using rm75_touch_teleop::DifferentialIkInput;
using rm75_touch_teleop::GeometricJacobian;
using rm75_touch_teleop::JointVector;

DifferentialIkInput nominalInput() {
  DifferentialIkInput input;
  input.jacobian.setZero();
  input.jacobian.block<6, 6>(0, 0).setIdentity();
  input.jacobian(0, 6) = 0.4;
  input.position_lower.setConstant(-2.0);
  input.position_upper.setConstant(2.0);
  input.velocity_limit.setConstant(0.5);
  input.acceleration_limit.setConstant(1.5);
  input.dt = 0.01;
  return input;
}

TEST(BoxQpSolver, FindsConstrainedKktOptimum) {
  Eigen::Matrix2d h = Eigen::Matrix2d::Identity();
  Eigen::Vector2d g(-2.0, 1.0), lower(-0.5, -0.25), upper(0.7, 0.8);
  const auto result = rm75_touch_teleop::solveBoxQp(
      h, g, lower, upper, Eigen::Vector2d::Zero());
  ASSERT_EQ(result.status, rm75_touch_teleop::BoxQpStatus::SUCCESS);
  EXPECT_NEAR(result.solution[0], 0.7, 1e-9);
  EXPECT_NEAR(result.solution[1], -0.25, 1e-9);
  EXPECT_LE(result.kkt_residual, 1e-8);
}

TEST(BoxQpSolver, RejectsInfeasibleBoundsAndHonorsTimeout) {
  Eigen::Matrix2d h = Eigen::Matrix2d::Identity();
  Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  Eigen::Vector2d lower(1.0, -1.0), upper(0.0, 1.0);
  EXPECT_EQ(rm75_touch_teleop::solveBoxQp(h, zero, lower, upper, zero).status,
            rm75_touch_teleop::BoxQpStatus::INFEASIBLE_BOUNDS);
  rm75_touch_teleop::BoxQpOptions options;
  options.time_limit_sec = 1e-15;
  lower << -1.0, -1.0;
  const auto timeout = rm75_touch_teleop::solveBoxQp(
      h, Eigen::Vector2d(-1.0, 1.0), lower, upper, zero, options);
  EXPECT_EQ(timeout.status, rm75_touch_teleop::BoxQpStatus::TIMEOUT);
}

TEST(ConstrainedDifferentialIk, UsesSixBySevenQpAndFiniteOutput) {
  auto input = nominalInput();
  input.pose_error << 0.03, -0.01, 0.02, 0.1, -0.05, 0.02;
  const auto output = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, DifferentialIkConfig());
  ASSERT_TRUE(output.success()) << output.reason;
  EXPECT_TRUE(output.velocity.allFinite());
  EXPECT_EQ(output.velocity.size(), 7);
  EXPECT_TRUE((output.velocity.array() <= output.upper_bound.array() + 1e-10).all());
  EXPECT_TRUE((output.velocity.array() >= output.lower_bound.array() - 1e-10).all());
}

TEST(ConstrainedDifferentialIk, SingularAndNearSingularRemainConstrained) {
  auto input = nominalInput();
  input.pose_error.setConstant(0.1);
  DifferentialIkConfig config;
  input.jacobian.row(5).setZero();
  auto singular = rm75_touch_teleop::solveConstrainedDifferentialIk(input, config);
  ASSERT_TRUE(singular.success()) << singular.reason;
  EXPECT_NEAR(singular.minimum_singular_value, 0.0, 1e-12);
  EXPECT_GT(singular.adaptive_regularization, config.regularization);
  input.jacobian(5, 5) = 1e-5;
  auto near = rm75_touch_teleop::solveConstrainedDifferentialIk(input, config);
  ASSERT_TRUE(near.success()) << near.reason;
  EXPECT_TRUE(near.velocity.allFinite());
  EXPECT_LE(near.velocity.cwiseAbs().maxCoeff(), 0.0150001);
}

TEST(ConstrainedDifferentialIk, PostureVelocityIsInMeasuredJacobianNullspace) {
  auto input = nominalInput();
  input.measured_position << 0.2, -0.4, 0.1, 0.3, -0.2, 0.5, -0.6;
  input.previous_command = input.measured_position;
  input.posture_reference.setZero();
  const auto output = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, DifferentialIkConfig());
  ASSERT_TRUE(output.success()) << output.reason;
  EXPECT_GT(output.posture_velocity.norm(), 1e-6);
  EXPECT_LT((input.jacobian * output.posture_velocity).norm(), 1e-10);
}

TEST(ConstrainedDifferentialIk, NearSingularPostureProjectionIsContinuous) {
  auto input = nominalInput();
  input.jacobian(0, 6) = 0.0;
  input.measured_position[5] = 0.4;
  input.previous_command = input.measured_position;
  input.posture_reference.setZero();
  DifferentialIkConfig config;

  input.jacobian(5, 5) = 0.0;
  const auto singular = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, config);
  ASSERT_TRUE(singular.success()) << singular.reason;
  input.jacobian(5, 5) = 1e-7;
  const auto near = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, config);
  ASSERT_TRUE(near.success()) << near.reason;
  EXPECT_LT((near.posture_velocity - singular.posture_velocity).norm(), 1e-5);

  input.jacobian(5, 5) = config.singular_value_threshold * 1.01;
  const auto conditioned = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, config);
  ASSERT_TRUE(conditioned.success()) << conditioned.reason;
  EXPECT_LT(std::abs(conditioned.posture_velocity[5]), 1e-10);
}

TEST(ConstrainedDifferentialIk, VelocitySmoothingPenalizesFrameToFrameChange) {
  auto input = nominalInput();
  input.acceleration_limit.setConstant(100.0);
  input.previous_velocity[6] = 0.1;
  DifferentialIkConfig smooth;
  DifferentialIkConfig unsmoothed = smooth;
  unsmoothed.velocity_smoothing_weight = 0.0;
  const auto with_smoothing = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, smooth);
  const auto without_smoothing =
      rm75_touch_teleop::solveConstrainedDifferentialIk(input, unsmoothed);
  ASSERT_TRUE(with_smoothing.success()) << with_smoothing.reason;
  ASSERT_TRUE(without_smoothing.success()) << without_smoothing.reason;
  EXPECT_LT(std::abs(with_smoothing.velocity[6] - input.previous_velocity[6]),
            std::abs(without_smoothing.velocity[6] - input.previous_velocity[6]));
}

TEST(ConstrainedDifferentialIk, PostureReferenceFollowsMeasuredAtBoundedRate) {
  JointVector reference = JointVector::Zero();
  JointVector measured;
  measured << 1.0, -1.0, 0.2, -0.2, 0.0, 0.01, -0.01;
  const JointVector updated = rm75_touch_teleop::updatePostureReference(
      reference, measured, 0.3, 0.01);
  EXPECT_LE((updated - reference).cwiseAbs().maxCoeff(), 0.003000001);
  EXPECT_NEAR(updated[0], 0.003, 1e-12);
  EXPECT_NEAR(updated[1], -0.003, 1e-12);
  EXPECT_NEAR(updated[4], 0.0, 1e-12);
}

TEST(ConstrainedDifferentialIk, AllSevenSoftLimitsPreventOutwardMotion) {
  DifferentialIkConfig config;
  for (int joint = 0; joint < 7; ++joint) {
    auto lower_input = nominalInput();
    lower_input.jacobian.setZero();
    lower_input.measured_position[joint] = -1.99;
    lower_input.previous_command = lower_input.measured_position;
    auto lower = rm75_touch_teleop::solveConstrainedDifferentialIk(
        lower_input, config);
    ASSERT_TRUE(lower.success()) << "joint " << joint << ": " << lower.reason;
    EXPECT_GE(lower.velocity[joint], -0.0250001);

    auto upper_input = nominalInput();
    upper_input.jacobian.setZero();
    upper_input.measured_position[joint] = 1.99;
    upper_input.previous_command = upper_input.measured_position;
    auto upper = rm75_touch_teleop::solveConstrainedDifferentialIk(
        upper_input, config);
    ASSERT_TRUE(upper.success()) << "joint " << joint << ": " << upper.reason;
    EXPECT_LE(upper.velocity[joint], 0.0250001);
  }
}

TEST(ConstrainedDifferentialIk, ActualDtSetsAccelerationBounds) {
  for (double dt : {0.002, 0.01, 0.04}) {
    auto input = nominalInput();
    input.dt = dt;
    input.previous_velocity.setConstant(0.1);
    input.pose_error.setConstant(-1.0);
    const auto output = rm75_touch_teleop::solveConstrainedDifferentialIk(
        input, DifferentialIkConfig());
    ASSERT_TRUE(output.success()) << output.reason;
    const JointVector acceleration =
        (output.velocity - input.previous_velocity) / dt;
    EXPECT_LE(acceleration.cwiseAbs().maxCoeff(), 1.500001);
  }
}

TEST(ConstrainedDifferentialIk, DirectionReversalIsAccelerationContinuous) {
  auto input = nominalInput();
  DifferentialIkConfig config;
  input.pose_error.setConstant(1.0);
  auto forward = rm75_touch_teleop::solveConstrainedDifferentialIk(input, config);
  ASSERT_TRUE(forward.success());
  input.previous_velocity = forward.velocity;
  input.pose_error.setConstant(-1.0);
  auto reverse = rm75_touch_teleop::solveConstrainedDifferentialIk(input, config);
  ASSERT_TRUE(reverse.success());
  EXPECT_LE(((reverse.velocity - forward.velocity) / input.dt)
                .cwiseAbs().maxCoeff(), 1.500001);
}

TEST(ConstrainedDifferentialIk, RejectsInvalidFeedbackWithoutFallback) {
  auto input = nominalInput();
  input.measured_position[3] = std::numeric_limits<double>::quiet_NaN();
  const auto result = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, DifferentialIkConfig());
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.status, rm75_touch_teleop::BoxQpStatus::INVALID_INPUT);
  EXPECT_TRUE(result.velocity.isZero(0.0));
}

TEST(ConstrainedDifferentialIk, OfflineTrajectoryTracksAndKeepsRedundancyStable) {
  auto input = nominalInput();
  DifferentialIkConfig config;
  JointVector q = JointVector::Zero();
  JointVector qdot = JointVector::Zero();
  JointVector posture_reference = JointVector::Zero();
  Eigen::Matrix<double, 6, 1> target;
  target << 0.05, -0.03, 0.02, 0.08, -0.04, 0.03;
  double max_speed = 0.0, max_acceleration = 0.0, max_jump = 0.0;
  double max_solve_time_us = 0.0, solve_time_sum_us = 0.0;
  double max_redundant_joint = 0.0;
  for (int step = 0; step < 1000; ++step) {
    input.measured_position = q;
    input.previous_command = q;
    input.previous_velocity = qdot;
    input.posture_reference = posture_reference;
    input.pose_error = target - input.jacobian * q;
    const auto output = rm75_touch_teleop::solveConstrainedDifferentialIk(
        input, config);
    ASSERT_TRUE(output.success()) << "step=" << step << " " << output.reason;
    max_speed = std::max(max_speed, output.velocity.cwiseAbs().maxCoeff());
    max_acceleration = std::max(
        max_acceleration,
        ((output.velocity - qdot) / input.dt).cwiseAbs().maxCoeff());
    max_jump = std::max(max_jump,
                        (output.velocity * input.dt).cwiseAbs().maxCoeff());
    const double solve_time_us = output.solve_time_sec * 1e6;
    max_solve_time_us = std::max(max_solve_time_us, solve_time_us);
    solve_time_sum_us += solve_time_us;
    qdot = output.velocity;
    q += qdot * input.dt;
    max_redundant_joint = std::max(max_redundant_joint, std::abs(q[6]));
    posture_reference += 0.3 * input.dt * (q - posture_reference);
  }
  const double final_error = (target - input.jacobian * q).norm();
  EXPECT_LT(final_error, 0.005);
  EXPECT_LE(max_speed, 0.500001);
  EXPECT_LE(max_acceleration, 1.500001);
  EXPECT_LE(max_jump, 0.005001);
  EXPECT_LT(std::abs(q[6]), 0.03);
  RecordProperty("final_task_error", std::to_string(final_error));
  RecordProperty("max_joint_speed", std::to_string(max_speed));
  RecordProperty("max_joint_acceleration", std::to_string(max_acceleration));
  RecordProperty("max_command_step", std::to_string(max_jump));
  RecordProperty("max_redundant_joint", std::to_string(max_redundant_joint));
  RecordProperty("average_qp_solve_us",
                 std::to_string(solve_time_sum_us / 1000.0));
  RecordProperty("max_qp_solve_us", std::to_string(max_solve_time_us));
}

TEST(ConstrainedDifferentialIk, QpHonorsBoundsThatDlsDoesNotModel) {
  auto input = nominalInput();
  input.pose_error.setConstant(0.5);
  input.previous_velocity.setZero();
  input.acceleration_limit.setConstant(0.2);
  const auto qp = rm75_touch_teleop::solveConstrainedDifferentialIk(
      input, DifferentialIkConfig());
  ASSERT_TRUE(qp.success());
  const Eigen::VectorXd dls = rm75_touch_teleop::dampedDifferentialIkStep(
      input.jacobian, input.pose_error, 0.01, 1.0) / input.dt;
  EXPECT_LE(qp.velocity.cwiseAbs().maxCoeff(), 0.0020001);
  EXPECT_GT(dls.cwiseAbs().maxCoeff(), qp.velocity.cwiseAbs().maxCoeff() * 10.0);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
