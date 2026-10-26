#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <sstream>
#include <unistd.h>
#include "rm75_motion_planning/cartesian_path_generator.h"
#include "rm75_motion_planning/command_source_manager.h"
#include "rm75_motion_planning/continuous_ik_solver.h"
#include "rm75_motion_planning/moveit_collision_checker.h"
#include "rm75_motion_planning/persistent_ipopt_fallback.h"
#include "rm75_motion_planning/real_execution_metrics.h"
#include "rm75_motion_planning/runtime_guard.h"
#include "rm75_motion_planning/trajectory_parameterizer.h"
#include "rm75_motion_planning/trajectory_validator.h"

using namespace rm75_motion_planning;

namespace {
Pose makePose(const Eigen::Vector3d& p, const Eigen::Vector3d& axis,
              double angle) {
  Pose result = Pose::Identity();
  result.translation() = p;
  result.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return result;
}

DynamicLimits limits(int dof, double v = 1.0, double a = 0.5,
                     double j = 2.0) {
  return {Eigen::VectorXd::Constant(dof, v),
          Eigen::VectorXd::Constant(dof, a),
          Eigen::VectorXd::Constant(dof, j)};
}

class ThresholdCollisionChecker : public CollisionChecker {
 public:
  bool inCollision(const Eigen::VectorXd& q,
                   std::string* detail = nullptr) const override {
    const bool collision = q.size() > 0 && q[0] > .75;
    if (detail) *detail = collision ? "test obstacle" : "";
    return collision;
  }
};
}  // namespace

TEST(Se3, TranslationUsesAdaptiveSampleCount) {
  const Pose start = makePose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), 0);
  const Pose goal = makePose(Eigen::Vector3d(.023, 0, 0), Eigen::Vector3d::UnitZ(), 0);
  const auto path = Se3Interpolator::sample(start, goal, {.005, .05});
  ASSERT_EQ(path.size(), 6u);
  EXPECT_TRUE(path.front().matrix().isApprox(start.matrix(), 1e-14));
  EXPECT_TRUE(path.back().matrix().isApprox(goal.matrix(), 1e-14));
  for (std::size_t i = 1; i < path.size(); ++i)
    EXPECT_LE((path[i].translation() - path[i - 1].translation()).norm(), .005 + 1e-12);
}

TEST(Se3, SlerpNinetyDegrees) {
  const Pose start = Pose::Identity();
  const Pose goal = makePose(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), M_PI / 2);
  const Pose middle = Se3Interpolator::interpolate(start, goal, .5);
  EXPECT_NEAR(Se3Interpolator::rotationDistance(start.rotation(), middle.rotation()),
              M_PI / 4, 1e-12);
}

TEST(Se3, SlerpNearPiIsFiniteAndShortest) {
  const Pose start = Pose::Identity();
  const Pose goal = makePose(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 2, 3), M_PI - 1e-9);
  const auto path = Se3Interpolator::sample(start, goal, {.01, .05});
  ASSERT_GT(path.size(), 60u);
  for (const auto& pose : path) EXPECT_TRUE(pose.matrix().allFinite());
  EXPECT_NEAR(Se3Interpolator::rotationDistance(path.front().rotation(),
                                                path.back().rotation()),
              M_PI - 1e-9, 2e-8);
}

TEST(Se3, RandomLogExpRoundTrip) {
  std::mt19937 rng(7506);
  std::uniform_real_distribution<double> value(-1.0, 1.0);
  for (int i = 0; i < 100; ++i) {
    Eigen::Vector3d axis(value(rng), value(rng), value(rng));
    axis.normalize();
    const double angle = std::abs(value(rng)) * (M_PI - 1e-6);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    EXPECT_TRUE(Se3Interpolator::so3Exp(Se3Interpolator::so3Log(rotation))
                    .isApprox(rotation, 1e-9));
  }
}

TEST(Frames, UserAndToolTransformsAreApplied) {
  const Pose base_user = makePose({1, 0, 0}, Eigen::Vector3d::UnitZ(), M_PI / 2);
  const Pose user_tcp = makePose({.2, 0, 0}, Eigen::Vector3d::UnitZ(), 0);
  const Pose flange_tcp = makePose({0, 0, .1}, Eigen::Vector3d::UnitZ(), 0);
  const Pose flange = targetTcpToBaseFlange(base_user, user_tcp, flange_tcp);
  EXPECT_TRUE((flange * flange_tcp).matrix().isApprox(
      (base_user * user_tcp).matrix(), 1e-12));
}

TEST(Path, LineEndpointsAndStepBounds) {
  const Pose a = Pose::Identity();
  const Pose b = makePose({.031, -.01, .02}, Eigen::Vector3d::UnitY(), .4);
  const auto path = CartesianPathGenerator::line(a, b, {.005, .05});
  EXPECT_TRUE(path.front().matrix().isApprox(a.matrix()));
  EXPECT_TRUE(path.back().matrix().isApprox(b.matrix()));
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE((path[i].translation() - path[i - 1].translation()).norm(), .005 + 1e-12);
    EXPECT_LE(Se3Interpolator::rotationDistance(path[i - 1].rotation(),
                                                path[i].rotation()), .05 + 1e-12);
  }
}

TEST(Path, CirclePassesThroughViaAndIsContinuous) {
  const Pose start = makePose({1, 0, 0}, Eigen::Vector3d::UnitZ(), 0);
  const Pose via = makePose({0, 1, 0}, Eigen::Vector3d::UnitZ(), .4);
  const Pose goal = makePose({-1, 0, 0}, Eigen::Vector3d::UnitZ(), .8);
  const auto path = CartesianPathGenerator::circularArc(start, via, goal, {.05, .05});
  ASSERT_GT(path.size(), 3u);
  EXPECT_TRUE(std::any_of(path.begin(), path.end(), [&](const Pose& p) {
    return p.matrix().isApprox(via.matrix(), 1e-12);
  }));
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_NEAR(path[i].translation().norm(), 1.0, 1e-12);
    EXPECT_LE((path[i].translation() - path[i - 1].translation()).norm(), .051);
  }
}

TEST(Path, CircleRejectsCollinearPoints) {
  EXPECT_THROW(CartesianPathGenerator::circularArc(
                   makePose({0, 0, 0}, Eigen::Vector3d::UnitX(), 0),
                   makePose({1, 0, 0}, Eigen::Vector3d::UnitX(), 0),
                   makePose({2, 0, 0}, Eigen::Vector3d::UnitX(), 0)),
               std::invalid_argument);
}

TEST(Path, MultiWaypointPreservesOrderAndNoDuplicates) {
  PoseVector waypoints{
      makePose({0, 0, 0}, Eigen::Vector3d::UnitZ(), 0),
      makePose({.02, 0, 0}, Eigen::Vector3d::UnitZ(), .1),
      makePose({.02, .03, 0}, Eigen::Vector3d::UnitZ(), .2)};
  const auto path = CartesianPathGenerator::multiWaypoint(waypoints, {.005, .05});
  for (const auto& waypoint : waypoints)
    EXPECT_TRUE(std::any_of(path.begin(), path.end(), [&](const Pose& p) {
      return p.matrix().isApprox(waypoint.matrix(), 1e-12);
    }));
  for (std::size_t i = 1; i < path.size(); ++i)
    EXPECT_FALSE(path[i].matrix().isApprox(path[i - 1].matrix(), 1e-14));
}

TEST(ContinuousIk, TracksReachablePathUsingPreviousSolution) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  Eigen::VectorXd q0(7), q1(7);
  q0 << .2, -.5, .25, -.7, .3, .6, -.2;
  q1 = q0; q1[0] += .03; q1[3] -= .02;
  const auto p0 = model.forward(q0), p1 = model.forward(q1);
  Pose start = p0.isometry(), goal = p1.isometry();
  const auto poses = CartesianPathGenerator::line(start, goal, {.002, .02});
  ContinuousIkSolver solver(model);
  rm75_kinematics::IkOptions ik; ik.position_tolerance = 2e-4;
  ik.orientation_tolerance = 2e-3; ik.timeout_sec = .1;
  const auto result = solver.solve(poses, q0, {}, ik);
  ASSERT_TRUE(result.success) << result.failure_reason;
  ASSERT_EQ(result.joints.size(), result.solved_poses.size());
  EXPECT_LT((model.forward(result.joints.back()).position - p1.position).norm(), 3e-4);
}

TEST(ContinuousIk, NearSingularSmallPathRemainsFinite) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q1 = q0; q1[0] = .005; q1[2] = -.004;
  const auto poses = CartesianPathGenerator::line(model.forward(q0).isometry(),
                                                   model.forward(q1).isometry(),
                                                   {.001, .01});
  ContinuousIkSolver solver(model);
  const auto result = solver.solve(poses, q0);
  ASSERT_TRUE(result.success) << result.failure_reason;
  for (const auto& q : result.joints) EXPECT_TRUE(q.allFinite());
}

TEST(ContinuousIk, UnreachableTargetFailsWithoutSkipping) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
  PoseVector poses{model.forward(q0).isometry(), Pose::Identity()};
  poses.back().translation() = Eigen::Vector3d(10, 10, 10);
  ContinuousIkOptions options; options.max_refinement_depth = 1;
  options.enable_ipopt_fallback = false;
  rm75_kinematics::IkOptions ik; ik.timeout_sec = .01; ik.max_iterations = 20;
  const auto result = ContinuousIkSolver(model).solve(poses, q0, options, ik);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.joints.size(), 1u);
  EXPECT_FALSE(result.failure_reason.empty());
}

TEST(Trajectory, QuinticMeetsVelocityAccelerationAndJerkLimits) {
  JointPath path;
  path.push_back(Eigen::VectorXd::Zero(7));
  path.push_back(Eigen::VectorXd::Constant(7, .4));
  path.push_back(Eigen::VectorXd::Constant(7, .2));
  const auto dynamic = limits(7);
  const auto trajectory = TrajectoryParameterizer::parameterize(path, dynamic, .02);
  ASSERT_GT(trajectory.size(), 3u);
  EXPECT_TRUE(trajectory.front().position.isApprox(path.front()));
  EXPECT_TRUE(trajectory.back().position.isApprox(path.back(), 1e-12));
  for (const auto& point : trajectory) {
    EXPECT_LE(point.velocity.cwiseAbs().maxCoeff(), 1.0 + 1e-10);
    EXPECT_LE(point.acceleration.cwiseAbs().maxCoeff(), .5 + 1e-10);
    EXPECT_LE(point.jerk.cwiseAbs().maxCoeff(), 2.0 + 1e-10);
  }
}

TEST(Trajectory, EveryInputWaypointIsReachedExactly) {
  JointPath path{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Constant(2, .2),
                 Eigen::VectorXd::Constant(2, -.1)};
  const auto trajectory = TrajectoryParameterizer::parameterize(path, limits(2), .02);
  for (const auto& waypoint : path)
    EXPECT_TRUE(std::any_of(trajectory.begin(), trajectory.end(),
                            [&](const TrajectoryPoint& point) {
      return point.position.isApprox(waypoint, 1e-12);
    }));
}

TEST(Trajectory, MinimumDurationRegeneratesFixedPeriodSamples) {
  JointPath path{Eigen::VectorXd::Zero(2), Eigen::VectorXd::Constant(2, .01)};
  const auto dynamic = limits(2, 1.0, 1.0, 10.0);
  const auto trajectory = TrajectoryParameterizer::parameterizeAtLeastDuration(
      path, dynamic, 1.0, .02);
  ASSERT_GT(trajectory.size(), 2u);
  EXPECT_GE(trajectory.back().time, 1.0);
  for (std::size_t index = 1; index < trajectory.size(); ++index)
    EXPECT_NEAR(trajectory[index].time - trajectory[index - 1].time,
                .02, 1e-12);
  for (const auto& point : trajectory) {
    EXPECT_LE(point.velocity.cwiseAbs().maxCoeff(), 1.0 + 1e-12);
    EXPECT_LE(point.acceleration.cwiseAbs().maxCoeff(), 1.0 + 1e-12);
    EXPECT_LE(point.jerk.cwiseAbs().maxCoeff(), 10.0 + 1e-12);
  }
}

TEST(Validator, AcceptsValidFkAndDynamics) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  Eigen::VectorXd q0(7), q1(7);
  q0 << .2, -.5, .25, -.7, .3, .6, -.2;
  q1 = q0; q1[6] += .1;
  JointPath joints{q0, q1};
  PoseVector poses{model.forward(q0).isometry(), model.forward(q1).isometry()};
  const auto dynamic = limits(7);
  const auto timed = TrajectoryParameterizer::parameterize(joints, dynamic);
  const auto report = TrajectoryValidator(model).validate(poses, joints, timed, dynamic);
  EXPECT_TRUE(report.valid);
  EXPECT_LT(report.max_position_error_m, 1e-12);
  EXPECT_LT(report.max_orientation_error_rad, 1e-12);
  EXPECT_GE(report.min_joint_limit_margin_rad, 0.0);
}

TEST(Validator, DetectsJointJumpAndFkMismatch) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7), q1 = q0;
  q1[0] = .8;
  JointPath joints{q0, q1};
  PoseVector poses{model.forward(q0).isometry(), model.forward(q0).isometry()};
  const auto dynamic = limits(7);
  const auto report = TrajectoryValidator(model).validate(
      poses, joints, TrajectoryParameterizer::parameterize(joints, dynamic),
      dynamic, .001, .01, .2);
  EXPECT_FALSE(report.valid);
  EXPECT_GT(report.max_joint_jump_rad, .2);
  EXPECT_GT(report.max_orientation_error_rad, .01);
}

TEST(Collision, PlanningSceneChecksSelfAndEnvironmentAtTimedSamples) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  MoveitCollisionChecker collision(RM75_TEST_URDF, RM75_TEST_SRDF);
  Eigen::VectorXd q0(7), q1(7);
  q0 << 0, .8395, 0, 1.4442, 0, .8513, 0;
  q1 = q0; q1[6] = .05;
  std::string detail;
  EXPECT_FALSE(collision.inCollision(q0, &detail)) << detail;
  JointPath joints{q0, q1};
  PoseVector poses{model.forward(q0).isometry(), model.forward(q1).isometry()};
  const auto dynamic = limits(7);
  const auto timed = TrajectoryParameterizer::parameterize(joints, dynamic);
  auto report = TrajectoryValidator(model, &collision).validate(
      poses, joints, timed, dynamic, .002, .034906585, .35, true);
  ASSERT_TRUE(report.valid);
  EXPECT_TRUE(report.collision_checked);
  EXPECT_TRUE(report.collision_free);
  EXPECT_EQ(report.collision_checks, timed.size());

  collision.addBox("blocking_box", Eigen::Vector3d(3, 3, 3),
                   Eigen::Isometry3d::Identity());
  report = TrajectoryValidator(model, &collision).validate(
      poses, joints, timed, dynamic, .002, .034906585, .35, true);
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.collision_free);
  EXPECT_FALSE(report.first_collision.empty());
  collision.removeObject("blocking_box");
  EXPECT_FALSE(collision.inCollision(q0, &detail)) << detail;
}

TEST(Collision, RequiredCheckerCannotBeSilentlyOmitted) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  const Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
  JointPath joints{q};
  PoseVector poses{model.forward(q).isometry()};
  const auto dynamic = limits(7);
  const auto timed = TrajectoryParameterizer::parameterize(joints, dynamic);
  const auto report = TrajectoryValidator(model).validate(
      poses, joints, timed, dynamic, .002, .034906585, .35, true);
  EXPECT_FALSE(report.valid);
  EXPECT_FALSE(report.collision_checked);
}

TEST(RuntimeGuard, AllowsFreshSafeCurrentAndNextState) {
  RuntimeGuardConfig config;
  config.lower = Eigen::VectorXd::Constant(7, -1.0);
  config.upper = Eigen::VectorXd::Constant(7, 1.0);
  ThresholdCollisionChecker collision;
  RuntimeGuard guard(config, &collision);
  RuntimeGuardInput input;
  input.current = Eigen::VectorXd::Zero(7);
  input.next_target = Eigen::VectorXd::Constant(7, .01);
  input.previous_target = Eigen::VectorXd::Zero(7);
  input.now_sec = 5.0;
  input.feedback_stamp_sec = 4.95;
  const auto result = guard.evaluate(input);
  EXPECT_EQ(result.decision, GuardDecision::ALLOW) << result.reason;
  EXPECT_TRUE(result.allow);
  EXPECT_TRUE(result.collision_checked);
}

TEST(RuntimeGuard, StopsOnTimeoutTrackingLimitAndCollision) {
  RuntimeGuardConfig config;
  config.lower = Eigen::VectorXd::Constant(7, -1.0);
  config.upper = Eigen::VectorXd::Constant(7, 1.0);
  ThresholdCollisionChecker collision;
  RuntimeGuard guard(config, &collision);
  RuntimeGuardInput input;
  input.current = Eigen::VectorXd::Zero(7);
  input.next_target = Eigen::VectorXd::Zero(7);
  input.previous_target = Eigen::VectorXd::Zero(7);
  input.now_sec = 1.0;
  input.feedback_stamp_sec = .8;
  const auto timeout = guard.evaluate(input);
  EXPECT_EQ(timeout.decision, GuardDecision::STOP);
  EXPECT_TRUE(timeout.timeout);
  EXPECT_FALSE(timeout.allow);

  input.feedback_stamp_sec = .95;
  input.previous_target[2] = .04;
  const auto tracking = guard.evaluate(input);
  EXPECT_EQ(tracking.decision, GuardDecision::STOP);
  EXPECT_TRUE(tracking.tracking_error);
  input.previous_target.setZero();
  input.next_target[1] = .99;
  const auto limit = guard.evaluate(input);
  EXPECT_EQ(limit.decision, GuardDecision::STOP);
  EXPECT_TRUE(limit.joint_limit);
  input.next_target.setZero();
  input.next_target[0] = .8;
  const auto collision_result = guard.evaluate(input);
  EXPECT_EQ(collision_result.decision, GuardDecision::STOP);
  EXPECT_TRUE(collision_result.collision);
  EXPECT_NE(collision_result.reason.find("next-target collision"),
            std::string::npos);
}

TEST(RuntimeGuard, PredictsNextCycleJointLimitFromVelocity) {
  RuntimeGuardConfig config;
  config.lower = Eigen::VectorXd::Constant(7, -1.0);
  config.upper = Eigen::VectorXd::Constant(7, 1.0);
  ThresholdCollisionChecker collision;
  RuntimeGuard guard(config, &collision);
  RuntimeGuardInput input;
  input.current = Eigen::VectorXd::Zero(7);
  input.current[4] = .95;
  input.next_target = input.current;
  input.previous_target = input.current;
  input.commanded_velocity = Eigen::VectorXd::Zero(7);
  input.commanded_velocity[4] = 3.0;
  input.cycle_sec = .02;
  input.now_sec = 3.0;
  input.feedback_stamp_sec = 2.99;
  const auto result = guard.evaluate(input);
  EXPECT_EQ(result.decision, GuardDecision::STOP);
  EXPECT_TRUE(result.joint_limit);
  EXPECT_LT(result.min_limit_margin_rad, 0.0);
}

TEST(RuntimeGuard, WarnsBeforeHardTrackingThreshold) {
  RuntimeGuardConfig config;
  config.lower = Eigen::VectorXd::Constant(7, -1.0);
  config.upper = Eigen::VectorXd::Constant(7, 1.0);
  ThresholdCollisionChecker collision;
  RuntimeGuard guard(config, &collision);
  RuntimeGuardInput input;
  input.current = Eigen::VectorXd::Zero(7);
  input.next_target = Eigen::VectorXd::Zero(7);
  input.previous_target = Eigen::VectorXd::Zero(7);
  input.previous_target[3] = .025;
  input.now_sec = 2.0;
  input.feedback_stamp_sec = 1.99;
  EXPECT_EQ(guard.evaluate(input).decision, GuardDecision::WARN);
}

TEST(RuntimeGuard, RecordsStopTimeAndDistance) {
  StopRecorder recorder;
  recorder.request(10.0, Eigen::VectorXd::Zero(7));
  Eigen::VectorXd moved = Eigen::VectorXd::Zero(7);
  moved[4] = .012;
  recorder.observe(10.18, moved);
  EXPECT_NEAR(recorder.elapsedSec(), .18, 1e-12);
  EXPECT_NEAR(recorder.stopLatencySec(), .18, 1e-12);
  EXPECT_NEAR(recorder.maxJointDistanceRad(), .012, 1e-12);
  recorder.observe(10.38, moved);
  EXPECT_NEAR(recorder.stopLatencySec(), .18, 1e-12);
}

TEST(RealExecutionMetrics, ComputesMeanRmsP95AndMaximum) {
  std::vector<double> samples;
  for (int value = 1; value <= 100; ++value)
    samples.push_back(static_cast<double>(value));
  const auto summary = summarizeDistribution(samples);
  EXPECT_EQ(summary.count, 100u);
  EXPECT_NEAR(summary.mean, 50.5, 1e-12);
  EXPECT_NEAR(summary.rms, std::sqrt(3383.5), 1e-12);
  EXPECT_NEAR(summary.maximum, 100.0, 1e-12);
  EXPECT_NEAR(summary.p95, 95.05, 1e-12);
  EXPECT_NEAR(summary.p99, 99.01, 1e-12);
  EXPECT_THROW(summarizeDistribution({}), std::invalid_argument);
}

TEST(RealExecutionMetrics, RejectsInconsistentTimestamps) {
  std::string reason;
  EXPECT_TRUE(timestampsAreConsistent(
      {1.02, 1.04, 1.06}, {1.01, 1.03, 1.05},
      {1.00, 1.02, 1.04}, &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_FALSE(timestampsAreConsistent(
      {1.02, 1.04, 1.06}, {1.01, 1.03, 1.05},
      {1.00, .99, 1.04}, &reason));
  EXPECT_EQ(reason, "timestamps are not monotonic");
}

TEST(RealExecutionMetrics, MeasuresFkPoseError) {
  rm75_kinematics::KinematicsModel model(RM75_TEST_URDF);
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q1 = q0;
  q1[6] = .01;
  const Pose desired = model.forward(q0).isometry();
  const Pose actual = model.forward(q1).isometry();
  const auto error = measurePoseError(desired, actual);
  EXPECT_TRUE(std::isfinite(error.position_m));
  EXPECT_TRUE(std::isfinite(error.orientation_rad));
  EXPECT_GT(error.orientation_rad, 0.0);
  EXPECT_NEAR(measurePoseError(desired, desired).position_m, 0.0, 1e-14);
}

TEST(RealExecutionMetrics, GeneratesRawCsvAndRunJson) {
  const std::string prefix = "/tmp/rm75-real-execution-" +
      std::to_string(static_cast<long long>(getpid()));
  const std::string csv_path = prefix + ".csv";
  const std::string json_path = prefix + ".json";
  std::remove(csv_path.c_str());
  std::remove(json_path.c_str());
  {
    RealExecutionCsvWriter writer(csv_path);
    RealExecutionSample sample;
    sample.run_id = "x-001";
    sample.scenario = "x";
    sample.repetition = 1;
    sample.requested_linear_speed_mps = .01;
    sample.requested_angular_speed_rad_s = .0872664626;
    sample.sample_time_sec = 10.02;
    sample.command_time_sec = 10.01;
    sample.feedback_time_sec = 10.00;
    sample.feedback_source_time_sec = 9.998;
    sample.feedback_source_time_valid = true;
    sample.command_published = true;
    sample.target_joints = Eigen::VectorXd::Zero(7);
    sample.target_velocity = Eigen::VectorXd::Constant(7, .01);
    sample.target_acceleration = Eigen::VectorXd::Zero(7);
    sample.actual_joints = Eigen::VectorXd::Constant(7, .001);
    sample.force_n = 1.2;
    sample.torque_nm = .04;
    writer.append(sample);
    EXPECT_EQ(writer.rowsWritten(), 1u);
  }
  writeRealExecutionRunJson(json_path, "x-001", "x", "PASS", 1,
                            {.001, .002}, "");
  std::ifstream csv_stream(csv_path);
  std::stringstream csv_text;
  csv_text << csv_stream.rdbuf();
  EXPECT_NE(csv_text.str().find("data_source,reference_frame"), std::string::npos);
  EXPECT_NE(csv_text.str().find("real_robot,base_link_to_Link7,x-001,x,1,0.01"),
            std::string::npos);
  std::ifstream json_stream(json_path);
  std::stringstream json_text;
  json_text << json_stream.rdbuf();
  EXPECT_NE(json_text.str().find("\"status\": \"PASS\""), std::string::npos);
  EXPECT_NE(json_text.str().find("\"samples\": 1"), std::string::npos);
  std::remove(csv_path.c_str());
  std::remove(json_path.c_str());
}

TEST(ExecutionGate, DefaultsDisarmedAndEnforcesSingleSource) {
  const Eigen::VectorXd command = Eigen::VectorXd::Zero(7);
  CommandSourceManager disarmed;
  ASSERT_TRUE(disarmed.acquire(CommandSource::MOVEIT));
  EXPECT_FALSE(disarmed.commandPublisherAllowed());
  std::string reason;
  EXPECT_FALSE(disarmed.accept(CommandSource::MOVEIT, command, &reason));
  EXPECT_EQ(reason, "execution is disarmed");

  CommandSourceManager armed(true);
  EXPECT_FALSE(armed.commandPublisherAllowed());
  ASSERT_TRUE(armed.acquire(CommandSource::TOUCH));
  EXPECT_TRUE(armed.commandPublisherAllowed());
  EXPECT_FALSE(armed.acquire(CommandSource::AUTONOMOUS, &reason));
  EXPECT_FALSE(armed.accept(CommandSource::MOVEIT, command, &reason));
  EXPECT_TRUE(armed.accept(CommandSource::TOUCH, command, &reason));
}

TEST(ExecutionGate, GuardedGatewayRequiresOwnershipAndSafety) {
  RuntimeGuardConfig config;
  config.lower = Eigen::VectorXd::Constant(7, -1.0);
  config.upper = Eigen::VectorXd::Constant(7, 1.0);
  ThresholdCollisionChecker collision;
  RuntimeGuard guard(config, &collision);
  CommandSourceManager sources(true);
  ASSERT_TRUE(sources.acquire(CommandSource::AUTONOMOUS));
  GuardedCommandGateway gateway(sources, guard);
  RuntimeGuardInput input;
  input.current = Eigen::VectorXd::Zero(7);
  input.next_target = Eigen::VectorXd::Constant(7, .01);
  input.previous_target = Eigen::VectorXd::Zero(7);
  input.now_sec = 4.0;
  input.feedback_stamp_sec = 3.99;
  EXPECT_TRUE(gateway.evaluate(CommandSource::AUTONOMOUS, input).accepted);
  EXPECT_FALSE(gateway.evaluate(CommandSource::TOUCH, input).accepted);
  input.next_target[0] = .8;
  const auto stopped = gateway.evaluate(CommandSource::AUTONOMOUS, input);
  EXPECT_FALSE(stopped.accepted);
  EXPECT_EQ(stopped.decision, GuardDecision::STOP);
  EXPECT_TRUE(stopped.guard.collision);
}

TEST(IpoptFallback, PersistentProcessProtocolAndCallback) {
  PersistentIpoptFallback fallback("/usr/bin/python3", RM75_FAKE_IPOPT_SERVER,
                                   "unused", "unused", 2000);
  ASSERT_TRUE(fallback.healthy());
  rm75_kinematics::Pose target;
  target.position = Eigen::Vector3d(.1, .2, .3);
  const Eigen::VectorXd seed = Eigen::VectorXd::LinSpaced(7, -.3, .3);
  const auto first = fallback.solve(target, seed);
  EXPECT_EQ(first.status, rm75_kinematics::IkStatus::SUCCESS);
  EXPECT_TRUE(first.q.isApprox(seed));
  const auto second = fallback.callback()(target, seed);
  EXPECT_EQ(second.status, rm75_kinematics::IkStatus::SUCCESS);
  EXPECT_TRUE(second.q.isApprox(seed));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
