#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "rm75_vertical_press/vertical_orientation.h"

TEST(VerticalOrientation, PointsToolZDownAndStaysRightHanded) {
  const Eigen::Matrix3d current =
      (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Eigen::Matrix3d result =
      rm75_vertical_press::verticalDownOrientation(current);

  EXPECT_TRUE(result.col(2).isApprox(Eigen::Vector3d(0, 0, -1), 1e-12));
  EXPECT_TRUE((result.transpose() * result).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_NEAR(result.determinant(), 1.0, 1e-12);
  EXPECT_GT(result.col(0).dot(Eigen::Vector3d(current(0, 0), current(1, 0), 0).normalized()),
            1.0 - 1e-12);
}

TEST(VerticalOrientation, HandlesVerticalCurrentXWithoutNan) {
  Eigen::Matrix3d current;
  current.col(0) = Eigen::Vector3d::UnitZ();
  current.col(1) = Eigen::Vector3d::UnitX();
  current.col(2) = Eigen::Vector3d::UnitY();
  const Eigen::Matrix3d result =
      rm75_vertical_press::verticalDownOrientation(current);

  EXPECT_TRUE(result.allFinite());
  EXPECT_TRUE(result.col(2).isApprox(Eigen::Vector3d(0, 0, -1), 1e-12));
  EXPECT_NEAR(result.determinant(), 1.0, 1e-12);
}

TEST(VerticalOrientation, TakesBoundedStepsTowardGoal) {
  const Eigen::Matrix3d current = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d goal =
      Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d first =
      rm75_vertical_press::stepTowardOrientation(current, goal, 0.1);

  EXPECT_NEAR(Eigen::Quaterniond(current).angularDistance(
                  Eigen::Quaterniond(first)), 0.1, 1e-12);
  EXPECT_NEAR(Eigen::Quaterniond(first).angularDistance(
                  Eigen::Quaterniond(goal)), 0.7, 1e-12);
  EXPECT_TRUE(rm75_vertical_press::stepTowardOrientation(first, goal, 1.0)
                  .isApprox(goal, 1e-12));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
