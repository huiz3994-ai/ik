#include <gtest/gtest.h>

#include "rm75_vertical_press/tool5_feed.h"

TEST(Tool5Feed, UsesDedicatedVelocityAtFiftyHertz) {
  EXPECT_DOUBLE_EQ(
      rm75_vertical_press::boundedTool5FeedStep(0.0002, 0.005, 0.02),
      0.0001);
}

TEST(Tool5Feed, RemainsMonotonicWhileDrilling) {
  double z = 0.5;
  for (int tick = 0; tick < 50; ++tick) {
    const double previous = z;
    z -= rm75_vertical_press::boundedTool5FeedStep(0.0002, 0.005, 0.02);
    EXPECT_LT(z, previous);
  }
  EXPECT_NEAR(z, 0.495, 1e-12);
}

TEST(Tool5Feed, CapsLargeTimerIntervalsByPerCycleStep) {
  EXPECT_DOUBLE_EQ(
      rm75_vertical_press::boundedTool5FeedStep(0.0002, 0.005, 1.0),
      0.0002);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
