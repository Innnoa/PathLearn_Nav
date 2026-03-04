#include <gtest/gtest.h>

#include "core/types.hpp"

namespace {

TEST(CoreTypesTest, DefaultValuesAreZero) {
  pathlearn::State2D state{};
  EXPECT_DOUBLE_EQ(state.pose.x, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.y, 0.0);
  EXPECT_DOUBLE_EQ(state.pose.theta, 0.0);
  EXPECT_DOUBLE_EQ(state.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(state.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(state.acceleration.x, 0.0);
  EXPECT_DOUBLE_EQ(state.acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(state.time_sec, 0.0);
}

TEST(CoreTypesTest, TrajectoryStoresPointsInOrder) {
  pathlearn::Trajectory trajectory{};
  pathlearn::TrajectoryPoint p1{};
  pathlearn::TrajectoryPoint p2{};
  p1.state.time_sec = 1.0;
  p2.state.time_sec = 2.0;
  trajectory.points.push_back(p1);
  trajectory.points.push_back(p2);

  ASSERT_EQ(trajectory.points.size(), 2u);
  EXPECT_DOUBLE_EQ(trajectory.points[0].state.time_sec, 1.0);
  EXPECT_DOUBLE_EQ(trajectory.points[1].state.time_sec, 2.0);
}

TEST(CoreTypesTest, StatusDefaultsToOk) {
  pathlearn::Status status{};
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
}

}  // namespace
