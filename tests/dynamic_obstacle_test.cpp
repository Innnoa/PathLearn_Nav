#include <gtest/gtest.h>

#include "impl/rule_based_dynamic_obstacle.hpp"

namespace {

TEST(RuleBasedDynamicObstacleTest, LineRulePrediction) {
  pathlearn::RuleBasedDynamicObstacle model;
  pathlearn::DynamicObstacleState state{};
  state.id = "obs_1";
  state.pose = {0.0, 0.0, 0.0};
  state.velocity = {1.0, 0.0};
  state.shape.radius = 0.5;

  ASSERT_EQ(model.Reset({state}).code, pathlearn::StatusCode::kOk);
  ASSERT_EQ(model.SetRuleLine("obs_1", {1.0, 0.0}).code,
            pathlearn::StatusCode::kOk);

  pathlearn::DynamicObstaclePrediction prediction{};
  ASSERT_EQ(model.Predict(0.0, 2.0, 1.0, &prediction).code,
            pathlearn::StatusCode::kOk);

  ASSERT_EQ(prediction.trajectories.size(), 1u);
  const auto& traj = prediction.trajectories[0];
  ASSERT_EQ(traj.points.size(), 3u);
  EXPECT_DOUBLE_EQ(traj.points[0].state.pose.x, 0.0);
  EXPECT_DOUBLE_EQ(traj.points[1].state.pose.x, 1.0);
  EXPECT_DOUBLE_EQ(traj.points[2].state.pose.x, 2.0);
}

TEST(RuleBasedDynamicObstacleTest, BounceRulePrediction) {
  pathlearn::RuleBasedDynamicObstacle model;
  pathlearn::DynamicObstacleState state{};
  state.id = "obs_2";
  state.pose = {0.0, 0.0, 0.0};
  state.velocity = {1.0, 0.0};
  state.shape.radius = 0.5;

  ASSERT_EQ(model.Reset({state}).code, pathlearn::StatusCode::kOk);
  ASSERT_EQ(model.SetRuleBounce("obs_2", {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}).code,
            pathlearn::StatusCode::kOk);

  pathlearn::DynamicObstaclePrediction prediction{};
  ASSERT_EQ(model.Predict(0.0, 4.0, 1.0, &prediction).code,
            pathlearn::StatusCode::kOk);

  const auto& traj = prediction.trajectories[0];
  EXPECT_DOUBLE_EQ(traj.points[0].state.pose.x, 0.0);
  EXPECT_DOUBLE_EQ(traj.points[1].state.pose.x, 1.0);
  EXPECT_DOUBLE_EQ(traj.points[2].state.pose.x, 2.0);
  EXPECT_DOUBLE_EQ(traj.points[3].state.pose.x, 1.0);
  EXPECT_DOUBLE_EQ(traj.points[4].state.pose.x, 0.0);
}

}  // namespace
