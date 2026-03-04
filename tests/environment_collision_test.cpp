#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "impl/simple_collision_checker.hpp"
#include "impl/simple_environment.hpp"

namespace {

std::string GetTestMapPath() {
  return std::string(PATHLEARN_TEST_DATA_DIR) + "/simple_map.txt";
}

TEST(SimpleEnvironmentTest, LoadAndQueries) {
  pathlearn::SimpleEnvironment env;
  pathlearn::Status status = env.LoadFromFile(GetTestMapPath());
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);

  pathlearn::MapInfo info = env.GetMapInfo();
  EXPECT_EQ(info.name, "test_map");
  EXPECT_TRUE(env.IsInsideBounds({0.0, 0.0, 0.0}));
  EXPECT_FALSE(env.IsInsideBounds({20.0, 0.0, 0.0}));

  bool occupied = false;
  status = env.IsOccupied({0.0, 0.0, 0.0}, &occupied);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(occupied);

  status = env.IsOccupied({2.0, 2.0, 0.0}, &occupied);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_FALSE(occupied);

  status = env.IsOccupied({20.0, 0.0, 0.0}, &occupied);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(occupied);

  double distance = 0.0;
  status = env.DistanceToNearestObstacle({0.0, 2.0, 0.0}, &distance);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_NEAR(distance, 1.0, 1e-6);

  bool collision = false;
  status = env.CheckSegmentCollision(
      {-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(collision);

  status = env.CheckSegmentCollision(
      {-2.0, 2.0, 0.0}, {2.0, 2.0, 0.0}, &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_FALSE(collision);
}

TEST(SimpleCollisionCheckerTest, StaticAndDynamicCollision) {
  pathlearn::SimpleEnvironment env;
  pathlearn::Status status = env.LoadFromFile(GetTestMapPath());
  ASSERT_EQ(status.code, pathlearn::StatusCode::kOk);

  pathlearn::CollisionContext context{};
  context.environment = &env;

  pathlearn::SimpleCollisionChecker checker;
  bool collision = false;
  status = checker.CheckState(context, {{0.0, 0.0, 0.0}, {}, {}, 0.0},
                              &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(collision);

  status = checker.CheckState(context, {{2.0, 2.0, 0.0}, {}, {}, 0.0},
                              &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_FALSE(collision);

  pathlearn::DynamicObstaclePrediction prediction{};
  prediction.start_time_sec = 0.0;
  prediction.time_step_sec = 1.0;
  pathlearn::DynamicObstacleTrajectory traj{};
  traj.id = "obs_1";
  traj.shape.radius = 0.5;
  pathlearn::TrajectoryPoint p0{};
  pathlearn::TrajectoryPoint p1{};
  pathlearn::TrajectoryPoint p2{};
  p0.state.pose = {0.0, 0.0, 0.0};
  p1.state.pose = {1.0, 1.0, 0.0};
  p2.state.pose = {2.0, 2.0, 0.0};
  p0.state.time_sec = 0.0;
  p1.state.time_sec = 1.0;
  p2.state.time_sec = 2.0;
  traj.points = {p0, p1, p2};
  prediction.trajectories.push_back(traj);

  context.prediction = &prediction;

  status = checker.CheckState(context, {{1.0, 1.0, 0.0}, {}, {}, 1.0},
                              &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(collision);

  context.prediction = nullptr;

  pathlearn::Trajectory trajectory{};
  pathlearn::TrajectoryPoint t0{};
  pathlearn::TrajectoryPoint t1{};
  t0.state.pose = {2.0, 0.0, 0.0};
  t1.state.pose = {2.0, 2.0, 0.0};
  t0.state.time_sec = 0.0;
  t1.state.time_sec = 1.0;
  trajectory.points = {t0, t1};

  context.robot_radius = 0.0;
  status = checker.CheckTrajectory(context, trajectory, &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_FALSE(collision);

  context.robot_radius = 1.0;
  status = checker.CheckTrajectory(context, trajectory, &collision);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_TRUE(collision);

  double min_distance = 0.0;
  context.robot_radius = 0.0;
  status = checker.MinimumDistance(context, trajectory, &min_distance);
  EXPECT_EQ(status.code, pathlearn::StatusCode::kOk);
  EXPECT_GT(min_distance, 0.0);
}

}  // namespace
