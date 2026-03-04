#include <gtest/gtest.h>

#include "impl/simple_collision_checker.hpp"
#include "impl/simple_astar_planner.hpp"
#include "impl/simple_environment.hpp"
#include "impl/simple_rrt_planner.hpp"

namespace {

std::string GetTestMapPath(const std::string& name) {
  return std::string(PATHLEARN_TEST_DATA_DIR) + "/" + name;
}

TEST(SimpleRrtPlannerTest, StraightLineSuccess) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("empty_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleRrtPlanner planner;

  pathlearn::PlannerRequest request{};
  request.start.pose = {0.0, 0.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 0.0, 0.0};
  request.limits.max_speed = 1.0;
  request.horizon_sec = 10.0;
  request.time_step_sec = 1.0;
  request.collision.environment = &env;
  request.collision_checker = &checker;

  pathlearn::PlannerResult result{};
  ASSERT_EQ(planner.Plan(request, &result).code, pathlearn::StatusCode::kOk);
  ASSERT_FALSE(result.trajectory.points.empty());
  EXPECT_GT(result.first_solution_iteration, 0);
  const auto& last = result.trajectory.points.back().state.pose;
  EXPECT_NEAR(last.x, request.goal.x, 1e-6);
  EXPECT_NEAR(last.y, request.goal.y, 1e-6);
}

TEST(SimpleRrtPlannerTest, StartCollisionFails) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("simple_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleRrtPlanner planner;

  pathlearn::PlannerRequest request{};
  request.start.pose = {0.0, 0.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 0.0, 0.0};
  request.limits.max_speed = 1.0;
  request.horizon_sec = 10.0;
  request.time_step_sec = 1.0;
  request.collision.environment = &env;
  request.collision_checker = &checker;

  pathlearn::PlannerResult result{};
  ASSERT_EQ(planner.Plan(request, &result).code, pathlearn::StatusCode::kCollision);
}

TEST(SimpleAStarPlannerTest, StraightLineSuccess) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("empty_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleAStarPlanner planner;

  pathlearn::PlannerRequest request{};
  request.start.pose = {-4.0, 0.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 0.0, 0.0};
  request.limits.max_speed = 1.0;
  request.horizon_sec = 20.0;
  request.time_step_sec = 1.0;
  request.collision.environment = &env;
  request.collision_checker = &checker;
  request.astar_grid_resolution = 0.5;
  request.astar_allow_diagonal = true;
  request.astar_heuristic_weight = 1.2;

  pathlearn::PlannerResult result{};
  ASSERT_EQ(planner.Plan(request, &result).code, pathlearn::StatusCode::kOk);
  EXPECT_GT(result.iterations, 0);
  EXPECT_GT(result.first_solution_iteration, 0);
  ASSERT_FALSE(result.trajectory.points.empty());
  const auto& last = result.trajectory.points.back().state.pose;
  EXPECT_NEAR(last.x, request.goal.x, 1e-6);
  EXPECT_NEAR(last.y, request.goal.y, 1e-6);
}

TEST(SimpleAStarPlannerTest, AvoidsCenterObstacle) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("simple_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleAStarPlanner planner;

  pathlearn::PlannerRequest request{};
  request.start.pose = {-4.0, 0.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 0.0, 0.0};
  request.limits.max_speed = 1.0;
  request.horizon_sec = 30.0;
  request.time_step_sec = 1.0;
  request.collision.environment = &env;
  request.collision_checker = &checker;
  request.collision.robot_radius = 0.2;
  request.astar_grid_resolution = 0.5;
  request.astar_allow_diagonal = true;
  request.astar_heuristic_weight = 1.2;

  pathlearn::PlannerResult result{};
  ASSERT_EQ(planner.Plan(request, &result).code, pathlearn::StatusCode::kOk);
  ASSERT_GE(result.trajectory.points.size(), 2u);

  bool hit = false;
  ASSERT_EQ(checker.CheckTrajectory(request.collision, result.trajectory, &hit).code,
            pathlearn::StatusCode::kOk);
  EXPECT_FALSE(hit);
}

TEST(SimpleAStarPlannerTest, SupportsLearnedHeuristicHint) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("simple_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleAStarPlanner planner;

  pathlearn::PlannerRequest request{};
  request.start.pose = {-4.0, -4.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 4.0, 0.0};
  request.limits.max_speed = 1.0;
  request.horizon_sec = 40.0;
  request.time_step_sec = 1.0;
  request.collision.environment = &env;
  request.collision_checker = &checker;
  request.astar_grid_resolution = 0.5;
  request.astar_allow_diagonal = true;
  request.astar_heuristic_weight = 1.2;
  request.nn_guidance_enabled = true;
  request.nn_model_path = GetTestMapPath("nn_hint_model.txt");
  request.nn_sample_probability = 0.8;
  request.nn_sample_distance = 1.0;

  pathlearn::PlannerResult result{};
  ASSERT_EQ(planner.Plan(request, &result).code, pathlearn::StatusCode::kOk);
  EXPECT_GT(result.iterations, 0);
  EXPECT_GT(result.first_solution_iteration, 0);
}

}  // namespace
