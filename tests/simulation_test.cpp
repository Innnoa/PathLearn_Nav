#include <gtest/gtest.h>

#include "impl/rule_based_dynamic_obstacle.hpp"
#include "impl/simple_collision_checker.hpp"
#include "impl/simple_environment.hpp"
#include "impl/simple_rrt_planner.hpp"
#include "impl/simple_simulation.hpp"

namespace {

std::string GetTestMapPath(const std::string& name) {
  return std::string(PATHLEARN_TEST_DATA_DIR) + "/" + name;
}

TEST(SimpleSimulationTest, ReachesGoalInEmptyMap) {
  pathlearn::SimpleEnvironment env;
  ASSERT_EQ(env.LoadFromFile(GetTestMapPath("empty_map.txt")).code,
            pathlearn::StatusCode::kOk);

  pathlearn::RuleBasedDynamicObstacle dynamic_model;
  pathlearn::SimpleCollisionChecker checker;
  pathlearn::SimpleRrtPlanner planner;

  pathlearn::SimulationConfig config{};
  config.horizon_sec = 5.0;
  config.time_step_sec = 1.0;
  config.max_steps = 10;
  config.goal_tolerance = 0.1;

  pathlearn::SimulationRequest request{};
  request.start.pose = {0.0, 0.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {4.0, 0.0, 0.0};
  request.limits.max_speed = 1.0;
  request.environment = &env;
  request.collision_checker = &checker;
  request.planner = &planner;
  request.dynamic_model = &dynamic_model;

  pathlearn::SimpleSimulation simulation;
  pathlearn::SimulationResult result{};
  ASSERT_EQ(simulation.Run(request, config, &result).code,
            pathlearn::StatusCode::kOk);
  EXPECT_EQ(result.metrics.success_rate, 1.0);
  EXPECT_EQ(result.metrics.collision_rate, 0.0);
  EXPECT_GT(result.first_solution_iteration, 0);
  ASSERT_FALSE(result.executed_trajectory.points.empty());
  const auto& last = result.executed_trajectory.points.back().state.pose;
  EXPECT_NEAR(last.x, request.goal.x, 1e-6);
  EXPECT_NEAR(last.y, request.goal.y, 1e-6);
}

}  // namespace
