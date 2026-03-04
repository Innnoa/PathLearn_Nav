#include <gtest/gtest.h>

#include "impl/simple_evaluator.hpp"

namespace {

TEST(EvaluationSummaryTest, AggregatesSuccessAndCollision) {
  std::vector<pathlearn::EvaluationRunResult> runs;
  runs.push_back({"scene_01", "scene_01", "line", 1u, false, {}, {},
                  {pathlearn::StatusCode::kOk, ""},
                  {1.0, 0.0, 2.0, 10.0, 0.3},
                  120});
  runs.push_back({"scene_01", "scene_01", "line", 2u, false, {}, {},
                  {pathlearn::StatusCode::kOk, ""},
                  {1.0, 0.0, 4.0, 14.0, 0.5},
                  80});
  runs.push_back({"scene_01", "scene_01", "line", 3u, false, {}, {},
                  {pathlearn::StatusCode::kCollision, "碰撞"},
                  {0.0, 1.0, 0.0, 0.0, 0.0},
                  0});

  const pathlearn::EvaluationSummary summary =
      pathlearn::SummarizeRuns(runs);

  EXPECT_EQ(summary.total_runs, 3);
  EXPECT_EQ(summary.success_runs, 2);
  EXPECT_EQ(summary.collision_runs, 1);
  EXPECT_EQ(summary.no_path_runs, 0);
  EXPECT_NEAR(summary.success_rate, 2.0 / 3.0, 1e-9);
  EXPECT_NEAR(summary.collision_rate, 1.0 / 3.0, 1e-9);
  EXPECT_DOUBLE_EQ(summary.avg_exec_time_sec, 3.0);
  EXPECT_DOUBLE_EQ(summary.avg_path_length, 12.0);
  EXPECT_DOUBLE_EQ(summary.avg_min_safety_distance, 0.4);
  EXPECT_DOUBLE_EQ(summary.avg_first_solution_iteration, 100.0);
}

TEST(EvaluationSummaryTest, HandlesNoSuccess) {
  std::vector<pathlearn::EvaluationRunResult> runs;
  runs.push_back({"scene_02", "scene_02", "bounce", 1u, true, {}, {},
                  {pathlearn::StatusCode::kNoPath, "无路径"},
                  {0.0, 0.0, 0.0, 0.0, 0.0},
                  0});
  runs.push_back({"scene_02", "scene_02", "bounce", 2u, true, {}, {},
                  {pathlearn::StatusCode::kNoPath, "无路径"},
                  {0.0, 0.0, 0.0, 0.0, 0.0},
                  0});

  const pathlearn::EvaluationSummary summary =
      pathlearn::SummarizeRuns(runs);

  EXPECT_EQ(summary.total_runs, 2);
  EXPECT_EQ(summary.success_runs, 0);
  EXPECT_EQ(summary.no_path_runs, 2);
  EXPECT_DOUBLE_EQ(summary.success_rate, 0.0);
  EXPECT_DOUBLE_EQ(summary.collision_rate, 0.0);
  EXPECT_DOUBLE_EQ(summary.avg_exec_time_sec, 0.0);
  EXPECT_DOUBLE_EQ(summary.avg_path_length, 0.0);
  EXPECT_DOUBLE_EQ(summary.avg_min_safety_distance, 0.0);
  EXPECT_DOUBLE_EQ(summary.avg_first_solution_iteration, 0.0);
}

}  // namespace
