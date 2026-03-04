#include <gtest/gtest.h>

#include "impl/cost_weight_learner.hpp"

namespace {

TEST(CostWeightLearnerTest, DisabledKeepsWeights) {
  pathlearn::CostWeightLearner learner;
  pathlearn::CostWeights weights{};
  weights.length_weight = 1.0;
  weights.safety_weight = 1.0;
  weights.time_weight = 1.0;
  weights.exploration_weight = 1.0;

  ASSERT_EQ(learner.Initialize(weights).code, pathlearn::StatusCode::kOk);
  learner.SetEnabled(false);

  pathlearn::LearningInput input{};
  input.current_weights = weights;
  input.metrics.collision_rate = 0.5;
  input.baseline_metrics.collision_rate = 0.0;

  pathlearn::LearningOutput output{};
  ASSERT_EQ(learner.Update(input, &output).code, pathlearn::StatusCode::kOk);
  EXPECT_DOUBLE_EQ(output.updated_weights.safety_weight, 1.0);
  EXPECT_DOUBLE_EQ(output.updated_weights.exploration_weight, 1.0);
}

TEST(CostWeightLearnerTest, IncreasesWeightsOnWorseMetrics) {
  pathlearn::CostWeightLearner learner;
  pathlearn::CostWeights weights{};
  weights.length_weight = 1.0;
  weights.safety_weight = 1.0;
  weights.time_weight = 1.0;
  weights.exploration_weight = 1.0;

  ASSERT_EQ(learner.Initialize(weights).code, pathlearn::StatusCode::kOk);

  pathlearn::LearningInput input{};
  input.current_weights = weights;
  input.metrics.avg_path_length = 12.0;
  input.baseline_metrics.avg_path_length = 10.0;
  input.metrics.collision_rate = 0.2;
  input.baseline_metrics.collision_rate = 0.0;
  input.metrics.min_safety_distance = 0.2;
  input.baseline_metrics.min_safety_distance = 0.5;
  input.metrics.avg_exec_time_sec = 2.0;
  input.baseline_metrics.avg_exec_time_sec = 1.0;
  input.metrics.success_rate = 0.0;
  input.baseline_metrics.success_rate = 1.0;

  pathlearn::LearningOutput output{};
  ASSERT_EQ(learner.Update(input, &output).code, pathlearn::StatusCode::kOk);
  EXPECT_GT(output.updated_weights.length_weight, weights.length_weight);
  EXPECT_GT(output.updated_weights.safety_weight, weights.safety_weight);
  EXPECT_GT(output.updated_weights.time_weight, weights.time_weight);
  EXPECT_GT(output.updated_weights.exploration_weight,
            weights.exploration_weight);
}

}  // namespace
