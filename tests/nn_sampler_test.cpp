#include <gtest/gtest.h>

#include "impl/nn_sampler.hpp"

namespace {

TEST(NnSamplerTest, LoadAndSuggestFromDefaultModel) {
  pathlearn::NnSampler sampler;
  const std::string model_path =
      std::string(PATHLEARN_TEST_DATA_DIR) + "/../../data/models/nn_sampler_default.txt";
  ASSERT_EQ(sampler.LoadModel(model_path).code,
            pathlearn::StatusCode::kOk);

  std::vector<pathlearn::CircleObstacle> obstacles;
  obstacles.push_back({{0.5, 0.0, 0.0}, 0.2});
  obstacles.push_back({{0.0, 0.5, 0.0}, 0.2});

  pathlearn::Bounds2D bounds{-1.0, 1.0, -1.0, 1.0};
  pathlearn::Pose2D current{0.0, 0.0, 0.0};
  pathlearn::Pose2D goal{1.0, 1.0, 0.0};

  const auto suggestion = sampler.SuggestSample(
      current,
      goal,
      obstacles,
      0.5,
      bounds);
  ASSERT_TRUE(suggestion.valid);
  EXPECT_GE(suggestion.target.x, bounds.min_x);
  EXPECT_LE(suggestion.target.x, bounds.max_x);
  EXPECT_GE(suggestion.target.y, bounds.min_y);
  EXPECT_LE(suggestion.target.y, bounds.max_y);
}

TEST(NnSamplerTest, InvalidModelPathFails) {
  pathlearn::NnSampler sampler;
  EXPECT_NE(sampler.LoadModel("data/models/not_exists.txt").code,
            pathlearn::StatusCode::kOk);
  EXPECT_FALSE(sampler.IsReady());
}

}  // namespace
