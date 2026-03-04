#pragma once

#include <unordered_map>
#include <vector>

#include "core/dynamic_obstacle.hpp"

namespace pathlearn {

// 规则模型：支持直线/往返两类规则
class RuleBasedDynamicObstacle final : public DynamicObstacleModel {
 public:
  RuleBasedDynamicObstacle() = default;
  ~RuleBasedDynamicObstacle() override = default;

  Status Reset(const std::vector<DynamicObstacleState>& initial_states) override;
  Status Update(TimeSec current_time_sec) override;
  Status Predict(
      TimeSec start_time_sec,
      TimeSec horizon_sec,
      TimeSec time_step_sec,
      DynamicObstaclePrediction* prediction) const override;
  Status GetStates(std::vector<DynamicObstacleState>* states) const override;

  // 设置规则参数：直线规则仅使用方向单位向量；往返规则使用端点
  Status SetRuleLine(
      const std::string& id,
      const Vector2D& direction_unit);
  Status SetRuleBounce(
      const std::string& id,
      const Pose2D& start,
      const Pose2D& end);

 private:
  struct LineRule {
    Vector2D direction_unit{};
  };

  struct BounceRule {
    Pose2D start{};
    Pose2D end{};
    bool forward = true;
  };

  std::unordered_map<std::string, LineRule> line_rules_{};
  std::unordered_map<std::string, BounceRule> bounce_rules_{};
  std::unordered_map<std::string, DynamicObstacleState> states_{};
  bool has_states_ = false;
  TimeSec last_update_time_sec_ = 0.0;
};

}  // namespace pathlearn
