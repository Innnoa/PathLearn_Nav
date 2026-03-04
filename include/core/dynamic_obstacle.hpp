#pragma once

#include <vector>

#include "core/types.hpp"

namespace pathlearn {

// 动态障碍模型接口：规则模型可在实现中体现
class DynamicObstacleModel {
 public:
  virtual ~DynamicObstacleModel() = default;

  // 重置动态障碍初始状态
  virtual Status Reset(const std::vector<DynamicObstacleState>& initial_states) = 0;

  // 基于规则更新内部状态
  virtual Status Update(TimeSec current_time_sec) = 0;

  // 预测未来障碍轨迹
  virtual Status Predict(
      TimeSec start_time_sec,
      TimeSec horizon_sec,
      TimeSec time_step_sec,
      DynamicObstaclePrediction* prediction) const = 0;

  // 获取当前障碍状态快照（用于可视化/仿真）
  virtual Status GetStates(std::vector<DynamicObstacleState>* states) const = 0;
};

}  // namespace pathlearn
