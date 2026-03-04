#pragma once

#include "core/types.hpp"

namespace pathlearn {

// 学习输入：基于评测指标与当前权重进行更新
struct LearningInput {
  EvaluationMetrics metrics{};
  CostWeights current_weights{};
  EvaluationMetrics baseline_metrics{};
};

// 学习输出：更新后的权重
struct LearningOutput {
  CostWeights updated_weights{};
};

// 学习模块接口：用于代价权重学习
class LearningModule {
 public:
  virtual ~LearningModule() = default;

  // 初始化学习模块
  virtual Status Initialize(const CostWeights& initial_weights) = 0;

  // 依据输入更新权重
  virtual Status Update(const LearningInput& input, LearningOutput* output) = 0;

  // 允许关闭学习用于对比基线
  virtual void SetEnabled(bool enabled) = 0;
};

}  // namespace pathlearn
