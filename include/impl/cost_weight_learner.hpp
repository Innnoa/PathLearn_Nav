#pragma once

#include "core/learning.hpp"

namespace pathlearn {

// 代价权重学习器：根据指标趋势调整权重
class CostWeightLearner final : public LearningModule {
 public:
  CostWeightLearner() = default;
  ~CostWeightLearner() override = default;

  Status Initialize(const CostWeights& initial_weights) override;
  Status Update(const LearningInput& input, LearningOutput* output) override;
  void SetEnabled(bool enabled) override;

 private:
  bool enabled_ = true;
  CostWeights weights_{};
};

}  // namespace pathlearn
