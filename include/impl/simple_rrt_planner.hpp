#pragma once

#include <random>

#include "core/planner.hpp"

namespace pathlearn {

// 简易采样规划器：基于 RRT 的连续空间规划
class SimpleRrtPlanner final : public Planner {
 public:
  SimpleRrtPlanner();
  ~SimpleRrtPlanner() override = default;

  Status Plan(const PlannerRequest& request, PlannerResult* result) override;

  // 设置随机种子用于可复现测试
  void SetRandomSeed(unsigned int seed);

 private:
  std::mt19937 rng_;
};

}  // namespace pathlearn
