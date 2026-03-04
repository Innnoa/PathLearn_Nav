#pragma once

#include "core/planner.hpp"

namespace pathlearn {

// 简易 A* 规划器：在连续边界上构建规则网格并执行 A* 搜索
// 支持使用轻量神经网络输出对启发函数进行方向性修正。
class SimpleAStarPlanner final : public Planner {
 public:
  SimpleAStarPlanner() = default;
  ~SimpleAStarPlanner() override = default;

  Status Plan(const PlannerRequest& request, PlannerResult* result) override;
};

}  // namespace pathlearn
