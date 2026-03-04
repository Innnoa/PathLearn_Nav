#pragma once

#include "core/collision.hpp"

namespace pathlearn {

// 简单碰撞检测器：基于离散时间点进行检查
class SimpleCollisionChecker final : public CollisionChecker {
 public:
  SimpleCollisionChecker() = default;
  ~SimpleCollisionChecker() override = default;

  Status CheckState(
      const CollisionContext& context,
      const State2D& state,
      bool* collision) const override;

  Status CheckTrajectory(
      const CollisionContext& context,
      const Trajectory& trajectory,
      bool* collision) const override;

  Status MinimumDistance(
      const CollisionContext& context,
      const Trajectory& trajectory,
      double* distance) const override;
};

}  // namespace pathlearn
