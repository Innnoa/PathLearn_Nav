#pragma once

#include "core/types.hpp"

namespace pathlearn {

// 碰撞检测接口：结合静态环境与动态障碍预测进行查询
class CollisionChecker {
 public:
  virtual ~CollisionChecker() = default;

  // 状态是否碰撞（可支持仅静态或静态+动态）
  virtual Status CheckState(
      const CollisionContext& context,
      const State2D& state,
      bool* collision) const = 0;

  // 轨迹是否碰撞（可支持仅静态或静态+动态）
  virtual Status CheckTrajectory(
      const CollisionContext& context,
      const Trajectory& trajectory,
      bool* collision) const = 0;

  // 轨迹最小安全距离
  virtual Status MinimumDistance(
      const CollisionContext& context,
      const Trajectory& trajectory,
      double* distance) const = 0;
};

}  // namespace pathlearn
