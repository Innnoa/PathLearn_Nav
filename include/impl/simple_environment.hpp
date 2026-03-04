#pragma once

#include <vector>

#include "core/environment.hpp"

namespace pathlearn {

// 简单连续空间环境：圆形障碍 + 轴对齐边界
class SimpleEnvironment final : public Environment {
 public:
  SimpleEnvironment() = default;
  ~SimpleEnvironment() override = default;

  Status LoadFromFile(const std::string& path) override;
  MapInfo GetMapInfo() const override;
  bool IsInsideBounds(const Pose2D& pose) const override;
  Status IsOccupied(const Pose2D& pose, bool* occupied) const override;
  Status DistanceToNearestObstacle(const Pose2D& pose, double* distance) const override;
  Status GetStaticObstacles(std::vector<CircleObstacle>* obstacles) const override;
  Status CheckSegmentCollision(
      const Pose2D& start,
      const Pose2D& end,
      bool* collision) const override;

 private:
  bool loaded_ = false;
  MapInfo map_info_{};
  std::vector<CircleObstacle> obstacles_{};
};

}  // namespace pathlearn
