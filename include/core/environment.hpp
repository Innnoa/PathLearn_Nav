#pragma once

#include <string>
#include <vector>

#include "core/types.hpp"

namespace pathlearn {

// 环境接口：只负责静态地图与障碍查询
class Environment {
 public:
  virtual ~Environment() = default;

  // 从文件加载环境（返回状态用于错误说明）
  virtual Status LoadFromFile(const std::string& path) = 0;

  // 获取地图信息
  virtual MapInfo GetMapInfo() const = 0;

  // 是否在地图边界内
  virtual bool IsInsideBounds(const Pose2D& pose) const = 0;

  // 查询当前位置是否与静态障碍碰撞
  virtual Status IsOccupied(const Pose2D& pose, bool* occupied) const = 0;

  // 查询到最近静态障碍的距离
  virtual Status DistanceToNearestObstacle(const Pose2D& pose, double* distance) const = 0;

  // 获取静态障碍列表（用于可视化）
  virtual Status GetStaticObstacles(std::vector<CircleObstacle>* obstacles) const = 0;

  // 线段与静态障碍是否相交
  virtual Status CheckSegmentCollision(
      const Pose2D& start,
      const Pose2D& end,
      bool* collision) const = 0;
};

}  // namespace pathlearn
