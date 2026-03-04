#include "impl/simple_environment.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

namespace pathlearn {
namespace {

double SquaredDistance(double x1, double y1, double x2, double y2) {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return dx * dx + dy * dy;
}

double DistancePointToSegment(
    double px,
    double py,
    double x1,
    double y1,
    double x2,
    double y2) {
  const double vx = x2 - x1;
  const double vy = y2 - y1;
  const double wx = px - x1;
  const double wy = py - y1;
  const double len2 = vx * vx + vy * vy;
  if (len2 <= 1e-12) {
    return std::sqrt(SquaredDistance(px, py, x1, y1));
  }
  double t = (wx * vx + wy * vy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  const double cx = x1 + t * vx;
  const double cy = y1 + t * vy;
  return std::sqrt(SquaredDistance(px, py, cx, cy));
}

}  // namespace

Status SimpleEnvironment::LoadFromFile(const std::string& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return {StatusCode::kInvalidInput, "无法打开地图文件"};
  }

  MapInfo info{};
  std::vector<CircleObstacle> obstacles;
  bool has_name = false;
  bool has_bounds = false;

  std::string token;
  while (input >> token) {
    if (token == "name") {
      input >> info.name;
      has_name = true;
    } else if (token == "bounds") {
      input >> info.bounds.min_x >> info.bounds.max_x >> info.bounds.min_y >>
          info.bounds.max_y;
      has_bounds = true;
    } else if (token == "obstacles") {
      int count = 0;
      input >> count;
      if (count < 0) {
        return {StatusCode::kInvalidInput, "障碍物数量无效"};
      }
      for (int i = 0; i < count; ++i) {
        std::string type;
        input >> type;
        if (type != "circle") {
          return {StatusCode::kInvalidInput, "仅支持 circle 障碍物"};
        }
        CircleObstacle obstacle{};
        input >> obstacle.center.x >> obstacle.center.y >> obstacle.radius;
        obstacles.push_back(obstacle);
      }
    } else {
      return {StatusCode::kInvalidInput, "未知地图字段"};
    }
  }

  if (!has_name || !has_bounds) {
    return {StatusCode::kInvalidInput, "地图缺少 name 或 bounds"};
  }

  map_info_ = info;
  obstacles_ = std::move(obstacles);
  loaded_ = true;
  return {StatusCode::kOk, ""};
}

MapInfo SimpleEnvironment::GetMapInfo() const {
  return map_info_;
}

bool SimpleEnvironment::IsInsideBounds(const Pose2D& pose) const {
  const auto& b = map_info_.bounds;
  return pose.x >= b.min_x && pose.x <= b.max_x && pose.y >= b.min_y &&
      pose.y <= b.max_y;
}

Status SimpleEnvironment::IsOccupied(const Pose2D& pose, bool* occupied) const {
  if (!occupied) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!loaded_) {
    return {StatusCode::kNotReady, "环境未加载"};
  }
  if (!IsInsideBounds(pose)) {
    *occupied = true;
    return {StatusCode::kOk, ""};
  }

  for (const auto& obstacle : obstacles_) {
    const double dist2 = SquaredDistance(
        pose.x, pose.y, obstacle.center.x, obstacle.center.y);
    if (dist2 <= obstacle.radius * obstacle.radius) {
      *occupied = true;
      return {StatusCode::kOk, ""};
    }
  }

  *occupied = false;
  return {StatusCode::kOk, ""};
}

Status SimpleEnvironment::DistanceToNearestObstacle(
    const Pose2D& pose,
    double* distance) const {
  if (!distance) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!loaded_) {
    return {StatusCode::kNotReady, "环境未加载"};
  }
  if (!IsInsideBounds(pose)) {
    return {StatusCode::kInvalidInput, "位置超出边界"};
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto& obstacle : obstacles_) {
    const double dist = std::sqrt(SquaredDistance(
        pose.x, pose.y, obstacle.center.x, obstacle.center.y));
    const double clearance = std::max(0.0, dist - obstacle.radius);
    min_distance = std::min(min_distance, clearance);
  }

  const auto& b = map_info_.bounds;
  const double boundary_distance = std::min(
      std::min(pose.x - b.min_x, b.max_x - pose.x),
      std::min(pose.y - b.min_y, b.max_y - pose.y));
  min_distance = std::min(min_distance, boundary_distance);

  *distance = min_distance;
  return {StatusCode::kOk, ""};
}

Status SimpleEnvironment::GetStaticObstacles(
    std::vector<CircleObstacle>* obstacles) const {
  if (!obstacles) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!loaded_) {
    return {StatusCode::kNotReady, "环境未加载"};
  }
  *obstacles = obstacles_;
  return {StatusCode::kOk, ""};
}

Status SimpleEnvironment::CheckSegmentCollision(
    const Pose2D& start,
    const Pose2D& end,
    bool* collision) const {
  if (!collision) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!loaded_) {
    return {StatusCode::kNotReady, "环境未加载"};
  }

  if (!IsInsideBounds(start) || !IsInsideBounds(end)) {
    *collision = true;
    return {StatusCode::kOk, ""};
  }

  for (const auto& obstacle : obstacles_) {
    const double dist = DistancePointToSegment(
        obstacle.center.x,
        obstacle.center.y,
        start.x,
        start.y,
        end.x,
        end.y);
    if (dist <= obstacle.radius) {
      *collision = true;
      return {StatusCode::kOk, ""};
    }
  }

  *collision = false;
  return {StatusCode::kOk, ""};
}

}  // namespace pathlearn
