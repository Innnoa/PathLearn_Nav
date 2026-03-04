#include "impl/simple_collision_checker.hpp"

#include "core/environment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

Status CheckDynamicState(
    const CollisionContext& context,
    const State2D& state,
    bool* collision) {
  if (!collision) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  *collision = false;

  if (!context.prediction) {
    return {StatusCode::kOk, ""};
  }
  if (context.prediction->time_step_sec <= 0.0) {
    return {StatusCode::kInvalidInput, "时间步长无效"};
  }

  const double dt = context.prediction->time_step_sec;
  const double start_time = context.prediction->start_time_sec;

  for (const auto& traj : context.prediction->trajectories) {
    if (traj.points.empty()) {
      continue;
    }
    const double offset = (state.time_sec - start_time) / dt;
    const int raw_index = static_cast<int>(std::floor(offset + 0.5));
    const int max_index = static_cast<int>(traj.points.size()) - 1;
    const int index = std::clamp(raw_index, 0, max_index);
    const auto& obs_state = traj.points[index].state;

    const double dist2 = SquaredDistance(
        state.pose.x, state.pose.y, obs_state.pose.x, obs_state.pose.y);
    const double radius = context.robot_radius + traj.shape.radius;
    if (dist2 <= radius * radius) {
      *collision = true;
      return {StatusCode::kOk, ""};
    }
  }

  return {StatusCode::kOk, ""};
}

}  // namespace

Status SimpleCollisionChecker::CheckState(
    const CollisionContext& context,
    const State2D& state,
    bool* collision) const {
  if (!collision) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!context.environment) {
    return {StatusCode::kInvalidInput, "环境未设置"};
  }

  bool occupied = false;
  Status status = context.environment->IsOccupied(state.pose, &occupied);
  if (status.code != StatusCode::kOk) {
    return status;
  }
  if (occupied) {
    *collision = true;
    return {StatusCode::kOk, ""};
  }

  if (context.robot_radius > 0.0) {
    double clearance = 0.0;
    status = context.environment->DistanceToNearestObstacle(state.pose,
                                                            &clearance);
    if (status.code != StatusCode::kOk) {
      return status;
    }
    // 机器人半径作为静态障碍膨胀距离，确保与障碍保持最小间隙。
    if (clearance <= context.robot_radius) {
      *collision = true;
      return {StatusCode::kOk, ""};
    }
  }

  return CheckDynamicState(context, state, collision);
}

Status SimpleCollisionChecker::CheckTrajectory(
    const CollisionContext& context,
    const Trajectory& trajectory,
    bool* collision) const {
  if (!collision) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!context.environment) {
    return {StatusCode::kInvalidInput, "环境未设置"};
  }

  std::vector<CircleObstacle> obstacles;
  Status status = context.environment->GetStaticObstacles(&obstacles);
  if (status.code != StatusCode::kOk) {
    return status;
  }

  // 先做线段级静态障碍检测，避免仅检测采样点导致穿墙。
  if (trajectory.points.size() >= 2) {
    for (size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto& p0 = trajectory.points[i - 1].state.pose;
      const auto& p1 = trajectory.points[i].state.pose;
      for (const auto& obstacle : obstacles) {
        const double dist = DistancePointToSegment(
            obstacle.center.x,
            obstacle.center.y,
            p0.x,
            p0.y,
            p1.x,
            p1.y);
        const double radius = obstacle.radius + context.robot_radius;
        if (dist <= radius) {
          *collision = true;
          return {StatusCode::kOk, ""};
        }
      }
    }
  }

  for (const auto& point : trajectory.points) {
    bool hit = false;
    status = CheckState(context, point.state, &hit);
    if (status.code != StatusCode::kOk) {
      return status;
    }
    if (hit) {
      *collision = true;
      return {StatusCode::kOk, ""};
    }
  }
  *collision = false;
  return {StatusCode::kOk, ""};
}

Status SimpleCollisionChecker::MinimumDistance(
    const CollisionContext& context,
    const Trajectory& trajectory,
    double* distance) const {
  if (!distance) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!context.environment) {
    return {StatusCode::kInvalidInput, "环境未设置"};
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto& point : trajectory.points) {
    double static_distance = 0.0;
    Status status =
        context.environment->DistanceToNearestObstacle(point.state.pose,
                                                       &static_distance);
    if (status.code != StatusCode::kOk) {
      return status;
    }
    min_distance = std::min(min_distance, static_distance);

    if (context.prediction && context.prediction->time_step_sec > 0.0) {
      for (const auto& traj : context.prediction->trajectories) {
        if (traj.points.empty()) {
          continue;
        }
        const double dt = context.prediction->time_step_sec;
        const double start_time = context.prediction->start_time_sec;
        const double offset = (point.state.time_sec - start_time) / dt;
        const int raw_index = static_cast<int>(std::floor(offset + 0.5));
        const int max_index = static_cast<int>(traj.points.size()) - 1;
        const int index = std::clamp(raw_index, 0, max_index);
        const auto& obs_state = traj.points[index].state;
        const double dist = std::sqrt(SquaredDistance(
            point.state.pose.x,
            point.state.pose.y,
            obs_state.pose.x,
            obs_state.pose.y));
        const double clearance =
            std::max(0.0, dist - context.robot_radius - traj.shape.radius);
        min_distance = std::min(min_distance, clearance);
      }
    }
  }

  *distance = min_distance;
  return {StatusCode::kOk, ""};
}

}  // namespace pathlearn
