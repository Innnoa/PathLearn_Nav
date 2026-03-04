#include "impl/simple_astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

#include "core/environment.hpp"
#include "impl/nn_sampler.hpp"

namespace pathlearn {
namespace {

struct GridNode {
  double g = std::numeric_limits<double>::infinity();
  double f = std::numeric_limits<double>::infinity();
  int parent = -1;
  double time_sec = 0.0;
  bool closed = false;
};

struct OpenItem {
  double f = std::numeric_limits<double>::infinity();
  double g = std::numeric_limits<double>::infinity();
  int index = -1;
  int serial = 0;
};

struct OpenItemLess {
  bool operator()(const OpenItem& lhs, const OpenItem& rhs) const {
    if (lhs.f != rhs.f) {
      return lhs.f > rhs.f;
    }
    if (lhs.g != rhs.g) {
      return lhs.g > rhs.g;
    }
    return lhs.serial > rhs.serial;
  }
};

double Distance2D(const Pose2D& a, const Pose2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double Dot2D(double ax, double ay, double bx, double by) {
  return ax * bx + ay * by;
}

double Norm2D(double x, double y) {
  return std::sqrt(x * x + y * y);
}

bool BuildSegmentTrajectory(
    const Pose2D& from,
    const Pose2D& to,
    double from_time,
    double to_time,
    Trajectory* trajectory) {
  if (!trajectory) {
    return false;
  }
  const double dt = to_time - from_time;
  if (dt <= 1e-9) {
    return false;
  }
  const Vector2D velocity{
      (to.x - from.x) / dt,
      (to.y - from.y) / dt,
  };
  Trajectory segment{};
  segment.points.resize(2);
  segment.points[0].state.pose = from;
  segment.points[0].state.pose.theta = std::atan2(velocity.y, velocity.x);
  segment.points[0].state.velocity = velocity;
  segment.points[0].state.acceleration = {0.0, 0.0};
  segment.points[0].state.time_sec = from_time;
  segment.points[1].state.pose = to;
  segment.points[1].state.pose.theta = std::atan2(velocity.y, velocity.x);
  segment.points[1].state.velocity = velocity;
  segment.points[1].state.acceleration = {0.0, 0.0};
  segment.points[1].state.time_sec = to_time;
  *trajectory = std::move(segment);
  return true;
}

}  // namespace

Status SimpleAStarPlanner::Plan(
    const PlannerRequest& request,
    PlannerResult* result) {
  if (!result) {
    return {StatusCode::kInvalidInput, "结果指针为空"};
  }
  result->status = {StatusCode::kNoPath, "未找到可行路径"};
  result->trajectory.points.clear();
  result->planning_time_ms = 0.0;
  result->iterations = 0;
  result->first_solution_iteration = 0;
  result->debug_edges.clear();

  if (!request.collision.environment) {
    return {StatusCode::kInvalidInput, "环境未设置"};
  }
  if (!request.collision_checker) {
    return {StatusCode::kInvalidInput, "碰撞检测器未设置"};
  }
  if (request.limits.max_speed <= 0.0) {
    return {StatusCode::kInvalidInput, "速度上限无效"};
  }
  if (request.horizon_sec <= 0.0) {
    return {StatusCode::kInvalidInput, "规划时间窗无效"};
  }

  const auto start_clock = std::chrono::steady_clock::now();
  const MapInfo map_info = request.collision.environment->GetMapInfo();
  const Bounds2D& bounds = map_info.bounds;
  const double span_x = bounds.max_x - bounds.min_x;
  const double span_y = bounds.max_y - bounds.min_y;
  if (span_x <= 0.0 || span_y <= 0.0) {
    return {StatusCode::kInvalidInput, "地图边界无效"};
  }

  double resolution = request.astar_grid_resolution;
  if (resolution <= 0.0) {
    resolution = std::clamp(
        request.limits.max_speed * std::max(request.time_step_sec, 0.1) * 0.7,
        0.25,
        2.0);
  }
  if (resolution <= 1e-6) {
    return {StatusCode::kInvalidInput, "A* 网格分辨率无效"};
  }

  const int width = static_cast<int>(std::floor(span_x / resolution)) + 1;
  const int height = static_cast<int>(std::floor(span_y / resolution)) + 1;
  if (width < 2 || height < 2) {
    return {StatusCode::kInvalidInput, "A* 网格尺寸过小"};
  }
  const size_t cell_count =
      static_cast<size_t>(width) * static_cast<size_t>(height);
  if (cell_count > 2000000u) {
    return {StatusCode::kInvalidInput, "A* 网格过密，建议增大 astar_resolution"};
  }

  auto ToIndex = [&](int gx, int gy) {
    return gy * width + gx;
  };
  auto GridToPose = [&](int gx, int gy) {
    Pose2D pose{};
    pose.x = bounds.min_x + static_cast<double>(gx) * resolution;
    pose.y = bounds.min_y + static_cast<double>(gy) * resolution;
    pose.theta = 0.0;
    return pose;
  };
  auto PoseToGrid = [&](const Pose2D& pose, int* gx, int* gy) {
    if (!gx || !gy) {
      return;
    }
    const double fx = (pose.x - bounds.min_x) / resolution;
    const double fy = (pose.y - bounds.min_y) / resolution;
    int x = static_cast<int>(std::llround(fx));
    int y = static_cast<int>(std::llround(fy));
    x = std::clamp(x, 0, width - 1);
    y = std::clamp(y, 0, height - 1);
    *gx = x;
    *gy = y;
  };

  bool start_collision = false;
  Status status = request.collision_checker->CheckState(
      request.collision,
      request.start,
      &start_collision);
  if (status.code != StatusCode::kOk) {
    return status;
  }
  if (start_collision) {
    return {StatusCode::kCollision, "起点碰撞"};
  }

  State2D goal_state{};
  goal_state.pose = request.goal;
  goal_state.time_sec = request.start.time_sec;
  bool goal_collision = false;
  status = request.collision_checker->CheckState(
      request.collision,
      goal_state,
      &goal_collision);
  if (status.code != StatusCode::kOk) {
    return status;
  }
  if (goal_collision) {
    return {StatusCode::kCollision, "终点碰撞"};
  }

  int start_gx = 0;
  int start_gy = 0;
  int goal_gx = 0;
  int goal_gy = 0;
  PoseToGrid(request.start.pose, &start_gx, &start_gy);
  PoseToGrid(request.goal, &goal_gx, &goal_gy);
  const int start_index = ToIndex(start_gx, start_gy);
  const int goal_index = ToIndex(goal_gx, goal_gy);

  std::vector<uint8_t> blocked(cell_count, 0);
  std::vector<double> clearance(cell_count, 0.0);
  for (int gy = 0; gy < height; ++gy) {
    for (int gx = 0; gx < width; ++gx) {
      const int index = ToIndex(gx, gy);
      State2D state{};
      state.pose = GridToPose(gx, gy);
      state.time_sec = request.start.time_sec;
      bool hit = false;
      status = request.collision_checker->CheckState(
          request.collision,
          state,
          &hit);
      if (status.code != StatusCode::kOk) {
        return status;
      }
      if (hit) {
        blocked[static_cast<size_t>(index)] = 1;
        continue;
      }
      double static_clearance = 0.0;
      status = request.collision.environment->DistanceToNearestObstacle(
          state.pose,
          &static_clearance);
      if (status.code != StatusCode::kOk) {
        return status;
      }
      clearance[static_cast<size_t>(index)] =
          std::isfinite(static_clearance) ? std::max(0.0, static_clearance)
                                          : 0.0;
    }
  }
  blocked[static_cast<size_t>(start_index)] = 0;
  blocked[static_cast<size_t>(goal_index)] = 0;

  std::vector<CircleObstacle> static_obstacles;
  status = request.collision.environment->GetStaticObstacles(&static_obstacles);
  if (status.code != StatusCode::kOk) {
    return status;
  }

  NnSampler nn_sampler;
  bool nn_ready = false;
  if (request.nn_guidance_enabled) {
    status = nn_sampler.LoadModel(request.nn_model_path);
    if (status.code == StatusCode::kOk) {
      nn_ready = true;
    } else if (request.debug_log_console) {
      std::cerr << "[A*] 学习启发模型加载失败，降级为欧式启发: "
                << status.message << std::endl;
    }
  }

  auto PoseForIndex = [&](int index) {
    if (index == start_index) {
      return request.start.pose;
    }
    if (index == goal_index) {
      return request.goal;
    }
    const int gx = index % width;
    const int gy = index / width;
    return GridToPose(gx, gy);
  };

  auto HeuristicCost = [&](const Pose2D& pose) {
    const double distance = Distance2D(pose, request.goal);
    const double base_cost =
        request.cost_weights.length_weight * distance +
        request.cost_weights.time_weight *
            (distance / std::max(request.limits.max_speed, 1e-6));
    const double exploration = std::clamp(
        request.cost_weights.exploration_weight,
        1.0,
        8.0);
    const double weighted_scale = std::clamp(
        request.astar_heuristic_weight * (1.0 + 0.06 * (exploration - 1.0)),
        0.6,
        3.0);
    double heuristic = base_cost * weighted_scale;

    if (nn_ready) {
      const double suggest_distance = request.nn_sample_distance > 0.0
          ? request.nn_sample_distance
          : std::max(2.0 * resolution, 0.5);
      const NnSampler::Result suggestion = nn_sampler.SuggestSample(
          pose,
          request.goal,
          static_obstacles,
          suggest_distance,
          bounds);
      if (suggestion.valid) {
        const double hint_x = suggestion.target.x - pose.x;
        const double hint_y = suggestion.target.y - pose.y;
        const double goal_x = request.goal.x - pose.x;
        const double goal_y = request.goal.y - pose.y;
        const double hint_norm = Norm2D(hint_x, hint_y);
        const double goal_norm = Norm2D(goal_x, goal_y);
        if (hint_norm > 1e-9 && goal_norm > 1e-9) {
          const double align =
              Dot2D(hint_x, hint_y, goal_x, goal_y) /
              (hint_norm * goal_norm);
          const double strength =
              std::clamp(request.nn_sample_probability, 0.0, 1.0);
          // 方向越一致，启发越“激进”，以减少扩展节点。
          const double factor =
              std::clamp(1.0 - 0.28 * strength * align, 0.65, 1.35);
          heuristic *= factor;
        }
      }
    }
    return std::max(0.0, heuristic);
  };

  auto PushDebugEdge = [&](const Pose2D& from, const Pose2D& to) {
    if (!request.enable_debug) {
      return;
    }
    const int limit = request.debug_edge_limit > 0 ? request.debug_edge_limit : 8000;
    if (limit > 0 &&
        static_cast<int>(result->debug_edges.size()) >= limit) {
      return;
    }
    result->debug_edges.push_back({from, to});
  };

  std::vector<GridNode> nodes(cell_count);
  std::priority_queue<OpenItem, std::vector<OpenItem>, OpenItemLess> open_set;
  int serial = 0;
  nodes[static_cast<size_t>(start_index)].g = 0.0;
  nodes[static_cast<size_t>(start_index)].time_sec = request.start.time_sec;
  nodes[static_cast<size_t>(start_index)].f = HeuristicCost(request.start.pose);
  open_set.push(
      {nodes[static_cast<size_t>(start_index)].f, 0.0, start_index, serial++});

  const int default_max_expansions = std::clamp(
      static_cast<int>(cell_count * 4),
      2000,
      1500000);
  const int max_expansions = request.max_iterations_override > 0
      ? std::clamp(request.max_iterations_override, 500, 3000000)
      : default_max_expansions;

  const bool allow_diagonal = request.astar_allow_diagonal;
  const int neighbor_dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int neighbor_dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const int neighbor_count = allow_diagonal ? 8 : 4;

  const int log_every = request.debug_log_every > 0 ? request.debug_log_every : 5000;
  int expansions = 0;
  int found_index = -1;

  while (!open_set.empty() && expansions < max_expansions) {
    const OpenItem top = open_set.top();
    open_set.pop();
    GridNode& node = nodes[static_cast<size_t>(top.index)];
    if (node.closed) {
      continue;
    }
    if (top.g > node.g + 1e-9) {
      continue;
    }
    node.closed = true;
    expansions += 1;

    if (request.debug_log_console &&
        (expansions == 1 || expansions % log_every == 0)) {
      std::cout << "[A*] expand=" << expansions
                << " open=" << open_set.size()
                << " current_f=" << node.f
                << " current_g=" << node.g
                << std::endl;
    }

    if (top.index == goal_index) {
      found_index = top.index;
      break;
    }

    const int cx = top.index % width;
    const int cy = top.index / width;
    const Pose2D from_pose = PoseForIndex(top.index);
    const double from_time = node.time_sec;

    for (int dir = 0; dir < neighbor_count; ++dir) {
      const int nx = cx + neighbor_dx[dir];
      const int ny = cy + neighbor_dy[dir];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      const int next_index = ToIndex(nx, ny);
      GridNode& next_node = nodes[static_cast<size_t>(next_index)];
      if (next_node.closed) {
        continue;
      }
      if (blocked[static_cast<size_t>(next_index)] != 0 &&
          next_index != goal_index) {
        continue;
      }

      const Pose2D to_pose = PoseForIndex(next_index);
      const double step_distance = Distance2D(from_pose, to_pose);
      if (step_distance <= 1e-9) {
        continue;
      }
      const double step_time = step_distance / request.limits.max_speed;
      const double to_time = from_time + step_time;
      if (to_time - request.start.time_sec > request.horizon_sec + 1e-9) {
        continue;
      }

      Trajectory segment{};
      if (!BuildSegmentTrajectory(from_pose, to_pose, from_time, to_time, &segment)) {
        continue;
      }
      bool segment_collision = false;
      status = request.collision_checker->CheckTrajectory(
          request.collision,
          segment,
          &segment_collision);
      if (status.code != StatusCode::kOk) {
        return status;
      }
      if (segment_collision) {
        continue;
      }
      PushDebugEdge(from_pose, to_pose);

      double step_cost =
          request.cost_weights.length_weight * step_distance +
          request.cost_weights.time_weight * step_time;

      const double nearest_clearance = clearance[static_cast<size_t>(next_index)];
      step_cost += request.cost_weights.safety_weight *
          (0.04 / std::pow(nearest_clearance + 1e-3, 2.0));

      if (request.cost_weights.smoothness_weight > 0.0 &&
          node.parent >= 0) {
        const Pose2D prev_pose = PoseForIndex(node.parent);
        const double vx1 = from_pose.x - prev_pose.x;
        const double vy1 = from_pose.y - prev_pose.y;
        const double vx2 = to_pose.x - from_pose.x;
        const double vy2 = to_pose.y - from_pose.y;
        const double n1 = Norm2D(vx1, vy1);
        const double n2 = Norm2D(vx2, vy2);
        if (n1 > 1e-9 && n2 > 1e-9) {
          const double cos_angle = std::clamp(
              Dot2D(vx1, vy1, vx2, vy2) / (n1 * n2),
              -1.0,
              1.0);
          const double turn_angle = std::acos(cos_angle);
          step_cost += request.cost_weights.smoothness_weight * 0.1 * turn_angle;
        }
      }

      const double candidate_g = node.g + step_cost;
      if (candidate_g + 1e-9 >= next_node.g) {
        continue;
      }

      next_node.parent = top.index;
      next_node.g = candidate_g;
      next_node.time_sec = to_time;
      next_node.f = candidate_g + HeuristicCost(to_pose);
      open_set.push({next_node.f, next_node.g, next_index, serial++});
    }
  }

  result->iterations = expansions;
  if (found_index < 0) {
    result->status = {StatusCode::kNoPath, "A* 未找到可行路径"};
    const auto end_clock = std::chrono::steady_clock::now();
    result->planning_time_ms = std::chrono::duration<double, std::milli>(
                                   end_clock - start_clock)
                                   .count();
    return result->status;
  }

  std::vector<int> reversed_index_path;
  reversed_index_path.reserve(256);
  int cursor = found_index;
  while (cursor >= 0) {
    reversed_index_path.push_back(cursor);
    if (cursor == start_index) {
      break;
    }
    cursor = nodes[static_cast<size_t>(cursor)].parent;
  }
  if (reversed_index_path.empty() || reversed_index_path.back() != start_index) {
    return {StatusCode::kInternalError, "A* 路径回溯失败"};
  }
  std::reverse(reversed_index_path.begin(), reversed_index_path.end());

  Trajectory trajectory{};
  trajectory.points.reserve(reversed_index_path.size());
  for (size_t i = 0; i < reversed_index_path.size(); ++i) {
    const int index = reversed_index_path[i];
    TrajectoryPoint point{};
    point.state.pose = PoseForIndex(index);
    point.state.time_sec = nodes[static_cast<size_t>(index)].time_sec;
    point.state.velocity = {0.0, 0.0};
    point.state.acceleration = {0.0, 0.0};
    trajectory.points.push_back(point);
  }

  if (trajectory.points.size() == 1) {
    trajectory.points[0].state.time_sec = request.start.time_sec;
  } else {
    for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
      const Pose2D& from = trajectory.points[i].state.pose;
      const Pose2D& to = trajectory.points[i + 1].state.pose;
      double dt =
          trajectory.points[i + 1].state.time_sec - trajectory.points[i].state.time_sec;
      if (dt <= 1e-9) {
        dt = std::max(request.time_step_sec, 0.1);
        trajectory.points[i + 1].state.time_sec =
            trajectory.points[i].state.time_sec + dt;
      }
      Vector2D velocity{};
      velocity.x = (to.x - from.x) / dt;
      velocity.y = (to.y - from.y) / dt;
      const double heading = std::atan2(velocity.y, velocity.x);
      trajectory.points[i].state.velocity = velocity;
      trajectory.points[i].state.pose.theta = heading;
      trajectory.points[i + 1].state.velocity = velocity;
      trajectory.points[i + 1].state.pose.theta = heading;
    }
    trajectory.points.back().state.acceleration = {0.0, 0.0};
  }

  result->trajectory = std::move(trajectory);
  result->status = {StatusCode::kOk, ""};
  result->first_solution_iteration = expansions;
  const auto end_clock = std::chrono::steady_clock::now();
  result->planning_time_ms =
      std::chrono::duration<double, std::milli>(end_clock - start_clock).count();
  return result->status;
}

}  // namespace pathlearn
