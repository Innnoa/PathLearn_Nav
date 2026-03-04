#include "impl/simple_rrt_planner.hpp"

#include "core/environment.hpp"
#include "impl/nn_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <limits>
#include <vector>

namespace pathlearn {
namespace {

struct TreeNode {
  State2D state{};
  int parent = -1;
};

struct CandidateSolution {
  Trajectory trajectory;
  double cost = std::numeric_limits<double>::infinity();
  int iterations = 0;
};

const auto kProcessStartTime = std::chrono::steady_clock::now();

long long ProcessElapsedMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - kProcessStartTime)
      .count();
}

std::string FormatWallTimeNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
#if defined(_WIN32)
  localtime_s(&local_tm, &now_time);
#else
  localtime_r(&now_time, &local_tm);
#endif
  char buffer[32]{};
  if (std::strftime(
          buffer,
          sizeof(buffer),
          "%Y-%m-%dT%H:%M:%S",
          &local_tm) == 0) {
    return "unknown";
  }
  return std::string(buffer);
}

double Distance2D(const Pose2D& a, const Pose2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ComputeTrajectoryLength(const Trajectory& trajectory) {
  double length = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    length += Distance2D(trajectory.points[i - 1].state.pose,
                         trajectory.points[i].state.pose);
  }
  return length;
}

double ComputeTrajectorySmoothness(const Trajectory& trajectory) {
  if (trajectory.points.size() < 3) {
    return 0.0;
  }
  double smoothness = 0.0;
  for (size_t i = 2; i < trajectory.points.size(); ++i) {
    const auto& prev = trajectory.points[i - 2].state.pose;
    const auto& curr = trajectory.points[i - 1].state.pose;
    const auto& next = trajectory.points[i].state.pose;
    const double angle1 = std::atan2(curr.y - prev.y, curr.x - prev.x);
    const double angle2 = std::atan2(next.y - curr.y, next.x - curr.x);
    double delta = std::fabs(angle2 - angle1);
    if (delta > M_PI) {
      delta = 2.0 * M_PI - delta;
    }
    smoothness += delta;
  }
  return smoothness;
}

bool EvaluateTrajectoryCost(
    const PlannerRequest& request,
    const Trajectory& trajectory,
    double* out_cost) {
  if (!out_cost) {
    return false;
  }
  if (!request.collision_checker) {
    return false;
  }
  const double length = ComputeTrajectoryLength(trajectory);
  const double smoothness = ComputeTrajectorySmoothness(trajectory);
  const double total_time = trajectory.points.empty()
      ? 0.0
      : trajectory.points.back().state.time_sec -
          trajectory.points.front().state.time_sec;

  double min_clearance = 0.0;
  Status status = request.collision_checker->MinimumDistance(
      request.collision,
      trajectory,
      &min_clearance);
  if (status.code != StatusCode::kOk) {
    return false;
  }
  if (!std::isfinite(min_clearance)) {
    min_clearance = 0.0;
  }

  const double clearance = min_clearance + 1e-3;
  const double safety_cost = 1.0 / (clearance * clearance);
  const CostWeights& weights = request.cost_weights;
  *out_cost = weights.length_weight * length +
      weights.time_weight * total_time +
      weights.smoothness_weight * smoothness +
      weights.safety_weight * safety_cost;
  return true;
}

bool BuildLinearTrajectory(
    const State2D& start,
    const Pose2D& goal,
    TimeSec time_step_sec,
    double max_speed,
    TimeSec horizon_sec,
    Trajectory* trajectory) {
  if (!trajectory || time_step_sec <= 0.0 || max_speed <= 0.0) {
    return false;
  }
  const double distance = Distance2D(start.pose, goal);
  const int steps = static_cast<int>(std::ceil(distance / (max_speed * time_step_sec)));
  if (steps <= 0) {
    trajectory->points = {TrajectoryPoint{start}};
    return true;
  }
  const double total_time = steps * time_step_sec;
  if (total_time > horizon_sec) {
    return false;
  }

  Vector2D velocity{};
  velocity.x = (goal.x - start.pose.x) / total_time;
  velocity.y = (goal.y - start.pose.y) / total_time;

  trajectory->points.clear();
  trajectory->points.reserve(steps + 1);
  for (int i = 0; i <= steps; ++i) {
    const TimeSec t = start.time_sec + i * time_step_sec;
    TrajectoryPoint point{};
    point.state.pose.x = start.pose.x + velocity.x * (t - start.time_sec);
    point.state.pose.y = start.pose.y + velocity.y * (t - start.time_sec);
    point.state.pose.theta = 0.0;
    point.state.velocity = velocity;
    point.state.acceleration = {0.0, 0.0};
    point.state.time_sec = t;
    trajectory->points.push_back(point);
  }
  return true;
}

bool BuildAxisAlignedTrajectory(
    const State2D& start,
    const Pose2D& goal,
    TimeSec time_step_sec,
    double max_speed,
    TimeSec horizon_sec,
    Trajectory* trajectory) {
  if (!trajectory) {
    return false;
  }
  const Pose2D corners[2] = {
    {goal.x, start.pose.y, 0.0},
    {start.pose.x, goal.y, 0.0},
  };

  for (const auto& corner : corners) {
    Trajectory first;
    if (!BuildLinearTrajectory(start, corner, time_step_sec, max_speed, horizon_sec, &first)) {
      continue;
    }
    if (first.points.empty()) {
      continue;
    }
    const State2D corner_state = first.points.back().state;
    Trajectory second;
    if (!BuildLinearTrajectory(corner_state, goal, time_step_sec, max_speed, horizon_sec, &second)) {
      continue;
    }
    if (second.points.empty()) {
      continue;
    }
    const double total_time = second.points.back().state.time_sec - start.time_sec;
    if (total_time > horizon_sec) {
      continue;
    }

    trajectory->points = std::move(first.points);
    trajectory->points.insert(
        trajectory->points.end(),
        second.points.begin() + 1,
        second.points.end());
    return true;
  }

  return false;
}

Status CheckTrajectoryCollision(
    const PlannerRequest& request,
    const Trajectory& trajectory,
    bool* collision) {
  if (!request.collision_checker) {
    return {StatusCode::kInvalidInput, "未提供碰撞检测器"};
  }
  return request.collision_checker->CheckTrajectory(
      request.collision,
      trajectory,
      collision);
}

}  // namespace

SimpleRrtPlanner::SimpleRrtPlanner() : rng_(42u) {}

void SimpleRrtPlanner::SetRandomSeed(unsigned int seed) {
  rng_.seed(seed);
}

Status SimpleRrtPlanner::Plan(const PlannerRequest& request, PlannerResult* result) {
  if (!result) {
    return {StatusCode::kInvalidInput, "结果指针为空"};
  }
  if (!request.collision.environment) {
    return {StatusCode::kInvalidInput, "环境未设置"};
  }
  if (!request.collision_checker) {
    return {StatusCode::kInvalidInput, "碰撞检测器未设置"};
  }
  if (request.time_step_sec <= 0.0 || request.horizon_sec <= 0.0) {
    return {StatusCode::kInvalidInput, "时间参数无效"};
  }
  if (request.limits.max_speed <= 0.0) {
    return {StatusCode::kInvalidInput, "速度上限无效"};
  }

  result->debug_edges.clear();
  const bool enable_debug = request.enable_debug;
  const int debug_limit = request.debug_edge_limit > 0
      ? request.debug_edge_limit
      : 8000;
  auto PushDebugEdge = [&](const Pose2D& from, const Pose2D& to) {
    if (!enable_debug) {
      return;
    }
    if (debug_limit > 0 &&
        static_cast<int>(result->debug_edges.size()) >= debug_limit) {
      return;
    }
    result->debug_edges.push_back({from, to});
  };
  const bool log_console = request.debug_log_console;
  const int log_every = request.debug_log_every > 0 ? request.debug_log_every : 200;
  const int plan_id = request.debug_plan_id;
  int rejected_max_steps = 0;
  int rejected_collision = 0;
  int accepted_edges = 0;

  bool occupied = false;
  Status status = request.collision.environment->IsOccupied(
      request.start.pose,
      &occupied);
  if (status.code != StatusCode::kOk) {
    return status;
  }
  if (occupied) {
    return {StatusCode::kCollision, "起点碰撞"};
  }

  status = request.collision.environment->IsOccupied(request.goal, &occupied);
  if (status.code != StatusCode::kOk) {
    return status;
  }
  if (occupied) {
    return {StatusCode::kCollision, "终点碰撞"};
  }

  CandidateSolution best_solution{};
  int solutions_found = 0;
  int first_solution_iter = -1;
  int max_solutions = 3;
  int max_iterations_after_goal = 3000;

  Trajectory straight;
  if (BuildLinearTrajectory(request.start,
                            request.goal,
                            request.time_step_sec,
                            request.limits.max_speed,
                            request.horizon_sec,
                            &straight)) {
    bool collision = false;
    status = CheckTrajectoryCollision(request, straight, &collision);
    if (status.code != StatusCode::kOk) {
      return status;
    }
    if (!collision) {
      double cost = 0.0;
      if (EvaluateTrajectoryCost(request, straight, &cost)) {
        best_solution.trajectory = straight;
        best_solution.cost = cost;
        best_solution.iterations = 1;
        solutions_found = 1;
        first_solution_iter = 0;
      }
    }
  }

  Trajectory axis_aligned;
  if (BuildAxisAlignedTrajectory(request.start,
                                 request.goal,
                                 request.time_step_sec,
                                 request.limits.max_speed,
                                 request.horizon_sec,
                                 &axis_aligned)) {
    bool collision = false;
    status = CheckTrajectoryCollision(request, axis_aligned, &collision);
    if (status.code != StatusCode::kOk) {
      return status;
    }
    if (!collision) {
      double cost = 0.0;
      if (EvaluateTrajectoryCost(request, axis_aligned, &cost)) {
        if (cost < best_solution.cost) {
          best_solution.trajectory = axis_aligned;
          best_solution.cost = cost;
          best_solution.iterations = 1;
        }
        solutions_found = std::max(solutions_found, 1);
        if (first_solution_iter < 0) {
          first_solution_iter = 0;
        }
      }
    }
  }

  const MapInfo map_info = request.collision.environment->GetMapInfo();
  const double step_distance = request.limits.max_speed * request.time_step_sec;
  const int max_steps = static_cast<int>(std::floor(request.horizon_sec / request.time_step_sec));
  const double exploration =
      std::clamp(request.cost_weights.exploration_weight, 1.0, 8.0);
  const int base_iterations = 30000;
  const double base_goal_bias = 0.22;
  const int base_solutions = 2;
  const int base_iterations_after_goal = 3000;
  const int max_iterations = std::clamp(
      static_cast<int>(std::round(base_iterations * exploration)),
      6000,
      300000);
  int effective_max_iterations = max_iterations;
  if (request.max_iterations_override > 0) {
    effective_max_iterations = std::clamp(
        request.max_iterations_override,
        1000,
        2000000);
  }
  const double goal_bias = std::clamp(
      base_goal_bias + 0.05 * (exploration - 1.0),
      0.1,
      0.6);
  max_solutions = std::clamp(
      static_cast<int>(std::round(base_solutions * exploration)),
      base_solutions,
      10);
  max_iterations_after_goal = std::clamp(
      static_cast<int>(std::round(base_iterations_after_goal * exploration)),
      base_iterations_after_goal,
      30000);
  double corridor_bias = std::clamp(0.32 + 0.06 * (exploration - 1.0), 0.18, 0.6);
  // 保证一定比例的均匀采样，避免走廊偏置在狭窄通道下失效。
  const double min_uniform_prob = 0.3;
  const double max_biased_prob = 1.0 - min_uniform_prob;
  if (goal_bias + corridor_bias > max_biased_prob) {
    corridor_bias = std::max(0.0, max_biased_prob - goal_bias);
  }
  const double corridor_half_width =
      std::clamp(2.5 * std::sqrt(exploration), 1.5, 8.0);

  const double corridor_length = Distance2D(request.start.pose, request.goal);
  const bool corridor_enabled = corridor_length > 1e-6;
  const double corridor_ux = corridor_enabled
      ? (request.goal.x - request.start.pose.x) / corridor_length
      : 0.0;
  const double corridor_uy = corridor_enabled
      ? (request.goal.y - request.start.pose.y) / corridor_length
      : 0.0;
  const double corridor_px = -corridor_uy;
  const double corridor_py = corridor_ux;
  const double axis_length = std::abs(request.goal.x - request.start.pose.x) +
      std::abs(request.goal.y - request.start.pose.y);
  const bool axis_enabled = axis_length > 1e-6;

  std::uniform_real_distribution<double> dist_x(map_info.bounds.min_x, map_info.bounds.max_x);
  std::uniform_real_distribution<double> dist_y(map_info.bounds.min_y, map_info.bounds.max_y);
  std::uniform_real_distribution<double> dist_prob(0.0, 1.0);
  std::uniform_real_distribution<double> dist_t(0.0, corridor_length);
  std::uniform_real_distribution<double> dist_axis_t(0.0, axis_length);
  std::uniform_real_distribution<double> dist_offset(-corridor_half_width, corridor_half_width);

  std::vector<TreeNode> tree;
  tree.push_back({request.start, -1});

  std::vector<CircleObstacle> static_obstacles;
  bool nn_active = false;
  NnSampler nn_sampler;
  if (request.nn_guidance_enabled && !request.nn_model_path.empty() &&
      request.nn_sample_probability > 0.0) {
    Status load_status = nn_sampler.LoadModel(request.nn_model_path);
    if (load_status.code == StatusCode::kOk) {
      Status obs_status = request.collision.environment->GetStaticObstacles(
          &static_obstacles);
      if (obs_status.code == StatusCode::kOk) {
        nn_active = true;
      }
    }
  }
  const double nn_prob = std::clamp(request.nn_sample_probability, 0.0, 1.0);
  const double nn_distance = request.nn_sample_distance > 1e-6
      ? request.nn_sample_distance
      : std::max(step_distance * 2.0, 0.5);

  for (int iter = 0; iter < effective_max_iterations; ++iter) {
    Pose2D sample{};
    const char* sample_tag = "uniform";
    const double pick = dist_prob(rng_);
    const bool pick_nn = nn_active && pick < nn_prob;
    if (pick_nn) {
      NnSampler::Result nn_result = nn_sampler.SuggestSample(
          request.start.pose,
          request.goal,
          static_obstacles,
          nn_distance,
          map_info.bounds);
      if (nn_result.valid) {
        sample = nn_result.target;
        sample_tag = "nn";
      } else {
        sample.x = dist_x(rng_);
        sample.y = dist_y(rng_);
      }
    } else if (pick < goal_bias) {
      sample = request.goal;
      sample_tag = "goal";
    } else if (corridor_enabled && pick < goal_bias + corridor_bias) {
      sample_tag = "corridor";
      if (axis_enabled && dist_prob(rng_) < 0.5) {
        const double t = dist_axis_t(rng_);
        const double offset = dist_offset(rng_);
        const Pose2D corner1{request.goal.x, request.start.pose.y, 0.0};
        const Pose2D corner2{request.start.pose.x, request.goal.y, 0.0};
        const bool use_first = dist_prob(rng_) < 0.5;
        const Pose2D a = request.start.pose;
        const Pose2D b = use_first ? corner1 : corner2;
        const Pose2D c = request.goal;
        const double seg1 = Distance2D(a, b);
        const double seg2 = Distance2D(b, c);
        if (seg1 > 1e-6 && t <= seg1) {
          const double ratio = t / seg1;
          sample.x = a.x + (b.x - a.x) * ratio;
          sample.y = a.y + (b.y - a.y) * ratio;
          if (std::abs(b.y - a.y) < 1e-6) {
            sample.y += offset;
          } else {
            sample.x += offset;
          }
        } else if (seg2 > 1e-6) {
          const double t2 = std::max(0.0, t - seg1);
          const double ratio = std::min(1.0, t2 / seg2);
          sample.x = b.x + (c.x - b.x) * ratio;
          sample.y = b.y + (c.y - b.y) * ratio;
          if (std::abs(c.y - b.y) < 1e-6) {
            sample.y += offset;
          } else {
            sample.x += offset;
          }
        } else {
          sample = request.goal;
        }
      } else {
        const double t = dist_t(rng_);
        const double offset = dist_offset(rng_);
        sample.x = request.start.pose.x + corridor_ux * t + corridor_px * offset;
        sample.y = request.start.pose.y + corridor_uy * t + corridor_py * offset;
      }
      sample.x = std::clamp(sample.x, map_info.bounds.min_x, map_info.bounds.max_x);
      sample.y = std::clamp(sample.y, map_info.bounds.min_y, map_info.bounds.max_y);
    } else {
      sample.x = dist_x(rng_);
      sample.y = dist_y(rng_);
    }

    int nearest_index = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
      const double dist = Distance2D(tree[i].state.pose, sample);
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest_index = i;
      }
    }

    const State2D nearest_state = tree[nearest_index].state;
    const int current_step = static_cast<int>(
        std::round(nearest_state.time_sec / request.time_step_sec));
    if (current_step + 1 > max_steps) {
      rejected_max_steps += 1;
      if (log_console && (iter % log_every == 0)) {
        std::cout << "[RRT p" << plan_id << "] iter " << iter
                  << " sample=" << sample_tag
                  << " tree=" << tree.size()
                  << " accept=" << accepted_edges
                  << " rej_step=" << rejected_max_steps
                  << " rej_col=" << rejected_collision
                  << " solutions=" << solutions_found
                  << " elapsed_total_ms=" << ProcessElapsedMs()
                  << " wall_time=" << FormatWallTimeNow()
                  << std::endl;
      }
      continue;
    }

    Pose2D new_pose = nearest_state.pose;
    const double dist_to_sample = Distance2D(nearest_state.pose, sample);
    if (dist_to_sample > 1e-6) {
      const double step = std::min(step_distance, dist_to_sample);
      const double dx = (sample.x - nearest_state.pose.x) / dist_to_sample;
      const double dy = (sample.y - nearest_state.pose.y) / dist_to_sample;
      new_pose.x += dx * step;
      new_pose.y += dy * step;
    }

    State2D new_state{};
    new_state.pose = new_pose;
    new_state.time_sec = nearest_state.time_sec + request.time_step_sec;
    new_state.velocity = {0.0, 0.0};
    new_state.acceleration = {0.0, 0.0};

    Trajectory edge{};
    edge.points = {TrajectoryPoint{nearest_state}, TrajectoryPoint{new_state}};
    bool collision = false;
    status = CheckTrajectoryCollision(request, edge, &collision);
    if (status.code != StatusCode::kOk || collision) {
      rejected_collision += 1;
      if (log_console && (iter % log_every == 0)) {
        std::cout << "[RRT p" << plan_id << "] iter " << iter
                  << " sample=" << sample_tag
                  << " tree=" << tree.size()
                  << " accept=" << accepted_edges
                  << " rej_step=" << rejected_max_steps
                  << " rej_col=" << rejected_collision
                  << " solutions=" << solutions_found
                  << " elapsed_total_ms=" << ProcessElapsedMs()
                  << " wall_time=" << FormatWallTimeNow()
                  << std::endl;
      }
      continue;
    }

    tree.push_back({new_state, nearest_index});
    const int new_index = static_cast<int>(tree.size()) - 1;
    accepted_edges += 1;
    PushDebugEdge(nearest_state.pose, new_state.pose);

    if (log_console && (iter % log_every == 0)) {
      std::cout << "[RRT p" << plan_id << "] iter " << iter
                << " sample=" << sample_tag
                << " tree=" << tree.size()
                << " accept=" << accepted_edges
                << " rej_step=" << rejected_max_steps
                << " rej_col=" << rejected_collision
                << " solutions=" << solutions_found
                << " elapsed_total_ms=" << ProcessElapsedMs()
                << " wall_time=" << FormatWallTimeNow()
                << std::endl;
    }

    if (Distance2D(new_state.pose, request.goal) <= step_distance) {
      State2D goal_state{};
      goal_state.pose = request.goal;
      goal_state.time_sec = new_state.time_sec + request.time_step_sec;
      Trajectory connect_edge{};
      connect_edge.points = {TrajectoryPoint{new_state}, TrajectoryPoint{goal_state}};
      bool goal_collision = false;
      status = CheckTrajectoryCollision(request, connect_edge, &goal_collision);
      if (status.code == StatusCode::kOk && !goal_collision) {
        tree.push_back({goal_state, new_index});
        const int goal_index = static_cast<int>(tree.size()) - 1;
        PushDebugEdge(new_state.pose, goal_state.pose);

        std::vector<TrajectoryPoint> points;
        for (int idx = goal_index; idx >= 0; idx = tree[idx].parent) {
          points.push_back(TrajectoryPoint{tree[idx].state});
          if (tree[idx].parent < 0) {
            break;
          }
        }
        std::reverse(points.begin(), points.end());
        Trajectory candidate{};
        candidate.points = std::move(points);
        double cost = 0.0;
        if (EvaluateTrajectoryCost(request, candidate, &cost)) {
          if (cost < best_solution.cost) {
            best_solution.trajectory = std::move(candidate);
            best_solution.cost = cost;
            best_solution.iterations = iter + 1;
          }
          solutions_found += 1;
          if (log_console) {
            std::cout << "[RRT p" << plan_id << "] found solution"
                      << " iter=" << iter
                      << " cost=" << best_solution.cost
                      << " solutions=" << solutions_found
                      << " elapsed_total_ms=" << ProcessElapsedMs()
                      << " wall_time=" << FormatWallTimeNow()
                      << std::endl;
          }
          if (first_solution_iter < 0) {
            first_solution_iter = iter;
          }
          if (solutions_found >= max_solutions) {
            break;
          }
        }
      }
    }

    if (first_solution_iter >= 0 &&
        iter - first_solution_iter >= max_iterations_after_goal) {
      break;
    }
  }

  if (!best_solution.trajectory.points.empty()) {
    result->status = {StatusCode::kOk, ""};
    result->trajectory = std::move(best_solution.trajectory);
    result->iterations = best_solution.iterations;
    result->first_solution_iteration =
        first_solution_iter >= 0 ? (first_solution_iter + 1) : 0;
    result->planning_time_ms = 0.0;
    return result->status;
  }

  result->status = {StatusCode::kNoPath, "未找到可行路径"};
  result->trajectory.points.clear();
  result->first_solution_iteration = 0;
  return result->status;
}

}  // namespace pathlearn
