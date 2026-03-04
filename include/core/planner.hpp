#pragma once

#include <string>
#include <vector>

#include "core/collision.hpp"

namespace pathlearn {

// 规划请求
struct PlannerRequest {
  State2D start{};
  Pose2D goal{};
  Limits2D limits{};
  TimeSec horizon_sec = 0.0;
  TimeSec time_step_sec = 0.0;
  CollisionContext collision{};
  const CollisionChecker* collision_checker = nullptr;
  CostWeights cost_weights{};
  bool enable_debug = false;
  int debug_edge_limit = 0;
  bool debug_log_console = false;
  int debug_log_every = 0;
  int debug_plan_id = 0;
  int max_iterations_override = 0;
  bool nn_guidance_enabled = false;
  std::string nn_model_path;
  double nn_sample_probability = 0.0;
  double nn_sample_distance = 0.0;
  // A* 网格分辨率（<=0 表示内部自动估算）
  double astar_grid_resolution = 0.0;
  // A* 邻接方式：true=8邻接，false=4邻接
  bool astar_allow_diagonal = true;
  // A* 启发权重（>0）
  double astar_heuristic_weight = 1.0;
};

// 规划结果
struct PlannerResult {
  Status status{};
  Trajectory trajectory{};
  double planning_time_ms = 0.0;
  int iterations = 0;
  int first_solution_iteration = 0;
  std::vector<LineSegment2D> debug_edges;
};

// 规划器接口：支持可替换实现
class Planner {
 public:
  virtual ~Planner() = default;

  // 生成带时间维度的可执行轨迹
  virtual Status Plan(const PlannerRequest& request, PlannerResult* result) = 0;
};

}  // namespace pathlearn
