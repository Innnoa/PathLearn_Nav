#pragma once

#include <vector>

#include "core/collision.hpp"
#include "core/dynamic_obstacle.hpp"
#include "core/environment.hpp"
#include "core/learning.hpp"
#include "core/planner.hpp"
#include "core/visualization.hpp"

namespace pathlearn {

// 仿真参数
struct SimulationConfig {
  TimeSec horizon_sec = 0.0;
  TimeSec time_step_sec = 0.0;
  int max_steps = 0;
  int max_wait_steps = 0;
  double goal_tolerance = 0.0;
};

// 仿真输入
struct SimulationRequest {
  State2D start{};
  Pose2D goal{};
  Limits2D limits{};
  double robot_radius = 0.0;
  CostWeights cost_weights{};
  EvaluationMetrics baseline_metrics{};
  std::vector<DynamicObstacleState> dynamic_obstacles;
  bool show_planner_tree = false;
  int planner_debug_edge_limit = 0;
  bool planner_trace_console = false;
  int planner_trace_every = 0;
  int planner_max_iterations_override = 0;
  bool planner_nn_guidance_enabled = false;
  std::string planner_nn_model_path;
  double planner_nn_sample_probability = 0.0;
  double planner_nn_sample_distance = 0.0;
  // A* 网格分辨率（<=0 自动估算）
  double planner_astar_resolution = 0.0;
  // A* 是否允许对角邻接
  bool planner_astar_diagonal = true;
  // A* 启发权重（>0）
  double planner_astar_heuristic_weight = 1.0;

  const Environment* environment = nullptr;
  const CollisionChecker* collision_checker = nullptr;
  Planner* planner = nullptr;
  DynamicObstacleModel* dynamic_model = nullptr;
  LearningModule* learner = nullptr;
  Visualizer* visualizer = nullptr;
  VisualizationConfig visualization_config{};
};

// 仿真输出
struct SimulationResult {
  Status status{};
  Trajectory executed_trajectory{};
  Trajectory last_planned_trajectory{};
  std::vector<LineSegment2D> last_planner_debug_edges{};
  EvaluationMetrics metrics{};
  CostWeights updated_weights{};
  int steps = 0;
  int replans = 0;
  double total_planning_time_ms = 0.0;
  int first_solution_iteration = 0;
  int last_plan_iterations = 0;
};

// 仿真接口
class Simulation {
 public:
  virtual ~Simulation() = default;

  // 执行单次仿真并输出结果
  virtual Status Run(
      const SimulationRequest& request,
      const SimulationConfig& config,
      SimulationResult* result) = 0;
};

}  // namespace pathlearn
