#include "impl/simple_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pathlearn {
namespace {

double Distance2D(const Pose2D& a, const Pose2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

Status ValidateRequest(
    const SimulationRequest& request,
    const SimulationConfig& config) {
  if (!request.environment || !request.collision_checker ||
      !request.planner || !request.dynamic_model) {
    return {StatusCode::kInvalidInput, "仿真依赖未设置"};
  }
  if (config.horizon_sec <= 0.0 || config.time_step_sec <= 0.0) {
    return {StatusCode::kInvalidInput, "仿真时间参数无效"};
  }
  if (config.max_steps <= 0) {
    return {StatusCode::kInvalidInput, "仿真步数无效"};
  }
  if (config.max_wait_steps < 0) {
    return {StatusCode::kInvalidInput, "等待步数无效"};
  }
  if (request.limits.max_speed <= 0.0) {
    return {StatusCode::kInvalidInput, "速度上限无效"};
  }
  return {StatusCode::kOk, ""};
}

void InitMetrics(EvaluationMetrics* metrics) {
  if (!metrics) {
    return;
  }
  metrics->success_rate = 0.0;
  metrics->collision_rate = 0.0;
  metrics->avg_exec_time_sec = 0.0;
  metrics->avg_path_length = 0.0;
  metrics->min_safety_distance = 0.0;
}

}  // namespace

Status SimpleSimulation::Run(
    const SimulationRequest& request,
    const SimulationConfig& config,
    SimulationResult* result) {
  if (!result) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }

  result->executed_trajectory.points.clear();
  result->last_planned_trajectory.points.clear();
  result->last_planner_debug_edges.clear();
  result->total_planning_time_ms = 0.0;
  result->first_solution_iteration = 0;
  result->last_plan_iterations = 0;
  result->steps = 0;
  result->replans = 0;
  InitMetrics(&result->metrics);
  result->updated_weights = request.cost_weights;

  Status status = ValidateRequest(request, config);
  if (status.code != StatusCode::kOk) {
    result->status = status;
    return status;
  }

  status = request.dynamic_model->Reset(request.dynamic_obstacles);
  if (status.code != StatusCode::kOk) {
    result->status = status;
    return status;
  }

  if (request.visualizer) {
    std::vector<CircleObstacle> static_obstacles;
    status = request.environment->GetStaticObstacles(&static_obstacles);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      return status;
    }
    status = request.visualizer->Initialize(
        request.visualization_config,
        request.environment->GetMapInfo(),
        static_obstacles);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      return status;
    }
  }

  State2D current = request.start;
  result->executed_trajectory.points.push_back(TrajectoryPoint{current});

  if (request.visualizer) {
    VisualizationFrame frame{};
    status = request.dynamic_model->GetStates(&frame.dynamic_obstacles);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      return status;
    }
    frame.executed_trajectory = result->executed_trajectory;
    frame.current_state = current;
    frame.goal = request.goal;
    status = request.visualizer->Render(frame);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      return status;
    }
  }

  double total_length = 0.0;
  double min_safety_distance = std::numeric_limits<double>::infinity();
  bool collision = false;
  bool success = false;
  int remaining_wait_steps = config.max_wait_steps;
  const bool static_only_mode = request.dynamic_obstacles.empty();
  Trajectory active_plan{};
  size_t active_plan_next_index = 0;
  bool has_active_plan = false;
  std::vector<LineSegment2D> last_debug_edges;

  for (int step = 0; step < config.max_steps; ++step) {
    const TimeSec current_time = current.time_sec;
    status = request.dynamic_model->Update(current_time);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      break;
    }

    DynamicObstaclePrediction prediction{};
    status = request.dynamic_model->Predict(
        current_time,
        config.horizon_sec,
        config.time_step_sec,
        &prediction);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      break;
    }

    CollisionContext collision_context{};
    collision_context.environment = request.environment;
    collision_context.prediction = &prediction;
    collision_context.robot_radius = request.robot_radius;

    const bool need_replan =
        !static_only_mode || !has_active_plan ||
        active_plan_next_index >= active_plan.points.size();
    if (need_replan) {
      PlannerRequest plan_request{};
      plan_request.start = current;
      plan_request.goal = request.goal;
      plan_request.limits = request.limits;
      plan_request.horizon_sec = config.horizon_sec;
      plan_request.time_step_sec = config.time_step_sec;
      plan_request.collision = collision_context;
      plan_request.collision_checker = request.collision_checker;
      plan_request.cost_weights = request.cost_weights;
      plan_request.enable_debug = request.show_planner_tree;
      plan_request.debug_edge_limit = request.planner_debug_edge_limit;
      plan_request.debug_log_console = request.planner_trace_console;
      plan_request.debug_log_every = request.planner_trace_every;
      plan_request.max_iterations_override =
          request.planner_max_iterations_override;
      plan_request.nn_guidance_enabled = request.planner_nn_guidance_enabled;
      plan_request.nn_model_path = request.planner_nn_model_path;
      plan_request.nn_sample_probability = request.planner_nn_sample_probability;
      plan_request.nn_sample_distance = request.planner_nn_sample_distance;
      plan_request.astar_grid_resolution = request.planner_astar_resolution;
      plan_request.astar_allow_diagonal = request.planner_astar_diagonal;
      plan_request.astar_heuristic_weight =
          request.planner_astar_heuristic_weight;

      PlannerResult plan_result{};
      plan_request.debug_plan_id = result->replans + 1;
      status = request.planner->Plan(plan_request, &plan_result);
      result->total_planning_time_ms += plan_result.planning_time_ms;
      result->replans += 1;
      if (status.code != StatusCode::kOk) {
        result->last_planner_debug_edges = plan_result.debug_edges;
        result->last_plan_iterations = plan_result.iterations;
        if (request.visualizer && request.show_planner_tree) {
          VisualizationFrame frame{};
          Status debug_status = request.dynamic_model->GetStates(
              &frame.dynamic_obstacles);
          if (debug_status.code != StatusCode::kOk) {
            result->status = debug_status;
            break;
          }
          frame.executed_trajectory = result->executed_trajectory;
          frame.current_state = current;
          frame.goal = request.goal;
          frame.planner_debug_edges = plan_result.debug_edges;
          debug_status = request.visualizer->Render(frame);
          if (debug_status.code != StatusCode::kOk) {
            result->status = debug_status;
            break;
          }
        }
        if (status.code == StatusCode::kNoPath && remaining_wait_steps > 0) {
          State2D wait_state = current;
          wait_state.time_sec += config.time_step_sec;
          wait_state.velocity = {0.0, 0.0};
          wait_state.acceleration = {0.0, 0.0};

          bool wait_collision = false;
          Status wait_status = request.collision_checker->CheckState(
              collision_context,
              wait_state,
              &wait_collision);
          if (wait_status.code != StatusCode::kOk || wait_collision) {
            result->status = status;
            break;
          }

          Trajectory wait_segment{};
          wait_segment.points = {TrajectoryPoint{current}, TrajectoryPoint{wait_state}};
          double wait_clearance = 0.0;
          wait_status = request.collision_checker->MinimumDistance(
              collision_context,
              wait_segment,
              &wait_clearance);
          if (wait_status.code != StatusCode::kOk) {
            result->status = wait_status;
            break;
          }
          min_safety_distance = std::min(min_safety_distance, wait_clearance);

          current = wait_state;
          result->executed_trajectory.points.push_back(TrajectoryPoint{current});
          result->steps += 1;
          remaining_wait_steps -= 1;
          continue;
        }

        result->status = status;
        break;
      }
      result->last_planned_trajectory = plan_result.trajectory;
      result->last_planner_debug_edges = plan_result.debug_edges;
      result->last_plan_iterations = plan_result.iterations;
      if (result->first_solution_iteration <= 0 &&
          plan_result.first_solution_iteration > 0) {
        result->first_solution_iteration = plan_result.first_solution_iteration;
      }
      if (plan_result.trajectory.points.size() < 2) {
        result->status = {StatusCode::kNoPath, "规划轨迹点不足"};
        break;
      }

      active_plan = plan_result.trajectory;
      active_plan_next_index = 1;
      has_active_plan = true;
      last_debug_edges = plan_result.debug_edges;
      remaining_wait_steps = config.max_wait_steps;
    }

    if (request.visualizer) {
      VisualizationFrame frame{};
      status = request.dynamic_model->GetStates(&frame.dynamic_obstacles);
      if (status.code != StatusCode::kOk) {
        result->status = status;
        break;
      }
      frame.planned_trajectory = active_plan;
      frame.executed_trajectory = result->executed_trajectory;
      frame.current_state = current;
      frame.goal = request.goal;
      frame.planner_debug_edges = last_debug_edges;
      status = request.visualizer->Render(frame);
      if (status.code != StatusCode::kOk) {
        result->status = status;
        break;
      }
    }

    if (!has_active_plan || active_plan_next_index >= active_plan.points.size()) {
      result->status = {StatusCode::kNoPath, "规划轨迹点不足"};
      break;
    }

    const State2D next = active_plan.points[active_plan_next_index].state;
    active_plan_next_index += 1;
    Trajectory segment{};
    segment.points = {TrajectoryPoint{current}, TrajectoryPoint{next}};
    bool hit = false;
    status = request.collision_checker->CheckTrajectory(
        collision_context,
        segment,
        &hit);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      break;
    }
    if (hit) {
      collision = true;
      result->executed_trajectory.points.push_back(TrajectoryPoint{next});
      result->status = {StatusCode::kCollision, "执行轨迹发生碰撞"};
      break;
    }

    double clearance = 0.0;
    status = request.collision_checker->MinimumDistance(
        collision_context,
        segment,
        &clearance);
    if (status.code != StatusCode::kOk) {
      result->status = status;
      break;
    }
    min_safety_distance = std::min(min_safety_distance, clearance);

    total_length += Distance2D(current.pose, next.pose);
    current = next;
    result->executed_trajectory.points.push_back(TrajectoryPoint{current});
    result->steps += 1;

    if (Distance2D(current.pose, request.goal) <= config.goal_tolerance) {
      success = true;
      result->status = {StatusCode::kOk, ""};
      break;
    }
  }

  if (result->status.code == StatusCode::kOk && !success) {
    result->status = {StatusCode::kNoPath, "仿真步数耗尽"};
  }

  result->metrics.success_rate = success ? 1.0 : 0.0;
  result->metrics.collision_rate = collision ? 1.0 : 0.0;
  result->metrics.avg_path_length = total_length;
  result->metrics.avg_exec_time_sec =
      result->executed_trajectory.points.empty()
          ? 0.0
          : result->executed_trajectory.points.back().state.time_sec -
              request.start.time_sec;
  result->metrics.min_safety_distance =
      std::isfinite(min_safety_distance) ? min_safety_distance : 0.0;

  if (request.learner) {
    LearningInput input{};
    input.metrics = result->metrics;
    input.current_weights = request.cost_weights;
    input.baseline_metrics = request.baseline_metrics;

    LearningOutput output{};
    status = request.learner->Update(input, &output);
    if (status.code == StatusCode::kOk) {
      result->updated_weights = output.updated_weights;
    } else {
      result->status = status;
      return status;
    }
  }

  return result->status;
}

}  // namespace pathlearn
