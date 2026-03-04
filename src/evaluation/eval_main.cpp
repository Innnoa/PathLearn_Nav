#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "impl/cost_weight_learner.hpp"
#include "impl/rule_based_dynamic_obstacle.hpp"
#include "impl/simple_collision_checker.hpp"
#include "impl/simple_config.hpp"
#include "impl/simple_environment.hpp"
#include "impl/simple_evaluator.hpp"
#include "impl/simple_astar_planner.hpp"
#include "impl/simple_rrt_planner.hpp"
#include "impl/simple_simulation.hpp"
#if PATHLEARN_WITH_SDL
#include "impl/sdl_visualizer.hpp"
#endif

namespace {

std::string GetArgValue(int argc, char** argv, const std::string& key) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key && i + 1 < argc) {
      return argv[i + 1];
    }
  }
  return "";
}

bool HasFlag(int argc, char** argv, const std::string& key) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == key) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> SplitCsv(const std::string& input) {
  std::vector<std::string> parts;
  std::string current;
  for (char ch : input) {
    if (ch == ',') {
      if (!current.empty()) {
        parts.push_back(current);
        current.clear();
      }
    } else {
      current.push_back(ch);
    }
  }
  if (!current.empty()) {
    parts.push_back(current);
  }
  return parts;
}

bool ParseIntArg(const std::string& value, int* out) {
  if (!out || value.empty()) {
    return false;
  }
  try {
    *out = std::stoi(value);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

bool ParseDoubleArg(const std::string& value, double* out) {
  if (!out || value.empty()) {
    return false;
  }
  try {
    *out = std::stod(value);
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

bool ParseOnOffArg(const std::string& value, bool* out) {
  if (!out) {
    return false;
  }
  std::string normalized;
  normalized.reserve(value.size());
  for (char ch : value) {
    normalized.push_back(static_cast<char>(std::tolower(
        static_cast<unsigned char>(ch))));
  }

  if (normalized == "on" || normalized == "true" || normalized == "1") {
    *out = true;
    return true;
  }
  if (normalized == "off" || normalized == "false" || normalized == "0") {
    *out = false;
    return true;
  }
  return false;
}

std::string NormalizeLower(std::string value) {
  for (char& ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return value;
}

std::string BuildTimestampString() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
#if defined(_WIN32)
  localtime_s(&local_tm, &now_time);
#else
  localtime_r(&now_time, &local_tm);
#endif
  std::ostringstream output;
  output << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
  return output.str();
}

std::string ReplaceTimestampToken(
    const std::string& value,
    const std::string& timestamp) {
  const std::string token = "{timestamp}";
  if (value.find(token) == std::string::npos) {
    return value;
  }
  std::string result = value;
  size_t pos = 0;
  while ((pos = result.find(token, pos)) != std::string::npos) {
    result.replace(pos, token.size(), timestamp);
    pos += timestamp.size();
  }
  return result;
}

struct EvalRunConfig {
  std::string planner_type = "rrt";
  double max_speed = 0.4;
  double time_step_sec = 0.5;
  double detour_factor = 3.0;
  int max_steps_override = 0;
  int max_wait_steps = 4;
  double exploration_weight = 6.0;
  int planner_max_iterations_override = 0;
  double planner_astar_resolution = 0.0;
  bool planner_astar_diagonal = true;
  double planner_astar_heuristic_weight = 1.0;
  bool planner_show_tree = false;
  int planner_tree_edge_limit = 8000;
  bool planner_trace_console = false;
  int planner_trace_every = 200;
  bool planner_nn_guidance_enabled = false;
  std::string planner_nn_model_path = "data/models/nn_sampler_default.txt";
  double planner_nn_sample_probability = 0.35;
  double planner_nn_sample_distance = 0.0;
  bool gui_enabled = false;
  bool gui_step_by_step = true;
  int gui_width_px = 900;
  int gui_height_px = 700;
  int gui_target_fps = 30;
};

struct Scenario {
  std::string id;
  std::string map_path;
  pathlearn::Pose2D start;
  pathlearn::Pose2D goal;
};

struct EvalPose {
  pathlearn::Pose2D start;
  pathlearn::Pose2D goal;
};

struct DynamicProfile {
  std::string name;
  std::vector<pathlearn::DynamicObstacleState> obstacles;
  std::vector<std::pair<std::string, pathlearn::Vector2D>> line_rules;
  std::vector<std::pair<std::string, std::pair<pathlearn::Pose2D, pathlearn::Pose2D>>> bounce_rules;
};

struct WeightLogEntry {
  std::string scenario_id;
  std::string profile;
  unsigned int seed = 0;
  bool learning_enabled = false;
  int epoch = 0;
  pathlearn::CostWeights initial_weights{};
  pathlearn::CostWeights updated_weights{};
};

struct RunVisualSnapshot {
  bool valid = false;
  pathlearn::VisualizationFrame frame{};
  int replans = 0;
  int steps = 0;
  int last_plan_iterations = 0;
  int first_solution_iteration = 0;
  std::string planner_tag = "RRT";
};

std::string StatusReason(pathlearn::StatusCode code) {
  switch (code) {
    case pathlearn::StatusCode::kOk:
      return "到达目标";
    case pathlearn::StatusCode::kCollision:
      return "发生碰撞";
    case pathlearn::StatusCode::kNoPath:
      return "未找到可行路径";
    case pathlearn::StatusCode::kInvalidInput:
      return "参数或输入无效";
    case pathlearn::StatusCode::kNotReady:
      return "窗口关闭或渲染不可用";
    case pathlearn::StatusCode::kInternalError:
      return "内部错误";
  }
  return "未知";
}

std::string BuildHudTitle(
    const Scenario& scenario,
    const DynamicProfile& profile,
    unsigned int seed,
    bool learning_enabled,
    int epoch,
    int learn_epochs) {
  std::ostringstream output;
  output << "场景 " << scenario.id
         << " | 规则 " << profile.name
         << " | seed " << seed
         << " | 模式 " << (learning_enabled ? "学习" : "基线");
  if (learning_enabled) {
    output << " | 学习轮 " << epoch << "/" << learn_epochs;
  }
  return output.str();
}

std::string BuildHudSubtitle(const std::string& planner_type, int plan_index) {
  const std::string normalized = NormalizeLower(planner_type);
  if (normalized == "astar") {
    return "A* 搜索运行中";
  }
  std::ostringstream output;
  output << "RRT p" << std::max(1, plan_index) << " 运行中";
  return output.str();
}

std::string BuildResultDialogText(
    const pathlearn::EvaluationRunResult& run,
    const RunVisualSnapshot& snapshot,
    const std::string& header,
    const std::string& action_hint) {
  const int plan_index = std::max(1, snapshot.replans);
  const std::string planner_tag =
      snapshot.planner_tag.empty() ? "RRT" : snapshot.planner_tag;
  std::ostringstream output;
  output << header << "\n";
  output << "状态 " << static_cast<int>(run.status.code)
         << "(" << StatusReason(run.status.code) << ")"
         << " | " << planner_tag;
  if (planner_tag == "RRT") {
    output << " p" << plan_index;
  }
  output
         << " | replans " << snapshot.replans
         << " | iter " << snapshot.last_plan_iterations
         << " | 首解 " << snapshot.first_solution_iteration << "\n";
  output << std::fixed << std::setprecision(4)
         << "success " << run.metrics.success_rate
         << " | collision " << run.metrics.collision_rate
         << " | time " << run.metrics.avg_exec_time_sec
         << " | length " << run.metrics.avg_path_length
         << " | min_safe " << run.metrics.min_safety_distance << "\n";
  output << action_hint;
  return output.str();
}

std::vector<Scenario> BuildScenarios(const std::string& base_dir) {
  std::vector<Scenario> scenarios;
  const pathlearn::Pose2D start{-33.6, -33.6, 0.0};
  const pathlearn::Pose2D goal{33.6, 33.6, 0.0};
  scenarios.push_back({"scene_01", base_dir + "/maps/scene_01.txt", start, goal});
  scenarios.push_back({"scene_02", base_dir + "/maps/scene_02.txt", start, goal});
  scenarios.push_back({"scene_03", base_dir + "/maps/scene_03.txt", start, goal});
  scenarios.push_back({"scene_04", base_dir + "/maps/scene_04.txt", start, goal});
  scenarios.push_back({"scene_05", base_dir + "/maps/scene_05.txt", start, goal});
  return scenarios;
}

Scenario BuildSingleScenario(const std::string& map_path, const EvalPose& pose) {
  std::filesystem::path path_obj(map_path);
  std::string scenario_id = path_obj.stem().string();
  if (scenario_id.empty()) {
    scenario_id = "custom_map";
  }
  return {scenario_id, map_path, pose.start, pose.goal};
}

void OverrideScenarioPoses(
    const EvalPose& pose,
    std::vector<Scenario>* scenarios) {
  if (!scenarios) {
    return;
  }
  for (auto& scenario : *scenarios) {
    scenario.start = pose.start;
    scenario.goal = pose.goal;
  }
}

DynamicProfile BuildLineProfile(double /*max_speed*/) {
  DynamicProfile profile;
  profile.name = "line";
  // 动态障碍已禁用：保留名称但不生成障碍
  return profile;
}

DynamicProfile BuildBounceProfile(double /*max_speed*/) {
  DynamicProfile profile;
  profile.name = "bounce";
  // 动态障碍已禁用：保留名称但不生成障碍
  return profile;
}

DynamicProfile BuildStaticProfile() {
  DynamicProfile profile;
  profile.name = "static";
  return profile;
}

void ApplyProfileRules(
    const DynamicProfile& profile,
    pathlearn::RuleBasedDynamicObstacle* model) {
  if (!model) {
    return;
  }
  for (const auto& rule : profile.line_rules) {
    model->SetRuleLine(rule.first, rule.second);
  }
  for (const auto& rule : profile.bounce_rules) {
    model->SetRuleBounce(rule.first, rule.second.first, rule.second.second);
  }
}

pathlearn::SimulationConfig BuildConfig(
    const pathlearn::SimulationRequest& request,
    const EvalRunConfig& run_config) {
  pathlearn::SimulationConfig config{};
  config.time_step_sec = run_config.time_step_sec;

  const double dx = request.goal.x - request.start.pose.x;
  const double dy = request.goal.y - request.start.pose.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double min_time = request.limits.max_speed > 0.0
      ? distance / request.limits.max_speed
      : distance;

  const double detour_factor = run_config.detour_factor;
  config.horizon_sec = min_time * detour_factor + 2.0;
  if (config.horizon_sec < 10.0) {
    config.horizon_sec = 10.0;
  }
  config.max_steps = 200;
  config.max_wait_steps = run_config.max_wait_steps;
  const double buffer_time = 5.0;
  const int desired_steps = static_cast<int>(
      std::ceil((config.horizon_sec + buffer_time) / config.time_step_sec));
  int planner_scaled_steps = desired_steps;
  if (NormalizeLower(run_config.planner_type) == "astar") {
    const double auto_resolution = std::clamp(
        request.limits.max_speed * std::max(config.time_step_sec, 0.1) * 0.7,
        0.25,
        2.0);
    const double effective_resolution =
        run_config.planner_astar_resolution > 0.0
        ? run_config.planner_astar_resolution
        : auto_resolution;
    const double nominal_step =
        std::max(0.1, request.limits.max_speed * config.time_step_sec);
    const double step_scale = std::max(1.0, nominal_step / effective_resolution);
    planner_scaled_steps = static_cast<int>(
        std::ceil(static_cast<double>(desired_steps) * step_scale));
  }
  if (planner_scaled_steps > config.max_steps) {
    config.max_steps = planner_scaled_steps;
  }
  if (run_config.max_steps_override > 0) {
    config.max_steps = run_config.max_steps_override;
  }
  config.goal_tolerance = 0.4;
  return config;
}

pathlearn::EvaluationRunResult RunOnce(
    const Scenario& scenario,
    const DynamicProfile& profile,
    unsigned int seed,
    const EvalRunConfig& run_config,
    bool learning_enabled,
    const pathlearn::EvaluationMetrics& baseline_metrics,
    const pathlearn::CostWeights& initial_weights,
    pathlearn::Visualizer* visualizer,
    const RunVisualSnapshot* baseline_snapshot,
    const RunVisualSnapshot* previous_learning_snapshot,
    const std::string& hud_title,
    const std::string& hud_subtitle,
    RunVisualSnapshot* visual_snapshot) {
  pathlearn::EvaluationRunResult run{};
  run.scenario_id = scenario.id;
  run.map_name = std::filesystem::path(scenario.map_path).filename().string();
  run.profile = profile.name;
  run.seed = seed;
  run.learning_enabled = learning_enabled;
  run.initial_weights = initial_weights;

  pathlearn::SimpleEnvironment environment;
  pathlearn::Status status = environment.LoadFromFile(scenario.map_path);
  if (status.code != pathlearn::StatusCode::kOk) {
    run.status = status;
    return run;
  }

  pathlearn::RuleBasedDynamicObstacle dynamic_model;
  ApplyProfileRules(profile, &dynamic_model);

  pathlearn::SimpleCollisionChecker collision_checker;
  pathlearn::SimpleRrtPlanner rrt_planner;
  rrt_planner.SetRandomSeed(seed);
  pathlearn::SimpleAStarPlanner astar_planner;
  pathlearn::Planner* planner = nullptr;
  const std::string normalized_planner = NormalizeLower(run_config.planner_type);
  if (normalized_planner == "astar") {
    planner = &astar_planner;
  } else {
    planner = &rrt_planner;
  }
  pathlearn::CostWeightLearner learner;

  pathlearn::CostWeights weights = initial_weights;
  learner.Initialize(weights);
  learner.SetEnabled(learning_enabled);

  pathlearn::SimulationRequest request{};
  request.start.pose = scenario.start;
  request.start.time_sec = 0.0;
  request.goal = scenario.goal;
  request.limits.max_speed = run_config.max_speed;
  request.robot_radius = 0.2;
  request.cost_weights = weights;
  request.baseline_metrics = baseline_metrics;
  request.dynamic_obstacles = profile.obstacles;
  request.environment = &environment;
  request.collision_checker = &collision_checker;
  request.planner = planner;
  request.dynamic_model = &dynamic_model;
  request.learner = learning_enabled ? &learner : nullptr;
  request.show_planner_tree = run_config.planner_show_tree;
  request.planner_debug_edge_limit = run_config.planner_tree_edge_limit;
  request.planner_trace_console = run_config.planner_trace_console;
  request.planner_trace_every = run_config.planner_trace_every;
  request.planner_max_iterations_override =
      run_config.planner_max_iterations_override;
  request.planner_nn_guidance_enabled =
      run_config.planner_nn_guidance_enabled;
  request.planner_nn_model_path = run_config.planner_nn_model_path;
  request.planner_nn_sample_probability =
      run_config.planner_nn_sample_probability;
  request.planner_nn_sample_distance = run_config.planner_nn_sample_distance;
  request.planner_astar_resolution = run_config.planner_astar_resolution;
  request.planner_astar_diagonal = run_config.planner_astar_diagonal;
  request.planner_astar_heuristic_weight =
      run_config.planner_astar_heuristic_weight;
  request.visualizer = visualizer;
  request.visualization_config.width_px = run_config.gui_width_px;
  request.visualization_config.height_px = run_config.gui_height_px;
  request.visualization_config.target_fps = run_config.gui_target_fps;

#if PATHLEARN_WITH_SDL
  auto* sdl_visualizer = dynamic_cast<pathlearn::SdlVisualizer*>(visualizer);
  if (sdl_visualizer) {
    sdl_visualizer->ClearOverlayLayers();
    if (baseline_snapshot && baseline_snapshot->valid) {
      pathlearn::VisualOverlayLayer layer{};
      layer.planned_trajectory = baseline_snapshot->frame.planned_trajectory;
      layer.executed_trajectory = baseline_snapshot->frame.executed_trajectory;
      layer.planner_debug_edges = baseline_snapshot->frame.planner_debug_edges;
      layer.planned_color = {80, 120, 190, 80};
      layer.executed_color = {90, 140, 200, 80};
      layer.tree_color = {150, 205, 255, 75};
      sdl_visualizer->AddOverlayLayer(layer);
    }
    if (previous_learning_snapshot && previous_learning_snapshot->valid) {
      pathlearn::VisualOverlayLayer layer{};
      layer.planned_trajectory = previous_learning_snapshot->frame.planned_trajectory;
      layer.executed_trajectory = previous_learning_snapshot->frame.executed_trajectory;
      layer.planner_debug_edges = previous_learning_snapshot->frame.planner_debug_edges;
      layer.planned_color = {120, 130, 220, 110};
      layer.executed_color = {100, 180, 120, 110};
      layer.tree_color = {150, 205, 255, 95};
      sdl_visualizer->AddOverlayLayer(layer);
    }
    sdl_visualizer->SetHudText(hud_title, hud_subtitle);
  }
#else
  (void)baseline_snapshot;
  (void)previous_learning_snapshot;
  (void)hud_title;
  (void)hud_subtitle;
#endif

  pathlearn::SimulationConfig config = BuildConfig(request, run_config);
  pathlearn::SimpleSimulation simulation;
  pathlearn::SimulationResult result{};
  status = simulation.Run(request, config, &result);
  run.status = status;
  run.metrics = result.metrics;
  run.updated_weights = result.updated_weights;
  run.first_solution_iteration = result.first_solution_iteration;

  if (visual_snapshot) {
    visual_snapshot->valid = true;
    visual_snapshot->frame.dynamic_obstacles = request.dynamic_obstacles;
    visual_snapshot->frame.planned_trajectory = result.last_planned_trajectory;
    visual_snapshot->frame.executed_trajectory = result.executed_trajectory;
    if (!result.executed_trajectory.points.empty()) {
      visual_snapshot->frame.current_state =
          result.executed_trajectory.points.back().state;
    } else {
      visual_snapshot->frame.current_state = request.start;
    }
    visual_snapshot->frame.goal = request.goal;
    visual_snapshot->frame.planner_debug_edges = result.last_planner_debug_edges;
    visual_snapshot->replans = result.replans;
    visual_snapshot->steps = result.steps;
    visual_snapshot->last_plan_iterations = result.last_plan_iterations;
    visual_snapshot->first_solution_iteration = result.first_solution_iteration;
    visual_snapshot->planner_tag =
        normalized_planner == "astar" ? "A*" : "RRT";
  }
  return run;
}

void WriteRunsCsv(
    const std::string& path,
    const std::vector<pathlearn::EvaluationRunResult>& runs) {
  std::ofstream output(path);
  output << "scenario_id,map_name,profile,seed,learning_enabled,status_code,"
            "success_rate,collision_rate,avg_exec_time_sec,avg_path_length,"
            "min_safety_distance,first_solution_iteration\n";
  output << std::fixed << std::setprecision(6);
  for (const auto& run : runs) {
    output << run.scenario_id << ","
           << run.map_name << ","
           << run.profile << ","
           << run.seed << ","
           << (run.learning_enabled ? 1 : 0) << ","
           << static_cast<int>(run.status.code) << ","
           << run.metrics.success_rate << ","
           << run.metrics.collision_rate << ","
           << run.metrics.avg_exec_time_sec << ","
           << run.metrics.avg_path_length << ","
           << run.metrics.min_safety_distance << ","
           << run.first_solution_iteration << "\n";
  }
}

void WriteWeightsCsv(
    const std::string& path,
    const std::vector<WeightLogEntry>& logs) {
  std::ofstream output(path);
  output << "run_index,scenario_id,profile,seed,learning_enabled,epoch,"
            "initial_length_weight,initial_smoothness_weight,initial_safety_weight,"
            "initial_time_weight,initial_exploration_weight,updated_length_weight,"
            "updated_smoothness_weight,updated_safety_weight,updated_time_weight,"
            "updated_exploration_weight\n";
  output << std::fixed << std::setprecision(6);
  int index = 0;
  for (const auto& log : logs) {
    output << index << ","
           << log.scenario_id << ","
           << log.profile << ","
           << log.seed << ","
           << (log.learning_enabled ? 1 : 0) << ","
           << log.epoch << ","
           << log.initial_weights.length_weight << ","
           << log.initial_weights.smoothness_weight << ","
           << log.initial_weights.safety_weight << ","
           << log.initial_weights.time_weight << ","
           << log.initial_weights.exploration_weight << ","
           << log.updated_weights.length_weight << ","
           << log.updated_weights.smoothness_weight << ","
           << log.updated_weights.safety_weight << ","
           << log.updated_weights.time_weight << ","
           << log.updated_weights.exploration_weight << "\n";
    index += 1;
  }
}

void WriteSummaryCsv(
    const std::string& path,
    const std::vector<pathlearn::EvaluationSummary>& summaries) {
  std::ofstream output(path);
  output << "scenario_id,map_name,profile,learning_enabled,total_runs,"
            "success_runs,collision_runs,no_path_runs,success_rate,"
            "collision_rate,avg_exec_time_sec,avg_path_length,"
            "avg_min_safety_distance,avg_first_solution_iteration\n";
  output << std::fixed << std::setprecision(6);
  for (const auto& summary : summaries) {
    output << summary.scenario_id << ","
           << summary.map_name << ","
           << summary.profile << ","
           << (summary.learning_enabled ? 1 : 0) << ","
           << summary.total_runs << ","
           << summary.success_runs << ","
           << summary.collision_runs << ","
           << summary.no_path_runs << ","
           << summary.success_rate << ","
           << summary.collision_rate << ","
           << summary.avg_exec_time_sec << ","
           << summary.avg_path_length << ","
           << summary.avg_min_safety_distance << ","
           << summary.avg_first_solution_iteration << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
#ifdef PATHLEARN_DATA_DIR
  const std::string base_dir = PATHLEARN_DATA_DIR;
  std::string config_path =
      std::string(PATHLEARN_DATA_DIR) + "/config/pathlearn_eval.conf";
#else
  const std::string base_dir = "data";
  std::string config_path = "data/config/pathlearn_eval.conf";
#endif

  int seeds = 20;
  int seed_start = 1;
  int maps_limit = 0;
  EvalRunConfig run_config{};
  int log_every = 10;
  int learn_epochs = 2;
  std::string out_prefix = "data/eval";
  std::string mode = "both";
  std::string profiles_arg = "all";
  std::string map_arg;
  EvalPose pose{};
  pose.start = {-33.0, -33.0, 0.0};
  pose.goal = {33.0, 33.0, 0.0};

  const std::string config_arg = GetArgValue(argc, argv, "--config");
  if (!config_arg.empty()) {
    config_path = config_arg;
  }
  pathlearn::config::ConfigMap config_values;
  const pathlearn::config::LoadResult config_result =
      pathlearn::config::LoadConfigFile(config_path, &config_values);
  if (config_result.status == pathlearn::config::LoadStatus::kInvalidFormat ||
      config_result.status == pathlearn::config::LoadStatus::kIoError) {
    std::cerr << "加载配置文件失败: " << config_path
              << " 行 " << config_result.line
              << " 原因: " << config_result.message
              << std::endl;
    return 1;
  }
  if (!config_arg.empty() &&
      config_result.status == pathlearn::config::LoadStatus::kNotFound) {
    std::cerr << "配置文件不存在: " << config_path << std::endl;
    return 1;
  }

#ifdef PATHLEARN_DATA_DIR
  const std::filesystem::path project_root =
      std::filesystem::path(PATHLEARN_DATA_DIR).parent_path();
#else
  const std::filesystem::path project_root = std::filesystem::current_path();
#endif
  std::filesystem::path config_base_dir = project_root;
  std::string config_base_dir_raw;
  if (pathlearn::config::TryGetString(
          config_values,
          "base_dir",
          &config_base_dir_raw) &&
      !config_base_dir_raw.empty()) {
    const std::filesystem::path raw_path(config_base_dir_raw);
    config_base_dir = raw_path.is_absolute()
        ? raw_path
        : (project_root / raw_path);
  }
  config_base_dir = config_base_dir.lexically_normal();

  auto ResolveConfigPath = [&](std::string* value) {
    if (!value || value->empty()) {
      return;
    }
    std::filesystem::path path(*value);
    if (path.is_relative()) {
      path = config_base_dir / path;
    }
    *value = path.lexically_normal().string();
  };

  auto ApplyConfigString = [&](const std::string& key, std::string* target) {
    std::string value;
    if (pathlearn::config::TryGetString(config_values, key, &value)) {
      *target = value;
    }
  };
  auto ApplyConfigInt = [&](const std::string& key, int* target) {
    int value = 0;
    if (pathlearn::config::TryGetInt(config_values, key, &value)) {
      *target = value;
    }
  };
  auto ApplyConfigDouble = [&](const std::string& key, double* target) {
    double value = 0.0;
    if (pathlearn::config::TryGetDouble(config_values, key, &value)) {
      *target = value;
    }
  };
  auto ApplyConfigBool = [&](const std::string& key, bool* target) {
    bool value = false;
    if (pathlearn::config::TryGetBool(config_values, key, &value)) {
      *target = value;
    }
  };

  ApplyConfigInt("seeds", &seeds);
  ApplyConfigInt("seed_start", &seed_start);
  ApplyConfigInt("maps_limit", &maps_limit);
  ApplyConfigString("planner", &run_config.planner_type);
  ApplyConfigDouble("speed", &run_config.max_speed);
  ApplyConfigInt("log_every", &log_every);
  ApplyConfigInt("learn_epochs", &learn_epochs);
  ApplyConfigString("out_prefix", &out_prefix);
  ApplyConfigString("mode", &mode);
  ApplyConfigString("profiles", &profiles_arg);
  ApplyConfigString("map", &map_arg);
  ApplyConfigDouble("time_step", &run_config.time_step_sec);
  ApplyConfigDouble("detour", &run_config.detour_factor);
  ApplyConfigInt("max_steps", &run_config.max_steps_override);
  ApplyConfigInt("max_wait_steps", &run_config.max_wait_steps);
  ApplyConfigDouble("exploration", &run_config.exploration_weight);
  ApplyConfigInt("rrt_max_iters", &run_config.planner_max_iterations_override);
  ApplyConfigDouble("astar_resolution", &run_config.planner_astar_resolution);
  ApplyConfigBool("astar_diagonal", &run_config.planner_astar_diagonal);
  ApplyConfigDouble("astar_heuristic_weight",
                    &run_config.planner_astar_heuristic_weight);
  ApplyConfigInt("tree_limit", &run_config.planner_tree_edge_limit);
  ApplyConfigInt("trace_every", &run_config.planner_trace_every);
  ApplyConfigBool("show_tree", &run_config.planner_show_tree);
  ApplyConfigBool("nn_guide", &run_config.planner_nn_guidance_enabled);
  ApplyConfigString("nn_model", &run_config.planner_nn_model_path);
  ApplyConfigDouble("nn_prob", &run_config.planner_nn_sample_probability);
  ApplyConfigDouble("nn_distance", &run_config.planner_nn_sample_distance);
  ApplyConfigDouble("start_x", &pose.start.x);
  ApplyConfigDouble("start_y", &pose.start.y);
  ApplyConfigDouble("start_theta", &pose.start.theta);
  ApplyConfigDouble("goal_x", &pose.goal.x);
  ApplyConfigDouble("goal_y", &pose.goal.y);
  ApplyConfigDouble("goal_theta", &pose.goal.theta);
  ApplyConfigBool("gui", &run_config.gui_enabled);
  ApplyConfigBool("gui_step", &run_config.gui_step_by_step);
  ApplyConfigInt("gui_width", &run_config.gui_width_px);
  ApplyConfigInt("gui_height", &run_config.gui_height_px);
  ApplyConfigInt("fps", &run_config.gui_target_fps);

  if (pathlearn::config::HasKey(config_values, "map")) {
    ResolveConfigPath(&map_arg);
  }
  if (pathlearn::config::HasKey(config_values, "out_prefix")) {
    ResolveConfigPath(&out_prefix);
  }
  if (pathlearn::config::HasKey(config_values, "nn_model")) {
    ResolveConfigPath(&run_config.planner_nn_model_path);
  }

  std::string trace_from_config;
  ApplyConfigString("trace", &trace_from_config);
  if (trace_from_config == "console") {
    run_config.planner_trace_console = true;
  } else if (trace_from_config == "off") {
    run_config.planner_trace_console = false;
  }

  const std::string seeds_arg = GetArgValue(argc, argv, "--seeds");
  ParseIntArg(seeds_arg, &seeds);
  const std::string seed_start_arg = GetArgValue(argc, argv, "--seed-start");
  ParseIntArg(seed_start_arg, &seed_start);
  const std::string maps_arg = GetArgValue(argc, argv, "--maps-limit");
  ParseIntArg(maps_arg, &maps_limit);
  const std::string speed_arg = GetArgValue(argc, argv, "--speed");
  ParseDoubleArg(speed_arg, &run_config.max_speed);
  const std::string log_every_arg = GetArgValue(argc, argv, "--log-every");
  ParseIntArg(log_every_arg, &log_every);
  const std::string learn_epochs_arg = GetArgValue(argc, argv, "--learn-epochs");
  ParseIntArg(learn_epochs_arg, &learn_epochs);
  const std::string out_arg = GetArgValue(argc, argv, "--out-prefix");
  if (!out_arg.empty()) {
    out_prefix = out_arg;
  }
  const std::string mode_arg = GetArgValue(argc, argv, "--mode");
  if (!mode_arg.empty()) {
    mode = mode_arg;
  }
  const std::string profiles_value = GetArgValue(argc, argv, "--profiles");
  if (!profiles_value.empty()) {
    profiles_arg = profiles_value;
  }
  const std::string map_value = GetArgValue(argc, argv, "--map");
  if (!map_value.empty()) {
    map_arg = map_value;
  }
  const std::string planner_value = GetArgValue(argc, argv, "--planner");
  if (!planner_value.empty()) {
    run_config.planner_type = planner_value;
  }

  ParseDoubleArg(GetArgValue(argc, argv, "--time-step"), &run_config.time_step_sec);
  ParseDoubleArg(GetArgValue(argc, argv, "--detour"), &run_config.detour_factor);
  ParseIntArg(GetArgValue(argc, argv, "--max-steps"), &run_config.max_steps_override);
  ParseIntArg(GetArgValue(argc, argv, "--max-wait-steps"), &run_config.max_wait_steps);
  ParseDoubleArg(GetArgValue(argc, argv, "--exploration"), &run_config.exploration_weight);
  ParseIntArg(GetArgValue(argc, argv, "--rrt-max-iters"),
              &run_config.planner_max_iterations_override);
  ParseDoubleArg(GetArgValue(argc, argv, "--astar-resolution"),
                 &run_config.planner_astar_resolution);
  const std::string astar_diagonal_arg =
      GetArgValue(argc, argv, "--astar-diagonal");
  if (!astar_diagonal_arg.empty()) {
    bool parsed_diagonal = true;
    if (ParseOnOffArg(astar_diagonal_arg, &parsed_diagonal)) {
      run_config.planner_astar_diagonal = parsed_diagonal;
    }
  }
  ParseDoubleArg(GetArgValue(argc, argv, "--astar-heuristic-weight"),
                 &run_config.planner_astar_heuristic_weight);
  ParseIntArg(GetArgValue(argc, argv, "--tree-limit"),
              &run_config.planner_tree_edge_limit);
  ParseIntArg(GetArgValue(argc, argv, "--trace-every"),
              &run_config.planner_trace_every);

  const std::string trace_arg = GetArgValue(argc, argv, "--trace");
  if (trace_arg == "console") {
    run_config.planner_trace_console = true;
  } else if (trace_arg == "off") {
    run_config.planner_trace_console = false;
  }
  if (HasFlag(argc, argv, "--show-tree")) {
    run_config.planner_show_tree = true;
  }

  const std::string nn_guide_arg = GetArgValue(argc, argv, "--nn-guide");
  if (!nn_guide_arg.empty()) {
    bool parsed_nn_guide = false;
    if (ParseOnOffArg(nn_guide_arg, &parsed_nn_guide)) {
      run_config.planner_nn_guidance_enabled = parsed_nn_guide;
    }
  }

  const std::string nn_model_arg = GetArgValue(argc, argv, "--nn-model");
  if (!nn_model_arg.empty()) {
    run_config.planner_nn_model_path = nn_model_arg;
  }
  ParseDoubleArg(GetArgValue(argc, argv, "--nn-prob"),
                 &run_config.planner_nn_sample_probability);
  ParseDoubleArg(GetArgValue(argc, argv, "--nn-distance"),
                 &run_config.planner_nn_sample_distance);

  ParseDoubleArg(GetArgValue(argc, argv, "--start-x"), &pose.start.x);
  ParseDoubleArg(GetArgValue(argc, argv, "--start-y"), &pose.start.y);
  ParseDoubleArg(GetArgValue(argc, argv, "--start-theta"), &pose.start.theta);
  ParseDoubleArg(GetArgValue(argc, argv, "--goal-x"), &pose.goal.x);
  ParseDoubleArg(GetArgValue(argc, argv, "--goal-y"), &pose.goal.y);
  ParseDoubleArg(GetArgValue(argc, argv, "--goal-theta"), &pose.goal.theta);

  const std::string gui_arg = GetArgValue(argc, argv, "--gui");
  if (!gui_arg.empty()) {
    bool parsed_gui = false;
    if (ParseOnOffArg(gui_arg, &parsed_gui)) {
      run_config.gui_enabled = parsed_gui;
    }
  } else if (HasFlag(argc, argv, "--gui")) {
    run_config.gui_enabled = true;
  }
  const std::string gui_step_arg = GetArgValue(argc, argv, "--gui-step");
  if (!gui_step_arg.empty()) {
    bool parsed_step = true;
    if (ParseOnOffArg(gui_step_arg, &parsed_step)) {
      run_config.gui_step_by_step = parsed_step;
    }
  }
  ParseIntArg(GetArgValue(argc, argv, "--gui-width"), &run_config.gui_width_px);
  ParseIntArg(GetArgValue(argc, argv, "--gui-height"), &run_config.gui_height_px);
  ParseIntArg(GetArgValue(argc, argv, "--fps"), &run_config.gui_target_fps);

  if (seeds <= 0) {
    seeds = 20;
  }
  if (seed_start < 0) {
    seed_start = 1;
  }
  if (run_config.max_speed <= 0.0) {
    run_config.max_speed = 0.4;
  }
  run_config.planner_type = NormalizeLower(run_config.planner_type);
  if (run_config.planner_type != "rrt" &&
      run_config.planner_type != "astar") {
    std::cerr << "planner 参数仅支持 rrt/astar，使用默认 rrt。" << std::endl;
    run_config.planner_type = "rrt";
  }
  if (log_every <= 0) {
    log_every = 10;
  }
  if (learn_epochs <= 0) {
    learn_epochs = 1;
  }
  if (run_config.time_step_sec <= 0.0) {
    run_config.time_step_sec = 0.5;
  }
  if (run_config.detour_factor < 1.0) {
    run_config.detour_factor = 3.0;
  }
  if (run_config.max_steps_override < 0) {
    run_config.max_steps_override = 0;
  }
  if (run_config.max_wait_steps < 0) {
    run_config.max_wait_steps = 4;
  }
  if (run_config.exploration_weight < 1.0 ||
      run_config.exploration_weight > 8.0) {
    run_config.exploration_weight = 6.0;
  }
  if (run_config.planner_max_iterations_override < 0) {
    run_config.planner_max_iterations_override = 0;
  }
  if (run_config.planner_astar_resolution < 0.0) {
    run_config.planner_astar_resolution = 0.0;
  }
  if (run_config.planner_astar_heuristic_weight <= 0.0) {
    run_config.planner_astar_heuristic_weight = 1.0;
  }
  if (run_config.planner_tree_edge_limit < 0) {
    run_config.planner_tree_edge_limit = 8000;
  }
  if (run_config.planner_trace_every < 1) {
    run_config.planner_trace_every = 200;
  }
  if (run_config.planner_nn_sample_probability < 0.0 ||
      run_config.planner_nn_sample_probability > 1.0) {
    run_config.planner_nn_sample_probability = 0.35;
  }
  if (run_config.planner_nn_sample_distance < 0.0) {
    run_config.planner_nn_sample_distance = 0.0;
  }
  if (run_config.gui_width_px < 480) {
    run_config.gui_width_px = 900;
  }
  if (run_config.gui_height_px < 360) {
    run_config.gui_height_px = 700;
  }
  if (run_config.gui_target_fps < 0) {
    run_config.gui_target_fps = 30;
  }
  if (run_config.gui_enabled) {
    // GUI 评测默认展示树，便于观察当前轮与历史叠加。
    run_config.planner_show_tree = true;
  }
#if !PATHLEARN_WITH_SDL
  if (run_config.gui_enabled) {
    std::cerr << "当前构建未启用 SDL，可视化参数已忽略。" << std::endl;
    run_config.gui_enabled = false;
  }
#endif

  std::vector<DynamicProfile> profiles;
  const std::vector<std::string> profile_tokens =
      profiles_arg == "all" ? std::vector<std::string>{"line", "bounce"}
                            : SplitCsv(profiles_arg);
  for (const auto& token : profile_tokens) {
    if (token == "line") {
      profiles.push_back(BuildLineProfile(run_config.max_speed));
    } else if (token == "bounce") {
      profiles.push_back(BuildBounceProfile(run_config.max_speed));
    } else if (token == "static" || token == "none") {
      profiles.push_back(BuildStaticProfile());
    }
  }
  if (profiles.empty()) {
    std::cerr << "未匹配动态规则，使用默认 all。" << std::endl;
    profiles.push_back(BuildLineProfile(run_config.max_speed));
    profiles.push_back(BuildBounceProfile(run_config.max_speed));
  }

  std::vector<Scenario> scenarios;
  if (!map_arg.empty()) {
    if (!std::filesystem::exists(map_arg)) {
      std::cerr << "地图文件不存在: " << map_arg << std::endl;
      return 1;
    }
    scenarios.push_back(BuildSingleScenario(map_arg, pose));
  } else {
    scenarios = BuildScenarios(base_dir);
    OverrideScenarioPoses(pose, &scenarios);
    if (maps_limit > 0 && maps_limit < static_cast<int>(scenarios.size())) {
      scenarios.resize(static_cast<size_t>(maps_limit));
    }
  }

  bool run_baseline = (mode == "both" || mode == "baseline");
  bool run_learning = (mode == "both" || mode == "learning");
  if (!run_baseline && !run_learning) {
    std::cerr << "模式无效，使用 both。" << std::endl;
    run_baseline = true;
    run_learning = true;
  }

  out_prefix = ReplaceTimestampToken(out_prefix, BuildTimestampString());

  const int run_count_per_seed =
      (run_baseline ? 1 : 0) + (run_learning ? learn_epochs : 0);
  const int total_runs = static_cast<int>(scenarios.size()) *
      static_cast<int>(profiles.size()) * seeds * run_count_per_seed;
  int completed_runs = 0;

  std::cout << "评测配置: maps=" << scenarios.size()
            << " profiles=" << profiles.size()
            << " seeds=" << seeds
            << " mode=" << mode
            << " planner=" << run_config.planner_type
            << " learn_epochs=" << learn_epochs
            << " log_every=" << log_every
            << " map=" << (map_arg.empty() ? "<default-list>" : map_arg)
            << " speed=" << run_config.max_speed
            << " time_step=" << run_config.time_step_sec
            << " detour=" << run_config.detour_factor
            << " max_steps=" << run_config.max_steps_override
            << " max_wait_steps=" << run_config.max_wait_steps
            << " exploration=" << run_config.exploration_weight
            << " rrt_max_iters=" << run_config.planner_max_iterations_override
            << " astar_resolution=" << run_config.planner_astar_resolution
            << " astar_diagonal="
            << (run_config.planner_astar_diagonal ? "on" : "off")
            << " astar_heuristic_weight="
            << run_config.planner_astar_heuristic_weight
            << " gui=" << (run_config.gui_enabled ? "on" : "off")
            << " gui_step=" << (run_config.gui_step_by_step ? "on" : "off")
            << " gui_size=" << run_config.gui_width_px << "x" << run_config.gui_height_px
            << " fps=" << run_config.gui_target_fps
            << " start=(" << pose.start.x << "," << pose.start.y << ")"
            << " goal=(" << pose.goal.x << "," << pose.goal.y << ")"
            << " total_runs=" << total_runs
            << std::endl;

  const auto LogProgress = [&](const pathlearn::EvaluationRunResult& run) {
    if (total_runs <= 0 || log_every <= 0) {
      return;
    }
    if (completed_runs % log_every != 0 &&
        completed_runs != total_runs) {
      return;
    }
    std::cout << "进度 " << completed_runs << "/" << total_runs
              << " 场景 " << run.scenario_id
              << " 规则 " << run.profile
              << " 种子 " << run.seed
              << " 学习 " << (run.learning_enabled ? "on" : "off")
              << " 状态 " << static_cast<int>(run.status.code)
              << std::endl;
  };

  std::vector<pathlearn::EvaluationRunResult> all_runs;
  std::vector<WeightLogEntry> weight_logs;
  weight_logs.reserve(static_cast<size_t>(total_runs));
  bool stop_requested = false;

#if PATHLEARN_WITH_SDL
  pathlearn::SdlVisualizer eval_visualizer_impl;
  pathlearn::SdlVisualizer* eval_visualizer =
      run_config.gui_enabled ? &eval_visualizer_impl : nullptr;
#else
  pathlearn::Visualizer* eval_visualizer = nullptr;
#endif

  for (const auto& scenario : scenarios) {
    if (stop_requested) {
      break;
    }
    for (const auto& profile : profiles) {
      if (stop_requested) {
        break;
      }
      pathlearn::CostWeights learning_weights{};
      learning_weights.length_weight = 1.0;
      learning_weights.smoothness_weight = 1.0;
      learning_weights.safety_weight = 1.0;
      learning_weights.time_weight = 1.0;
      learning_weights.exploration_weight = run_config.exploration_weight;
      for (int i = 0; i < seeds; ++i) {
        if (stop_requested) {
          break;
        }
        const unsigned int seed =
            static_cast<unsigned int>(seed_start + i);
        pathlearn::EvaluationMetrics baseline_metrics{};
        RunVisualSnapshot baseline_snapshot{};
        RunVisualSnapshot previous_learning_snapshot{};

        if (run_baseline || !run_learning) {
          pathlearn::CostWeights baseline_weights{};
          baseline_weights.length_weight = 1.0;
          baseline_weights.smoothness_weight = 1.0;
          baseline_weights.safety_weight = 1.0;
          baseline_weights.time_weight = 1.0;
          baseline_weights.exploration_weight = run_config.exploration_weight;
          const std::string hud_title = BuildHudTitle(
              scenario,
              profile,
              seed,
              false,
              0,
              learn_epochs);
          const std::string hud_subtitle =
              BuildHudSubtitle(run_config.planner_type, 1);
          RunVisualSnapshot baseline_round_snapshot{};
          auto baseline = RunOnce(
              scenario, profile, seed, run_config, false, baseline_metrics,
              baseline_weights,
              eval_visualizer,
              nullptr,
              nullptr,
              hud_title,
              hud_subtitle,
              &baseline_round_snapshot);
          baseline_snapshot = baseline_round_snapshot;
          baseline_metrics = baseline.metrics;
          all_runs.push_back(baseline);
          weight_logs.push_back({baseline.scenario_id,
                                 baseline.profile,
                                 baseline.seed,
                                 baseline.learning_enabled,
                                 0,
                                 baseline.initial_weights,
                                 baseline.updated_weights});
          completed_runs += 1;
          LogProgress(baseline);

#if PATHLEARN_WITH_SDL
          if (eval_visualizer &&
              !run_learning &&
              baseline_snapshot.valid &&
              run_config.gui_step_by_step) {
            const bool has_more_runs = completed_runs < total_runs;
            pathlearn::EvalControlAction action = has_more_runs
                ? pathlearn::EvalControlAction::kContinue
                : pathlearn::EvalControlAction::kStop;
            const std::string header = has_more_runs
                ? "基线轮次已完成。"
                : "基线评测已完成。";
            const std::string action_hint = has_more_runs
                ? "点击“进行下一轮”继续，或“结束评测”停止。"
                : "点击“关闭窗口”结束评测。";
            const std::string prompt = BuildResultDialogText(
                baseline,
                baseline_snapshot,
                header,
                action_hint);
            const pathlearn::Status wait_status = eval_visualizer->WaitForControl(
                baseline_snapshot.frame,
                has_more_runs,
                &action,
                prompt);
            if (wait_status.code != pathlearn::StatusCode::kOk) {
              stop_requested = true;
            } else if (action == pathlearn::EvalControlAction::kStop) {
              stop_requested = true;
            }
          }
#endif
        }

        if (run_learning) {
          pathlearn::EvaluationRunResult last_learning{};
          bool has_learning_result = false;
          for (int epoch = 0; epoch < learn_epochs; ++epoch) {
            if (stop_requested) {
              break;
            }
            const int current_epoch = epoch + 1;
            const std::string hud_title = BuildHudTitle(
                scenario,
                profile,
                seed,
                true,
                current_epoch,
                learn_epochs);
            const std::string hud_subtitle =
                BuildHudSubtitle(run_config.planner_type, 1);
            RunVisualSnapshot current_snapshot{};
            auto learning = RunOnce(
                scenario, profile, seed, run_config, true, baseline_metrics,
                learning_weights,
                eval_visualizer,
                baseline_snapshot.valid ? &baseline_snapshot : nullptr,
                previous_learning_snapshot.valid ? &previous_learning_snapshot
                                                 : nullptr,
                hud_title,
                hud_subtitle,
                &current_snapshot);
            weight_logs.push_back({learning.scenario_id,
                                   learning.profile,
                                   learning.seed,
                                   learning.learning_enabled,
                                   epoch,
                                   learning.initial_weights,
                                   learning.updated_weights});
            learning_weights = learning.updated_weights;
            last_learning = learning;
            has_learning_result = true;
            previous_learning_snapshot = current_snapshot;
            completed_runs += 1;
            LogProgress(learning);

#if PATHLEARN_WITH_SDL
            if (eval_visualizer && run_config.gui_step_by_step &&
                current_snapshot.valid) {
              const bool has_more_runs = completed_runs < total_runs;
              pathlearn::EvalControlAction action = has_more_runs
                  ? pathlearn::EvalControlAction::kContinue
                  : pathlearn::EvalControlAction::kStop;
              std::ostringstream header;
              if (has_more_runs) {
                header << "学习轮 " << current_epoch << "/" << learn_epochs
                       << " 已完成。";
              } else {
                header << "学习评测已完成。";
              }
              const std::string action_hint = has_more_runs
                  ? "点击“进行下一轮”继续，或“结束评测”停止。"
                  : "点击“关闭窗口”结束评测。";
              const std::string prompt = BuildResultDialogText(
                  learning,
                  current_snapshot,
                  header.str(),
                  action_hint);
              const pathlearn::Status wait_status = eval_visualizer->WaitForControl(
                  current_snapshot.frame,
                  has_more_runs,
                  &action,
                  prompt);
              if (wait_status.code != pathlearn::StatusCode::kOk ||
                  action == pathlearn::EvalControlAction::kStop) {
                stop_requested = true;
              }
            }
#endif
          }
          if (has_learning_result) {
            all_runs.push_back(last_learning);
          }
        }
      }
    }
  }

  std::map<std::string, std::vector<pathlearn::EvaluationRunResult>> groups;
  for (const auto& run : all_runs) {
    const std::string key = run.scenario_id + "|" + run.profile + "|" +
        (run.learning_enabled ? "1" : "0");
    groups[key].push_back(run);
  }

  std::vector<pathlearn::EvaluationSummary> summaries;
  summaries.reserve(groups.size());
  for (const auto& [key, runs] : groups) {
    summaries.push_back(pathlearn::SummarizeRuns(runs));
  }

  const std::filesystem::path prefix_path(out_prefix);
  const std::filesystem::path parent = prefix_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }

  const std::string runs_path = out_prefix + "_runs.csv";
  const std::string summary_path = out_prefix + "_summary.csv";
  const std::string weights_path = out_prefix + "_weights.csv";
  WriteRunsCsv(runs_path, all_runs);
  WriteSummaryCsv(summary_path, summaries);
  WriteWeightsCsv(weights_path, weight_logs);

  std::cout << "评测完成，输出: " << runs_path << " 与 " << summary_path
            << " 与 " << weights_path
            << std::endl;
  return 0;
}
