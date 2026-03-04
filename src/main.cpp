#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "impl/cost_weight_learner.hpp"
#include "impl/rule_based_dynamic_obstacle.hpp"
#include "impl/simple_collision_checker.hpp"
#include "impl/simple_config.hpp"
#include "impl/simple_environment.hpp"
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

struct LearningRoundLog {
  int round_index = 0;
  pathlearn::Status status{};
  pathlearn::EvaluationMetrics metrics{};
  pathlearn::CostWeights initial_weights{};
  pathlearn::CostWeights updated_weights{};
  int replans = 0;
  int steps = 0;
};

}  // namespace

int main(int argc, char** argv) {
#ifdef PATHLEARN_DATA_DIR
  std::string map_path = std::string(PATHLEARN_DATA_DIR) + "/demo_map.txt";
  std::string config_path =
      std::string(PATHLEARN_DATA_DIR) + "/config/pathlearn_app.conf";
#else
  std::string map_path = "data/demo_map.txt";
  std::string config_path = "data/config/pathlearn_app.conf";
#endif
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

  if (pathlearn::config::TryGetString(config_values, "map", &map_path)) {
    ResolveConfigPath(&map_path);
  }
  const std::string map_arg = GetArgValue(argc, argv, "--map");
  if (!map_arg.empty()) {
    map_path = map_arg;
  }

  std::string planner_type = "rrt";
  ApplyConfigString("planner", &planner_type);
  const std::string planner_arg = GetArgValue(argc, argv, "--planner");
  if (!planner_arg.empty()) {
    planner_type = planner_arg;
  }
  planner_type = NormalizeLower(planner_type);
  if (planner_type != "rrt" && planner_type != "astar") {
    std::cerr << "planner 参数仅支持 rrt/astar，使用默认 rrt。"
              << std::endl;
    planner_type = "rrt";
  }

  bool disable_gui = false;
  bool hold_gui = false;
  ApplyConfigBool("no_gui", &disable_gui);
  ApplyConfigBool("hold", &hold_gui);
  if (HasFlag(argc, argv, "--no-gui")) {
    disable_gui = true;
  }
  if (HasFlag(argc, argv, "--hold")) {
    hold_gui = true;
  }

  double max_speed = 0.4;
  ApplyConfigDouble("speed", &max_speed);
  const std::string speed_arg = GetArgValue(argc, argv, "--speed");
  if (!speed_arg.empty()) {
    try {
      max_speed = std::stod(speed_arg);
    } catch (const std::exception&) {
      std::cerr << "速度参数无效，使用默认值 0.4。" << std::endl;
      max_speed = 0.4;
    }
  }
  if (max_speed <= 0.0) {
    std::cerr << "速度参数需大于 0，使用默认值 0.4。" << std::endl;
    max_speed = 0.4;
  }

  int target_fps = 15;
  ApplyConfigInt("fps", &target_fps);
  const std::string fps_arg = GetArgValue(argc, argv, "--fps");
  if (!fps_arg.empty()) {
    try {
      target_fps = std::stoi(fps_arg);
    } catch (const std::exception&) {
      std::cerr << "帧率参数无效，使用默认值 15。" << std::endl;
      target_fps = 15;
    }
  }
  if (target_fps < 0) {
    std::cerr << "帧率参数需为非负值，使用默认值 15。" << std::endl;
    target_fps = 15;
  }

  double time_step_sec = 0.5;
  ApplyConfigDouble("time_step", &time_step_sec);
  const std::string time_step_arg = GetArgValue(argc, argv, "--time-step");
  if (!time_step_arg.empty()) {
    try {
      time_step_sec = std::stod(time_step_arg);
    } catch (const std::exception&) {
      std::cerr << "时间步长参数无效，使用默认值 0.5。" << std::endl;
      time_step_sec = 0.5;
    }
  }
  if (time_step_sec <= 0.0) {
    std::cerr << "时间步长需大于 0，使用默认值 0.5。" << std::endl;
    time_step_sec = 0.5;
  }

  double detour_factor = 3.0;
  ApplyConfigDouble("detour", &detour_factor);
  const std::string detour_arg = GetArgValue(argc, argv, "--detour");
  if (!detour_arg.empty()) {
    try {
      detour_factor = std::stod(detour_arg);
    } catch (const std::exception&) {
      std::cerr << "绕行系数参数无效，使用默认值 3.0。" << std::endl;
      detour_factor = 3.0;
    }
  }
  if (detour_factor < 1.0) {
    std::cerr << "绕行系数需 >= 1.0，使用默认值 3.0。" << std::endl;
    detour_factor = 3.0;
  }

  int max_steps_override = 0;
  ApplyConfigInt("max_steps", &max_steps_override);
  const std::string max_steps_arg = GetArgValue(argc, argv, "--max-steps");
  if (!max_steps_arg.empty()) {
    try {
      max_steps_override = std::stoi(max_steps_arg);
    } catch (const std::exception&) {
      std::cerr << "最大步数参数无效，使用自动估算。" << std::endl;
      max_steps_override = 0;
    }
  }
  if (max_steps_override < 0) {
    std::cerr << "最大步数需为非负值，使用自动估算。" << std::endl;
    max_steps_override = 0;
  }

  int max_wait_steps = 4;
  ApplyConfigInt("max_wait_steps", &max_wait_steps);
  const std::string max_wait_steps_arg = GetArgValue(argc, argv, "--max-wait-steps");
  if (!max_wait_steps_arg.empty()) {
    try {
      max_wait_steps = std::stoi(max_wait_steps_arg);
    } catch (const std::exception&) {
      std::cerr << "max-wait-steps 参数无效，使用默认值 4。" << std::endl;
      max_wait_steps = 4;
    }
  }
  if (max_wait_steps < 0) {
    std::cerr << "max-wait-steps 需为非负值，使用默认值 4。" << std::endl;
    max_wait_steps = 4;
  }

  double exploration_weight = 6.0;
  ApplyConfigDouble("exploration", &exploration_weight);
  const std::string exploration_arg = GetArgValue(argc, argv, "--exploration");
  if (!exploration_arg.empty()) {
    try {
      exploration_weight = std::stod(exploration_arg);
    } catch (const std::exception&) {
      std::cerr << "探索权重参数无效，使用默认值 6.0。" << std::endl;
      exploration_weight = 6.0;
    }
  }
  if (exploration_weight < 1.0 || exploration_weight > 8.0) {
    std::cerr << "探索权重需在 [1, 8]，使用默认值 6.0。" << std::endl;
    exploration_weight = 6.0;
  }

  bool show_tree = false;
  ApplyConfigBool("show_tree", &show_tree);
  if (HasFlag(argc, argv, "--show-tree")) {
    show_tree = true;
  }
  int tree_edge_limit = 8000;
  ApplyConfigInt("tree_limit", &tree_edge_limit);
  const std::string tree_limit_arg = GetArgValue(argc, argv, "--tree-limit");
  if (!tree_limit_arg.empty()) {
    try {
      tree_edge_limit = std::stoi(tree_limit_arg);
    } catch (const std::exception&) {
      std::cerr << "树边数上限参数无效，使用默认值 8000。" << std::endl;
      tree_edge_limit = 8000;
    }
  }
  if (tree_edge_limit < 0) {
    std::cerr << "树边数上限需为非负值，使用默认值 8000。" << std::endl;
    tree_edge_limit = 8000;
  }

  bool trace_console = false;
  std::string trace_config;
  ApplyConfigString("trace", &trace_config);
  if (trace_config == "console") {
    trace_console = true;
  }
  int trace_every = 200;
  ApplyConfigInt("trace_every", &trace_every);
  const std::string trace_arg = GetArgValue(argc, argv, "--trace");
  if (!trace_arg.empty()) {
    if (trace_arg == "console") {
      trace_console = true;
    } else if (trace_arg == "off") {
      trace_console = false;
    } else if (trace_arg != "off") {
      std::cerr << "trace 参数仅支持 console/off，使用 off。" << std::endl;
    }
  }
  const std::string trace_every_arg = GetArgValue(argc, argv, "--trace-every");
  if (!trace_every_arg.empty()) {
    try {
      trace_every = std::stoi(trace_every_arg);
    } catch (const std::exception&) {
      std::cerr << "trace-every 参数无效，使用默认值 200。" << std::endl;
      trace_every = 200;
    }
  }
  if (trace_every < 1) {
    std::cerr << "trace-every 需 >= 1，使用默认值 200。" << std::endl;
    trace_every = 200;
  }

  int rrt_max_iters_override = 0;
  ApplyConfigInt("rrt_max_iters", &rrt_max_iters_override);
  const std::string rrt_max_iters_arg = GetArgValue(argc, argv, "--rrt-max-iters");
  if (!rrt_max_iters_arg.empty()) {
    try {
      rrt_max_iters_override = std::stoi(rrt_max_iters_arg);
    } catch (const std::exception&) {
      std::cerr << "rrt-max-iters 参数无效，使用默认预算。" << std::endl;
      rrt_max_iters_override = 0;
    }
  }
  if (rrt_max_iters_override < 0) {
    std::cerr << "rrt-max-iters 需为非负值，使用默认预算。" << std::endl;
    rrt_max_iters_override = 0;
  }

  double astar_resolution = 0.0;
  ApplyConfigDouble("astar_resolution", &astar_resolution);
  const std::string astar_resolution_arg =
      GetArgValue(argc, argv, "--astar-resolution");
  if (!astar_resolution_arg.empty()) {
    try {
      astar_resolution = std::stod(astar_resolution_arg);
    } catch (const std::exception&) {
      std::cerr << "astar-resolution 参数无效，使用自动估算。" << std::endl;
      astar_resolution = 0.0;
    }
  }
  if (astar_resolution < 0.0) {
    std::cerr << "astar-resolution 需为非负值，使用自动估算。"
              << std::endl;
    astar_resolution = 0.0;
  }

  bool astar_diagonal = true;
  ApplyConfigBool("astar_diagonal", &astar_diagonal);
  const std::string astar_diagonal_arg =
      GetArgValue(argc, argv, "--astar-diagonal");
  if (!astar_diagonal_arg.empty()) {
    bool parsed_diagonal = true;
    if (ParseOnOffArg(astar_diagonal_arg, &parsed_diagonal)) {
      astar_diagonal = parsed_diagonal;
    } else {
      std::cerr << "astar-diagonal 参数仅支持 on/off/true/false/1/0，使用默认 on。"
                << std::endl;
      astar_diagonal = true;
    }
  }

  double astar_heuristic_weight = 1.0;
  ApplyConfigDouble("astar_heuristic_weight", &astar_heuristic_weight);
  const std::string astar_heuristic_weight_arg =
      GetArgValue(argc, argv, "--astar-heuristic-weight");
  if (!astar_heuristic_weight_arg.empty()) {
    try {
      astar_heuristic_weight = std::stod(astar_heuristic_weight_arg);
    } catch (const std::exception&) {
      std::cerr << "astar-heuristic-weight 参数无效，使用默认值 1.0。"
                << std::endl;
      astar_heuristic_weight = 1.0;
    }
  }
  if (astar_heuristic_weight <= 0.0) {
    std::cerr << "astar-heuristic-weight 需大于 0，使用默认值 1.0。"
              << std::endl;
    astar_heuristic_weight = 1.0;
  }

  bool nn_guidance_enabled = false;
  ApplyConfigBool("nn_guide", &nn_guidance_enabled);
  const std::string nn_guide_arg = GetArgValue(argc, argv, "--nn-guide");
  if (!nn_guide_arg.empty()) {
    bool parsed_nn_guide = false;
    if (ParseOnOffArg(nn_guide_arg, &parsed_nn_guide)) {
      nn_guidance_enabled = parsed_nn_guide;
    } else {
      std::cerr << "nn-guide 参数仅支持 on/off/true/false/1/0，使用默认 off。"
                << std::endl;
      nn_guidance_enabled = false;
    }
  }

  std::string nn_model_path = "data/models/nn_sampler_default.txt";
  if (pathlearn::config::TryGetString(
          config_values,
          "nn_model",
          &nn_model_path)) {
    ResolveConfigPath(&nn_model_path);
  }
  const std::string nn_model_arg = GetArgValue(argc, argv, "--nn-model");
  if (!nn_model_arg.empty()) {
    nn_model_path = nn_model_arg;
  }

  double nn_sample_probability = 0.35;
  ApplyConfigDouble("nn_prob", &nn_sample_probability);
  const std::string nn_prob_arg = GetArgValue(argc, argv, "--nn-prob");
  if (!nn_prob_arg.empty()) {
    try {
      nn_sample_probability = std::stod(nn_prob_arg);
    } catch (const std::exception&) {
      std::cerr << "nn-prob 参数无效，使用默认值 0.35。" << std::endl;
      nn_sample_probability = 0.35;
    }
  }
  if (nn_sample_probability < 0.0 || nn_sample_probability > 1.0) {
    std::cerr << "nn-prob 需在 [0,1]，使用默认值 0.35。" << std::endl;
    nn_sample_probability = 0.35;
  }

  double nn_sample_distance = 0.0;
  ApplyConfigDouble("nn_distance", &nn_sample_distance);
  const std::string nn_distance_arg = GetArgValue(argc, argv, "--nn-distance");
  if (!nn_distance_arg.empty()) {
    try {
      nn_sample_distance = std::stod(nn_distance_arg);
    } catch (const std::exception&) {
      std::cerr << "nn-distance 参数无效，使用自动估算。" << std::endl;
      nn_sample_distance = 0.0;
    }
  }
  if (nn_sample_distance < 0.0) {
    std::cerr << "nn-distance 需为非负值，使用自动估算。" << std::endl;
    nn_sample_distance = 0.0;
  }

  bool learning_enabled = true;
  ApplyConfigBool("learning", &learning_enabled);
  const std::string learning_arg = GetArgValue(argc, argv, "--learning");
  if (!learning_arg.empty()) {
    bool parsed_learning = true;
    if (ParseOnOffArg(learning_arg, &parsed_learning)) {
      learning_enabled = parsed_learning;
    } else {
      std::cerr << "learning 参数仅支持 on/off/true/false/1/0，使用默认 on。"
                << std::endl;
    }
  }

  int learn_rounds = 1;
  ApplyConfigInt("learn_rounds", &learn_rounds);
  const std::string learn_rounds_arg = GetArgValue(argc, argv, "--learn-rounds");
  if (!learn_rounds_arg.empty()) {
    try {
      learn_rounds = std::stoi(learn_rounds_arg);
    } catch (const std::exception&) {
      std::cerr << "learn-rounds 参数无效，使用默认值 1。" << std::endl;
      learn_rounds = 1;
    }
  }
  if (learn_rounds < 1) {
    std::cerr << "learn-rounds 需 >= 1，使用默认值 1。" << std::endl;
    learn_rounds = 1;
  }

  unsigned int seed = 42u;
  {
    int seed_from_config = 42;
    ApplyConfigInt("seed", &seed_from_config);
    if (seed_from_config >= 0) {
      seed = static_cast<unsigned int>(seed_from_config);
    }
  }
  const std::string seed_arg = GetArgValue(argc, argv, "--seed");
  if (!seed_arg.empty()) {
    try {
      seed = static_cast<unsigned int>(std::stoul(seed_arg));
    } catch (const std::exception&) {
      std::cerr << "随机种子参数无效，使用默认值 42。" << std::endl;
      seed = 42u;
    }
  }

  pathlearn::SimpleEnvironment environment;
  pathlearn::Status status = environment.LoadFromFile(map_path);
  if (status.code != pathlearn::StatusCode::kOk) {
    std::cerr << "加载地图失败: " << status.message << std::endl;
    return 1;
  }

  // 动态障碍已禁用：保持空列表与无规则模型
  std::vector<pathlearn::DynamicObstacleState> obstacles;
  pathlearn::RuleBasedDynamicObstacle dynamic_model;

  pathlearn::SimpleCollisionChecker collision_checker;
  pathlearn::SimpleRrtPlanner rrt_planner;
  rrt_planner.SetRandomSeed(seed);
  pathlearn::SimpleAStarPlanner astar_planner;
  pathlearn::Planner* planner = nullptr;
  if (planner_type == "astar") {
    planner = &astar_planner;
  } else {
    planner = &rrt_planner;
  }
  pathlearn::CostWeightLearner learner;

  pathlearn::CostWeights weights{};
  weights.length_weight = 1.0;
  weights.safety_weight = 1.0;
  weights.time_weight = 1.0;
  weights.exploration_weight = exploration_weight;
  learner.Initialize(weights);

  pathlearn::SimulationRequest request{};
  request.start.pose = {-33.0, -33.0, 0.0};
  request.start.time_sec = 0.0;
  request.goal = {33.0, 33.0, 0.0};
  request.limits.max_speed = max_speed;
  request.robot_radius = 0.2;
  request.cost_weights = weights;
  request.dynamic_obstacles = obstacles;
  request.environment = &environment;
  request.collision_checker = &collision_checker;
  request.planner = planner;
  request.dynamic_model = &dynamic_model;
  request.learner = nullptr;
  request.show_planner_tree = show_tree;
  request.planner_debug_edge_limit = tree_edge_limit;
  request.planner_trace_console = trace_console;
  request.planner_trace_every = trace_every;
  request.planner_max_iterations_override = rrt_max_iters_override;
  request.planner_nn_guidance_enabled = nn_guidance_enabled;
  request.planner_nn_model_path = nn_model_path;
  request.planner_nn_sample_probability = nn_sample_probability;
  request.planner_nn_sample_distance = nn_sample_distance;
  request.planner_astar_resolution = astar_resolution;
  request.planner_astar_diagonal = astar_diagonal;
  request.planner_astar_heuristic_weight = astar_heuristic_weight;

  const double dx = request.goal.x - request.start.pose.x;
  const double dy = request.goal.y - request.start.pose.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  const double min_time = request.limits.max_speed > 0.0
      ? distance / request.limits.max_speed
      : distance;

  pathlearn::SimulationConfig config{};
  config.time_step_sec = time_step_sec;
  config.horizon_sec = min_time * detour_factor + 2.0;
  if (config.horizon_sec < 10.0) {
    config.horizon_sec = 10.0;
  }
  config.max_steps = 200;
  config.max_wait_steps = max_wait_steps;
  const double buffer_time = 5.0;
  const int desired_steps = static_cast<int>(
      std::ceil((config.horizon_sec + buffer_time) / config.time_step_sec));
  int planner_scaled_steps = desired_steps;
  if (planner_type == "astar") {
    const double auto_resolution = std::clamp(
        request.limits.max_speed * std::max(config.time_step_sec, 0.1) * 0.7,
        0.25,
        2.0);
    const double effective_resolution =
        astar_resolution > 0.0 ? astar_resolution : auto_resolution;
    const double nominal_step =
        std::max(0.1, request.limits.max_speed * config.time_step_sec);
    const double step_scale = std::max(1.0, nominal_step / effective_resolution);
    planner_scaled_steps = static_cast<int>(
        std::ceil(static_cast<double>(desired_steps) * step_scale));
  }
  if (planner_scaled_steps > config.max_steps) {
    config.max_steps = planner_scaled_steps;
  }
  if (max_steps_override > 0) {
    if (max_steps_override < planner_scaled_steps) {
      std::cerr << "最大步数低于估算值，可能导致仿真提前结束。" << std::endl;
    }
    config.max_steps = max_steps_override;
  }
  config.goal_tolerance = 0.4;

#if PATHLEARN_WITH_SDL
  pathlearn::SdlVisualizer visualizer;
  if (!disable_gui) {
    request.visualizer = &visualizer;
    request.visualization_config.width_px = 900;
    request.visualization_config.height_px = 700;
    request.visualization_config.target_fps = target_fps;
  }
#else
  if (!disable_gui) {
    std::cout << "SDL2 未启用，使用无 GUI 模式运行。" << std::endl;
  }
#endif

  pathlearn::SimpleSimulation simulation;
  pathlearn::SimulationResult result{};
  pathlearn::CostWeights rolling_weights = weights;
  pathlearn::EvaluationMetrics baseline_metrics{};
  bool baseline_ready = false;
  std::vector<LearningRoundLog> round_logs;
  round_logs.reserve(static_cast<size_t>(learn_rounds));

  for (int round = 1; round <= learn_rounds; ++round) {
    request.cost_weights = rolling_weights;
    request.baseline_metrics =
        baseline_ready ? baseline_metrics : pathlearn::EvaluationMetrics{};

    if (learning_enabled) {
      learner.SetEnabled(true);
      request.learner = &learner;
    } else {
      learner.SetEnabled(false);
      request.learner = nullptr;
    }

    pathlearn::SimulationResult round_result{};
    pathlearn::Status round_status = simulation.Run(request, config, &round_result);

    LearningRoundLog round_log{};
    round_log.round_index = round;
    round_log.status = round_status;
    round_log.metrics = round_result.metrics;
    round_log.initial_weights = request.cost_weights;
    round_log.updated_weights = round_result.updated_weights;
    round_log.replans = round_result.replans;
    round_log.steps = round_result.steps;
    round_logs.push_back(round_log);

    status = round_status;
    result = round_result;

    if (!baseline_ready) {
      baseline_metrics = round_result.metrics;
      baseline_ready = true;
    }
    if (learning_enabled) {
      rolling_weights = round_result.updated_weights;
    }

    if (round_status.code == pathlearn::StatusCode::kInvalidInput ||
        round_status.code == pathlearn::StatusCode::kInternalError ||
        round_status.code == pathlearn::StatusCode::kNotReady) {
      break;
    }
  }

  std::cout << "仿真状态: " << static_cast<int>(status.code)
            << " " << status.message << std::endl;
  const std::string end_reason = StatusReason(status.code);
  std::cout << "结束原因: " << end_reason << std::endl;
  std::cout << "成功率: " << result.metrics.success_rate
            << " 碰撞率: " << result.metrics.collision_rate
            << " 执行时间: " << result.metrics.avg_exec_time_sec
            << " 路径长度: " << result.metrics.avg_path_length
            << " 最小安全距离: " << result.metrics.min_safety_distance
            << std::endl;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << "学习配置: learning=" << (learning_enabled ? "on" : "off")
            << " learn_rounds=" << learn_rounds
            << " executed_rounds=" << round_logs.size()
            << std::endl;
  std::cout << "规划器配置: planner=" << planner_type
            << " astar_resolution=" << astar_resolution
            << " astar_diagonal=" << (astar_diagonal ? "on" : "off")
            << " astar_heuristic_weight=" << astar_heuristic_weight
            << " rrt_max_iters=" << rrt_max_iters_override
            << " max_wait_steps=" << max_wait_steps
            << std::endl;
  std::cout << "NN配置: nn_guide=" << (nn_guidance_enabled ? "on" : "off")
            << " nn_model=\"" << nn_model_path << "\""
            << " nn_prob=" << nn_sample_probability
            << " nn_distance=" << nn_sample_distance
            << std::endl;
  if (!round_logs.empty()) {
    const auto& base_round = round_logs.front();
    for (const auto& log : round_logs) {
      std::cout << "[学习轮 " << log.round_index << "]"
                << " 状态=" << static_cast<int>(log.status.code)
                << "(" << StatusReason(log.status.code) << ")"
                << " success=" << log.metrics.success_rate
                << " collision=" << log.metrics.collision_rate
                << " time=" << log.metrics.avg_exec_time_sec
                << " length=" << log.metrics.avg_path_length
                << " min_safe=" << log.metrics.min_safety_distance
                << " replans=" << log.replans
                << " steps=" << log.steps
                << std::endl;

      std::cout << "  权重变化"
                << " L " << log.initial_weights.length_weight
                << "->" << log.updated_weights.length_weight
                << " S " << log.initial_weights.safety_weight
                << "->" << log.updated_weights.safety_weight
                << " T " << log.initial_weights.time_weight
                << "->" << log.updated_weights.time_weight
                << " Smooth " << log.initial_weights.smoothness_weight
                << "->" << log.updated_weights.smoothness_weight
                << " Explore " << log.initial_weights.exploration_weight
                << "->" << log.updated_weights.exploration_weight
                << std::endl;

      std::cout << "  相对首轮Δ"
                << " success="
                << (log.metrics.success_rate - base_round.metrics.success_rate)
                << " collision="
                << (log.metrics.collision_rate - base_round.metrics.collision_rate)
                << " time="
                << (log.metrics.avg_exec_time_sec -
                    base_round.metrics.avg_exec_time_sec)
                << " length="
                << (log.metrics.avg_path_length -
                    base_round.metrics.avg_path_length)
                << " min_safe="
                << (log.metrics.min_safety_distance -
                    base_round.metrics.min_safety_distance)
                << std::endl;
    }
  }

  if (hold_gui && request.visualizer && !disable_gui) {
    std::cout << "窗口保持打开，关闭窗口或按 ESC 退出。" << std::endl;
    request.visualizer->WaitForClose();
  }

  return status.code == pathlearn::StatusCode::kOk ? 0 : 1;
}
