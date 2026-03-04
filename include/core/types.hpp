#pragma once

#include <string>
#include <vector>

namespace pathlearn {

// 时间单位：秒
using TimeSec = double;

// 二维位姿（全向模型中 theta 可置 0）
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
};

// 二维线段（用于调试可视化）
struct LineSegment2D {
  Pose2D from{};
  Pose2D to{};
};

// 二维向量
struct Vector2D {
  double x = 0.0;
  double y = 0.0;
};

// 二维状态：位姿 + 速度 + 加速度 + 时间
struct State2D {
  Pose2D pose{};
  Vector2D velocity{};
  Vector2D acceleration{};
  TimeSec time_sec = 0.0;
};

// 轨迹点
struct TrajectoryPoint {
  State2D state{};
};

// 轨迹（按时间升序）
struct Trajectory {
  std::vector<TrajectoryPoint> points;
};

// 运动约束
struct Limits2D {
  double max_speed = 0.0;
  double max_accel = 0.0;
  double max_jerk = 0.0;
};

// 环境边界
struct Bounds2D {
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
};

// 地图信息
struct MapInfo {
  std::string name;
  Bounds2D bounds{};
};

// 静态圆形障碍
struct CircleObstacle {
  Pose2D center{};
  double radius = 0.0;
};

// 圆形障碍（简化表示）
struct CircleShape {
  double radius = 0.0;
};

// 动态障碍当前状态
struct DynamicObstacleState {
  std::string id;
  Pose2D pose{};
  Vector2D velocity{};
  Vector2D acceleration{};
  CircleShape shape{};
  // 规则模型的模式标记（由上层定义含义）
  std::string rule_tag;
};

// 单个动态障碍的预测轨迹
struct DynamicObstacleTrajectory {
  std::string id;
  CircleShape shape{};
  std::vector<TrajectoryPoint> points;
};

// 动态障碍预测结果
struct DynamicObstaclePrediction {
  TimeSec start_time_sec = 0.0;
  TimeSec time_step_sec = 0.0;
  std::vector<DynamicObstacleTrajectory> trajectories;
};

// 评测指标
struct EvaluationMetrics {
  double success_rate = 0.0;
  double collision_rate = 0.0;
  double avg_exec_time_sec = 0.0;
  double avg_path_length = 0.0;
  double min_safety_distance = 0.0;
};

// 代价权重
struct CostWeights {
  double length_weight = 1.0;
  double smoothness_weight = 1.0;
  double safety_weight = 1.0;
  double time_weight = 1.0;
  double exploration_weight = 1.0;
};

// 通用状态码
enum class StatusCode {
  kOk = 0,
  kInvalidInput,
  kNotReady,
  kNoPath,
  kCollision,
  kInternalError
};

// 通用状态（成功由 code==kOk 表示）
struct Status {
  StatusCode code = StatusCode::kOk;
  std::string message;
};

// 碰撞查询上下文
struct CollisionContext {
  const class Environment* environment = nullptr;
  const DynamicObstaclePrediction* prediction = nullptr;
  // 机器人半径，默认按点模型处理
  double robot_radius = 0.0;
};

}  // namespace pathlearn
