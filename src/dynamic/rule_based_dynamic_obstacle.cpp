#include "impl/rule_based_dynamic_obstacle.hpp"

#include <algorithm>
#include <cmath>

namespace pathlearn {
namespace {

double VectorNorm(const Vector2D& v) {
  return std::sqrt(v.x * v.x + v.y * v.y);
}

Vector2D Normalize(const Vector2D& v) {
  const double norm = VectorNorm(v);
  if (norm <= 1e-9) {
    return {0.0, 0.0};
  }
  return {v.x / norm, v.y / norm};
}

}  // namespace

Status RuleBasedDynamicObstacle::Reset(
    const std::vector<DynamicObstacleState>& initial_states) {
  states_.clear();
  for (const auto& state : initial_states) {
    states_[state.id] = state;
  }
  has_states_ = true;
  last_update_time_sec_ = 0.0;
  return {StatusCode::kOk, ""};
}

Status RuleBasedDynamicObstacle::SetRuleLine(
    const std::string& id,
    const Vector2D& direction_unit) {
  if (id.empty()) {
    return {StatusCode::kInvalidInput, "障碍物 ID 为空"};
  }
  const Vector2D normalized = Normalize(direction_unit);
  if (VectorNorm(normalized) <= 1e-9) {
    return {StatusCode::kInvalidInput, "直线规则方向无效"};
  }
  line_rules_[id] = {normalized};
  bounce_rules_.erase(id);
  return {StatusCode::kOk, ""};
}

Status RuleBasedDynamicObstacle::SetRuleBounce(
    const std::string& id,
    const Pose2D& start,
    const Pose2D& end) {
  if (id.empty()) {
    return {StatusCode::kInvalidInput, "障碍物 ID 为空"};
  }
  if (start.x == end.x && start.y == end.y) {
    return {StatusCode::kInvalidInput, "往返规则端点无效"};
  }
  bounce_rules_[id] = {start, end, true};
  line_rules_.erase(id);
  return {StatusCode::kOk, ""};
}

Status RuleBasedDynamicObstacle::Update(TimeSec current_time_sec) {
  if (!has_states_) {
    return {StatusCode::kNotReady, "未初始化障碍物状态"};
  }
  if (current_time_sec < last_update_time_sec_) {
    return {StatusCode::kInvalidInput, "时间回退"};
  }

  const double dt = current_time_sec - last_update_time_sec_;
  last_update_time_sec_ = current_time_sec;

  for (auto& [id, state] : states_) {
    const auto line_it = line_rules_.find(id);
    const auto bounce_it = bounce_rules_.find(id);

    if (line_it != line_rules_.end()) {
      const auto& rule = line_it->second;
      const double speed = VectorNorm(state.velocity);
      state.velocity = {rule.direction_unit.x * speed,
                        rule.direction_unit.y * speed};
      state.pose.x += state.velocity.x * dt;
      state.pose.y += state.velocity.y * dt;
    } else if (bounce_it != bounce_rules_.end()) {
      auto& rule = bounce_it->second;
      const double speed = VectorNorm(state.velocity);
      const Pose2D& from = rule.forward ? rule.start : rule.end;
      const Pose2D& to = rule.forward ? rule.end : rule.start;
      const Vector2D dir = Normalize({to.x - from.x, to.y - from.y});
      state.velocity = {dir.x * speed, dir.y * speed};
      const double distance_to_end =
          VectorNorm({to.x - state.pose.x, to.y - state.pose.y});
      if (distance_to_end <= speed * dt + 1e-6) {
        state.pose.x = to.x;
        state.pose.y = to.y;
        rule.forward = !rule.forward;
      } else {
        state.pose.x += state.velocity.x * dt;
        state.pose.y += state.velocity.y * dt;
      }
    } else {
      // 无规则：按当前速度与加速度更新
      state.pose.x += state.velocity.x * dt + 0.5 * state.acceleration.x * dt * dt;
      state.pose.y += state.velocity.y * dt + 0.5 * state.acceleration.y * dt * dt;
      state.velocity.x += state.acceleration.x * dt;
      state.velocity.y += state.acceleration.y * dt;
    }
  }

  return {StatusCode::kOk, ""};
}

Status RuleBasedDynamicObstacle::Predict(
    TimeSec start_time_sec,
    TimeSec horizon_sec,
    TimeSec time_step_sec,
    DynamicObstaclePrediction* prediction) const {
  if (!prediction) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!has_states_) {
    return {StatusCode::kNotReady, "未初始化障碍物状态"};
  }
  if (horizon_sec <= 0.0 || time_step_sec <= 0.0) {
    return {StatusCode::kInvalidInput, "预测参数无效"};
  }

  prediction->start_time_sec = start_time_sec;
  prediction->time_step_sec = time_step_sec;
  prediction->trajectories.clear();

  const int steps = static_cast<int>(std::ceil(horizon_sec / time_step_sec));
  for (const auto& [id, state] : states_) {
    DynamicObstacleTrajectory traj{};
    traj.id = id;
    traj.shape = state.shape;
    traj.points.reserve(steps + 1);

    DynamicObstacleState temp = state;
    bool bounce_forward = true;
    auto bounce_it = bounce_rules_.find(id);
    if (bounce_it != bounce_rules_.end()) {
      bounce_forward = bounce_it->second.forward;
    }

    for (int i = 0; i <= steps; ++i) {
      const TimeSec t = start_time_sec + i * time_step_sec;
      TrajectoryPoint point{};
      point.state.pose = temp.pose;
      point.state.velocity = temp.velocity;
      point.state.acceleration = temp.acceleration;
      point.state.time_sec = t;
      traj.points.push_back(point);

      const auto line_it = line_rules_.find(id);
      if (line_it != line_rules_.end()) {
        const auto& rule = line_it->second;
        const double speed = VectorNorm(temp.velocity);
        temp.velocity = {rule.direction_unit.x * speed,
                         rule.direction_unit.y * speed};
        temp.pose.x += temp.velocity.x * time_step_sec;
        temp.pose.y += temp.velocity.y * time_step_sec;
        continue;
      }

      if (bounce_it != bounce_rules_.end()) {
        const auto& rule = bounce_it->second;
        const Pose2D& from = bounce_forward ? rule.start : rule.end;
        const Pose2D& to = bounce_forward ? rule.end : rule.start;
        const Vector2D dir = Normalize({to.x - from.x, to.y - from.y});
        const double speed = VectorNorm(temp.velocity);
        temp.velocity = {dir.x * speed, dir.y * speed};
        const double distance_to_end =
            VectorNorm({to.x - temp.pose.x, to.y - temp.pose.y});
        if (distance_to_end <= speed * time_step_sec + 1e-6) {
          temp.pose.x = to.x;
          temp.pose.y = to.y;
          bounce_forward = !bounce_forward;
        } else {
          temp.pose.x += temp.velocity.x * time_step_sec;
          temp.pose.y += temp.velocity.y * time_step_sec;
        }
        continue;
      }

      temp.pose.x += temp.velocity.x * time_step_sec +
          0.5 * temp.acceleration.x * time_step_sec * time_step_sec;
      temp.pose.y += temp.velocity.y * time_step_sec +
          0.5 * temp.acceleration.y * time_step_sec * time_step_sec;
      temp.velocity.x += temp.acceleration.x * time_step_sec;
      temp.velocity.y += temp.acceleration.y * time_step_sec;
    }

    prediction->trajectories.push_back(std::move(traj));
  }

  return {StatusCode::kOk, ""};
}

Status RuleBasedDynamicObstacle::GetStates(
    std::vector<DynamicObstacleState>* states) const {
  if (!states) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!has_states_) {
    return {StatusCode::kNotReady, "未初始化障碍物状态"};
  }
  states->clear();
  states->reserve(states_.size());
  for (const auto& [id, state] : states_) {
    states->push_back(state);
  }
  return {StatusCode::kOk, ""};
}

}  // namespace pathlearn
