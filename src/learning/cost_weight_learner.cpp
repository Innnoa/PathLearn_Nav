#include "impl/cost_weight_learner.hpp"

#include <algorithm>

namespace pathlearn {
namespace {

constexpr double kMinWeight = 0.2;
constexpr double kMaxWeight = 20.0;
constexpr double kLengthGain = 0.15;
constexpr double kSafetyGain = 0.25;
constexpr double kSafetyDistanceGain = 0.2;
constexpr double kTimeGain = 0.15;
constexpr double kStagnationTol = 1e-6;
constexpr double kStagnationGain = 0.02;
constexpr double kStagnationSafetyGain = 0.03;
constexpr double kMinExploration = 1.0;
constexpr double kMaxExploration = 8.0;
constexpr double kExplorationGain = 0.5;
constexpr double kExplorationDecay = 0.02;

double ClampWeight(double value) {
  return std::clamp(value, kMinWeight, kMaxWeight);
}

double ClampExploration(double value) {
  return std::clamp(value, kMinExploration, kMaxExploration);
}

}  // namespace

Status CostWeightLearner::Initialize(const CostWeights& initial_weights) {
  weights_ = initial_weights;
  return {StatusCode::kOk, ""};
}

Status CostWeightLearner::Update(
    const LearningInput& input,
    LearningOutput* output) {
  if (!output) {
    return {StatusCode::kInvalidInput, "输出指针为空"};
  }
  if (!enabled_) {
    output->updated_weights = input.current_weights;
    return {StatusCode::kOk, ""};
  }

  CostWeights next = input.current_weights;

  if (input.baseline_metrics.avg_path_length > 0.0) {
    if (input.metrics.avg_path_length >
        input.baseline_metrics.avg_path_length + kStagnationTol) {
      next.length_weight =
          ClampWeight(next.length_weight * (1.0 + kLengthGain));
    } else if (input.metrics.avg_path_length >=
               input.baseline_metrics.avg_path_length - kStagnationTol) {
      next.length_weight =
          ClampWeight(next.length_weight * (1.0 + kStagnationGain));
    }
  }

  if (input.baseline_metrics.collision_rate >= 0.0) {
    if (input.metrics.collision_rate >
        input.baseline_metrics.collision_rate + kStagnationTol) {
      next.safety_weight =
          ClampWeight(next.safety_weight * (1.0 + kSafetyGain));
    } else if (input.metrics.collision_rate >=
               input.baseline_metrics.collision_rate - kStagnationTol) {
      next.safety_weight =
          ClampWeight(next.safety_weight * (1.0 + kStagnationSafetyGain));
    }
  }

  if (input.baseline_metrics.min_safety_distance > 0.0) {
    if (input.metrics.min_safety_distance <
        input.baseline_metrics.min_safety_distance - kStagnationTol) {
      next.safety_weight =
          ClampWeight(next.safety_weight * (1.0 + kSafetyDistanceGain));
    } else if (input.metrics.min_safety_distance <=
               input.baseline_metrics.min_safety_distance + kStagnationTol) {
      next.safety_weight =
          ClampWeight(next.safety_weight * (1.0 + kStagnationSafetyGain));
    }
  }

  if (input.baseline_metrics.avg_exec_time_sec > 0.0) {
    if (input.metrics.avg_exec_time_sec >
        input.baseline_metrics.avg_exec_time_sec + kStagnationTol) {
      next.time_weight =
          ClampWeight(next.time_weight * (1.0 + kTimeGain));
    } else if (input.metrics.avg_exec_time_sec >=
               input.baseline_metrics.avg_exec_time_sec - kStagnationTol) {
      next.time_weight =
          ClampWeight(next.time_weight * (1.0 + kStagnationGain));
    }
  }

  if (input.metrics.success_rate <= kStagnationTol) {
    next.exploration_weight =
        ClampExploration(next.exploration_weight * (1.0 + kExplorationGain));
  } else if (input.baseline_metrics.success_rate > 0.0 &&
             input.metrics.success_rate + kStagnationTol <
                 input.baseline_metrics.success_rate) {
    next.exploration_weight =
        ClampExploration(next.exploration_weight * (1.0 + kExplorationGain));
  } else {
    next.exploration_weight =
        ClampExploration(next.exploration_weight * (1.0 - kExplorationDecay));
  }

  weights_ = next;
  output->updated_weights = next;
  return {StatusCode::kOk, ""};
}

void CostWeightLearner::SetEnabled(bool enabled) {
  enabled_ = enabled;
}

}  // namespace pathlearn
