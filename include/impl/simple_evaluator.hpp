#pragma once

#include <string>
#include <vector>

#include "core/types.hpp"

namespace pathlearn {

// 单次评测结果
struct EvaluationRunResult {
  std::string scenario_id;
  std::string map_name;
  std::string profile;
  unsigned int seed = 0;
  bool learning_enabled = false;
  CostWeights initial_weights{};
  CostWeights updated_weights{};
  Status status{};
  EvaluationMetrics metrics{};
  int first_solution_iteration = 0;
};

// 评测汇总结果
struct EvaluationSummary {
  std::string scenario_id;
  std::string map_name;
  std::string profile;
  bool learning_enabled = false;
  int total_runs = 0;
  int success_runs = 0;
  int collision_runs = 0;
  int no_path_runs = 0;
  double success_rate = 0.0;
  double collision_rate = 0.0;
  double avg_exec_time_sec = 0.0;
  double avg_path_length = 0.0;
  double avg_min_safety_distance = 0.0;
  double avg_first_solution_iteration = 0.0;
};

// 汇总同一组评测结果（默认使用首条结果的元信息）
EvaluationSummary SummarizeRuns(
    const std::vector<EvaluationRunResult>& runs);

}  // namespace pathlearn
