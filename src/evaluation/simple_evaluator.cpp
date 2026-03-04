#include "impl/simple_evaluator.hpp"

namespace pathlearn {
namespace {

bool IsSuccess(const Status& status) {
  return status.code == StatusCode::kOk;
}

}  // namespace

EvaluationSummary SummarizeRuns(
    const std::vector<EvaluationRunResult>& runs) {
  EvaluationSummary summary{};
  if (runs.empty()) {
    return summary;
  }

  summary.scenario_id = runs.front().scenario_id;
  summary.map_name = runs.front().map_name;
  summary.profile = runs.front().profile;
  summary.learning_enabled = runs.front().learning_enabled;

  double total_exec_time = 0.0;
  double total_path_length = 0.0;
  double total_min_safety = 0.0;
  double total_first_solution_iteration = 0.0;
  int success_count = 0;

  for (const auto& run : runs) {
    summary.total_runs += 1;
    if (run.status.code == StatusCode::kCollision) {
      summary.collision_runs += 1;
    }
    if (run.status.code == StatusCode::kNoPath) {
      summary.no_path_runs += 1;
    }
    if (IsSuccess(run.status)) {
      summary.success_runs += 1;
      success_count += 1;
      total_exec_time += run.metrics.avg_exec_time_sec;
      total_path_length += run.metrics.avg_path_length;
      total_min_safety += run.metrics.min_safety_distance;
      total_first_solution_iteration +=
          static_cast<double>(run.first_solution_iteration);
    }
  }

  if (summary.total_runs > 0) {
    summary.success_rate =
        static_cast<double>(summary.success_runs) /
        static_cast<double>(summary.total_runs);
    summary.collision_rate =
        static_cast<double>(summary.collision_runs) /
        static_cast<double>(summary.total_runs);
  }

  // 平均值仅统计成功样本，避免失败样本拉低指标。
  if (success_count > 0) {
    summary.avg_exec_time_sec = total_exec_time / success_count;
    summary.avg_path_length = total_path_length / success_count;
    summary.avg_min_safety_distance = total_min_safety / success_count;
    summary.avg_first_solution_iteration =
        total_first_solution_iteration / success_count;
  }

  return summary;
}

}  // namespace pathlearn
