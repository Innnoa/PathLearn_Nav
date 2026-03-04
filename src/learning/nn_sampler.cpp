#include "impl/nn_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace pathlearn {
namespace {

constexpr int kExpectedInputDim = 8;

double Distance2D(const Pose2D& a, const Pose2D& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool ReadNumbers(
    std::istream& input,
    int expected,
    std::vector<double>* out) {
  if (!out || expected < 0) {
    return false;
  }
  out->clear();
  out->reserve(static_cast<size_t>(expected));
  for (int i = 0; i < expected; ++i) {
    double value = 0.0;
    if (!(input >> value)) {
      return false;
    }
    out->push_back(value);
  }
  return true;
}

}  // namespace

double NnSampler::Relu(double value) {
  return value > 0.0 ? value : 0.0;
}

std::vector<double> NnSampler::BuildInput(
    const Pose2D& current,
    const Pose2D& goal,
    const std::vector<CircleObstacle>& static_obstacles) {
  std::vector<double> input;
  input.reserve(static_cast<size_t>(kExpectedInputDim));

  const double dx_goal = goal.x - current.x;
  const double dy_goal = goal.y - current.y;
  const double distance_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
  const double norm = distance_goal > 1e-9 ? distance_goal : 1.0;

  input.push_back(dx_goal / norm);
  input.push_back(dy_goal / norm);
  input.push_back(dx_goal / 100.0);
  input.push_back(dy_goal / 100.0);

  std::vector<std::pair<double, const CircleObstacle*>> nearest;
  nearest.reserve(static_obstacles.size());
  const Pose2D current_pose{current.x, current.y, 0.0};
  for (const auto& obstacle : static_obstacles) {
    nearest.push_back({Distance2D(current_pose, obstacle.center), &obstacle});
  }
  std::sort(nearest.begin(), nearest.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.first < rhs.first;
  });

  for (int i = 0; i < 2; ++i) {
    if (i < static_cast<int>(nearest.size()) && nearest[i].second) {
      const auto* obstacle = nearest[i].second;
      input.push_back((obstacle->center.x - current.x) / 100.0);
      input.push_back((obstacle->center.y - current.y) / 100.0);
    } else {
      input.push_back(0.0);
      input.push_back(0.0);
    }
  }

  if (static_cast<int>(input.size()) < kExpectedInputDim) {
    input.resize(static_cast<size_t>(kExpectedInputDim), 0.0);
  }
  return input;
}

Status NnSampler::LoadModel(const std::string& model_path) {
  if (model_path.empty()) {
    loaded_ = false;
    loaded_path_.clear();
    layers_.clear();
    return {StatusCode::kInvalidInput, "模型路径为空"};
  }
  if (loaded_ && model_path == loaded_path_) {
    return {StatusCode::kOk, ""};
  }

  std::ifstream input(model_path);
  if (!input.is_open()) {
    loaded_ = false;
    loaded_path_.clear();
    layers_.clear();
    return {StatusCode::kNotReady, "无法打开神经网络模型文件"};
  }

  std::string magic;
  int version = 0;
  int num_layers = 0;
  input >> magic >> version >> num_layers;
  if (!input || magic != "PATHLEARN_NN" || version != 1 || num_layers <= 0) {
    loaded_ = false;
    loaded_path_.clear();
    layers_.clear();
    return {StatusCode::kInvalidInput, "神经网络模型格式无效"};
  }

  std::vector<Layer> parsed_layers;
  parsed_layers.reserve(static_cast<size_t>(num_layers));
  for (int layer_index = 0; layer_index < num_layers; ++layer_index) {
    std::string tag;
    int in_dim = 0;
    int out_dim = 0;
    input >> tag >> in_dim >> out_dim;
    if (!input || tag != "layer" || in_dim <= 0 || out_dim <= 0) {
      loaded_ = false;
      loaded_path_.clear();
      layers_.clear();
      return {StatusCode::kInvalidInput, "神经网络层定义无效"};
    }

    Layer layer;
    layer.in_dim = in_dim;
    layer.out_dim = out_dim;
    const int total_weights = in_dim * out_dim;
    if (!ReadNumbers(input, total_weights, &layer.weights)) {
      loaded_ = false;
      loaded_path_.clear();
      layers_.clear();
      return {StatusCode::kInvalidInput, "神经网络权重数量不足"};
    }
    if (!ReadNumbers(input, out_dim, &layer.bias)) {
      loaded_ = false;
      loaded_path_.clear();
      layers_.clear();
      return {StatusCode::kInvalidInput, "神经网络偏置数量不足"};
    }
    parsed_layers.push_back(std::move(layer));
  }

  if (parsed_layers.front().in_dim != kExpectedInputDim) {
    loaded_ = false;
    loaded_path_.clear();
    layers_.clear();
    return {StatusCode::kInvalidInput, "神经网络输入维度不匹配"};
  }
  if (parsed_layers.back().out_dim != 2) {
    loaded_ = false;
    loaded_path_.clear();
    layers_.clear();
    return {StatusCode::kInvalidInput, "神经网络输出维度需为2"};
  }

  for (size_t i = 1; i < parsed_layers.size(); ++i) {
    if (parsed_layers[i - 1].out_dim != parsed_layers[i].in_dim) {
      loaded_ = false;
      loaded_path_.clear();
      layers_.clear();
      return {StatusCode::kInvalidInput, "神经网络层连接维度不一致"};
    }
  }

  layers_ = std::move(parsed_layers);
  loaded_ = true;
  loaded_path_ = model_path;
  return {StatusCode::kOk, ""};
}

NnSampler::Result NnSampler::SuggestSample(
    const Pose2D& current,
    const Pose2D& goal,
    const std::vector<CircleObstacle>& static_obstacles,
    double sample_distance,
    const Bounds2D& bounds) const {
  Result result{};
  if (!loaded_ || layers_.empty()) {
    return result;
  }
  if (sample_distance <= 1e-6) {
    return result;
  }

  std::vector<double> values = BuildInput(current, goal, static_obstacles);
  for (size_t layer_index = 0; layer_index < layers_.size(); ++layer_index) {
    const Layer& layer = layers_[layer_index];
    if (static_cast<int>(values.size()) != layer.in_dim) {
      return result;
    }

    std::vector<double> next(static_cast<size_t>(layer.out_dim), 0.0);
    for (int out_index = 0; out_index < layer.out_dim; ++out_index) {
      double sum = layer.bias[static_cast<size_t>(out_index)];
      const size_t row_start = static_cast<size_t>(out_index * layer.in_dim);
      for (int in_index = 0; in_index < layer.in_dim; ++in_index) {
        sum += layer.weights[row_start + static_cast<size_t>(in_index)] *
            values[static_cast<size_t>(in_index)];
      }
      const bool is_last = layer_index + 1 == layers_.size();
      next[static_cast<size_t>(out_index)] = is_last ? sum : Relu(sum);
    }
    values = std::move(next);
  }

  if (values.size() != 2) {
    return result;
  }

  const double vx = values[0];
  const double vy = values[1];
  const double norm = std::sqrt(vx * vx + vy * vy);
  if (!std::isfinite(norm) || norm <= 1e-9) {
    return result;
  }

  result.target = current;
  result.target.x += sample_distance * vx / norm;
  result.target.y += sample_distance * vy / norm;
  result.target.x = std::clamp(result.target.x, bounds.min_x, bounds.max_x);
  result.target.y = std::clamp(result.target.y, bounds.min_y, bounds.max_y);
  result.valid = true;
  return result;
}

}  // namespace pathlearn
