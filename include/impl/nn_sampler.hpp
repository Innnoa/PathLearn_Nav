#pragma once

#include <string>
#include <vector>

#include "core/types.hpp"

namespace pathlearn {

// 轻量神经网络引导采样器：基于文本权重做前向推理
class NnSampler {
 public:
  struct Result {
    bool valid = false;
    Pose2D target{};
  };

  NnSampler() = default;

  // 加载模型（文本格式），重复调用时若路径相同且已加载则直接复用
  Status LoadModel(const std::string& model_path);

  // 基于当前状态、目标和局部障碍生成引导采样点
  Result SuggestSample(
      const Pose2D& current,
      const Pose2D& goal,
      const std::vector<CircleObstacle>& static_obstacles,
      double sample_distance,
      const Bounds2D& bounds) const;

  bool IsReady() const { return loaded_; }
  const std::string& LoadedPath() const { return loaded_path_; }

 private:
  struct Layer {
    int in_dim = 0;
    int out_dim = 0;
    std::vector<double> weights;  // 行主序：out_dim x in_dim
    std::vector<double> bias;     // out_dim
  };

  bool loaded_ = false;
  std::string loaded_path_;
  std::vector<Layer> layers_;

  static double Relu(double value);
  static std::vector<double> BuildInput(
      const Pose2D& current,
      const Pose2D& goal,
      const std::vector<CircleObstacle>& static_obstacles);
};

}  // namespace pathlearn
