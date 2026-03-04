#pragma once

#include <vector>

#include "core/types.hpp"

namespace pathlearn {

// 可视化配置
struct VisualizationConfig {
  int width_px = 800;
  int height_px = 600;
  int margin_px = 20;
  int target_fps = 30;
};

// 单帧可视化输入
struct VisualizationFrame {
  std::vector<DynamicObstacleState> dynamic_obstacles;
  Trajectory planned_trajectory;
  Trajectory executed_trajectory;
  State2D current_state{};
  Pose2D goal{};
  std::vector<LineSegment2D> planner_debug_edges;
};

// 可视化接口
class Visualizer {
 public:
  virtual ~Visualizer() = default;

  // 初始化渲染器（传入静态信息）
  virtual Status Initialize(
      const VisualizationConfig& config,
      const MapInfo& map_info,
      const std::vector<CircleObstacle>& static_obstacles) = 0;

  // 渲染单帧
  virtual Status Render(const VisualizationFrame& frame) = 0;

  // 关闭并释放资源
  virtual void Shutdown() = 0;

  // 等待用户关闭窗口（默认空实现）
  virtual void WaitForClose() {}
};

}  // namespace pathlearn
