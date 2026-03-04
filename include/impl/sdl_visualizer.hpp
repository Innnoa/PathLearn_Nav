#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "core/visualization.hpp"

struct SDL_Window;
struct SDL_Renderer;
struct SDL_Color;
struct SDL_Rect;
union SDL_Event;
struct TTF_Font;

namespace pathlearn {

// RGBA 颜色
struct VisualColor {
  std::uint8_t r = 255;
  std::uint8_t g = 255;
  std::uint8_t b = 255;
  std::uint8_t a = 255;
};

// 叠加层：用于显示上一轮/基线轨迹与树
struct VisualOverlayLayer {
  Trajectory planned_trajectory;
  Trajectory executed_trajectory;
  std::vector<LineSegment2D> planner_debug_edges;
  VisualColor planned_color{90, 150, 240, 120};
  VisualColor executed_color{70, 170, 90, 120};
  VisualColor tree_color{140, 140, 210, 90};
};

// 评测交互动作
enum class EvalControlAction {
  kContinue = 0,
  kStop = 1,
};

// 基于 SDL2 的简易可视化
class SdlVisualizer final : public Visualizer {
 public:
  SdlVisualizer() = default;
  ~SdlVisualizer() override;

  Status Initialize(
      const VisualizationConfig& config,
      const MapInfo& map_info,
      const std::vector<CircleObstacle>& static_obstacles) override;
  Status Render(const VisualizationFrame& frame) override;
  void Shutdown() override;
  void WaitForClose() override;

  // 配置 HUD 顶部文字
  void SetHudText(const std::string& title, const std::string& subtitle);

  // 管理叠加层（用于保留上一轮结果）
  void ClearOverlayLayers();
  void AddOverlayLayer(const VisualOverlayLayer& layer);

  // 评测轮次控制：允许继续下一轮或结束评测
  Status WaitForControl(
      const VisualizationFrame& frame,
      bool allow_continue,
      EvalControlAction* action,
      const std::string& prompt);

 private:
  bool initialized_ = false;
  bool closed_ = false;
  VisualizationConfig config_{};
  MapInfo map_info_{};
  std::vector<CircleObstacle> static_obstacles_{};
  double scale_ = 1.0;
  double offset_x_ = 0.0;
  double offset_y_ = 0.0;
  SDL_Window* window_ = nullptr;
  SDL_Renderer* renderer_ = nullptr;
  std::vector<VisualOverlayLayer> overlay_layers_{};
  std::string hud_title_;
  std::string hud_subtitle_;
  bool ttf_ready_ = false;
  TTF_Font* font_title_ = nullptr;
  TTF_Font* font_body_ = nullptr;

  void ComputeTransform();
  bool PumpEvents(std::vector<SDL_Event>* events);
  void DrawFrameInternal(
      const VisualizationFrame& frame,
      bool apply_delay,
      bool present_frame);
  void DrawCircle(const Pose2D& center, double radius, const SDL_Color& color);
  void DrawTrajectory(const Trajectory& trajectory, const SDL_Color& color);
  void DrawDebugEdges(const std::vector<LineSegment2D>& edges,
                      const SDL_Color& color);
  void DrawBounds(const SDL_Color& color);
  void DrawStaticObstacles();
  void DrawDynamicObstacles(const std::vector<DynamicObstacleState>& obstacles);
  void DrawHud();
  void DrawText(
      const std::string& text,
      int x,
      int y,
      const SDL_Color& color,
      bool use_title_font);
  void DrawButton(
      const SDL_Rect& rect,
      const SDL_Color& fill,
      const SDL_Color& border,
      const std::string& label);
  bool IsPointInRect(int x, int y, const SDL_Rect& rect) const;
  void EnsureTtfReady();
  void ReleaseFonts();
  std::pair<int, int> WorldToScreen(const Pose2D& pose) const;
};

}  // namespace pathlearn
