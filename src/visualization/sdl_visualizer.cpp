#include "impl/sdl_visualizer.hpp"

#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <vector>

#if PATHLEARN_WITH_SDL_TTF
#include <SDL2/SDL_ttf.h>
#endif

namespace pathlearn {
namespace {

SDL_Color MakeColor(Uint8 r, Uint8 g, Uint8 b, Uint8 a = 255) {
  SDL_Color color{};
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

SDL_Color ToSdlColor(const VisualColor& color) {
  return MakeColor(color.r, color.g, color.b, color.a);
}

bool IsSameBounds(const Bounds2D& lhs, const Bounds2D& rhs) {
  return std::fabs(lhs.min_x - rhs.min_x) < 1e-9 &&
      std::fabs(lhs.max_x - rhs.max_x) < 1e-9 &&
      std::fabs(lhs.min_y - rhs.min_y) < 1e-9 &&
      std::fabs(lhs.max_y - rhs.max_y) < 1e-9;
}

std::vector<std::string> CandidateFontPaths() {
  return {
      "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/noto-cjk/NotoSansCJK-Medium.ttc",
      "/usr/share/fonts/noto/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
      "/usr/share/fonts/noto/NotoSans-Regular.ttf",
      "/usr/share/fonts/TTF/DejaVuSans.ttf",
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
  };
}

std::vector<std::string> SplitLines(const std::string& text) {
  std::vector<std::string> lines;
  std::string current;
  for (char ch : text) {
    if (ch == '\n') {
      lines.push_back(current);
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  if (!current.empty()) {
    lines.push_back(current);
  }
  return lines;
}

}  // namespace

SdlVisualizer::~SdlVisualizer() {
  Shutdown();
}

Status SdlVisualizer::Initialize(
    const VisualizationConfig& config,
    const MapInfo& map_info,
    const std::vector<CircleObstacle>& static_obstacles) {
  if (initialized_) {
    const bool same_window = config.width_px == config_.width_px &&
        config.height_px == config_.height_px &&
        config.margin_px == config_.margin_px;
    const bool same_bounds = IsSameBounds(map_info.bounds, map_info_.bounds);
    if (!same_window || !same_bounds) {
      Shutdown();
    } else {
      config_ = config;
      map_info_ = map_info;
      static_obstacles_ = static_obstacles;
      ComputeTransform();
      closed_ = false;
      EnsureTtfReady();
      return {StatusCode::kOk, ""};
    }
  }

  config_ = config;
  map_info_ = map_info;
  static_obstacles_ = static_obstacles;

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    return {StatusCode::kInternalError, SDL_GetError()};
  }

  window_ = SDL_CreateWindow(
      "PathLearn Nav",
      SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED,
      config_.width_px,
      config_.height_px,
      SDL_WINDOW_SHOWN);
  if (!window_) {
    SDL_Quit();
    return {StatusCode::kInternalError, SDL_GetError()};
  }

  renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
  if (!renderer_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
    SDL_Quit();
    return {StatusCode::kInternalError, SDL_GetError()};
  }

  SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
  ComputeTransform();
  initialized_ = true;
  closed_ = false;
  EnsureTtfReady();
  return {StatusCode::kOk, ""};
}

Status SdlVisualizer::Render(const VisualizationFrame& frame) {
  if (!initialized_ || !renderer_) {
    return {StatusCode::kNotReady, "渲染器未初始化"};
  }

  PumpEvents(nullptr);
  if (closed_) {
    return {StatusCode::kNotReady, "窗口已关闭"};
  }
  DrawFrameInternal(frame, true, true);
  return {StatusCode::kOk, ""};
}

void SdlVisualizer::Shutdown() {
  ReleaseFonts();

  if (renderer_) {
    SDL_DestroyRenderer(renderer_);
    renderer_ = nullptr;
  }
  if (window_) {
    SDL_DestroyWindow(window_);
    window_ = nullptr;
  }
  if (initialized_) {
    SDL_Quit();
    initialized_ = false;
  }
}

void SdlVisualizer::WaitForClose() {
  if (!initialized_ || !renderer_) {
    return;
  }
  SDL_FlushEvents(SDL_FIRSTEVENT, SDL_LASTEVENT);
  SDL_PumpEvents();
  closed_ = false;
  if (window_) {
    SDL_ShowWindow(window_);
    SDL_RaiseWindow(window_);
  }
  while (!closed_) {
    SDL_Event event{};
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_KEYDOWN &&
          event.key.keysym.sym == SDLK_ESCAPE) {
        closed_ = true;
      }
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE) {
        closed_ = true;
      }
    }
    SDL_Delay(16);
  }
}

void SdlVisualizer::SetHudText(
    const std::string& title,
    const std::string& subtitle) {
  hud_title_ = title;
  hud_subtitle_ = subtitle;
}

void SdlVisualizer::ClearOverlayLayers() {
  overlay_layers_.clear();
}

void SdlVisualizer::AddOverlayLayer(const VisualOverlayLayer& layer) {
  overlay_layers_.push_back(layer);
}

Status SdlVisualizer::WaitForControl(
    const VisualizationFrame& frame,
    bool allow_continue,
    EvalControlAction* action,
    const std::string& prompt) {
  if (!action) {
    return {StatusCode::kInvalidInput, "动作输出指针为空"};
  }
  if (!initialized_ || !renderer_) {
    return {StatusCode::kNotReady, "渲染器未初始化"};
  }
  *action = EvalControlAction::kStop;
  closed_ = false;

  const std::vector<std::string> prompt_lines = SplitLines(prompt);
  const int line_height = ttf_ready_ ? 22 : 18;
  const int content_h = static_cast<int>(prompt_lines.size()) * line_height;
  const int panel_h = std::clamp(22 + content_h, 90, 240);
  const int button_h = 48;
  const int button_y = config_.height_px - button_h - 16;
  const int panel_bottom = button_y - 10;
  const int panel_y = std::max(20, panel_bottom - panel_h);
  SDL_Rect panel = {20, panel_y, config_.width_px - 40, panel_h};

  SDL_Rect continue_rect{};
  SDL_Rect stop_rect{};
  SDL_Rect close_rect{};

  if (allow_continue) {
    const int button_w = 180;
    const int gap = 20;
    const int total_w = button_w * 2 + gap;
    const int start_x = (config_.width_px - total_w) / 2;
    continue_rect = {start_x, button_y, button_w, button_h};
    stop_rect = {start_x + button_w + gap, button_y, button_w, button_h};
  } else {
    const int button_w = 220;
    close_rect = {
        (config_.width_px - button_w) / 2,
        button_y,
        button_w,
        button_h};
  }

  while (!closed_) {
    std::vector<SDL_Event> events;
    PumpEvents(&events);
    for (const auto& event : events) {
      if (event.type == SDL_MOUSEBUTTONDOWN &&
          event.button.button == SDL_BUTTON_LEFT) {
        const int mx = event.button.x;
        const int my = event.button.y;
        if (allow_continue) {
          if (IsPointInRect(mx, my, continue_rect)) {
            *action = EvalControlAction::kContinue;
            return {StatusCode::kOk, ""};
          }
          if (IsPointInRect(mx, my, stop_rect)) {
            *action = EvalControlAction::kStop;
            return {StatusCode::kOk, ""};
          }
        } else if (IsPointInRect(mx, my, close_rect)) {
          *action = EvalControlAction::kStop;
          return {StatusCode::kOk, ""};
        }
      }
    }

    if (closed_) {
      *action = EvalControlAction::kStop;
      return {StatusCode::kOk, ""};
    }

    DrawFrameInternal(frame, false, false);

    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 220);
    SDL_RenderFillRect(renderer_, &panel);
    SDL_SetRenderDrawColor(renderer_, 60, 60, 60, 220);
    SDL_RenderDrawRect(renderer_, &panel);

    if (!prompt_lines.empty()) {
      int text_y = panel.y + 10;
      for (const auto& line : prompt_lines) {
        DrawText(
            line,
            panel.x + 12,
            text_y,
            MakeColor(20, 20, 20, 255),
            false);
        text_y += line_height;
      }
    }

    if (allow_continue) {
      DrawButton(
          continue_rect,
          MakeColor(60, 150, 70, 220),
          MakeColor(30, 80, 40, 240),
          "进行下一轮");
      DrawButton(
          stop_rect,
          MakeColor(180, 80, 70, 220),
          MakeColor(110, 40, 35, 240),
          "结束评测");
    } else {
      DrawButton(
          close_rect,
          MakeColor(70, 90, 160, 220),
          MakeColor(40, 55, 105, 240),
          "关闭窗口");
    }

    SDL_RenderPresent(renderer_);
    SDL_Delay(16);
  }

  *action = EvalControlAction::kStop;
  return {StatusCode::kOk, ""};
}

void SdlVisualizer::ComputeTransform() {
  const double world_w = map_info_.bounds.max_x - map_info_.bounds.min_x;
  const double world_h = map_info_.bounds.max_y - map_info_.bounds.min_y;
  const double top_padding = static_cast<double>(config_.margin_px);
  const double bottom_padding = static_cast<double>(config_.margin_px);
  if (world_w <= 0.0 || world_h <= 0.0) {
    scale_ = 1.0;
    offset_x_ = config_.margin_px;
    offset_y_ = config_.height_px - bottom_padding;
    return;
  }

  const double usable_w = std::max(1.0, config_.width_px - 2.0 * config_.margin_px);
  const double usable_h = std::max(
      1.0,
      config_.height_px - top_padding - bottom_padding);
  const double scale_x = usable_w / world_w;
  const double scale_y = usable_h / world_h;
  scale_ = std::min(scale_x, scale_y);

  const double draw_w = world_w * scale_;
  const double draw_h = world_h * scale_;
  offset_x_ = (config_.width_px - draw_w) * 0.5;
  offset_y_ = top_padding + draw_h;
}

std::pair<int, int> SdlVisualizer::WorldToScreen(const Pose2D& pose) const {
  const double dx = (pose.x - map_info_.bounds.min_x) * scale_;
  const double dy = (pose.y - map_info_.bounds.min_y) * scale_;
  const int sx = static_cast<int>(std::round(offset_x_ + dx));
  const int sy = static_cast<int>(std::round(offset_y_ - dy));
  return {sx, sy};
}

bool SdlVisualizer::PumpEvents(std::vector<SDL_Event>* events) {
  SDL_Event event{};
  while (SDL_PollEvent(&event)) {
    if (events) {
      events->push_back(event);
    }
    if (event.type == SDL_QUIT) {
      closed_ = true;
    }
    if (event.type == SDL_KEYDOWN &&
        event.key.keysym.sym == SDLK_ESCAPE) {
      closed_ = true;
    }
  }
  return !closed_;
}

void SdlVisualizer::DrawFrameInternal(
    const VisualizationFrame& frame,
    bool apply_delay,
    bool present_frame) {
  ComputeTransform();
  SDL_SetRenderDrawColor(renderer_, 245, 245, 245, 255);
  SDL_RenderClear(renderer_);

  DrawBounds(MakeColor(60, 60, 60));
  DrawStaticObstacles();
  for (const auto& layer : overlay_layers_) {
    DrawDebugEdges(layer.planner_debug_edges, ToSdlColor(layer.tree_color));
    DrawTrajectory(layer.planned_trajectory, ToSdlColor(layer.planned_color));
    DrawTrajectory(layer.executed_trajectory, ToSdlColor(layer.executed_color));
  }
  DrawDebugEdges(frame.planner_debug_edges, MakeColor(255, 120, 190, 120));
  DrawTrajectory(frame.planned_trajectory, MakeColor(60, 120, 220, 220));
  DrawTrajectory(frame.executed_trajectory, MakeColor(40, 160, 80, 240));
  DrawDynamicObstacles(frame.dynamic_obstacles);
  DrawCircle(frame.current_state.pose, 0.2, MakeColor(255, 140, 0, 220));
  DrawCircle(frame.goal, 0.25, MakeColor(20, 20, 20, 240));
  DrawHud();

  if (present_frame) {
    SDL_RenderPresent(renderer_);
  }
  if (apply_delay && config_.target_fps > 0) {
    const int delay_ms = static_cast<int>(1000 / config_.target_fps);
    SDL_Delay(delay_ms);
  }
}

void SdlVisualizer::DrawCircle(
    const Pose2D& center,
    double radius,
    const SDL_Color& color) {
  if (radius <= 0.0) {
    return;
  }
  const auto [cx, cy] = WorldToScreen(center);
  const int r = std::max(1, static_cast<int>(std::round(radius * scale_)));

  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  const int segments = 36;
  const double step = 2.0 * std::acos(-1.0) / segments;
  for (int i = 0; i < segments; ++i) {
    const double a1 = i * step;
    const double a2 = (i + 1) * step;
    const int x1 = static_cast<int>(std::round(cx + r * std::cos(a1)));
    const int y1 = static_cast<int>(std::round(cy + r * std::sin(a1)));
    const int x2 = static_cast<int>(std::round(cx + r * std::cos(a2)));
    const int y2 = static_cast<int>(std::round(cy + r * std::sin(a2)));
    SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
  }
}

void SdlVisualizer::DrawTrajectory(
    const Trajectory& trajectory,
    const SDL_Color& color) {
  if (trajectory.points.size() < 2) {
    return;
  }
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto& prev = trajectory.points[i - 1].state.pose;
    const auto& curr = trajectory.points[i].state.pose;
    const auto [x1, y1] = WorldToScreen(prev);
    const auto [x2, y2] = WorldToScreen(curr);
    SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
  }
}

void SdlVisualizer::DrawDebugEdges(
    const std::vector<LineSegment2D>& edges,
    const SDL_Color& color) {
  if (edges.empty()) {
    return;
  }
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  for (const auto& edge : edges) {
    const auto [x1, y1] = WorldToScreen(edge.from);
    const auto [x2, y2] = WorldToScreen(edge.to);
    SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
  }
}

void SdlVisualizer::DrawBounds(const SDL_Color& color) {
  SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
  Pose2D bl{map_info_.bounds.min_x, map_info_.bounds.min_y, 0.0};
  Pose2D br{map_info_.bounds.max_x, map_info_.bounds.min_y, 0.0};
  Pose2D tr{map_info_.bounds.max_x, map_info_.bounds.max_y, 0.0};
  Pose2D tl{map_info_.bounds.min_x, map_info_.bounds.max_y, 0.0};
  const auto [x1, y1] = WorldToScreen(bl);
  const auto [x2, y2] = WorldToScreen(br);
  const auto [x3, y3] = WorldToScreen(tr);
  const auto [x4, y4] = WorldToScreen(tl);
  SDL_RenderDrawLine(renderer_, x1, y1, x2, y2);
  SDL_RenderDrawLine(renderer_, x2, y2, x3, y3);
  SDL_RenderDrawLine(renderer_, x3, y3, x4, y4);
  SDL_RenderDrawLine(renderer_, x4, y4, x1, y1);
}

void SdlVisualizer::DrawStaticObstacles() {
  const SDL_Color color = MakeColor(120, 120, 120, 220);
  for (const auto& obstacle : static_obstacles_) {
    DrawCircle(obstacle.center, obstacle.radius, color);
  }
}

void SdlVisualizer::DrawDynamicObstacles(
    const std::vector<DynamicObstacleState>& obstacles) {
  const SDL_Color color = MakeColor(200, 70, 70, 220);
  for (const auto& obstacle : obstacles) {
    DrawCircle(obstacle.pose, obstacle.shape.radius, color);
  }
}

void SdlVisualizer::DrawHud() {
  if (!window_) {
    return;
  }
  std::string title = "PathLearn Nav";
  if (!hud_title_.empty() || !hud_subtitle_.empty()) {
    title = hud_subtitle_.empty()
        ? hud_title_
        : hud_title_ + " | " + hud_subtitle_;
  }
  SDL_SetWindowTitle(window_, title.c_str());
}

void SdlVisualizer::DrawText(
    const std::string& text,
    int x,
    int y,
    const SDL_Color& color,
    bool use_title_font) {
  if (!ttf_ready_ || text.empty()) {
    return;
  }
#if PATHLEARN_WITH_SDL_TTF
  TTF_Font* font = use_title_font ? font_title_ : font_body_;
  if (!font) {
    return;
  }
  SDL_Surface* surface = TTF_RenderUTF8_Blended(font, text.c_str(), color);
  if (!surface) {
    return;
  }
  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
  if (!texture) {
    SDL_FreeSurface(surface);
    return;
  }
  SDL_Rect dst = {x, y, surface->w, surface->h};
  SDL_RenderCopy(renderer_, texture, nullptr, &dst);
  SDL_DestroyTexture(texture);
  SDL_FreeSurface(surface);
#else
  (void)x;
  (void)y;
  (void)color;
  (void)use_title_font;
#endif
}

void SdlVisualizer::DrawButton(
    const SDL_Rect& rect,
    const SDL_Color& fill,
    const SDL_Color& border,
    const std::string& label) {
  SDL_SetRenderDrawColor(renderer_, fill.r, fill.g, fill.b, fill.a);
  SDL_RenderFillRect(renderer_, &rect);
  SDL_SetRenderDrawColor(renderer_, border.r, border.g, border.b, border.a);
  SDL_RenderDrawRect(renderer_, &rect);
  if (!ttf_ready_) {
    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 230);
    const int cx = rect.x + rect.w / 2;
    const int cy = rect.y + rect.h / 2;
    if (label.find("下一轮") != std::string::npos) {
      SDL_Point pts[4] = {
          {cx - 10, cy - 12},
          {cx - 10, cy + 12},
          {cx + 12, cy},
          {cx - 10, cy - 12},
      };
      SDL_RenderDrawLines(renderer_, pts, 4);
    } else if (label.find("结束") != std::string::npos ||
               label.find("关闭") != std::string::npos) {
      SDL_RenderDrawLine(renderer_, cx - 10, cy - 10, cx + 10, cy + 10);
      SDL_RenderDrawLine(renderer_, cx + 10, cy - 10, cx - 10, cy + 10);
    } else {
      SDL_RenderDrawLine(renderer_, cx - 12, cy, cx + 12, cy);
    }
    return;
  }
#if PATHLEARN_WITH_SDL_TTF
  if (!font_body_ || label.empty()) {
    return;
  }
  int text_w = 0;
  int text_h = 0;
  if (TTF_SizeUTF8(font_body_, label.c_str(), &text_w, &text_h) != 0) {
    return;
  }
  const int tx = rect.x + std::max(0, (rect.w - text_w) / 2);
  const int ty = rect.y + std::max(0, (rect.h - text_h) / 2);
  DrawText(label, tx, ty, MakeColor(255, 255, 255, 255), false);
#endif
}

bool SdlVisualizer::IsPointInRect(int x, int y, const SDL_Rect& rect) const {
  return x >= rect.x && x <= rect.x + rect.w &&
      y >= rect.y && y <= rect.y + rect.h;
}

void SdlVisualizer::EnsureTtfReady() {
  ttf_ready_ = false;
#if PATHLEARN_WITH_SDL_TTF
  if (TTF_WasInit() == 0 && TTF_Init() != 0) {
    return;
  }

  std::string chosen_path;
  for (const auto& path : CandidateFontPaths()) {
    if (std::filesystem::exists(path)) {
      chosen_path = path;
      break;
    }
  }
  if (chosen_path.empty()) {
    return;
  }

  font_title_ = TTF_OpenFont(chosen_path.c_str(), 20);
  font_body_ = TTF_OpenFont(chosen_path.c_str(), 16);
  if (!font_title_ || !font_body_) {
    ReleaseFonts();
    return;
  }
  ttf_ready_ = true;
#endif
}

void SdlVisualizer::ReleaseFonts() {
#if PATHLEARN_WITH_SDL_TTF
  if (font_title_) {
    TTF_CloseFont(font_title_);
    font_title_ = nullptr;
  }
  if (font_body_) {
    TTF_CloseFont(font_body_);
    font_body_ = nullptr;
  }
  if (TTF_WasInit()) {
    TTF_Quit();
  }
#endif
  ttf_ready_ = false;
}

}  // namespace pathlearn
