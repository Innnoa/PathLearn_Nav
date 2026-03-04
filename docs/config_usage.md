# 配置文件使用说明
日期：2026-02-19  
执行者：Codex

## 概览
项目已支持通过配置文件管理 `pathlearn_app` 与 `pathlearn_eval` 参数。

相关文档：
- `docs/p0_protocol.md`：P0 冻结基线与评测协议（challenge/pass 两档预算与 CSV 口径）。
- `docs/p1_protocol.md`：P1 学习效果评测协议（baseline vs learning 对照口径）。

优先级：
1. 程序默认值
2. 配置文件
3. 命令行参数（最高）

## 默认配置路径
- `pathlearn_app`：`data/config/pathlearn_app.conf`
- `pathlearn_eval`：`data/config/pathlearn_eval.conf`

## 快速使用
1. 修改 `data/config/pathlearn_app.conf`
2. 直接运行 `./build/pathlearn_app`

或：
1. 修改 `data/config/pathlearn_eval.conf`
2. 直接运行 `./build/pathlearn_eval`

## 指定自定义配置文件
- `./build/pathlearn_app --config your_app.conf`
- `./build/pathlearn_eval --config your_eval.conf`

## 语法规则
- 行格式：`key = value`
- 注释：`#`、`;`、`//`
- 空行会忽略
- 键名大小写不敏感，`-` 与 `_` 视为同一形式
- 路径基准：可用 `base_dir` 指定配置内路径基准目录；未设置时默认项目根目录
- 若 `base_dir` 为相对路径，则按项目根目录解析（推荐 `base_dir = .`）

## `pathlearn_app.conf` 全部可配置项

| 配置键 | 对应命令行 | 说明 |
| --- | --- | --- |
| `base_dir` | - | 配置文件路径基准目录（影响 `map`、`nn_model` 等路径键） |
| `map` | `--map` | 地图文件路径 |
| `no_gui` | `--no-gui` | 是否关闭 GUI（`true/false`） |
| `hold` | `--hold` | 仿真结束后是否保持窗口 |
| `speed` | `--speed` | 机器人速度上限 |
| `fps` | `--fps` | GUI 目标帧率 |
| `time_step` | `--time-step` | 仿真时间步长（秒） |
| `detour` | `--detour` | 规划时域绕行系数 |
| `max_steps` | `--max-steps` | 仿真最大步数，`0` 表示自动估算 |
| `max_wait_steps` | `--max-wait-steps` | 单次运行无路径时允许等待重规划的最大次数，`0` 表示不等待重试 |
| `planner` | `--planner` | 规划器类型：`rrt/astar` |
| `exploration` | `--exploration` | 探索权重（建议范围 `[1,8]`） |
| `astar_resolution` | `--astar-resolution` | A* 网格分辨率，`0` 表示自动估算 |
| `astar_diagonal` | `--astar-diagonal` | A* 是否允许对角邻接（`on/off`） |
| `astar_heuristic_weight` | `--astar-heuristic-weight` | A* 启发权重（>0） |
| `show_tree` | `--show-tree` | 是否显示规划搜索树/扩展边 |
| `tree_limit` | `--tree-limit` | 树边显示上限 |
| `trace` | `--trace` | 规划日志开关：`off/console` |
| `trace_every` | `--trace-every` | 规划日志输出频率 |
| `rrt_max_iters` | `--rrt-max-iters` | RRT 最大迭代预算，`0` 为默认 |
| `nn_guide` | `--nn-guide` | 神经网络学习启发（A*）/引导采样（RRT）开关 |
| `nn_model` | `--nn-model` | 神经网络模型文件路径 |
| `nn_prob` | `--nn-prob` | NN 启发强度/采样概率 `[0,1]` |
| `nn_distance` | `--nn-distance` | NN 建议点偏移距离，`0` 为自动 |
| `learning` | `--learning` | 学习模块开关 |
| `learn_rounds` | `--learn-rounds` | 单次运行的学习轮数 |
| `seed` | `--seed` | 随机种子 |

## `pathlearn_eval.conf` 全部可配置项

| 配置键 | 对应命令行 | 说明 |
| --- | --- | --- |
| `base_dir` | - | 配置文件路径基准目录（影响 `map`、`out_prefix`、`nn_model`） |
| `map` | `--map` | 单地图评测路径，留空则跑内置地图列表 |
| `maps_limit` | `--maps-limit` | 内置地图数量限制，`0` 表示不限制 |
| `profiles` | `--profiles` | 规则集合：`static/line/bounce/all`，可逗号分隔 |
| `mode` | `--mode` | 评测模式：`baseline/learning/both` |
| `out_prefix` | `--out-prefix` | 输出 CSV 前缀 |
| `seeds` | `--seeds` | 评测种子数量 |
| `seed_start` | `--seed-start` | 起始种子 |
| `learn_epochs` | `--learn-epochs` | 学习模式每个种子的学习轮数 |
| `log_every` | `--log-every` | 进度日志间隔 |
| `start_x` | `--start-x` | 起点 x |
| `start_y` | `--start-y` | 起点 y |
| `start_theta` | `--start-theta` | 起点朝向 |
| `goal_x` | `--goal-x` | 终点 x |
| `goal_y` | `--goal-y` | 终点 y |
| `goal_theta` | `--goal-theta` | 终点朝向 |
| `planner` | `--planner` | 规划器类型：`rrt/astar` |
| `speed` | `--speed` | 机器人速度上限 |
| `time_step` | `--time-step` | 仿真时间步长（秒） |
| `detour` | `--detour` | 规划时域绕行系数 |
| `max_steps` | `--max-steps` | 仿真最大步数，`0` 表示自动估算 |
| `max_wait_steps` | `--max-wait-steps` | 单次运行无路径时允许等待重规划的最大次数 |
| `exploration` | `--exploration` | 探索权重（建议范围 `[1,8]`） |
| `rrt_max_iters` | `--rrt-max-iters` | RRT 最大迭代预算，`0` 为默认 |
| `astar_resolution` | `--astar-resolution` | A* 网格分辨率，`0` 表示自动估算 |
| `astar_diagonal` | `--astar-diagonal` | A* 是否允许对角邻接（`on/off`） |
| `astar_heuristic_weight` | `--astar-heuristic-weight` | A* 启发权重（>0） |
| `show_tree` | `--show-tree` | 是否显示规划树 |
| `tree_limit` | `--tree-limit` | 树边显示上限 |
| `trace` | `--trace` | 规划日志开关：`off/console` |
| `trace_every` | `--trace-every` | 规划日志输出频率 |
| `nn_guide` | `--nn-guide` | 神经网络学习启发（A*）/引导采样（RRT）开关 |
| `nn_model` | `--nn-model` | 神经网络模型文件路径 |
| `nn_prob` | `--nn-prob` | NN 启发强度/采样概率 `[0,1]` |
| `nn_distance` | `--nn-distance` | NN 建议点偏移距离，`0` 为自动 |
| `gui` | `--gui` | 是否启用 GUI |
| `gui_step` | `--gui-step` | GUI 是否逐轮手动继续 |
| `gui_width` | `--gui-width` | GUI 窗口宽度 |
| `gui_height` | `--gui-height` | GUI 窗口高度 |
| `fps` | `--fps` | GUI 目标帧率 |

## 备注
- `--config` 不对应配置键，只用于指定配置文件路径。
- 同名配置与命令行同时出现时，命令行优先。
- `base_dir` 仅作用于配置文件中的路径值；命令行传入路径仍按当前工作目录解析。
- `out_prefix` 支持占位符 `{timestamp}`，运行时自动替换为 `YYYYMMDD_HHMMSS`。
