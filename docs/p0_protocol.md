# P0 冻结基线与评测协议
日期：2026-02-20  
执行者：Codex

## 1. 目标
本文件用于冻结论文阶段 `P0` 的基线评测协议，确保后续实验可复现、可比对、可审计。

`P0` 仅覆盖 `baseline`（不启用学习）评测，固定单地图、固定起终点、固定种子区间，并定义两档 RRT 预算：
- `challenge`：`B_challenge = 300000`
- `pass`：`B_pass = 500000`

## 2. 固定输入（协议参数）
以下参数在 `P0` 中视为固定项，除非进入下一阶段并显式升级协议版本。

| 类别 | 固定值 |
| --- | --- |
| 地图 | `data/maps/scene_custom.txt` |
| 规则 | `static` |
| 评测模式 | `baseline` |
| 种子 | `seed_start = 1`, `seeds = 30`（即 1..30） |
| 起点 | `(-33, -33, 0)` |
| 终点 | `(33, 33, 0)` |
| 速度 | `speed = 1.0` |
| 时间步长 | `time_step = 1.0` |
| 绕行系数 | `detour = 4.0` |
| max_steps | `0`（自动估算） |
| max_wait_steps | `4`（无路径时最多等待重规划 4 次） |
| exploration | `8.0` |
| show_tree | `true` |
| tree_limit | `55000` |
| trace | `console` |
| trace_every | `20000` |
| GUI | `false` |

## 3. 配置文件与预算分层
### 3.1 challenge 配置
- 配置文件：`data/config/pathlearn_eval_p0_challenge.conf`
- 关键预算：`rrt_max_iters = 300000`
- 输出前缀：`out_prefix = data/BenchmarK/challenge/challenge`

### 3.2 pass 配置
- 配置文件：`data/config/pathlearn_eval_p0_pass.conf`
- 关键预算：`rrt_max_iters = 500000`
- 输出前缀：`out_prefix = data/BenchmarK/pass/pass`

## 4. 标准运行命令（项目根目录）
### 4.1 运行 challenge
```bash
./build/pathlearn_eval --config data/config/pathlearn_eval_p0_challenge.conf
```

### 4.2 运行 pass
```bash
./build/pathlearn_eval --config data/config/pathlearn_eval_p0_pass.conf
```

## 5. 输出产物（冻结命名）
### 5.1 challenge 产物
- `data/BenchmarK/challenge/challenge_runs.csv`
- `data/BenchmarK/challenge/challenge_summary.csv`
- `data/BenchmarK/challenge/challenge_weights.csv`

### 5.2 pass 产物
- `data/BenchmarK/pass/pass_runs.csv`
- `data/BenchmarK/pass/pass_summary.csv`
- `data/BenchmarK/pass/pass_weights.csv`

## 6. CSV 字段口径
### 6.1 runs.csv（逐次运行记录）
关键字段：
- `status_code`：状态码（`0=成功`, `3=无可行路径`, `4=碰撞`）
- `success_rate`：单次运行成功率（当前协议中单次运行，取值通常为 `0/1`）
- `collision_rate`：单次运行碰撞率（当前协议中通常为 `0/1`）
- `avg_exec_time_sec`：执行时间（秒）
- `avg_path_length`：路径长度
- `min_safety_distance`：最小安全距离
- `first_solution_iteration`：首个可行解的 RRT 迭代数（失败为 `0`）

### 6.2 summary.csv（聚合结果）
关键字段：
- `total_runs`：总样本数
- `success_runs`：成功样本数
- `collision_runs`：碰撞样本数
- `no_path_runs`：无路径样本数
- `success_rate = success_runs / total_runs`
- `collision_rate = collision_runs / total_runs`
- `avg_exec_time_sec`、`avg_path_length`、`avg_min_safety_distance`、`avg_first_solution_iteration`：
  仅对成功样本取平均（代码实现见 `src/evaluation/simple_evaluator.cpp`）。

### 6.3 weights.csv（权重记录）
`P0` 为 baseline，`learning_enabled = 0`，通常会看到初始权重与更新权重一致。  
该文件在 `P0` 主要用于保持输出结构统一，便于后续学习实验横向对比。

## 7. 当前冻结快照（2026-02-20）
### 7.1 challenge（300000）
来源：`data/BenchmarK/challenge/challenge_summary.csv`
- `total_runs = 30`
- `success_runs = 3`
- `no_path_runs = 27`
- `success_rate = 0.100000`
- `avg_exec_time_sec = 294.333333`
- `avg_path_length = 284.303773`
- `avg_min_safety_distance = 0.203021`
- `avg_first_solution_iteration = 270393.666667`

### 7.2 pass（500000）
来源：`data/BenchmarK/pass/pass_summary.csv`
- `total_runs = 30`
- `success_runs = 30`
- `no_path_runs = 0`
- `success_rate = 1.000000`
- `avg_exec_time_sec = 300.800000`
- `avg_path_length = 287.896293`
- `avg_min_safety_distance = 0.203442`
- `avg_first_solution_iteration = 402755.233333`

## 8. 冻结规则
- `P0` 阶段不得修改固定输入参数、预算分层、输出命名与统计口径。
- 若需修改任一项，必须新建协议版本（例如 `P1`），并保留 `P0` 结果用于对照。
