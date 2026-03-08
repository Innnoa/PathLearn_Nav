# [DATA_TAG] KEEP | CORE | data目录总清单

日期：2026-03-08 10:33:20  
执行者：Codex

## 使用原则
- 不修改 CSV/地图/模型本体结构，避免影响程序读取。
- 通过 `DATA_INDEX.csv` 统一标记文件级用途与保留等级。
- 论文主证据优先引用 `CORE` 条目。

## 统计
- 当前保留文件总数：150
- `CORE`：81
- `AUX`：55
- `HISTORY`：8
- `META`：6
- 已删除条目：11

## 论文主保留项（CORE）
| 路径 | 作用 |
| --- | --- |
| `data/config/pathlearn_app.conf` | 主程序演示运行配置 |
| `data/config/pathlearn_eval_p2_astar_baseline.conf` | A*基线配置 |
| `data/config/pathlearn_eval_p2_astar_heuristic.conf` | A*+NN启发配置 |
| `data/config/pathlearn_eval_p2_astar_learning.conf` | 学习轮配置 |
| `data/maps/scene_custom.txt` | 主实验地图（迷宫） |
| `data/maps/scene_custom.tmj` | 地图编辑源文件 |
| `data/maps/walls_32.tsx` | Tiled tileset定义 |
| `data/models/nn_sampler_v2.txt` | 候选模型v2 |
| `data/models/nn_sampler_v3.txt` | 候选模型v3 |
| `data/models/nn_sampler_v4_20260305_071430.txt` | 夜跑训练模型v4 |
| `data/models/nn_sampler_v5_20260305_071430.txt` | 夜跑训练模型v5 |
| `data/nn_dataset/nn_train_samples.csv` | 训练集 |
| `data/nn_dataset/nn_val_samples.csv` | 验证集 |
| `data/nn_dataset/nn_test_samples.csv` | 测试集 |
| `data/BenchmarK/p2_astar/tune/budget_sweep_20260302_224634/` | 预算扫描结果与B_challenge选择依据 |
| `data/BenchmarK/p2_astar/challenge/challenge_B19500_20260303_192240/` | 挑战预算正式对比（核心结论） |
| `data/BenchmarK/p2_astar/pass/pass_B500000_20260304_223704/` | 通过预算正式对比（稳态结论） |

## 已删除项
| 路径 | 原因 |
| --- | --- |
| `data/BenchmarK/legacy_rrt` | 移除旧RRT阶段目录，非当前论文主线 |
| `data/BenchmarK/p1` | 移除P1历史目录，非当前论文主线 |
| `data/BenchmarK/p2_astar/overnight/overnight_smoke_222413` | 移除冒烟目录，非正式结果 |
| `data/BenchmarK/p2_astar/overnight/overnight_24h_smoke_wrapper_071146` | 移除包装脚本冒烟目录，非正式结果 |
| `data/eval_config_run_20260302_125223_runs.csv` | 移除临时配置运行产物 |
| `data/eval_config_run_20260302_125223_summary.csv` | 移除临时配置运行产物 |
| `data/eval_config_run_20260302_125223_weights.csv` | 移除临时配置运行产物 |
| `data/eval_config_run_20260304_222109_runs.csv` | 移除临时配置运行产物 |
| `data/eval_config_run_20260304_222109_summary.csv` | 移除临时配置运行产物 |
| `data/eval_config_run_20260304_222109_weights.csv` | 移除临时配置运行产物 |
| `data/BenchmarK/p2_astar/challenge/challenge_B19500_20260303_110912` | 移除空目录 |

## 总表
- 全量文件级标记见：`data/DATA_INDEX.csv`
