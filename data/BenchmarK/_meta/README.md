# BenchmarK 数据目录说明
日期：2026-03-01  
执行者：Codex

## 目录职责
- `legacy_rrt/`：历史 RRT 阶段结果归档（用于论文“前期基线与演进”）
- `p1/`：P1 阶段历史结果（原有目录，保持不动）
- `p2_astar/`：当前 A* 主线正式评测结果
  - `baseline/`：A* + 非学习（nn_guide=off）
  - `heuristic/`：A* + 启发（nn_guide=on，mode=baseline）
  - `learning/`：A* + 学习轮（nn_guide=on，mode=learning）
  - `compare/`：后续对比汇总/图表导出
- `_meta/`：实验索引与说明

## 约定
1. 每次评测通过 `out_prefix` 生成 `{runs,summary,weights}` 三件套。
2. 评测后在 `experiment_index.csv` 追加一行，记录配置与输出前缀。
3. 论文中引用时优先使用 `p2_astar/`，`legacy_rrt/` 仅作为历史对照。
