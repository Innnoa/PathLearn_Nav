# [DATA_TAG] KEEP | AUX | BenchmarK 子目录说明
# BenchmarK 数据目录说明
日期：2026-03-08  
执行者：Codex

## 当前目录职责
- `p2_astar/tune/`：预算扫描与阈值选择证据（确定 `B_challenge=19500`）。
- `p2_astar/challenge/`：挑战预算正式对比结果（论文核心证据）。
- `p2_astar/pass/`：通过预算正式对比结果（论文核心证据）。
- `p2_astar/overnight/`：长时运行过程输出（补充证据）。
- `p2_astar/baseline/` 与 `p2_astar/heuristic/`：小样本历史对比（辅助）。
- `_meta/`：实验索引与说明文件。

## 清理状态
1. `legacy_rrt/` 与 `p1/` 已删除，不再作为当前论文主证据链。
2. 冒烟目录已删除：`overnight_smoke_222413`、`overnight_24h_smoke_wrapper_071146`。
3. 论文主引用优先顺序：`challenge` > `pass` > `tune` > `overnight`。
