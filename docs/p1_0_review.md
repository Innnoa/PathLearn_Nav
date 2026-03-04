# P1-0 结果复盘与下一步
日期：2026-02-24  
执行者：Codex

## 1. 输入数据
- challenge summary：`data/BenchmarK/p1/challenge/p1_challenge_20260222_164653_summary.csv`
- pass summary：`data/BenchmarK/p1/pass/p1_pass_20260223_111502_summary.csv`
- challenge runs：`data/BenchmarK/p1/challenge/p1_challenge_20260222_164653_runs.csv`
- pass runs：`data/BenchmarK/p1/pass/p1_pass_20260223_111502_runs.csv`

## 2. 协议完整性检查
- challenge：`runs=61行(含表头), summary=2行数据, weights=181行(含表头)`，完整。
- pass：`runs=61行(含表头), summary=2行数据, weights=181行(含表头)`，完整。
- `mode=both` 对照完整：`learning_enabled=0/1` 两组均存在。
- 种子覆盖：`1..30`。

## 3. 核心结果
### 3.1 challenge（300000）
- baseline：`success_rate=0.000000`，`avg_first_solution_iteration=0.000000`
- learning：`success_rate=0.000000`，`avg_first_solution_iteration=0.000000`
- 差值（learning-baseline）：
  - `Δsuccess_rate=+0.000000`
  - `Δcollision_rate=+0.000000`
  - `Δavg_min_safety_distance=+0.000000`
  - `Δavg_first_solution_iteration=+0.000000`

说明：challenge 档 baseline 与 learning 全部 `NoPath`，学习收益无法显现。

### 3.2 pass（500000）
- baseline：`success_rate=0.966667`，`avg_first_solution_iteration=408573.965517`
- learning：`success_rate=1.000000`，`avg_first_solution_iteration=316450.866667`
- 差值（learning-baseline）：
  - `Δsuccess_rate=+0.033333`
  - `Δcollision_rate=+0.000000`
  - `Δavg_min_safety_distance=-0.000609`
  - `Δavg_first_solution_iteration=-92123.098850`

说明：pass 档学习有正向收益（成功率提升，首解迭代下降）。

## 4. P1-0 判定
按 `docs/p1_protocol.md` 的门槛，`P1-0` 在 challenge 档未满足“持平/提升且首解下降”的判定条件。  
结论：进入 `P1-1` 调优分支（先在 challenge 档恢复可分辨信号）。

## 5. 下一步（P1-1）
### 5.1 调优目标
- 在不改预算分层（challenge=300000）的前提下，让 challenge 档出现可比较样本（至少 baseline 或 learning 有部分成功）。
- 优先提升学习可观察性：`success_rate` 与 `avg_first_solution_iteration`。

### 5.2 已准备配置
- `data/config/pathlearn_eval_p1_challenge_tune.conf`

### 5.3 建议执行顺序
1. 先跑小样本调优（10 seeds）确认方向：
```bash
./build/pathlearn_eval --config data/config/pathlearn_eval_p1_challenge_tune.conf
```
2. 若 learning 相对 baseline 有改进，再把 `seeds` 扩大到 30 做正式评测。
3. 产出新的 `summary.csv` 后，按本文件第 3 节同口径比较并更新结论。
