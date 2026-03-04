# P1 学习效果评测协议（冻结版）
日期：2026-02-20  
执行者：Codex

## 1. 目标
`P1` 的目标是验证“学习机制”在固定预算下是否带来可量化收益。  
本阶段先聚焦于**同采样策略条件下**的学习效果：`baseline` vs `learning`（`mode=both`）。

说明：本协议中 `nn_guide` 固定开启，用于让 baseline 与 learning 使用一致采样引导，仅比较学习迭代对结果的影响。

## 2. 固定实验矩阵
两档预算、同一地图、同一起终点、同一种子区间：

| 档位 | 预算 | 配置文件 | 输出前缀 |
| --- | --- | --- | --- |
| challenge | `rrt_max_iters=300000` | `data/config/pathlearn_eval_p1_challenge.conf` | `data/BenchmarK/p1/challenge/p1_challenge_{timestamp}` |
| pass | `rrt_max_iters=500000` | `data/config/pathlearn_eval_p1_pass.conf` | `data/BenchmarK/p1/pass/p1_pass_{timestamp}` |

## 3. 固定输入参数
- 地图：`data/maps/scene_custom.txt`
- 规则：`static`
- 模式：`both`（每个种子同时产出 baseline 与 learning 对照）
- 种子：`seed_start=1`, `seeds=30`
- 学习轮次：`learn_epochs=5`
- 起点：`(-33, -33, 0)`
- 终点：`(33, 33, 0)`
- 速度/时步：`speed=1.0`, `time_step=1.0`
- 绕行系数：`detour=4.0`
- exploration：`8.0`
- max_steps：`0`（自动估算）
- max_wait_steps：`0`（无路径时最多等待重规划 4 次）
- NN 引导：`nn_guide=true`, `nn_model=data/models/nn_sampler_default.txt`

## 4. 运行命令
### 4.1 challenge
```bash
./build/pathlearn_eval --config data/config/pathlearn_eval_p1_challenge.conf
```

### 4.2 pass
```bash
./build/pathlearn_eval --config data/config/pathlearn_eval_p1_pass.conf
```

## 5. P1-0 你要完成的事项
1. 运行 challenge 与 pass 各 1 次，生成带时间戳的 `runs/summary/weights` 三件套。
2. 从两个 `summary.csv` 中提取 baseline 与 learning 的四个核心指标：
   - `success_rate`
   - `collision_rate`
   - `avg_min_safety_distance`
   - `avg_first_solution_iteration`
3. 形成 P1-0 结论：
   - 若 learning 在 challenge 上 `success_rate` 提升或持平，且 `avg_first_solution_iteration` 下降，则进入 P1-1（训练样本扩展与模型版本化）。
   - 若 learning 无提升，先保留协议不变，进入参数调优（`learn_epochs`、学习增益、NN 采样概率）再复测。

## 6. 验收口径（P1-0）
- 数据完整：每个档位都包含 `runs/summary/weights`。
- 样本完整：每组 `total_runs=30`，种子覆盖 `1..30`。
- 统计一致：summary 字段与 runs 聚合结果一致。
- 对照完整：同一 `scenario/profile` 下同时有 `learning_enabled=0` 与 `learning_enabled=1` 两组结果。

## 7. 冻结规则
- `P1-0` 期间不改地图、不改起终点、不改种子区间、不改预算分层。
- 允许改动项仅限：学习相关参数（如 `learn_epochs`、学习增益）与模型文件版本。
- 若修改固定输入项，必须升级协议版本（例如 `P1-1`）并保留本协议结果作为对照。

## 8. 阶段文档
- `docs/p1_0_review.md`：P1-0 结果复盘与 P1-1 调优入口。
