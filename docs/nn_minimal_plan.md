# NN 引导采样最小落地方案
日期：2026-02-08  
执行者：Codex  

## 目标
在不替换现有 RRT 主流程的前提下，引入“神经网络引导采样”，实现：
- 维持可行性（仍由碰撞检测与 RRT 约束保证）
- 提高难图搜索效率（减少首解迭代数）
- 保持工程可解释、可复现

## 已实现能力
- CLI 新增参数（`pathlearn_app`）：
  - `--nn-guide on/off`：开启或关闭 NN 引导
  - `--nn-model <path>`：模型路径（文本格式）
  - `--nn-prob <0~1>`：NN 采样概率
  - `--nn-distance <d>`：NN 采样步长（0 表示自动）
- 规划器接入：
  - 在 `simple_rrt_planner` 采样阶段按概率选择 NN 采样
  - NN 不可用时自动回退到原采样策略
- 模型加载：
  - `PATHLEARN_NN 1` 文本模型格式
  - 多层感知机前向推理（ReLU）

## 模型格式
示例（两层网络）：

```text
PATHLEARN_NN 1 2
layer 8 16
<16*8 个权重>
<16 个偏置>
layer 16 2
<2*16 个权重>
<2 个偏置>
```

## 训练脚本
脚本：`tools/train_nn_sampler.py`

输入 CSV 字段：
- `current_x,current_y,goal_x,goal_y`
- `obs1_dx,obs1_dy,obs2_dx,obs2_dy`
- `target_dx,target_dy`

示例命令：

```bash
python3 tools/train_nn_sampler.py \
  --in data/nn_train_samples.csv \
  --out data/models/nn_sampler_custom.txt \
  --epochs 120 --lr 0.03 --hidden 16
```

## 运行示例

```bash
./build/pathlearn_app \
  --map data/maps/scene_custom.txt \
  --nn-guide on \
  --nn-model data/models/nn_sampler_default.txt \
  --nn-prob 0.35 \
  --nn-distance 1.2 \
  --exploration 8 \
  --rrt-max-iters 1000000 \
  --trace console --trace-every 20000
```

## 建议评测
- 对照组：`--nn-guide off`
- 实验组：`--nn-guide on`
- 同地图同种子比较：
  - 成功率
  - 首解出现迭代数（看 trace 日志）
  - 规划时间（可后续补统计）
  - 路径长度与最小安全距离

