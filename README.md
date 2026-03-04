# PathLearn_Nav

一个面向二维导航场景的路径规划与评测项目，使用 C++17 + CMake 构建，支持：

- RRT 与 A* 两种规划器
- 静态/动态障碍仿真
- GUI 可视化（SDL2，可选）
- 批量评测与 CSV 输出（`runs/summary/weights`）
- 神经网络引导采样/启发（可选）

项目内置了从单次仿真到批量实验、从数据集生成到简易模型训练的完整流程，适合课程设计、算法对比和参数调优。

## 1. 环境依赖

基础依赖：

- CMake >= 3.16
- 支持 C++17 的编译器（GCC/Clang）
- `pkg-config`
- Python 3（用于脚本和数据处理）

可选依赖（启用 GUI 时）：

- `SDL2`
- `SDL2_ttf`（可选，缺失时会自动降级为无文字叠加）

Ubuntu/Debian 示例：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libsdl2-dev libsdl2-ttf-dev python3
```

## 2. 构建

在项目根目录执行：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPATHLEARN_BUILD_TESTS=ON \
  -DPATHLEARN_WITH_SDL=ON \
  -DPATHLEARN_WITH_SDL_TTF=ON

cmake --build build -j"$(nproc)"
```

如果你只做无界面评测，可以关闭 SDL：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPATHLEARN_BUILD_TESTS=ON \
  -DPATHLEARN_WITH_SDL=OFF
cmake --build build -j"$(nproc)"
```

构建产物：

- `build/pathlearn_app`：交互/单次仿真入口
- `build/pathlearn_eval`：批量评测入口
- `build/pathlearn_tests`：单元测试二进制

## 3. 运行

### 3.1 单次仿真（App）

```bash
./build/pathlearn_app --config data/config/pathlearn_app.conf
```

示例（命令行覆盖配置项）：

```bash
./build/pathlearn_app \
  --config data/config/pathlearn_app.conf \
  --planner astar \
  --map data/maps/scene_custom.txt \
  --nn-guide on \
  --nn-model data/models/nn_sampler_default.txt \
  --no-gui
```

### 3.2 批量评测（Eval）

```bash
./build/pathlearn_eval --config data/config/pathlearn_eval.conf
```

评测会按 `out_prefix` 输出三类文件：

- `*_runs.csv`：逐次运行记录
- `*_summary.csv`：聚合统计
- `*_weights.csv`：学习权重日志

配置语法与完整参数列表见：

- `docs/config_usage.md`
- `data/config/*.conf`

## 4. 测试

```bash
ctest --test-dir build --output-on-failure
```

测试源码位于 `tests/`，当前覆盖：

- 核心类型与环境/碰撞
- 动态障碍
- RRT/A* 规划
- 仿真与评测
- 学习与 NN 采样

## 5. 常用实验脚本

以下脚本均在项目根目录执行：

1) 公平重跑（baseline / heuristic-v2 / heuristic-v3）：

```bash
bash script/run_p2_astar_fair_rerun.sh
```

2) A* 预算扫描（多预算自动对比）：

```bash
bash script/run_p2_astar_budget_sweep.sh
```

3) challenge 档正式对比：

```bash
bash script/run_p2_astar_challenge_compare.sh
```

4) 仅统计成功样本口径：

```bash
python3 script/analyze_runs_success_only.py \
  --runs data/xxx_runs.csv data/yyy_runs.csv \
  --labels baseline heuristic \
  --out data/compare.csv
```

## 6. NN 数据与模型工具

生成训练数据（基于地图与 A* 轨迹采样）：

```bash
python3 tools/gen_nn_dataset.py \
  --map data/maps/scene_custom.txt \
  --out data/nn_dataset/nn_train_samples.csv \
  --rows 12000
```

训练最小 MLP 并导出 PathLearn 文本模型：

```bash
python3 tools/train_nn_sampler.py \
  --in data/nn_dataset/nn_train_samples.csv \
  --out data/models/nn_sampler_custom.txt \
  --epochs 120 --lr 0.03 --hidden 16
```

## 7. 目录结构

```text
PathLearn_Nav/
├── include/               # 核心接口与实现头文件
├── src/                   # 环境、规划、仿真、评测、可视化实现
├── tests/                 # GTest 测试
├── data/
│   ├── maps/              # 地图
│   ├── config/            # app/eval 配置
│   ├── models/            # NN 模型
│   └── BenchmarK/         # 实验输出与归档
├── script/                # 实验批处理脚本
├── tools/                 # 数据生成/模型训练工具
└── docs/                  # 协议与说明文档
```

## 8. 相关文档

- `docs/config_usage.md`：配置文件和参数总览
- `docs/p0_protocol.md`：P0 基线评测协议
- `docs/p1_protocol.md`：P1 学习评测协议
- `docs/p1_0_review.md`：P1-0 结果复盘
- `docs/nn_minimal_plan.md`：NN 引导采样最小落地方案

