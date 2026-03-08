#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

# 24h 长时任务默认参数（可通过环境变量覆盖）
export RUN_TRAIN="${RUN_TRAIN:-1}"
export RUN_SCREEN="${RUN_SCREEN:-1}"
export RUN_CHALLENGE="${RUN_CHALLENGE:-1}"
export RUN_PASS="${RUN_PASS:-1}"

# 固定预算：用于论文中 challenge / pass 两档。
export B_CHALLENGE="${B_CHALLENGE:-19500}"
export B_PASS="${B_PASS:-500000}"

# 样本规模：默认约 24h 负载（机器性能不同会有差异）。
export SCREEN_SEEDS="${SCREEN_SEEDS:-30}"
export SCREEN_SEED_START="${SCREEN_SEED_START:-1}"
export CHALLENGE_SEEDS="${CHALLENGE_SEEDS:-240}"
export PASS_SEEDS="${PASS_SEEDS:-160}"

# 输出目录统一带时间戳，避免覆盖历史结果。
export TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
export OUT_ROOT="${OUT_ROOT:-data/BenchmarK/p2_astar/overnight/overnight_24h_${TAG}}"

echo "启动 24h 任务:"
echo "  TAG=$TAG"
echo "  OUT_ROOT=$OUT_ROOT"
echo "  RUN_TRAIN=$RUN_TRAIN RUN_SCREEN=$RUN_SCREEN RUN_CHALLENGE=$RUN_CHALLENGE RUN_PASS=$RUN_PASS"
echo "  B_CHALLENGE=$B_CHALLENGE B_PASS=$B_PASS"
echo "  SCREEN_SEEDS=$SCREEN_SEEDS CHALLENGE_SEEDS=$CHALLENGE_SEEDS PASS_SEEDS=$PASS_SEEDS"
echo

exec ./script/run_p2_astar_overnight_24h.sh
