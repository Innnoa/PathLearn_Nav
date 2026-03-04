#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
SEEDS="${SEEDS:-10}"
SEED_START="${SEED_START:-1}"

BASE_CONF="data/config/pathlearn_eval_p2_astar_baseline.conf"
HINT_CONF="data/config/pathlearn_eval_p2_astar_heuristic.conf"
EVAL_BIN="./build/pathlearn_eval"
MODEL_V2="data/models/nn_sampler_v2.txt"
MODEL_V3="data/models/nn_sampler_v3.txt"

echo "== 公平重跑开始 =="
echo "ROOT_DIR=$ROOT_DIR"
echo "TAG=$TAG SEEDS=$SEEDS SEED_START=$SEED_START"
echo

if [[ ! -x "$EVAL_BIN" ]]; then
  echo "错误: 未找到可执行文件 $EVAL_BIN"
  echo "请先构建: cmake --build build -j\$(nproc)"
  exit 1
fi

if [[ ! -f "$BASE_CONF" ]]; then
  echo "错误: 缺少配置文件 $BASE_CONF"
  exit 1
fi

if [[ ! -f "$HINT_CONF" ]]; then
  echo "错误: 缺少配置文件 $HINT_CONF"
  exit 1
fi

if [[ ! -f "$MODEL_V2" ]]; then
  echo "错误: 缺少模型文件 $MODEL_V2"
  exit 1
fi

if [[ ! -f "$MODEL_V3" ]]; then
  echo "错误: 缺少模型文件 $MODEL_V3"
  exit 1
fi

BASE_PREFIX="data/BenchmarK/p2_astar/baseline/astar_base_s${SEEDS}_${TAG}"
V2_PREFIX="data/BenchmarK/p2_astar/heuristic/astar_hint_v2_s${SEEDS}_${TAG}"
V3_PREFIX="data/BenchmarK/p2_astar/heuristic/astar_hint_v3_s${SEEDS}_${TAG}"

echo "1/3 baseline（不使用NN）"
"$EVAL_BIN" \
  --config "$BASE_CONF" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --out-prefix "$BASE_PREFIX"

echo
echo "2/3 heuristic + v2"
"$EVAL_BIN" \
  --config "$HINT_CONF" \
  --nn-model "$MODEL_V2" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --out-prefix "$V2_PREFIX"

echo
echo "3/3 heuristic + v3"
"$EVAL_BIN" \
  --config "$HINT_CONF" \
  --nn-model "$MODEL_V3" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --out-prefix "$V3_PREFIX"

echo
echo "== 公平重跑完成 =="
echo "baseline summary: ${BASE_PREFIX}_summary.csv"
echo "heuristic(v2) summary: ${V2_PREFIX}_summary.csv"
echo "heuristic(v3) summary: ${V3_PREFIX}_summary.csv"
