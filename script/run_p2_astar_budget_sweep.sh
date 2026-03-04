#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

EVAL_BIN="${EVAL_BIN:-./build/pathlearn_eval}"
BASE_CONF="${BASE_CONF:-data/config/pathlearn_eval_p2_astar_baseline.conf}"
HINT_CONF="${HINT_CONF:-data/config/pathlearn_eval_p2_astar_heuristic.conf}"
MODEL_V3="${MODEL_V3:-data/models/nn_sampler_v3.txt}"
ANALYZER="${ANALYZER:-script/analyze_runs_success_only.py}"

SEEDS="${SEEDS:-10}"
SEED_START="${SEED_START:-1}"
BUDGETS="${BUDGETS:-19300 19400 19500 19600 19700 19800}"
TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
OUT_ROOT="${OUT_ROOT:-data/BenchmarK/p2_astar/tune/budget_sweep_${TAG}}"

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
if [[ ! -f "$MODEL_V3" ]]; then
  echo "错误: 缺少模型文件 $MODEL_V3"
  exit 1
fi
if [[ ! -f "$ANALYZER" ]]; then
  echo "错误: 缺少分析脚本 $ANALYZER"
  exit 1
fi

mkdir -p "$OUT_ROOT"

COMBINED_CSV="${OUT_ROOT}/budget_sweep_compare.csv"
HEADER_WRITTEN=0

echo "== A* 预算扫描开始 =="
echo "ROOT_DIR=$ROOT_DIR"
echo "SEEDS=$SEEDS SEED_START=$SEED_START"
echo "BUDGETS=$BUDGETS"
echo "OUT_ROOT=$OUT_ROOT"
echo

for B in $BUDGETS; do
  echo "===== B=${B} ====="

  BASE_PREFIX="${OUT_ROOT}/astar_base_B${B}"
  HINT_PREFIX="${OUT_ROOT}/astar_v3_B${B}"
  COMPARE_CSV="${OUT_ROOT}/compare_B${B}.csv"

  "$EVAL_BIN" \
    --config "$BASE_CONF" \
    --seeds "$SEEDS" \
    --seed-start "$SEED_START" \
    --rrt-max-iters "$B" \
    --out-prefix "$BASE_PREFIX"

  "$EVAL_BIN" \
    --config "$HINT_CONF" \
    --nn-model "$MODEL_V3" \
    --seeds "$SEEDS" \
    --seed-start "$SEED_START" \
    --rrt-max-iters "$B" \
    --out-prefix "$HINT_PREFIX"

  python3 "$ANALYZER" \
    --runs "${BASE_PREFIX}_runs.csv" "${HINT_PREFIX}_runs.csv" \
    --labels "base_B${B}" "v3_B${B}" \
    --out "$COMPARE_CSV"

  if [[ "$HEADER_WRITTEN" -eq 0 ]]; then
    echo "budget,$(head -n 1 "$COMPARE_CSV")" > "$COMBINED_CSV"
    HEADER_WRITTEN=1
  fi
  while IFS= read -r line; do
    [[ -z "$line" ]] && continue
    echo "${B},${line}" >> "$COMBINED_CSV"
  done < <(tail -n +2 "$COMPARE_CSV")

  python3 - <<'PY' "$COMPARE_CSV" "$B"
import csv
import sys

compare_csv = sys.argv[1]
budget = sys.argv[2]
with open(compare_csv, "r", encoding="utf-8", newline="") as f:
    rows = list(csv.DictReader(f))
if len(rows) < 2:
    print(f"[B={budget}] 警告: 对比文件记录不足")
    raise SystemExit(0)
base = rows[0]
hint = rows[1]
base_sr = float(base["success_rate"])
hint_sr = float(hint["success_rate"])
base_it = float(base["avg_first_iter_success_only"])
hint_it = float(hint["avg_first_iter_success_only"])
delta_sr = hint_sr - base_sr
delta_it = hint_it - base_it
print(
    f"[B={budget}] success_rate base={base_sr:.3f} v3={hint_sr:.3f} "
    f"delta={delta_sr:+.3f} | "
    f"first_iter(success_only) base={base_it:.1f} v3={hint_it:.1f} "
    f"delta={delta_it:+.1f}"
)
PY

  echo
done

echo "== A* 预算扫描完成 =="
echo "汇总文件: $COMBINED_CSV"
echo "单预算对比: ${OUT_ROOT}/compare_B*.csv"
