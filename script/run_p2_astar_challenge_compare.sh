#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

EVAL_BIN="${EVAL_BIN:-./build/pathlearn_eval}"
BASE_CONF="${BASE_CONF:-data/config/pathlearn_eval_p2_astar_baseline.conf}"
HINT_CONF="${HINT_CONF:-data/config/pathlearn_eval_p2_astar_heuristic.conf}"
ANALYZER="${ANALYZER:-script/analyze_runs_success_only.py}"
MODEL_V2="${MODEL_V2:-data/models/nn_sampler_v2.txt}"
MODEL_V3="${MODEL_V3:-data/models/nn_sampler_v3.txt}"

B_CHALLENGE="${B_CHALLENGE:-19500}"
SEEDS="${SEEDS:-50}"
SEED_START="${SEED_START:-1}"
TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
OUT_ROOT="${OUT_ROOT:-data/BenchmarK/p2_astar/challenge/challenge_B${B_CHALLENGE}_${TAG}}"

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
if [[ ! -f "$ANALYZER" ]]; then
  echo "错误: 缺少分析脚本 $ANALYZER"
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

mkdir -p "$OUT_ROOT"

BASE_PREFIX="${OUT_ROOT}/astar_base_s${SEEDS}_B${B_CHALLENGE}_${TAG}"
V2_PREFIX="${OUT_ROOT}/astar_hint_v2_s${SEEDS}_B${B_CHALLENGE}_${TAG}"
V3_PREFIX="${OUT_ROOT}/astar_hint_v3_s${SEEDS}_B${B_CHALLENGE}_${TAG}"

echo "== A* 挑战预算正式对比开始 =="
echo "ROOT_DIR=$ROOT_DIR"
echo "B_CHALLENGE=$B_CHALLENGE SEEDS=$SEEDS SEED_START=$SEED_START TAG=$TAG"
echo "OUT_ROOT=$OUT_ROOT"
echo

echo "1/3 baseline（不使用NN）"
"$EVAL_BIN" \
  --config "$BASE_CONF" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --rrt-max-iters "$B_CHALLENGE" \
  --out-prefix "$BASE_PREFIX"

echo
echo "2/3 heuristic + v2"
"$EVAL_BIN" \
  --config "$HINT_CONF" \
  --nn-model "$MODEL_V2" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --rrt-max-iters "$B_CHALLENGE" \
  --out-prefix "$V2_PREFIX"

echo
echo "3/3 heuristic + v3"
"$EVAL_BIN" \
  --config "$HINT_CONF" \
  --nn-model "$MODEL_V3" \
  --seeds "$SEEDS" \
  --seed-start "$SEED_START" \
  --rrt-max-iters "$B_CHALLENGE" \
  --out-prefix "$V3_PREFIX"

SUCCESS_ONLY_CSV="${OUT_ROOT}/challenge_compare_success_only.csv"
python3 "$ANALYZER" \
  --runs \
  "${BASE_PREFIX}_runs.csv" \
  "${V2_PREFIX}_runs.csv" \
  "${V3_PREFIX}_runs.csv" \
  --labels baseline v2 v3 \
  --out "$SUCCESS_ONLY_CSV"

SUMMARY_CSV="${OUT_ROOT}/challenge_compare_summary.csv"
python3 - <<'PY' "${BASE_PREFIX}_summary.csv" "${V2_PREFIX}_summary.csv" "${V3_PREFIX}_summary.csv" "$SUMMARY_CSV"
import csv
import sys
from pathlib import Path

base_path = Path(sys.argv[1])
v2_path = Path(sys.argv[2])
v3_path = Path(sys.argv[3])
out_path = Path(sys.argv[4])

inputs = [("baseline", base_path), ("v2", v2_path), ("v3", v3_path)]
rows = []
for label, path in inputs:
    with path.open("r", encoding="utf-8", newline="") as f:
        row = next(csv.DictReader(f))
    rows.append(
        {
            "label": label,
            "summary_path": str(path),
            "total_runs": row["total_runs"],
            "success_rate": row["success_rate"],
            "collision_rate": row["collision_rate"],
            "avg_exec_time_sec": row["avg_exec_time_sec"],
            "avg_path_length": row["avg_path_length"],
            "avg_min_safety_distance": row["avg_min_safety_distance"],
            "avg_first_solution_iteration": row["avg_first_solution_iteration"],
        }
    )

fieldnames = [
    "label",
    "summary_path",
    "total_runs",
    "success_rate",
    "collision_rate",
    "avg_exec_time_sec",
    "avg_path_length",
    "avg_min_safety_distance",
    "avg_first_solution_iteration",
]
out_path.parent.mkdir(parents=True, exist_ok=True)
with out_path.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    writer.writeheader()
    for row in rows:
        writer.writerow(row)

base = rows[0]
for candidate in rows[1:]:
    delta_sr = float(candidate["success_rate"]) - float(base["success_rate"])
    delta_it = (
        float(candidate["avg_first_solution_iteration"])
        - float(base["avg_first_solution_iteration"])
    )
    print(
        f"[{candidate['label']}] Δsuccess_rate={delta_sr:+.6f} "
        f"Δavg_first_solution_iteration={delta_it:+.1f}"
    )
PY

echo
echo "== A* 挑战预算正式对比完成 =="
echo "baseline runs: ${BASE_PREFIX}_runs.csv"
echo "v2 runs: ${V2_PREFIX}_runs.csv"
echo "v3 runs: ${V3_PREFIX}_runs.csv"
echo "success-only 对比: $SUCCESS_ONLY_CSV"
echo "summary 对比: $SUMMARY_CSV"
