#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

EVAL_BIN="${EVAL_BIN:-./build/pathlearn_eval}"
TRAIN_SCRIPT="${TRAIN_SCRIPT:-tools/train_nn_sampler.py}"
ANALYZER="${ANALYZER:-script/analyze_runs_success_only.py}"
BASE_CONF="${BASE_CONF:-data/config/pathlearn_eval_p2_astar_baseline.conf}"
HINT_CONF="${HINT_CONF:-data/config/pathlearn_eval_p2_astar_heuristic.conf}"
CHALLENGE_SCRIPT="${CHALLENGE_SCRIPT:-script/run_p2_astar_challenge_compare.sh}"
PASS_SCRIPT="${PASS_SCRIPT:-script/run_p2_astar_pass_compare.sh}"

DATASET_TRAIN="${DATASET_TRAIN:-data/nn_dataset/nn_train_samples.csv}"

B_CHALLENGE="${B_CHALLENGE:-19500}"
B_PASS="${B_PASS:-500000}"

SCREEN_SEEDS="${SCREEN_SEEDS:-20}"
SCREEN_SEED_START="${SCREEN_SEED_START:-1}"

CHALLENGE_SEEDS="${CHALLENGE_SEEDS:-100}"
PASS_SEEDS="${PASS_SEEDS:-80}"

RUN_TRAIN="${RUN_TRAIN:-1}"
RUN_SCREEN="${RUN_SCREEN:-1}"
RUN_CHALLENGE="${RUN_CHALLENGE:-1}"
RUN_PASS="${RUN_PASS:-1}"

TAG="${TAG:-$(date +%Y%m%d_%H%M%S)}"
OUT_ROOT="${OUT_ROOT:-data/BenchmarK/p2_astar/overnight/overnight_${TAG}}"
SCREEN_DIR="${OUT_ROOT}/screen"
FINAL_DIR="${OUT_ROOT}/final"

mkdir -p "$OUT_ROOT" "$SCREEN_DIR" "$FINAL_DIR"
LOG_FILE="${OUT_ROOT}/overnight.log"

exec > >(tee -a "$LOG_FILE") 2>&1

echo "== A* 24h 训练/评测流水线开始 =="
echo "ROOT_DIR=$ROOT_DIR"
echo "TAG=$TAG"
echo "B_CHALLENGE=$B_CHALLENGE B_PASS=$B_PASS"
echo "SCREEN_SEEDS=$SCREEN_SEEDS CHALLENGE_SEEDS=$CHALLENGE_SEEDS PASS_SEEDS=$PASS_SEEDS"
echo "OUT_ROOT=$OUT_ROOT"
echo "LOG_FILE=$LOG_FILE"
echo

if [[ ! -x "$EVAL_BIN" ]]; then
  echo "错误: 未找到可执行文件 $EVAL_BIN"
  echo "请先构建: cmake --build build -j\$(nproc)"
  exit 1
fi
if [[ ! -f "$TRAIN_SCRIPT" ]]; then
  echo "错误: 未找到训练脚本 $TRAIN_SCRIPT"
  exit 1
fi
if [[ ! -f "$ANALYZER" ]]; then
  echo "错误: 未找到分析脚本 $ANALYZER"
  exit 1
fi
if [[ ! -f "$BASE_CONF" || ! -f "$HINT_CONF" ]]; then
  echo "错误: 缺少 A* 配置文件（baseline/heuristic）"
  exit 1
fi
if [[ ! -f "$CHALLENGE_SCRIPT" || ! -f "$PASS_SCRIPT" ]]; then
  echo "错误: 缺少 challenge/pass 对比脚本"
  exit 1
fi
if [[ ! -f "$DATASET_TRAIN" ]]; then
  echo "错误: 缺少训练数据 $DATASET_TRAIN"
  exit 1
fi

# 候选模型（会在 RUN_TRAIN=1 时追加 v4/v5）
declare -a MODEL_PATHS=(
  "data/models/nn_sampler_v2.txt"
  "data/models/nn_sampler_v3.txt"
)

if [[ "$RUN_TRAIN" == "1" ]]; then
  echo "== 阶段1: 训练新模型(v4/v5) =="
  MODEL_V4="data/models/nn_sampler_v4_${TAG}.txt"
  MODEL_V5="data/models/nn_sampler_v5_${TAG}.txt"

  python3 "$TRAIN_SCRIPT" \
    --in "$DATASET_TRAIN" \
    --out "$MODEL_V4" \
    --epochs 280 \
    --lr 0.012 \
    --hidden 48 \
    --seed 42

  python3 "$TRAIN_SCRIPT" \
    --in "$DATASET_TRAIN" \
    --out "$MODEL_V5" \
    --epochs 360 \
    --lr 0.010 \
    --hidden 64 \
    --seed 42

  MODEL_PATHS+=("$MODEL_V4" "$MODEL_V5")
else
  echo "== 阶段1: 跳过训练（RUN_TRAIN=$RUN_TRAIN）=="
fi
echo

# 过滤存在的候选模型
declare -a CANDIDATES=()
for m in "${MODEL_PATHS[@]}"; do
  if [[ -f "$m" ]]; then
    CANDIDATES+=("$m")
  fi
done
if [[ ${#CANDIDATES[@]} -eq 0 ]]; then
  echo "错误: 无可用候选模型"
  exit 1
fi

BEST_MODEL_A=""
BEST_MODEL_B=""

if [[ "$RUN_SCREEN" == "1" ]]; then
  echo "== 阶段2: 小样本筛选（challenge 预算）=="

  BASE_PREFIX="${SCREEN_DIR}/screen_base_s${SCREEN_SEEDS}_B${B_CHALLENGE}"
  "$EVAL_BIN" \
    --config "$BASE_CONF" \
    --seeds "$SCREEN_SEEDS" \
    --seed-start "$SCREEN_SEED_START" \
    --rrt-max-iters "$B_CHALLENGE" \
    --out-prefix "$BASE_PREFIX"

  SCREEN_ROWS_CSV="${SCREEN_DIR}/screen_rows.csv"
  echo "label,model_path,runs_path,summary_path" > "$SCREEN_ROWS_CSV"
  echo "baseline,,${BASE_PREFIX}_runs.csv,${BASE_PREFIX}_summary.csv" >> "$SCREEN_ROWS_CSV"

  idx=0
  for model in "${CANDIDATES[@]}"; do
    idx=$((idx + 1))
    label="m${idx}"
    prefix="${SCREEN_DIR}/screen_${label}_s${SCREEN_SEEDS}_B${B_CHALLENGE}"
    echo "筛选模型 ${label}: ${model}"
    "$EVAL_BIN" \
      --config "$HINT_CONF" \
      --nn-model "$model" \
      --seeds "$SCREEN_SEEDS" \
      --seed-start "$SCREEN_SEED_START" \
      --rrt-max-iters "$B_CHALLENGE" \
      --out-prefix "$prefix"
    echo "${label},${model},${prefix}_runs.csv,${prefix}_summary.csv" >> "$SCREEN_ROWS_CSV"
  done

  # 基于 success_rate（降序）+ success-only 首解迭代（升序）选前两名。
  SCREEN_RANK_CSV="${SCREEN_DIR}/screen_rank.csv"
  python3 - <<'PY' "$SCREEN_ROWS_CSV" "$ANALYZER" "$SCREEN_RANK_CSV"
import csv
import subprocess
import sys
from pathlib import Path

rows_csv = Path(sys.argv[1])
analyzer = Path(sys.argv[2])
out_csv = Path(sys.argv[3])

rows = list(csv.DictReader(rows_csv.open("r", encoding="utf-8", newline="")))

records = []
for r in rows:
    label = r["label"]
    model_path = r["model_path"]
    runs_path = r["runs_path"]
    summary_path = r["summary_path"]

    # summary 指标
    with open(summary_path, "r", encoding="utf-8", newline="") as f:
        s = next(csv.DictReader(f))
    success_rate = float(s["success_rate"])
    collision_rate = float(s["collision_rate"])

    # success-only 指标
    proc = subprocess.run(
        [
            "python3",
            str(analyzer),
            "--runs",
            runs_path,
            "--labels",
            label,
        ],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    lines = [ln.strip() for ln in proc.stdout.splitlines() if ln.strip()]
    if len(lines) < 2:
        raise RuntimeError(f"analyzer 输出异常: {runs_path}")
    a = next(csv.DictReader(lines))
    first_iter_success = float(a["avg_first_iter_success_only"])
    exec_time_success = float(a["avg_exec_time_success_only"])
    success_runs = int(a["success_runs"])
    total_runs = int(a["total_runs"])

    # 无成功样本时将 first_iter 视为无穷大，避免被误选
    rank_first = first_iter_success if success_runs > 0 else 1e18

    records.append(
        {
            "label": label,
            "model_path": model_path,
            "runs_path": runs_path,
            "summary_path": summary_path,
            "total_runs": total_runs,
            "success_runs": success_runs,
            "success_rate": success_rate,
            "collision_rate": collision_rate,
            "avg_first_iter_success_only": first_iter_success,
            "avg_exec_time_success_only": exec_time_success,
            "_rank_first": rank_first,
        }
    )

records_sorted = sorted(
    records,
    key=lambda x: (
        -x["success_rate"],
        x["_rank_first"],
        x["avg_exec_time_success_only"],
    ),
)

fields = [
    "label",
    "model_path",
    "total_runs",
    "success_runs",
    "success_rate",
    "collision_rate",
    "avg_first_iter_success_only",
    "avg_exec_time_success_only",
    "runs_path",
    "summary_path",
]
with out_csv.open("w", encoding="utf-8", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    for rec in records_sorted:
        row = {k: rec.get(k, "") for k in fields}
        w.writerow(row)

print("screen ranking:")
for i, rec in enumerate(records_sorted, 1):
    print(
        f"{i:02d}. {rec['label']} model={rec['model_path'] or '(baseline)'} "
        f"success_rate={rec['success_rate']:.3f} "
        f"first_iter_succ={rec['avg_first_iter_success_only']:.1f}"
    )
PY

  echo "筛选排序文件: $SCREEN_RANK_CSV"

  # 选前两个“有模型路径”的记录
  BEST_MODEL_A="$(awk -F',' 'NR>1 && $2!=""{print $2; exit}' "$SCREEN_RANK_CSV")"
  BEST_MODEL_B="$(awk -F',' 'NR>1 && $2!=""{n+=1; if(n==2){print $2; exit}}' "$SCREEN_RANK_CSV")"

  if [[ -z "$BEST_MODEL_A" ]]; then
    echo "错误: 筛选后未得到可用模型A"
    exit 1
  fi
  if [[ -z "$BEST_MODEL_B" ]]; then
    BEST_MODEL_B="$BEST_MODEL_A"
  fi
else
  echo "== 阶段2: 跳过筛选（RUN_SCREEN=$RUN_SCREEN）=="
  # 回退到现有 v3 / v2
  BEST_MODEL_A="data/models/nn_sampler_v3.txt"
  BEST_MODEL_B="data/models/nn_sampler_v2.txt"
fi

echo
echo "选定模型:"
echo "  BEST_MODEL_A=$BEST_MODEL_A"
echo "  BEST_MODEL_B=$BEST_MODEL_B"
echo

if [[ "$RUN_CHALLENGE" == "1" ]]; then
  echo "== 阶段3: challenge 长时评测 =="
  CHALLENGE_OUT="${FINAL_DIR}/challenge_B${B_CHALLENGE}_s${CHALLENGE_SEEDS}"
  MODEL_V2="$BEST_MODEL_A" \
  MODEL_V3="$BEST_MODEL_B" \
  B_CHALLENGE="$B_CHALLENGE" \
  SEEDS="$CHALLENGE_SEEDS" \
  SEED_START="$SCREEN_SEED_START" \
  TAG="$TAG" \
  OUT_ROOT="$CHALLENGE_OUT" \
  "$CHALLENGE_SCRIPT"
  echo "challenge 输出目录: $CHALLENGE_OUT"
  echo
else
  echo "== 阶段3: 跳过 challenge（RUN_CHALLENGE=$RUN_CHALLENGE）=="
fi

if [[ "$RUN_PASS" == "1" ]]; then
  echo "== 阶段4: pass 长时评测 =="
  PASS_OUT="${FINAL_DIR}/pass_B${B_PASS}_s${PASS_SEEDS}"
  MODEL_V2="$BEST_MODEL_A" \
  MODEL_V3="$BEST_MODEL_B" \
  B_PASS="$B_PASS" \
  SEEDS="$PASS_SEEDS" \
  SEED_START="$SCREEN_SEED_START" \
  TAG="$TAG" \
  OUT_ROOT="$PASS_OUT" \
  "$PASS_SCRIPT"
  echo "pass 输出目录: $PASS_OUT"
  echo
else
  echo "== 阶段4: 跳过 pass（RUN_PASS=$RUN_PASS）=="
fi

echo "== A* 24h 训练/评测流水线完成 =="
echo "总目录: $OUT_ROOT"
echo "日志: $LOG_FILE"
echo "建议检查:"
echo "  1) ${SCREEN_DIR}/screen_rank.csv"
echo "  2) ${FINAL_DIR}/challenge_B${B_CHALLENGE}_s${CHALLENGE_SEEDS}/challenge_compare_summary.csv"
echo "  3) ${FINAL_DIR}/pass_B${B_PASS}_s${PASS_SEEDS}/pass_compare_summary.csv"
