#!/usr/bin/env python3
"""对 pathlearn_eval 的 runs.csv 做成功样本统计。"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path
from typing import Dict, List


def _mean(values: List[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def _pctl(sorted_values: List[float], p: float) -> float:
    if not sorted_values:
        return 0.0
    if len(sorted_values) == 1:
        return sorted_values[0]
    pos = (len(sorted_values) - 1) * p
    lo = int(pos)
    hi = min(lo + 1, len(sorted_values) - 1)
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def analyze_runs(path: Path) -> Dict[str, float]:
    rows: List[Dict[str, str]] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    total_runs = len(rows)
    success_rows = [r for r in rows if int(r.get("status_code", "-1")) == 0]
    success_runs = len(success_rows)

    success_rate = (float(success_runs) / float(total_runs)) if total_runs > 0 else 0.0
    collision_runs = sum(
        1
        for r in rows
        if float(r.get("collision_rate", "0") or "0") > 0.5
    )
    collision_rate = (
        float(collision_runs) / float(total_runs) if total_runs > 0 else 0.0
    )

    all_first_iters = [
        float(r.get("first_solution_iteration", "0") or "0") for r in rows
    ]
    succ_first_iters = [
        float(r.get("first_solution_iteration", "0") or "0")
        for r in success_rows
        if float(r.get("first_solution_iteration", "0") or "0") > 0.0
    ]
    succ_exec_time = [
        float(r.get("avg_exec_time_sec", "0") or "0") for r in success_rows
    ]
    succ_path_len = [
        float(r.get("avg_path_length", "0") or "0") for r in success_rows
    ]
    succ_min_safe = [
        float(r.get("min_safety_distance", "0") or "0") for r in success_rows
    ]

    succ_first_iters_sorted = sorted(succ_first_iters)
    succ_iter_median = statistics.median(succ_first_iters) if succ_first_iters else 0.0

    return {
        "total_runs": float(total_runs),
        "success_runs": float(success_runs),
        "success_rate": success_rate,
        "collision_rate": collision_rate,
        "avg_first_iter_all": _mean(all_first_iters),
        "avg_first_iter_success_only": _mean(succ_first_iters),
        "median_first_iter_success_only": succ_iter_median,
        "p90_first_iter_success_only": _pctl(succ_first_iters_sorted, 0.9),
        "avg_exec_time_success_only": _mean(succ_exec_time),
        "avg_path_length_success_only": _mean(succ_path_len),
        "avg_min_safety_success_only": _mean(succ_min_safe),
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="统计 runs.csv（重点：仅成功样本首解迭代数）"
    )
    parser.add_argument(
        "--runs",
        nargs="+",
        required=True,
        help="一个或多个 *_runs.csv 路径",
    )
    parser.add_argument(
        "--labels",
        nargs="*",
        default=[],
        help="每个 runs 的显示标签，数量需与 --runs 相同",
    )
    parser.add_argument(
        "--out",
        default="",
        help="可选：输出汇总 CSV 路径",
    )
    args = parser.parse_args()

    run_paths = [Path(p) for p in args.runs]
    for p in run_paths:
        if not p.is_file():
            raise FileNotFoundError(f"未找到 runs 文件: {p}")

    labels = args.labels
    if labels and len(labels) != len(run_paths):
        raise ValueError("--labels 数量需与 --runs 一致")
    if not labels:
        labels = [p.stem for p in run_paths]

    records = []
    for label, path in zip(labels, run_paths):
        stats = analyze_runs(path)
        stats["label"] = label
        stats["runs_path"] = str(path)
        records.append(stats)

    fields = [
        "label",
        "runs_path",
        "total_runs",
        "success_runs",
        "success_rate",
        "collision_rate",
        "avg_first_iter_all",
        "avg_first_iter_success_only",
        "median_first_iter_success_only",
        "p90_first_iter_success_only",
        "avg_exec_time_success_only",
        "avg_path_length_success_only",
        "avg_min_safety_success_only",
    ]

    print(",".join(fields))
    for rec in records:
        print(
            ",".join(
                [
                    str(rec.get("label", "")),
                    str(rec.get("runs_path", "")),
                    f"{rec['total_runs']:.0f}",
                    f"{rec['success_runs']:.0f}",
                    f"{rec['success_rate']:.6f}",
                    f"{rec['collision_rate']:.6f}",
                    f"{rec['avg_first_iter_all']:.6f}",
                    f"{rec['avg_first_iter_success_only']:.6f}",
                    f"{rec['median_first_iter_success_only']:.6f}",
                    f"{rec['p90_first_iter_success_only']:.6f}",
                    f"{rec['avg_exec_time_success_only']:.6f}",
                    f"{rec['avg_path_length_success_only']:.6f}",
                    f"{rec['avg_min_safety_success_only']:.6f}",
                ]
            )
        )

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            for rec in records:
                row = dict(rec)
                row["total_runs"] = int(rec["total_runs"])
                row["success_runs"] = int(rec["success_runs"])
                writer.writerow(row)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
