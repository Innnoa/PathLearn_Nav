#!/usr/bin/env python3
"""训练最小 NN 引导采样模型并导出为 PathLearn 文本格式。"""

import argparse
import csv
import math
import random
from pathlib import Path
from typing import List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train minimal NN sampler model")
    parser.add_argument("--in", dest="input_csv", required=True, help="输入轨迹 CSV")
    parser.add_argument("--out", dest="output_model", required=True, help="输出模型路径")
    parser.add_argument("--epochs", type=int, default=80, help="训练轮数")
    parser.add_argument("--lr", type=float, default=0.05, help="学习率")
    parser.add_argument("--hidden", type=int, default=16, help="隐藏层维度")
    parser.add_argument("--seed", type=int, default=42, help="随机种子")
    return parser.parse_args()


def relu(x: float) -> float:
    return x if x > 0.0 else 0.0


def relu_grad(x: float) -> float:
    return 1.0 if x > 0.0 else 0.0


def normalize(dx: float, dy: float) -> Tuple[float, float]:
    norm = math.sqrt(dx * dx + dy * dy)
    if norm < 1e-9:
        return 1.0, 0.0
    return dx / norm, dy / norm


def load_dataset(path: Path) -> List[Tuple[List[float], List[float]]]:
    rows: List[Tuple[List[float], List[float]]] = []
    with path.open("r", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        required = {
            "current_x",
            "current_y",
            "goal_x",
            "goal_y",
            "obs1_dx",
            "obs1_dy",
            "obs2_dx",
            "obs2_dy",
            "target_dx",
            "target_dy",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV 缺少字段: {sorted(missing)}")

        for line in reader:
            current_x = float(line["current_x"])
            current_y = float(line["current_y"])
            goal_x = float(line["goal_x"])
            goal_y = float(line["goal_y"])
            obs1_dx = float(line["obs1_dx"])
            obs1_dy = float(line["obs1_dy"])
            obs2_dx = float(line["obs2_dx"])
            obs2_dy = float(line["obs2_dy"])
            target_dx = float(line["target_dx"])
            target_dy = float(line["target_dy"])

            goal_dx = goal_x - current_x
            goal_dy = goal_y - current_y
            goal_ux, goal_uy = normalize(goal_dx, goal_dy)
            target_ux, target_uy = normalize(target_dx, target_dy)

            features = [
                goal_ux,
                goal_uy,
                goal_dx / 100.0,
                goal_dy / 100.0,
                obs1_dx / 100.0,
                obs1_dy / 100.0,
                obs2_dx / 100.0,
                obs2_dy / 100.0,
            ]
            label = [target_ux, target_uy]
            rows.append((features, label))
    return rows


def init_matrix(rows: int, cols: int, rng: random.Random) -> List[List[float]]:
    scale = 1.0 / math.sqrt(max(1, cols))
    return [[rng.uniform(-scale, scale) for _ in range(cols)] for _ in range(rows)]


def init_vector(size: int) -> List[float]:
    return [0.0 for _ in range(size)]


def matvec_mul(matrix: List[List[float]], vector: List[float]) -> List[float]:
    return [sum(w * x for w, x in zip(row, vector)) for row in matrix]


def add_bias(vector: List[float], bias: List[float]) -> List[float]:
    return [v + b for v, b in zip(vector, bias)]


def train_model(
    dataset: List[Tuple[List[float], List[float]]],
    epochs: int,
    lr: float,
    hidden_dim: int,
    seed: int,
) -> Tuple[List[List[float]], List[float], List[List[float]], List[float]]:
    rng = random.Random(seed)
    input_dim = 8
    output_dim = 2
    w1 = init_matrix(hidden_dim, input_dim, rng)
    b1 = init_vector(hidden_dim)
    w2 = init_matrix(output_dim, hidden_dim, rng)
    b2 = init_vector(output_dim)

    for epoch in range(epochs):
        rng.shuffle(dataset)
        total_loss = 0.0
        for features, target in dataset:
            z1 = add_bias(matvec_mul(w1, features), b1)
            h1 = [relu(v) for v in z1]
            pred = add_bias(matvec_mul(w2, h1), b2)

            error = [pred[i] - target[i] for i in range(output_dim)]
            loss = sum(e * e for e in error) / output_dim
            total_loss += loss

            grad_pred = [2.0 * e / output_dim for e in error]

            grad_w2 = [[grad_pred[o] * h1[i] for i in range(hidden_dim)]
                       for o in range(output_dim)]
            grad_b2 = grad_pred[:]

            grad_h1 = [0.0 for _ in range(hidden_dim)]
            for i in range(hidden_dim):
                grad_h1[i] = sum(w2[o][i] * grad_pred[o] for o in range(output_dim))

            grad_z1 = [grad_h1[i] * relu_grad(z1[i]) for i in range(hidden_dim)]
            grad_w1 = [[grad_z1[h] * features[i] for i in range(input_dim)]
                       for h in range(hidden_dim)]
            grad_b1 = grad_z1[:]

            for o in range(output_dim):
                for i in range(hidden_dim):
                    w2[o][i] -= lr * grad_w2[o][i]
                b2[o] -= lr * grad_b2[o]

            for h in range(hidden_dim):
                for i in range(input_dim):
                    w1[h][i] -= lr * grad_w1[h][i]
                b1[h] -= lr * grad_b1[h]

        avg_loss = total_loss / max(1, len(dataset))
        if epoch % max(1, epochs // 8) == 0 or epoch + 1 == epochs:
            print(f"epoch={epoch + 1} loss={avg_loss:.6f}")

    return w1, b1, w2, b2


def flatten(matrix: List[List[float]]) -> List[float]:
    values: List[float] = []
    for row in matrix:
        values.extend(row)
    return values


def save_model(path: Path, w1, b1, w2, b2):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        file.write("PATHLEARN_NN 1 2\n")
        file.write(f"layer {len(w1[0])} {len(w1)}\n")
        file.write(" ".join(f"{v:.8f}" for v in flatten(w1)) + "\n")
        file.write(" ".join(f"{v:.8f}" for v in b1) + "\n")
        file.write(f"layer {len(w2[0])} {len(w2)}\n")
        file.write(" ".join(f"{v:.8f}" for v in flatten(w2)) + "\n")
        file.write(" ".join(f"{v:.8f}" for v in b2) + "\n")


def main() -> None:
    args = parse_args()
    dataset = load_dataset(Path(args.input_csv))
    if not dataset:
        raise SystemExit("训练数据为空")

    w1, b1, w2, b2 = train_model(
        dataset=dataset,
        epochs=max(1, args.epochs),
        lr=max(1e-5, args.lr),
        hidden_dim=max(2, args.hidden),
        seed=args.seed,
    )
    save_model(Path(args.output_model), w1, b1, w2, b2)
    print(f"saved={args.output_model}")


if __name__ == "__main__":
    main()
