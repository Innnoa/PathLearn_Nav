#!/usr/bin/env python3
"""Generate NN training samples from map trajectories."""

import argparse
import csv
import heapq
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


@dataclass
class CircleObstacle:
    x: float
    y: float
    radius: float


@dataclass
class MapData:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    obstacles: List[CircleObstacle]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate NN dataset from A* trajectories on map")
    parser.add_argument("--map", required=True, help="PathLearn map file")
    parser.add_argument("--out", required=True, help="Output CSV path")
    parser.add_argument("--rows", type=int, default=12000, help="Target row count")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--grid", type=float, default=0.3, help="Grid resolution")
    parser.add_argument("--robot-radius", type=float, default=0.2, help="Robot radius")
    parser.add_argument(
        "--min-goal-dist",
        type=float,
        default=18.0,
        help="Minimum start-goal distance in world units")
    parser.add_argument(
        "--path-step",
        type=int,
        default=1,
        help="Keep every N-th path segment to reduce correlation")
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=20000,
        help="Maximum random start-goal attempts")
    parser.add_argument(
        "--diag",
        choices=["on", "off"],
        default="on",
        help="Use 8-neighbor or 4-neighbor A*")
    parser.add_argument(
        "--progress-every",
        type=int,
        default=1000,
        help="Progress print interval by generated rows")
    parser.add_argument("--val-out", default="", help="Optional validation CSV path")
    parser.add_argument("--test-out", default="", help="Optional test CSV path")
    parser.add_argument(
        "--val-ratio",
        type=float,
        default=0.1,
        help="Validation split ratio when --val-out is set")
    parser.add_argument(
        "--test-ratio",
        type=float,
        default=0.1,
        help="Test split ratio when --test-out is set")
    return parser.parse_args()


def load_map(path: Path) -> MapData:
    tokens = path.read_text(encoding="utf-8").split()
    min_x = max_x = min_y = max_y = 0.0
    has_bounds = False
    obstacles: List[CircleObstacle] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        index += 1
        if token == "name":
            if index >= len(tokens):
                raise ValueError("invalid map: missing name value")
            index += 1
            continue
        if token == "bounds":
            if index + 3 >= len(tokens):
                raise ValueError("invalid map: incomplete bounds")
            min_x = float(tokens[index + 0])
            max_x = float(tokens[index + 1])
            min_y = float(tokens[index + 2])
            max_y = float(tokens[index + 3])
            has_bounds = True
            index += 4
            continue
        if token == "obstacles":
            if index >= len(tokens):
                raise ValueError("invalid map: missing obstacle count")
            count = int(tokens[index])
            index += 1
            for _ in range(count):
                if index + 3 >= len(tokens):
                    raise ValueError("invalid map: incomplete obstacle data")
                obstacle_type = tokens[index]
                index += 1
                if obstacle_type != "circle":
                    raise ValueError("only circle obstacles are supported")
                cx = float(tokens[index + 0])
                cy = float(tokens[index + 1])
                radius = float(tokens[index + 2])
                index += 3
                obstacles.append(CircleObstacle(cx, cy, radius))
            continue
        raise ValueError(f"invalid map: unknown token '{token}'")

    if not has_bounds:
        raise ValueError("invalid map: missing bounds")
    if min_x >= max_x or min_y >= max_y:
        raise ValueError("invalid map: bounds are not increasing")
    return MapData(min_x, max_x, min_y, max_y, obstacles)


def world_distance(ax: float, ay: float, bx: float, by: float) -> float:
    dx = ax - bx
    dy = ay - by
    return math.sqrt(dx * dx + dy * dy)


class GridWorld:
    def __init__(
        self,
        map_data: MapData,
        resolution: float,
        robot_radius: float,
        allow_diag: bool,
    ) -> None:
        self.map = map_data
        self.resolution = resolution
        self.robot_radius = max(0.0, robot_radius)
        self.allow_diag = allow_diag

        span_x = self.map.max_x - self.map.min_x
        span_y = self.map.max_y - self.map.min_y
        self.width = int(math.floor(span_x / self.resolution)) + 1
        self.height = int(math.floor(span_y / self.resolution)) + 1
        if self.width < 2 or self.height < 2:
            raise ValueError("grid too small")

        self.cell_count = self.width * self.height
        self.blocked = [False] * self.cell_count
        self.free_indices: List[int] = []
        self.neighbors: List[List[Tuple[int, float]]] = [
            [] for _ in range(self.cell_count)
        ]

        self._build_occupancy()
        self._build_neighbors()
        if not self.free_indices:
            raise ValueError("no free cell in grid")

    def index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def coords(self, index: int) -> Tuple[int, int]:
        gx = index % self.width
        gy = index // self.width
        return gx, gy

    def to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.map.min_x + gx * self.resolution
        y = self.map.min_y + gy * self.resolution
        return x, y

    def from_world(self, x: float, y: float) -> int:
        gx = int(round((x - self.map.min_x) / self.resolution))
        gy = int(round((y - self.map.min_y) / self.resolution))
        gx = max(0, min(self.width - 1, gx))
        gy = max(0, min(self.height - 1, gy))
        return self.index(gx, gy)

    def _is_occupied(self, x: float, y: float) -> bool:
        if x < self.map.min_x or x > self.map.max_x:
            return True
        if y < self.map.min_y or y > self.map.max_y:
            return True
        for obstacle in self.map.obstacles:
            if world_distance(x, y, obstacle.x, obstacle.y) <= (
                obstacle.radius + self.robot_radius
            ):
                return True
        return False

    def _build_occupancy(self) -> None:
        for gy in range(self.height):
            for gx in range(self.width):
                idx = self.index(gx, gy)
                x, y = self.to_world(gx, gy)
                blocked = self._is_occupied(x, y)
                self.blocked[idx] = blocked
                if not blocked:
                    self.free_indices.append(idx)

    def _build_neighbors(self) -> None:
        offsets_4 = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        offsets_diag = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
        offsets = offsets_4 + (offsets_diag if self.allow_diag else [])
        for gy in range(self.height):
            for gx in range(self.width):
                idx = self.index(gx, gy)
                if self.blocked[idx]:
                    continue
                neighbors: List[Tuple[int, float]] = []
                for dx, dy in offsets:
                    nx = gx + dx
                    ny = gy + dy
                    if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                        continue
                    nidx = self.index(nx, ny)
                    if self.blocked[nidx]:
                        continue
                    px, py = self.to_world(gx, gy)
                    qx, qy = self.to_world(nx, ny)
                    neighbors.append((nidx, world_distance(px, py, qx, qy)))
                self.neighbors[idx] = neighbors


def astar_path(grid: GridWorld, start_idx: int, goal_idx: int) -> Optional[List[int]]:
    if start_idx == goal_idx:
        return [start_idx]
    if grid.blocked[start_idx] or grid.blocked[goal_idx]:
        return None

    inf = float("inf")
    g_cost = [inf] * grid.cell_count
    parent = [-1] * grid.cell_count
    closed = [False] * grid.cell_count
    heap: List[Tuple[float, float, int, int]] = []

    sx, sy = grid.to_world(*grid.coords(start_idx))
    gx, gy = grid.to_world(*grid.coords(goal_idx))

    def heuristic(index: int) -> float:
        x, y = grid.to_world(*grid.coords(index))
        return world_distance(x, y, gx, gy)

    serial = 0
    g_cost[start_idx] = 0.0
    heapq.heappush(heap, (heuristic(start_idx), 0.0, serial, start_idx))
    serial += 1

    while heap:
        _, g_now, _, cur = heapq.heappop(heap)
        if closed[cur]:
            continue
        if g_now > g_cost[cur] + 1e-9:
            continue
        closed[cur] = True
        if cur == goal_idx:
            path: List[int] = []
            cursor = goal_idx
            while cursor >= 0:
                path.append(cursor)
                if cursor == start_idx:
                    break
                cursor = parent[cursor]
            if not path or path[-1] != start_idx:
                return None
            path.reverse()
            return path

        for nxt, step in grid.neighbors[cur]:
            if closed[nxt]:
                continue
            cand = g_cost[cur] + step
            if cand + 1e-9 >= g_cost[nxt]:
                continue
            g_cost[nxt] = cand
            parent[nxt] = cur
            heapq.heappush(heap, (cand + heuristic(nxt), cand, serial, nxt))
            serial += 1
    return None


def nearest_two_obstacles(
    obstacles: Sequence[CircleObstacle], x: float, y: float
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    if not obstacles:
        return (0.0, 0.0), (0.0, 0.0)
    ranked = sorted(
        obstacles,
        key=lambda obs: (obs.x - x) * (obs.x - x) + (obs.y - y) * (obs.y - y),
    )
    obs1 = ranked[0]
    obs2 = ranked[1] if len(ranked) > 1 else None
    p1 = (obs1.x - x, obs1.y - y)
    p2 = (0.0, 0.0) if obs2 is None else (obs2.x - x, obs2.y - y)
    return p1, p2


def path_to_rows(
    grid: GridWorld,
    path: Sequence[int],
    goal_idx: int,
    step: int,
) -> List[dict]:
    rows: List[dict] = []
    if len(path) < 2:
        return rows
    step = max(1, step)
    gx, gy = grid.to_world(*grid.coords(goal_idx))
    for i in range(0, len(path) - 1, step):
        cur = path[i]
        nxt = path[min(i + 1, len(path) - 1)]
        cx, cy = grid.to_world(*grid.coords(cur))
        nx, ny = grid.to_world(*grid.coords(nxt))
        tx = nx - cx
        ty = ny - cy
        if abs(tx) + abs(ty) < 1e-9:
            continue
        (obs1_dx, obs1_dy), (obs2_dx, obs2_dy) = nearest_two_obstacles(
            grid.map.obstacles, cx, cy
        )
        rows.append(
            {
                "current_x": f"{cx:.6f}",
                "current_y": f"{cy:.6f}",
                "goal_x": f"{gx:.6f}",
                "goal_y": f"{gy:.6f}",
                "obs1_dx": f"{obs1_dx:.6f}",
                "obs1_dy": f"{obs1_dy:.6f}",
                "obs2_dx": f"{obs2_dx:.6f}",
                "obs2_dy": f"{obs2_dy:.6f}",
                "target_dx": f"{tx:.6f}",
                "target_dy": f"{ty:.6f}",
            }
        )
    return rows


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
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
    ]
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def split_rows(
    rows: List[dict],
    val_ratio: float,
    test_ratio: float,
    rng: random.Random,
) -> Tuple[List[dict], List[dict], List[dict]]:
    val_ratio = max(0.0, min(0.8, val_ratio))
    test_ratio = max(0.0, min(0.8, test_ratio))
    if val_ratio + test_ratio >= 0.95:
        test_ratio = max(0.0, 0.95 - val_ratio)
    shuffled = list(rows)
    rng.shuffle(shuffled)
    total = len(shuffled)
    val_n = int(total * val_ratio)
    test_n = int(total * test_ratio)
    train_n = total - val_n - test_n
    train = shuffled[:train_n]
    val = shuffled[train_n : train_n + val_n]
    test = shuffled[train_n + val_n :]
    return train, val, test


def main() -> None:
    args = parse_args()
    if args.rows < 1:
        raise SystemExit("--rows must be >= 1")
    if args.grid <= 0.0:
        raise SystemExit("--grid must be > 0")

    rng = random.Random(args.seed)
    map_data = load_map(Path(args.map))
    grid = GridWorld(
        map_data=map_data,
        resolution=args.grid,
        robot_radius=args.robot_radius,
        allow_diag=(args.diag == "on"),
    )

    min_dist = max(1.0, args.min_goal_dist)
    rows: List[dict] = []
    attempts = 0
    success_paths = 0
    while len(rows) < args.rows and attempts < args.max_attempts:
        attempts += 1
        start_idx = rng.choice(grid.free_indices)
        goal_idx = rng.choice(grid.free_indices)
        if start_idx == goal_idx:
            continue
        sx, sy = grid.to_world(*grid.coords(start_idx))
        gx, gy = grid.to_world(*grid.coords(goal_idx))
        if world_distance(sx, sy, gx, gy) < min_dist:
            continue
        path = astar_path(grid, start_idx, goal_idx)
        if not path or len(path) < 2:
            continue

        sample_rows = path_to_rows(grid, path, goal_idx, args.path_step)
        if not sample_rows:
            continue
        rows.extend(sample_rows)
        success_paths += 1

        if args.progress_every > 0:
            last_chunk = len(rows) // args.progress_every
            prev_chunk = (len(rows) - len(sample_rows)) // args.progress_every
            if last_chunk > prev_chunk:
                print(
                    f"rows={len(rows)} paths={success_paths} attempts={attempts}"
                )

    if len(rows) < args.rows:
        raise SystemExit(
            f"insufficient rows: {len(rows)} < {args.rows}, "
            f"try larger --max-attempts or smaller --min-goal-dist"
        )
    rows = rows[: args.rows]

    out_path = Path(args.out)
    if args.val_out or args.test_out:
        train_rows, val_rows, test_rows = split_rows(
            rows,
            val_ratio=args.val_ratio,
            test_ratio=args.test_ratio,
            rng=rng,
        )
        write_csv(out_path, train_rows)
        if args.val_out:
            write_csv(Path(args.val_out), val_rows)
        if args.test_out:
            write_csv(Path(args.test_out), test_rows)
        print(
            "generated"
            f" train={len(train_rows)} val={len(val_rows)} test={len(test_rows)}"
            f" paths={success_paths} attempts={attempts}"
        )
    else:
        write_csv(out_path, rows)
        print(
            f"generated rows={len(rows)} paths={success_paths} attempts={attempts}"
        )


if __name__ == "__main__":
    main()
