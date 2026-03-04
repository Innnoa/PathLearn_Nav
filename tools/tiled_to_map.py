#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Convert Tiled JSON to PathLearn map.")
    parser.add_argument("--in", dest="input_path", required=True, help="Tiled JSON map file")
    parser.add_argument("--out", dest="output_path", required=True, help="Output map txt path")
    parser.add_argument(
        "--bounds",
        nargs=4,
        type=float,
        metavar=("MIN_X", "MAX_X", "MIN_Y", "MAX_Y"),
        help="Override bounds; if omitted, derived from map size",
    )
    parser.add_argument("--scale", type=float, default=1.0, help="Scale factor for coordinates")
    parser.add_argument(
        "--radius",
        type=float,
        default=0.0,
        help="Circle radius for each wall tile (0 -> auto)",
    )
    parser.add_argument(
        "--layer",
        type=str,
        default="",
        help="Only convert the specified tile layer name",
    )
    parser.add_argument(
        "--tile-subdiv",
        type=int,
        default=1,
        help="Subdivide each wall tile into N x N circle obstacles",
    )
    return parser.parse_args()


def load_tiled(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def get_layers(data, target_name):
    layers = []
    for layer in data.get("layers", []):
        if layer.get("type") != "tilelayer":
            continue
        if target_name and layer.get("name") != target_name:
            continue
        layers.append(layer)
    return layers


def compute_bounds(width, height, tile_w, tile_h, scale):
    world_w = width * tile_w * scale
    world_h = height * tile_h * scale
    return (-world_w / 2.0, world_w / 2.0, -world_h / 2.0, world_h / 2.0)


def tile_center(col, row, width, height, tile_w, tile_h, scale):
    x = (col + 0.5 - width / 2.0) * tile_w * scale
    y = (height / 2.0 - row - 0.5) * tile_h * scale
    return x, y


def main():
    args = parse_args()
    data = load_tiled(args.input_path)

    width = int(data.get("width", 0))
    height = int(data.get("height", 0))
    tile_w = int(data.get("tilewidth", 0))
    tile_h = int(data.get("tileheight", 0))
    if width <= 0 or height <= 0 or tile_w <= 0 or tile_h <= 0:
        raise SystemExit("Invalid Tiled map size or tile size.")

    layers = get_layers(data, args.layer)
    if not layers:
        raise SystemExit("No tile layers found. Ensure at least one tile layer exists.")

    if args.bounds:
        min_x, max_x, min_y, max_y = args.bounds
    else:
        min_x, max_x, min_y, max_y = compute_bounds(width, height, tile_w, tile_h, args.scale)

    if args.tile_subdiv < 1:
        raise SystemExit("--tile-subdiv must be >= 1.")

    radius = args.radius
    tile_size_world = min(tile_w, tile_h) * args.scale
    if radius <= 0.0:
        if args.tile_subdiv > 1:
            radius = tile_size_world * 0.6 / args.tile_subdiv
        else:
            radius = tile_size_world * 0.55

    obstacles = []
    seen = set()

    for layer in layers:
        if layer.get("encoding"):
            raise SystemExit("Only supports raw array tile data (no encoding).")
        tiles = layer.get("data", [])
        if len(tiles) != width * height:
            raise SystemExit("Tile data size mismatch.")
        for idx, gid in enumerate(tiles):
            if gid == 0:
                continue
            col = idx % width
            row = idx // width
            x, y = tile_center(col, row, width, height, tile_w, tile_h, args.scale)
            if args.tile_subdiv == 1:
                key = (round(x, 3), round(y, 3), round(radius, 3))
                if key in seen:
                    continue
                seen.add(key)
                obstacles.append((x, y, radius))
                continue

            step_x = tile_w * args.scale / args.tile_subdiv
            step_y = tile_h * args.scale / args.tile_subdiv
            for iy in range(args.tile_subdiv):
                for ix in range(args.tile_subdiv):
                    ox = (ix + 0.5 - args.tile_subdiv / 2.0) * step_x
                    oy = (iy + 0.5 - args.tile_subdiv / 2.0) * step_y
                    cx = x + ox
                    cy = y + oy
                    key = (round(cx, 3), round(cy, 3), round(radius, 3))
                    if key in seen:
                        continue
                    seen.add(key)
                    obstacles.append((cx, cy, radius))

    output_path = Path(args.output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"name {output_path.stem}",
        f"bounds {min_x:.1f} {max_x:.1f} {min_y:.1f} {max_y:.1f}",
        f"obstacles {len(obstacles)}",
    ]
    for x, y, r in obstacles:
        lines.append(f"circle {x:.3f} {y:.3f} {r:.3f}")

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"converted={len(obstacles)}")
    print(f"bounds=({min_x:.1f},{max_x:.1f},{min_y:.1f},{max_y:.1f})")
    print(f"radius={radius:.3f}")


if __name__ == "__main__":
    main()
