#!/usr/bin/env python3
import math
from pathlib import Path


def load_map_yaml(yaml_path):
    data = {}
    with Path(yaml_path).open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, val = line.split(":", 1)
            key = key.strip()
            val = val.strip()
            if key == "origin":
                val = val.strip("[]")
                data[key] = [float(x.strip()) for x in val.split(",")]
            elif key in ("resolution", "occupied_thresh", "free_thresh"):
                data[key] = float(val)
            elif key in ("negate",):
                data[key] = int(float(val))
            else:
                data[key] = val
    return data


def _read_tokens(f):
    for raw in f:
        line = raw.strip()
        if not line or line.startswith(b"#"):
            continue
        for token in line.split():
            yield token


def load_pgm(path):
    path = Path(path)
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"Unsupported PGM format: {magic!r}")

        tokens = _read_tokens(f)
        w = int(next(tokens))
        h = int(next(tokens))
        max_val = int(next(tokens))
        if max_val <= 0:
            raise ValueError("Invalid PGM max value")

        if magic == b"P2":
            vals = [int(next(tokens)) for _ in range(w * h)]
        else:
            raw = f.read(w * h)
            if len(raw) != w * h:
                raise ValueError("Unexpected P5 data length")
            vals = list(raw)

    grid = [vals[r * w : (r + 1) * w] for r in range(h)]
    return grid, w, h


def load_ros_map(map_yaml_path):
    cfg = load_map_yaml(map_yaml_path)
    yaml_path = Path(map_yaml_path)
    image_path = cfg["image"]
    if not Path(image_path).is_absolute():
        image_path = yaml_path.parent / image_path
    grid, w, h = load_pgm(image_path)
    return cfg, grid, w, h


def pixel_to_occ_prob(pixel_value, negate):
    if negate:
        return pixel_value / 255.0
    return (255.0 - pixel_value) / 255.0


def build_obstacle_mask(grid, occupied_thresh=0.65, negate=0):
    h = len(grid)
    w = len(grid[0])
    mask = [[False for _ in range(w)] for _ in range(h)]
    for y in range(h):
        for x in range(w):
            p = pixel_to_occ_prob(grid[y][x], negate)
            mask[y][x] = p >= occupied_thresh
    return mask


def map_to_world(mx, my, resolution, origin):
    wx = origin[0] + (mx + 0.5) * resolution
    wy = origin[1] + (my + 0.5) * resolution
    return wx, wy


def world_to_map(wx, wy, resolution, origin):
    mx = int(math.floor((wx - origin[0]) / resolution))
    my = int(math.floor((wy - origin[1]) / resolution))
    return mx, my
