#!/usr/bin/env python3
import argparse
import csv
import heapq
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from map_utils import build_obstacle_mask, load_ros_map, world_to_map


def load_rows(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def filter_path(rows, algo, point_id):
    out = []
    for r in rows:
        if r.get("algo") != algo:
            continue
        if point_id and r.get("point_id") != point_id:
            continue
        out.append(r)
    out.sort(key=lambda x: int(float(x.get("seq", 0))))
    return [(float(r["x"]), float(r["y"])) for r in out]


def distance_transform(obstacle_mask):
    h = len(obstacle_mask)
    w = len(obstacle_mask[0])
    inf = 1e18
    dist = [[inf for _ in range(w)] for _ in range(h)]
    pq = []

    for y in range(h):
        for x in range(w):
            if obstacle_mask[y][x]:
                dist[y][x] = 0.0
                heapq.heappush(pq, (0.0, x, y))

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[y][x]:
            continue
        for dx, dy, wgt in neighbors:
            nx = x + dx
            ny = y + dy
            if nx < 0 or nx >= w or ny < 0 or ny >= h:
                continue
            nd = d + wgt
            if nd < dist[ny][nx]:
                dist[ny][nx] = nd
                heapq.heappush(pq, (nd, nx, ny))
    return dist


def sample_clearance(path_xy, dist_cells, resolution, origin):
    h = len(dist_cells)
    w = len(dist_cells[0])
    out = []
    for wx, wy in path_xy:
        mx, my = world_to_map(wx, wy, resolution, origin)
        if 0 <= mx < w and 0 <= my < h:
            out.append(dist_cells[my][mx] * resolution)
        else:
            out.append(0.0)
    return out


def stats(path_xy, clearances):
    if not path_xy:
        return {"path_length_m": 0.0, "min_clearance_m": 0.0, "mean_clearance_m": 0.0}
    length = 0.0
    for i in range(1, len(path_xy)):
        dx = path_xy[i][0] - path_xy[i - 1][0]
        dy = path_xy[i][1] - path_xy[i - 1][1]
        length += math.hypot(dx, dy)
    min_c = min(clearances) if clearances else 0.0
    mean_c = sum(clearances) / len(clearances) if clearances else 0.0
    return {"path_length_m": length, "min_clearance_m": min_c, "mean_clearance_m": mean_c}


def check_path_collision(path_xy, obstacle_mask, resolution, origin, label):
    """Warn if any path point falls inside an obstacle (e.g. placeholder data)."""
    h, w = len(obstacle_mask), len(obstacle_mask[0])
    hits = []
    for wx, wy in path_xy:
        mx, my = world_to_map(wx, wy, resolution, origin)
        if 0 <= mx < w and 0 <= my < h and obstacle_mask[my][mx]:
            hits.append((mx, my))
    if hits:
        print(
            f"[WARNING] {label}: {len(hits)} path point(s) inside obstacles (e.g. placeholder data). "
            "Use real planner output for valid comparison."
        )
    return len(hits)


def main():
    parser = argparse.ArgumentParser(description="Compare path quality between baseline and improved.")
    parser.add_argument("--map-yaml", required=True, help="ROS map yaml path")
    parser.add_argument("--trace", required=True, help="Path trace CSV")
    parser.add_argument("--point-id", default="", help="Optional point_id filter")
    parser.add_argument("--overlay-out", default="figures/fig_path_overlay.png", help="Output path overlay figure")
    parser.add_argument(
        "--clearance-out",
        default="figures/fig_clearance_profile.png",
        help="Output clearance profile figure",
    )
    parser.add_argument(
        "--allow-collision",
        action="store_true",
        help="Allow plotting even if path points fall inside obstacles",
    )
    args = parser.parse_args()

    cfg, grid, _, _ = load_ros_map(args.map_yaml)
    rows = load_rows(args.trace)
    baseline = filter_path(rows, "baseline", args.point_id)
    improved = filter_path(rows, "improved", args.point_id)
    if not baseline or not improved:
        raise SystemExit("Both baseline and improved paths are required after filtering.")

    occ = build_obstacle_mask(grid, cfg.get("occupied_thresh", 0.65), cfg.get("negate", 0))
    dist_cells = distance_transform(occ)
    resolution = cfg.get("resolution", 0.05)
    origin = cfg.get("origin", [0.0, 0.0, 0.0])

    hit_b = check_path_collision(baseline, occ, resolution, origin, "baseline")
    hit_i = check_path_collision(improved, occ, resolution, origin, "improved")
    if (hit_b > 0 or hit_i > 0) and not args.allow_collision:
        raise SystemExit(
            "Collision points detected in input path trace. "
            "Please use real planner output or rerun with --allow-collision for debugging."
        )

    cb = sample_clearance(baseline, dist_cells, resolution, origin)
    ci = sample_clearance(improved, dist_cells, resolution, origin)
    sb = stats(baseline, cb)
    si = stats(improved, ci)

    # Overlay figure
    arr = np.array(grid, dtype=np.uint8)
    plt.figure(figsize=(10, 4))
    plt.imshow(np.flipud(arr), cmap="gray", origin="lower", vmin=0, vmax=255)
    bmap = [world_to_map(p[0], p[1], resolution, origin) for p in baseline]
    imap = [world_to_map(p[0], p[1], resolution, origin) for p in improved]
    bx = [p[0] for p in bmap]
    by = [p[1] for p in bmap]
    ix = [p[0] for p in imap]
    iy = [p[1] for p in imap]
    plt.plot(bx, by, "-o", markersize=2, linewidth=1.2, label="baseline", color="#1f77b4")
    plt.plot(ix, iy, "-o", markersize=2, linewidth=1.2, label="improved", color="#ff7f0e")
    plt.title("Path Overlay (map frame)")
    plt.xlabel("mx")
    plt.ylabel("my")
    plt.legend()
    plt.tight_layout()
    overlay_out = Path(args.overlay_out)
    overlay_out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(overlay_out, dpi=170)
    plt.close()

    # Clearance profile
    plt.figure(figsize=(8, 4))
    plt.plot(range(len(cb)), cb, label="baseline", color="#1f77b4")
    plt.plot(range(len(ci)), ci, label="improved", color="#ff7f0e")
    plt.title("Clearance Profile Along Path")
    plt.xlabel("path sample index")
    plt.ylabel("clearance (m)")
    plt.legend()
    plt.tight_layout()
    clearance_out = Path(args.clearance_out)
    clearance_out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(clearance_out, dpi=170)
    plt.close()

    print(f"Saved: {overlay_out}")
    print(f"Saved: {clearance_out}")
    print(
        "baseline_stats "
        f"path_length_m={sb['path_length_m']:.3f} "
        f"min_clearance_m={sb['min_clearance_m']:.3f} "
        f"mean_clearance_m={sb['mean_clearance_m']:.3f}"
    )
    print(
        "improved_stats "
        f"path_length_m={si['path_length_m']:.3f} "
        f"min_clearance_m={si['min_clearance_m']:.3f} "
        f"mean_clearance_m={si['mean_clearance_m']:.3f}"
    )


if __name__ == "__main__":
    main()
