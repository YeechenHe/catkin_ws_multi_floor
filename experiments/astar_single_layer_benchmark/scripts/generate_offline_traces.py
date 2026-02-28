#!/usr/bin/env python3
import argparse
import csv
import heapq
import math
from pathlib import Path

from map_utils import build_obstacle_mask, load_ros_map, map_to_world, world_to_map


def load_point(points_csv, point_id):
    with Path(points_csv).open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("point_id") == point_id:
                return row
    raise RuntimeError(f"point_id not found: {point_id}")


def obstacle_distance_cells(occ):
    h = len(occ)
    w = len(occ[0])
    inf = 10**9
    dist = [[inf for _ in range(w)] for _ in range(h)]
    pq = []
    for y in range(h):
        for x in range(w):
            if occ[y][x]:
                dist[y][x] = 0
                heapq.heappush(pq, (0, x, y))
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d != dist[y][x]:
            continue
        for dx, dy in nbrs:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            nd = d + 1
            if nd < dist[ny][nx]:
                dist[ny][nx] = nd
                heapq.heappush(pq, (nd, nx, ny))
    return dist


def heuristic(a, b, diagonal=False):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if diagonal:
        return max(dx, dy) + 0.41421356 * min(dx, dy)
    return dx + dy


def neighbors_baseline(x, y, w, h):
    for dx, dy, c in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)):
        nx, ny = x + dx, y + dy
        if 0 <= nx < w and 0 <= ny < h:
            yield nx, ny, c


def neighbors_improved(x, y, w, h, goal):
    # 4-neighbor + 目标方向两条对角（6邻域）
    base = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    dxg = goal[0] - x
    dyg = goal[1] - y
    if abs(dxg) >= abs(dyg) and dxg != 0:
        sx = 1 if dxg > 0 else -1
        base.append((sx, 1, 1.41421356))
        base.append((sx, -1, 1.41421356))
    elif dyg != 0:
        sy = 1 if dyg > 0 else -1
        base.append((1, sy, 1.41421356))
        base.append((-1, sy, 1.41421356))
    for dx, dy, c in base:
        nx, ny = x + dx, y + dy
        if 0 <= nx < w and 0 <= ny < h:
            yield nx, ny, c


def astar(occ, start, goal, mode, dist_to_obs):
    h = len(occ)
    w = len(occ[0])
    if occ[start[1]][start[0]] or occ[goal[1]][goal[0]]:
        raise RuntimeError("start or goal lies in obstacle")

    g_cost = {start: 0.0}
    parent = {start: None}
    pq = [(0.0, start)]
    visited = set()
    expansions = []

    while pq:
        _, cur = heapq.heappop(pq)
        if cur in visited:
            continue
        visited.add(cur)
        expansions.append(cur)
        if cur == goal:
            break

        if mode == "baseline":
            nbrs = neighbors_baseline(cur[0], cur[1], w, h)
            alpha = 1.0
            use_diag_h = False
            obs_w = 0.0
        else:
            nbrs = neighbors_improved(cur[0], cur[1], w, h, goal)
            alpha = 1.12
            use_diag_h = True
            obs_w = 0.35

        for nx, ny, step_c in nbrs:
            if occ[ny][nx]:
                continue
            # 对近障碍增加代价，鼓励绕障。
            d_obs = max(0, dist_to_obs[ny][nx])
            obs_penalty = obs_w / (1.0 + d_obs)
            ng = g_cost[cur] + step_c + obs_penalty
            nxt = (nx, ny)
            if ng < g_cost.get(nxt, 1e18):
                g_cost[nxt] = ng
                parent[nxt] = cur
                hf = heuristic(nxt, goal, diagonal=use_diag_h)
                heapq.heappush(pq, (ng + alpha * hf, nxt))

    if goal not in parent:
        raise RuntimeError(f"{mode} failed to find path")

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()
    return path, expansions


def write_path_trace(path_rows, out_csv):
    fieldnames = ["run_id", "algo", "point_id", "seq", "x", "y"]
    with Path(out_csv).open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(path_rows)


def write_expansion_trace(exp_rows, out_csv):
    fieldnames = ["run_id", "algo", "point_id", "step_idx", "mx", "my"]
    with Path(out_csv).open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(exp_rows)


def main():
    parser = argparse.ArgumentParser(description="Generate offline non-collision traces for plotting.")
    parser.add_argument("--map-yaml", required=True, help="map yaml path")
    parser.add_argument("--points-csv", required=True, help="point csv path")
    parser.add_argument("--point-id", required=True, help="point id, e.g. cross_zone_01")
    parser.add_argument("--path-out", default="results/path_trace.csv", help="output path trace csv")
    parser.add_argument("--expansion-out", default="results/expansion_trace.csv", help="output expansion trace csv")
    args = parser.parse_args()

    cfg, grid, w, h = load_ros_map(args.map_yaml)
    occ = build_obstacle_mask(grid, cfg.get("occupied_thresh", 0.65), cfg.get("negate", 0))
    resolution = cfg.get("resolution", 0.05)
    origin = cfg.get("origin", [0.0, 0.0, 0.0])

    row = load_point(args.points_csv, args.point_id)
    sx, sy = float(row["start_x"]), float(row["start_y"])
    gx, gy = float(row["goal_x"]), float(row["goal_y"])
    start = world_to_map(sx, sy, resolution, origin)
    goal = world_to_map(gx, gy, resolution, origin)
    if not (0 <= start[0] < w and 0 <= start[1] < h and 0 <= goal[0] < w and 0 <= goal[1] < h):
        raise RuntimeError("start/goal outside map bounds")

    dist = obstacle_distance_cells(occ)
    b_path, b_exp = astar(occ, start, goal, "baseline", dist)
    i_path, i_exp = astar(occ, start, goal, "improved", dist)

    path_rows = []
    for algo, run_id, path in (("baseline", 1, b_path), ("improved", 2, i_path)):
        for seq, (mx, my) in enumerate(path):
            wx, wy = map_to_world(mx, my, resolution, origin)
            path_rows.append(
                {"run_id": run_id, "algo": algo, "point_id": args.point_id, "seq": seq, "x": wx, "y": wy}
            )
    exp_rows = []
    for algo, run_id, exp in (("baseline", 1, b_exp), ("improved", 2, i_exp)):
        for step, (mx, my) in enumerate(exp, start=1):
            exp_rows.append(
                {"run_id": run_id, "algo": algo, "point_id": args.point_id, "step_idx": step, "mx": mx, "my": my}
            )

    Path(args.path_out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.expansion_out).parent.mkdir(parents=True, exist_ok=True)
    write_path_trace(path_rows, args.path_out)
    write_expansion_trace(exp_rows, args.expansion_out)
    print(f"Saved offline path trace: {Path(args.path_out).resolve()} rows={len(path_rows)}")
    print(f"Saved offline expansion trace: {Path(args.expansion_out).resolve()} rows={len(exp_rows)}")


if __name__ == "__main__":
    main()
