#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from map_utils import load_ros_map


def load_trace(path):
    with Path(path).open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def filter_rows(rows, algo, point_id, run_id):
    out = []
    for r in rows:
        if r.get("algo") != algo:
            continue
        if point_id and r.get("point_id") != point_id:
            continue
        if run_id and str(r.get("run_id")) != str(run_id):
            continue
        out.append(r)
    return out


def to_cells(rows):
    cells = []
    for r in rows:
        mx = int(float(r["mx"]))
        my = int(float(r["my"]))
        step = int(float(r.get("step_idx", 0)))
        cells.append((mx, my, step))
    return cells


def main():
    parser = argparse.ArgumentParser(description="Compare expansion grids between baseline and improved.")
    parser.add_argument("--map-yaml", required=True, help="ROS map yaml path")
    parser.add_argument("--trace", required=True, help="Expansion trace CSV")
    parser.add_argument("--point-id", default="", help="Optional point filter")
    parser.add_argument("--run-id", default="", help="Optional run filter")
    parser.add_argument("--out", default="figures/fig_expansion_overlay.png", help="Output figure path")
    args = parser.parse_args()

    _, grid, w, h = load_ros_map(args.map_yaml)
    rows = load_trace(args.trace)
    b = to_cells(filter_rows(rows, "baseline", args.point_id, args.run_id))
    i = to_cells(filter_rows(rows, "improved", args.point_id, args.run_id))
    if not b or not i:
        raise SystemExit("Both baseline and improved traces are required after filtering.")

    b_set = {(x, y) for x, y, _ in b}
    i_set = {(x, y) for x, y, _ in i}
    overlap = b_set & i_set
    b_only = b_set - i_set
    i_only = i_set - b_set

    arr = np.array(grid, dtype=np.uint8)
    bg = np.flipud(arr)

    fig = plt.figure(figsize=(10, 4))
    ax = fig.add_subplot(111)
    ax.imshow(bg, cmap="gray", origin="lower", vmin=0, vmax=255)

    if b_only:
        xb, yb = zip(*sorted(b_only))
        ax.scatter(xb, yb, s=6, c="#1f77b4", alpha=0.7, label=f"baseline only ({len(b_only)})")
    if i_only:
        xi, yi = zip(*sorted(i_only))
        ax.scatter(xi, yi, s=6, c="#ff7f0e", alpha=0.7, label=f"improved only ({len(i_only)})")
    if overlap:
        xo, yo = zip(*sorted(overlap))
        ax.scatter(xo, yo, s=6, c="#9467bd", alpha=0.6, label=f"overlap ({len(overlap)})")

    ax.set_xlim(0, w - 1)
    ax.set_ylim(0, h - 1)
    ax.set_title("Expansion Grid Comparison")
    ax.legend(loc="upper right")
    ax.set_xlabel("mx")
    ax.set_ylabel("my")
    plt.tight_layout()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=170)
    plt.close()

    print(f"Saved: {out}")
    print(f"baseline_unique={len(b_set)} improved_unique={len(i_set)} overlap={len(overlap)}")


if __name__ == "__main__":
    main()
