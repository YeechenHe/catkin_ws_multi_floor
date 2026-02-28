#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from map_utils import build_obstacle_mask, load_ros_map


def make_inflation_mask(obstacle_mask, inflation_cells):
    h = len(obstacle_mask)
    w = len(obstacle_mask[0])
    inflated = [[False for _ in range(w)] for _ in range(h)]
    if inflation_cells <= 0:
        return inflated

    obs = [(x, y) for y in range(h) for x in range(w) if obstacle_mask[y][x]]
    for ox, oy in obs:
        y0 = max(0, oy - inflation_cells)
        y1 = min(h - 1, oy + inflation_cells)
        x0 = max(0, ox - inflation_cells)
        x1 = min(w - 1, ox + inflation_cells)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                inflated[y][x] = True
    for y in range(h):
        for x in range(w):
            if obstacle_mask[y][x]:
                inflated[y][x] = False
    return inflated


def compute_density_and_labels(obstacle_mask, inflation_mask, density_radius, inflation_lambda, t_open, t_narrow):
    h = len(obstacle_mask)
    w = len(obstacle_mask[0])
    rho = np.zeros((h, w), dtype=np.float32)
    labels = np.zeros((h, w), dtype=np.uint8)  # 0=open, 1=normal, 2=narrow, 255=obstacle

    for y in range(h):
        for x in range(w):
            if obstacle_mask[y][x]:
                labels[y, x] = 255
                continue
            n_total = 0
            n_occ = 0
            n_inf = 0
            for yy in range(max(0, y - density_radius), min(h, y + density_radius + 1)):
                for xx in range(max(0, x - density_radius), min(w, x + density_radius + 1)):
                    n_total += 1
                    if obstacle_mask[yy][xx]:
                        n_occ += 1
                    elif inflation_mask[yy][xx]:
                        n_inf += 1
            d = (n_occ + inflation_lambda * n_inf) / max(1, n_total)
            rho[y, x] = d
            if d < t_open:
                labels[y, x] = 0
            elif d >= t_narrow:
                labels[y, x] = 2
            else:
                labels[y, x] = 1

    return rho, labels


def save_density_figure(rho, obstacle_mask, out_png):
    view = rho.copy()
    view[np.array(obstacle_mask, dtype=bool)] = np.nan
    plt.figure(figsize=(10, 4))
    plt.imshow(view, origin="lower", cmap="viridis")
    plt.colorbar(label="local obstacle density rho")
    plt.title("Density Heatmap")
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def save_label_figure(labels, out_png):
    # 0=open(green),1=normal(yellow),2=narrow(red),255=obstacle(black)
    rgb = np.zeros((labels.shape[0], labels.shape[1], 3), dtype=np.float32)
    rgb[labels == 0] = [0.25, 0.75, 0.30]
    rgb[labels == 1] = [0.95, 0.85, 0.25]
    rgb[labels == 2] = [0.92, 0.30, 0.25]
    rgb[labels == 255] = [0.0, 0.0, 0.0]

    plt.figure(figsize=(10, 4))
    plt.imshow(rgb, origin="lower")
    plt.title("Region Labels (open/normal/narrow)")
    plt.tight_layout()
    plt.savefig(out_png, dpi=160)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Plot region recognition from map obstacle density.")
    parser.add_argument("--map-yaml", required=True, help="ROS map yaml path")
    parser.add_argument("--outdir", default="figures", help="Output figure directory")
    parser.add_argument("--density-radius", type=int, default=2, help="Density window radius (cells)")
    parser.add_argument("--inflation-lambda", type=float, default=0.5, help="Weight for inflated cells")
    parser.add_argument("--inflation-cells", type=int, default=2, help="Inflation radius in cells")
    parser.add_argument("--threshold-open", type=float, default=0.20, help="Open threshold")
    parser.add_argument("--threshold-narrow", type=float, default=0.45, help="Narrow threshold")
    args = parser.parse_args()

    cfg, grid, _, _ = load_ros_map(args.map_yaml)
    occ = build_obstacle_mask(
        grid,
        occupied_thresh=cfg.get("occupied_thresh", 0.65),
        negate=cfg.get("negate", 0),
    )
    inf = make_inflation_mask(occ, args.inflation_cells)
    rho, labels = compute_density_and_labels(
        occ,
        inf,
        args.density_radius,
        args.inflation_lambda,
        args.threshold_open,
        args.threshold_narrow,
    )

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    save_density_figure(rho, occ, outdir / "fig_density_heatmap.png")
    save_label_figure(labels, outdir / "fig_region_labels.png")
    print(f"Saved: {outdir / 'fig_density_heatmap.png'}")
    print(f"Saved: {outdir / 'fig_region_labels.png'}")


if __name__ == "__main__":
    main()
