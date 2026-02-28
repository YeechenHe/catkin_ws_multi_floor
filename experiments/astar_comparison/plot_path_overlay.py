#!/usr/bin/env python3
"""
将 Baseline A* 与 Improved A* 的路径叠加到同一张地图上，便于直观对比。

输入：
1) 地图 yaml（例如 map_level0.yaml）
2) Baseline 路径 CSV
3) Improved 路径 CSV

路径 CSV 格式（表头必须包含 x,y）：
seq,x,y
0,4.0,-5.0
1,4.1,-4.9
...
"""
from __future__ import annotations

import argparse
import ast
import csv
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


def parse_map_yaml(yaml_path: Path) -> Dict[str, object]:
    """解析最小 map_server yaml（image/resolution/origin）。"""
    data: Dict[str, object] = {}
    for raw in yaml_path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        k = k.strip()
        v = v.strip()
        if k == "image":
            data["image"] = v
        elif k == "resolution":
            data["resolution"] = float(v)
        elif k == "origin":
            data["origin"] = ast.literal_eval(v)
        elif k == "negate":
            data["negate"] = int(v)
        elif k == "occupied_thresh":
            data["occupied_thresh"] = float(v)
        elif k == "free_thresh":
            data["free_thresh"] = float(v)

    missing = [k for k in ("image", "resolution", "origin") if k not in data]
    if missing:
        raise ValueError(f"地图 yaml 缺少必要字段: {missing}")
    return data


def world_to_pixel(
    x: np.ndarray,
    y: np.ndarray,
    resolution: float,
    origin_xy: Tuple[float, float],
    map_h: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    世界坐标 -> 像素坐标
    map_server 的 origin 是地图左下角，图像数组原点在左上角，所以 y 需要翻转。
    """
    ox, oy = origin_xy
    px = (x - ox) / resolution
    py = map_h - (y - oy) / resolution
    return px, py


def load_path_csv(csv_path: Path):
    xs = []
    ys = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if "x" not in reader.fieldnames or "y" not in reader.fieldnames:
            raise ValueError(f"{csv_path} 缺少 x,y 列")
        for row in reader:
            xs.append(float(row["x"]))
            ys.append(float(row["y"]))
    return np.array(xs, dtype=float), np.array(ys, dtype=float)


def main():
    parser = argparse.ArgumentParser(description="绘制 Baseline vs Improved 路径叠加图")
    parser.add_argument("--map-yaml", required=True, help="地图 yaml 路径")
    parser.add_argument("--baseline", required=True, help="Baseline 路径 CSV（含 x,y）")
    parser.add_argument("--improved", required=True, help="Improved 路径 CSV（含 x,y）")
    parser.add_argument("--out", required=True, help="输出图片路径，例如 figures/overlay_case1.png")
    parser.add_argument("--title", default="Path Overlay: Baseline vs Improved", help="图标题")
    args = parser.parse_args()

    map_yaml = Path(args.map_yaml)
    base_csv = Path(args.baseline)
    imp_csv = Path(args.improved)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    cfg = parse_map_yaml(map_yaml)
    image_rel = Path(str(cfg["image"]))
    map_img_path = image_rel if image_rel.is_absolute() else (map_yaml.parent / image_rel)

    # 读取地图图像（pgm/png 均可）
    img = mpimg.imread(str(map_img_path))
    # 若为 RGB，转灰度
    if img.ndim == 3:
        img = img[..., 0]

    base_x_world, base_y_world = load_path_csv(base_csv)
    imp_x_world, imp_y_world = load_path_csv(imp_csv)

    resolution = float(cfg["resolution"])
    origin = cfg["origin"]
    ox, oy = float(origin[0]), float(origin[1])
    h = img.shape[0]

    bx, by = world_to_pixel(base_x_world, base_y_world, resolution, (ox, oy), h)
    ix, iy = world_to_pixel(imp_x_world, imp_y_world, resolution, (ox, oy), h)

    plt.figure(figsize=(8, 8))
    plt.imshow(img, cmap="gray", origin="upper")
    plt.plot(bx, by, color="tab:blue", linewidth=2.0, label="Baseline A*")
    plt.plot(ix, iy, color="tab:red", linewidth=2.0, label="Improved A*")

    # 标记起终点（以 baseline 起终点为准）
    if len(bx) > 0:
        plt.scatter([bx[0]], [by[0]], c="lime", s=45, marker="o", label="Start")
        plt.scatter([bx[-1]], [by[-1]], c="yellow", s=45, marker="X", label="Goal")

    plt.title(args.title)
    plt.legend(loc="upper right")
    plt.axis("off")
    plt.tight_layout()
    plt.savefig(str(out_path), dpi=220, bbox_inches="tight")
    plt.close()
    print(f"[OK] 已输出路径叠加图: {out_path.resolve()}")


if __name__ == "__main__":
    main()

