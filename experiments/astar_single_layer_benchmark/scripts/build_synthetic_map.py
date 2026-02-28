#!/usr/bin/env python3
import argparse
import csv
import random
from pathlib import Path


FREE = 254
OCC = 0


def draw_rect(grid, x0, y0, w, h, value):
    for y in range(max(0, y0), min(len(grid), y0 + h)):
        row = grid[y]
        for x in range(max(0, x0), min(len(row), x0 + w)):
            row[x] = value


def draw_border(grid, thickness=2):
    h = len(grid)
    w = len(grid[0])
    draw_rect(grid, 0, 0, w, thickness, OCC)
    draw_rect(grid, 0, h - thickness, w, thickness, OCC)
    draw_rect(grid, 0, 0, thickness, h, OCC)
    draw_rect(grid, w - thickness, 0, thickness, h, OCC)


def add_random_obstacles(grid, x0, y0, w, h, density, rng):
    area = w * h
    count = int(area * density / 60.0)
    for _ in range(count):
        ow = rng.randint(2, max(2, w // 8))
        oh = rng.randint(2, max(2, h // 8))
        ox = rng.randint(x0 + 1, max(x0 + 1, x0 + w - ow - 1))
        oy = rng.randint(y0 + 1, max(y0 + 1, y0 + h - oh - 1))
        draw_rect(grid, ox, oy, ow, oh, OCC)


def write_pgm_p2(path, grid):
    h = len(grid)
    w = len(grid[0])
    with path.open("w", encoding="ascii") as f:
        f.write("P2\n")
        f.write(f"{w} {h}\n")
        f.write("255\n")
        for row in grid:
            f.write(" ".join(str(v) for v in row))
            f.write("\n")


def write_yaml(path, image_name, resolution):
    content = (
        f"image: {image_name}\n"
        f"resolution: {resolution}\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(content, encoding="ascii")


def load_zones(csv_path):
    zones = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            zones.append(row)
    return zones


def main():
    parser = argparse.ArgumentParser(description="Generate single-layer synthetic grid map.")
    parser.add_argument("--scenario", required=True, help="Path to scenario CSV template.")
    parser.add_argument("--out-map", required=True, help="Output map prefix, e.g. maps/single_layer_eval")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters/cell")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for obstacle placement")
    args = parser.parse_args()

    scenario_path = Path(args.scenario)
    zones = load_zones(scenario_path)
    if not zones:
        raise SystemExit("No zones found in scenario CSV.")

    # Horizontal stitching of zones: open -> normal -> narrow by template order.
    margin = 10
    total_w = margin
    max_h = 0
    parsed = []
    for z in zones:
        w = int(z["width_cells"])
        h = int(z["height_cells"])
        scene = z.get("scene_type", "normal").strip()
        density = float(z.get("obstacle_density", 0.1))
        corridor = int(z.get("corridor_width_cells", 6))
        parsed.append((scene, w, h, density, corridor))
        total_w += w + margin
        max_h = max(max_h, h)
    total_h = max_h + 2 * margin

    grid = [[FREE for _ in range(total_w)] for _ in range(total_h)]
    draw_border(grid, thickness=2)
    rng = random.Random(args.seed)

    cursor_x = margin
    for i, (scene, w, h, density, corridor) in enumerate(parsed):
        top = margin + (max_h - h) // 2
        draw_rect(grid, cursor_x, top, w, h, FREE)

        # Zone walls
        draw_rect(grid, cursor_x, top, w, 1, OCC)
        draw_rect(grid, cursor_x, top + h - 1, w, 1, OCC)
        draw_rect(grid, cursor_x, top, 1, h, OCC)
        draw_rect(grid, cursor_x + w - 1, top, 1, h, OCC)

        # A central corridor opening to keep connectivity.
        mid = top + h // 2
        gap = max(2, corridor // 2)
        draw_rect(grid, cursor_x, mid - gap, 1, 2 * gap, FREE)
        draw_rect(grid, cursor_x + w - 1, mid - gap, 1, 2 * gap, FREE)

        # Add random obstacles but keep corridor clearer in narrow scene.
        local_density = density
        if scene == "narrow":
            local_density *= 1.2
        add_random_obstacles(grid, cursor_x + 1, top + 1, w - 2, h - 2, local_density, rng)

        # Carve a guaranteed horizontal band for cross-zone path.
        band_h = max(2, corridor)
        draw_rect(grid, cursor_x + 1, mid - band_h // 2, w - 2, band_h, FREE)
        cursor_x += w + margin

        # Connector wall in the margin, leave gap to enforce bottleneck-like transitions.
        if i < len(parsed) - 1:
            draw_rect(grid, cursor_x - margin, top, margin, h, OCC)
            draw_rect(grid, cursor_x - margin, mid - gap, margin, 2 * gap, FREE)

    out_prefix = Path(args.out_map)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = out_prefix.with_suffix(".pgm")
    yaml_path = out_prefix.with_suffix(".yaml")

    write_pgm_p2(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name, args.resolution)
    print(f"Generated map: {pgm_path}")
    print(f"Generated yaml: {yaml_path}")


if __name__ == "__main__":
    main()
