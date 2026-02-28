#!/usr/bin/env python3
import argparse
from pathlib import Path


FREE = 254
OCC = 0


def draw_rect(grid, x0, y0, w, h, value):
    h_grid = len(grid)
    w_grid = len(grid[0])
    for y in range(max(0, y0), min(h_grid, y0 + h)):
        row = grid[y]
        for x in range(max(0, x0), min(w_grid, x0 + w)):
            row[x] = value


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


def carve_vertical_gate(grid, wall_x, gate_y_center, gate_height):
    half = gate_height // 2
    draw_rect(grid, wall_x, gate_y_center - half, 1, gate_height, FREE)


def build_map(width, height):
    grid = [[FREE for _ in range(width)] for _ in range(height)]

    # Outer boundary
    draw_rect(grid, 0, 0, width, 2, OCC)
    draw_rect(grid, 0, height - 2, width, 2, OCC)
    draw_rect(grid, 0, 0, 2, height, OCC)
    draw_rect(grid, width - 2, 0, 2, height, OCC)

    # Zone separators: force path to pass through specified gates.
    wall1_x = 170  # open -> normal gate
    wall2_x = 295  # normal -> narrow gate
    wall3_x = 368  # narrow -> open gate
    draw_rect(grid, wall1_x, 2, 1, height - 4, OCC)
    draw_rect(grid, wall2_x, 2, 1, height - 4, OCC)
    draw_rect(grid, wall3_x, 2, 1, height - 4, OCC)

    # Gate widths: normal gate wider, narrow gate tight.
    carve_vertical_gate(grid, wall1_x, gate_y_center=100, gate_height=26)
    carve_vertical_gate(grid, wall2_x, gate_y_center=95, gate_height=8)
    carve_vertical_gate(grid, wall3_x, gate_y_center=102, gate_height=22)

    # Add anti-straight barriers between gates to avoid trivial straight-line crossing.
    # The center block forces detour choice (up or down), while keeping map traversable.
    draw_rect(grid, 215, 85, 34, 24, OCC)
    # Upper/lower side blocks break symmetry and form an S-like passage.
    draw_rect(grid, 255, 74, 18, 10, OCC)
    draw_rect(grid, 252, 112, 16, 10, OCC)
    # Small offset obstacles near gate entrances to discourage direct gate-to-gate line.
    draw_rect(grid, 176, 90, 14, 8, OCC)   # after wall1 gate, upper bias
    draw_rect(grid, 282, 98, 10, 8, OCC)   # before wall2 gate, lower bias

    # Transition bands: gradually increase clutter from open->normal and normal->narrow.
    # These bands reduce hard zone jumps and make region recognition more continuous.
    draw_rect(grid, 140, 35, 10, 12, OCC)
    draw_rect(grid, 148, 128, 8, 10, OCC)
    draw_rect(grid, 160, 58, 7, 9, OCC)
    draw_rect(grid, 274, 44, 8, 10, OCC)
    draw_rect(grid, 286, 136, 7, 9, OCC)
    draw_rect(grid, 292, 74, 6, 8, OCC)

    # Left open zone: sparse obstacles (low density).
    draw_rect(grid, 24, 28, 20, 14, OCC)
    draw_rect(grid, 70, 135, 16, 12, OCC)
    draw_rect(grid, 110, 40, 14, 14, OCC)

    # Middle normal zone: moderate clutter.
    draw_rect(grid, 188, 30, 10, 30, OCC)
    draw_rect(grid, 210, 118, 12, 30, OCC)
    draw_rect(grid, 236, 46, 12, 20, OCC)
    draw_rect(grid, 258, 104, 14, 22, OCC)
    draw_rect(grid, 272, 72, 10, 14, OCC)

    # Narrow zone core: tight tunnel around the narrow gate.
    # Horizontal walls form a channel to amplify local density and risk.
    draw_rect(grid, 300, 76, 64, 10, OCC)
    draw_rect(grid, 300, 109, 64, 10, OCC)
    # Keep a narrow traversable band in the middle (height ~14 cells).
    draw_rect(grid, 300, 86, 64, 23, FREE)
    # Add near-obstacle protrusions to create risk if path hugs edges.
    draw_rect(grid, 323, 86, 6, 9, OCC)
    draw_rect(grid, 338, 99, 6, 9, OCC)

    # Right open zone: sparse obstacles.
    draw_rect(grid, 404, 30, 20, 14, OCC)
    draw_rect(grid, 444, 128, 14, 12, OCC)
    draw_rect(grid, 426, 66, 10, 10, OCC)

    return grid


def main():
    parser = argparse.ArgumentParser(description="Build a contrast map for baseline vs improved A*.")
    parser.add_argument("--out-map", required=True, help="Output map prefix, e.g. maps/contrast_case1")
    parser.add_argument("--resolution", type=float, default=0.05, help="Map resolution in meters/cell")
    parser.add_argument("--width", type=int, default=480, help="Map width in cells")
    parser.add_argument("--height", type=int, default=200, help="Map height in cells")
    args = parser.parse_args()

    grid = build_map(args.width, args.height)
    out_prefix = Path(args.out_map)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = out_prefix.with_suffix(".pgm")
    yaml_path = out_prefix.with_suffix(".yaml")

    write_pgm_p2(pgm_path, grid)
    write_yaml(yaml_path, pgm_path.name, args.resolution)

    print(f"Generated map: {pgm_path}")
    print(f"Generated yaml: {yaml_path}")
    print("Recommended cross-zone point: start(mx,my)=(40,100), goal(mx,my)=(445,103)")


if __name__ == "__main__":
    main()
