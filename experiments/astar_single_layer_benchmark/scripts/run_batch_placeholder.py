#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Print batch command skeleton for baseline/improved runs.")
    parser.add_argument("--points", default="configs/experiment_points_template.csv", help="Points CSV path")
    parser.add_argument("--repeat-default", type=int, default=3, help="Fallback repeat count")
    args = parser.parse_args()

    points = Path(args.points)
    if not points.exists():
        raise SystemExit(f"Points file not found: {points}")

    with points.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    print("# Replace the command body with your ROS launch/service call")
    print("# Output should finally be appended into results/results.csv")
    print("")
    for row in rows:
        point_id = row["point_id"]
        scene = row["scene_type"]
        repeat = int(row.get("repeat") or args.repeat_default)
        for algo in ("baseline", "improved"):
            for i in range(1, repeat + 1):
                flag = "false" if algo == "baseline" else "true"
                print(
                    "echo "
                    f"\"TODO run point={point_id} scene={scene} algo={algo} repeat={i} "
                    f"use_improved_astar={flag}\""
                )


if __name__ == "__main__":
    main()
