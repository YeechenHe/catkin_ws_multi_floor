#!/usr/bin/env python3
"""
从 rosbag 导出 nav_msgs/Path 到本实验的 path_trace.csv 格式：
run_id,algo,point_id,seq,x,y

示例（先导 baseline，再导 improved）：
python3 scripts/export_path_trace_from_bag.py \
  --bag /path/to/case1_baseline.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --algo baseline \
  --point-id cross_zone_01 \
  --run-id 1 \
  --out results/path_trace.csv \
  --overwrite

python3 scripts/export_path_trace_from_bag.py \
  --bag /path/to/case1_improved.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --algo improved \
  --point-id cross_zone_01 \
  --run-id 2 \
  --out results/path_trace.csv
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def read_path_messages(bag_path: Path, topic: str):
    try:
        import rosbag  # type: ignore
    except Exception as e:
        raise RuntimeError("无法导入 rosbag，请在 ROS 环境中运行。") from e

    msgs = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            msgs.append((msg, t.to_sec()))
    return msgs


def select_message(messages, mode: str):
    if not messages:
        raise RuntimeError("未读取到 Path 消息")
    if mode == "first":
        return messages[0]
    if mode == "last":
        return messages[-1]
    if mode == "maxlen":
        return max(messages, key=lambda x: len(x[0].poses))
    raise ValueError(f"unknown select mode: {mode}")


def build_rows(path_msg, run_id: int, algo: str, point_id: str):
    rows = []
    for seq, pose_stamped in enumerate(path_msg.poses):
        p = pose_stamped.pose.position
        rows.append(
            {
                "run_id": run_id,
                "algo": algo,
                "point_id": point_id,
                "seq": seq,
                "x": float(p.x),
                "y": float(p.y),
            }
        )
    return rows


def write_rows(path: Path, rows, overwrite: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["run_id", "algo", "point_id", "seq", "x", "y"]

    if overwrite or not path.exists():
        mode = "w"
        write_header = True
    else:
        mode = "a"
        write_header = False

    with path.open(mode, encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description="导出本实验 path_trace.csv（从 rosbag 的 nav_msgs/Path）")
    parser.add_argument("--bag", required=True, help="bag 文件路径")
    parser.add_argument("--topic", required=True, help="Path 话题名，例如 /move_base/ImprovedAStarPlanner/plan")
    parser.add_argument("--algo", required=True, choices=["baseline", "improved"], help="算法标签")
    parser.add_argument("--point-id", required=True, help="点位 ID，例如 cross_zone_01")
    parser.add_argument("--run-id", required=True, type=int, help="运行 ID（整型）")
    parser.add_argument("--out", default="results/path_trace.csv", help="输出 CSV 路径")
    parser.add_argument("--select", choices=["first", "last", "maxlen"], default="maxlen", help="多条 Path 的选择策略")
    parser.add_argument("--overwrite", action="store_true", help="覆盖输出文件（否则追加）")
    args = parser.parse_args()

    bag_path = Path(args.bag)
    if not bag_path.exists():
        raise FileNotFoundError(f"找不到 bag 文件: {bag_path}")

    messages = read_path_messages(bag_path, args.topic)
    selected_msg, selected_ts = select_message(messages, args.select)
    rows = build_rows(selected_msg, args.run_id, args.algo, args.point_id)
    write_rows(Path(args.out), rows, args.overwrite)

    print(
        f"[OK] export done: out={Path(args.out).resolve()} rows={len(rows)} "
        f"algo={args.algo} point_id={args.point_id} run_id={args.run_id} msg_time={selected_ts:.3f}"
    )


if __name__ == "__main__":
    main()
