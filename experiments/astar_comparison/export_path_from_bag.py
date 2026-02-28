#!/usr/bin/env python3
"""
从 rosbag 中读取 nav_msgs/Path 话题，并导出为 x,y CSV（供路径叠加图使用）。

示例：
python3 export_path_from_bag.py \
  --bag case1_baseline.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --out baseline_case1.csv \
  --select last
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path


def _read_messages_from_bag(bag_path: Path, topic: str):
    try:
        import rosbag  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "无法导入 rosbag。请在 ROS 环境中运行，并确保 python3 可用 rosbag 模块。"
        ) from e

    msgs = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            msgs.append((msg, t.to_sec()))
    return msgs


def _path_msg_to_rows(path_msg):
    rows = []
    for i, pose_stamped in enumerate(path_msg.poses):
        p = pose_stamped.pose.position
        rows.append({"seq": i, "x": float(p.x), "y": float(p.y)})
    return rows


def main():
    parser = argparse.ArgumentParser(description="从 rosbag 导出 nav_msgs/Path 到 x,y CSV")
    parser.add_argument("--bag", required=True, help="rosbag 文件路径")
    parser.add_argument("--topic", required=True, help="nav_msgs/Path 话题名")
    parser.add_argument("--out", required=True, help="输出 CSV 路径")
    parser.add_argument(
        "--select",
        choices=["first", "last", "all", "maxlen"],
        default="maxlen",
        help="当 bag 中有多个 Path 消息时选择导出策略",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not bag_path.exists():
        raise FileNotFoundError(f"找不到 bag 文件: {bag_path}")

    msgs = _read_messages_from_bag(bag_path, args.topic)
    if not msgs:
        raise RuntimeError(f"在 bag 中未找到话题 {args.topic} 的消息")

    if args.select == "first":
        selected = [msgs[0]]
    elif args.select == "last":
        selected = [msgs[-1]]
    elif args.select == "maxlen":
        selected = [max(msgs, key=lambda item: len(item[0].poses))]
    else:
        selected = msgs

    if args.select in ("first", "last", "maxlen"):
        rows = _path_msg_to_rows(selected[0][0])
        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["seq", "x", "y"])
            writer.writeheader()
            writer.writerows(rows)
    else:
        # all 模式：把每条 Path 展开，并记录 path_idx / msg_time
        rows = []
        for path_idx, (msg, ts) in enumerate(selected):
            for seq, pose_stamped in enumerate(msg.poses):
                p = pose_stamped.pose.position
                rows.append(
                    {
                        "path_idx": path_idx,
                        "msg_time": ts,
                        "seq": seq,
                        "x": float(p.x),
                        "y": float(p.y),
                    }
                )
        with out_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["path_idx", "msg_time", "seq", "x", "y"])
            writer.writeheader()
            writer.writerows(rows)

    print(f"[OK] 导出完成: {out_path.resolve()} (rows={len(rows)})")


if __name__ == "__main__":
    main()

