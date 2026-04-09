#!/usr/bin/env python3
import argparse
import math
import re
from collections import defaultdict
from pathlib import Path

RAW_PATTERN = re.compile(
    r"\[ImprovedAStar\]\[RawMetrics\]\s+expanded=(?P<expanded>\d+)\s+"
    r"waypoints=(?P<waypoints>\d+)\s+"
    r"path_length_m=(?P<path_length>[0-9.]+)\s+"
    r"min_clearance_m=(?P<min_clearance>[0-9.]+)\s+"
    r"mean_clearance_m=(?P<mean_clearance>[0-9.]+)\s+"
    r"sum_abs_turn_rad=(?P<sum_abs_turn>[0-9.]+)\s+"
    r"turn_point_count=(?P<turn_points>\d+)\s+"
    r"danger_point_count=(?P<danger_points>\d+)"
)

SMOOTH_PATTERN = re.compile(
    r"\[ImprovedAStar\]\[SmoothMetrics\]\s+expanded=(?P<expanded>\d+)\s+"
    r"waypoints=(?P<waypoints>\d+)\s+"
    r"path_length_m=(?P<path_length>[0-9.]+)\s+"
    r"min_clearance_m=(?P<min_clearance>[0-9.]+)\s+"
    r"mean_clearance_m=(?P<mean_clearance>[0-9.]+)\s+"
    r"sum_abs_turn_rad=(?P<sum_abs_turn>[0-9.]+)\s+"
    r"turn_point_count=(?P<turn_points>\d+)\s+"
    r"danger_point_count=(?P<danger_points>\d+)"
)

PERT_LABELS = {
    "0_0_0": "none",
    "0.1_0.1_0.05": "small",
    "0.25_0.25_0.15": "medium",
    "0.4_0.4_0.2": "large",
}


class RunningStats:
    def __init__(self):
        self.values = []

    def add(self, value: float) -> None:
        self.values.append(float(value))

    def mean(self) -> float:
        if not self.values:
            return 0.0
        return sum(self.values) / len(self.values)

    def minimum(self) -> float:
        return min(self.values) if self.values else 0.0

    def maximum(self) -> float:
        return max(self.values) if self.values else 0.0


class MetricBucket:
    def __init__(self):
        self.metrics = defaultdict(RunningStats)
        self.pair_count = 0

    def add(self, data: dict) -> None:
        self.pair_count += 1
        for key, value in data.items():
            self.metrics[key].add(value)


def parse_log_pairs(log_path: Path):
    raw_entries = []
    smooth_entries = []
    with log_path.open("r", encoding="utf-8", errors="ignore") as file:
        for line in file:
            raw_match = RAW_PATTERN.search(line)
            if raw_match:
                raw_entries.append({k: float(v) for k, v in raw_match.groupdict().items()})
                continue
            smooth_match = SMOOTH_PATTERN.search(line)
            if smooth_match:
                smooth_entries.append({k: float(v) for k, v in smooth_match.groupdict().items()})

    pair_count = min(len(raw_entries), len(smooth_entries))
    pairs = []
    for index in range(pair_count):
        pairs.append((raw_entries[index], smooth_entries[index]))
    return pairs


def perturbation_key_from_name(log_name: str) -> str:
    match = re.search(r"pert_([^_]+_[^_]+_[^_]+)_run", log_name)
    if not match:
        return "unknown"
    return match.group(1)


def render_bucket(title: str, bucket: MetricBucket) -> str:
    lines = []
    lines.append(f"### {title}")
    lines.append("")
    lines.append(f"- 样本数：{bucket.pair_count}")
    lines.append("")
    lines.append("| 指标 | 平滑前 | 平滑后 | 变化 |")
    lines.append("| --- | ---: | ---: | ---: |")

    metric_order = [
        ("waypoints", "waypoints", False),
        ("expanded", "expanded_nodes", False),
        ("path_length", "path_length_m", True),
        ("min_clearance", "min_clearance_m", True),
        ("mean_clearance", "mean_clearance_m", True),
        ("sum_abs_turn", "sum_abs_turn_rad", True),
        ("turn_points", "turn_point_count", False),
        ("danger_points", "danger_point_count", False),
    ]

    for log_key, display_name, use_float in metric_order:
        raw_key = f"raw_{log_key}"
        smooth_key = f"smooth_{log_key}"
        raw_mean = bucket.metrics[raw_key].mean()
        smooth_mean = bucket.metrics[smooth_key].mean()
        delta = smooth_mean - raw_mean
        if use_float:
            lines.append(
                f"| `{display_name}` | {raw_mean:.3f} | {smooth_mean:.3f} | {delta:+.3f} |"
            )
        else:
            lines.append(
                f"| `{display_name}` | {raw_mean:.2f} | {smooth_mean:.2f} | {delta:+.2f} |"
            )

    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Summarize ImprovedAStar raw vs smoothed metrics from logs.")
    parser.add_argument("log_dir", help="Directory containing experiment log files")
    parser.add_argument("--method", default="spamcl", help="Method prefix to match, default spamcl")
    parser.add_argument("--output", default="", help="Optional markdown output path")
    args = parser.parse_args()

    log_dir = Path(args.log_dir)
    log_files = sorted(log_dir.glob(f"{args.method}_pert_*_run*.log"))
    if not log_files:
        raise SystemExit(f"No log files found in {log_dir} for method {args.method}")

    buckets = defaultdict(MetricBucket)

    for log_file in log_files:
        perturbation_key = perturbation_key_from_name(log_file.name)
        perturbation_label = PERT_LABELS.get(perturbation_key, perturbation_key)
        pairs = parse_log_pairs(log_file)
        for raw_data, smooth_data in pairs:
            combined = {
                "raw_waypoints": raw_data["waypoints"],
                "smooth_waypoints": smooth_data["waypoints"],
                "raw_expanded": raw_data["expanded"],
                "smooth_expanded": smooth_data["expanded"],
                "raw_path_length": raw_data["path_length"],
                "smooth_path_length": smooth_data["path_length"],
                "raw_min_clearance": raw_data["min_clearance"],
                "smooth_min_clearance": smooth_data["min_clearance"],
                "raw_mean_clearance": raw_data["mean_clearance"],
                "smooth_mean_clearance": smooth_data["mean_clearance"],
                "raw_sum_abs_turn": raw_data["sum_abs_turn"],
                "smooth_sum_abs_turn": smooth_data["sum_abs_turn"],
                "raw_turn_points": raw_data["turn_points"],
                "smooth_turn_points": smooth_data["turn_points"],
                "raw_danger_points": raw_data["danger_points"],
                "smooth_danger_points": smooth_data["danger_points"],
            }
            buckets[perturbation_label].add(combined)

    sections = [f"# {args.method} 改进A* 平滑前后统计汇总", ""]
    for label in ["none", "small", "medium", "large"]:
        if label in buckets:
            sections.append(render_bucket(label, buckets[label]))

    output_text = "\n".join(sections) + "\n"
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(output_text, encoding="utf-8")
        print(output_path)
    else:
        print(output_text)


if __name__ == "__main__":
    main()
