#!/usr/bin/env python3
import argparse
import csv
from collections import defaultdict
from pathlib import Path


def mean(values):
    return sum(values) / len(values) if values else 0.0


def load_rows(path):
    with path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def parse_float(row, key, default=0.0):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    return float(v)


def parse_int(row, key, default=0):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    return int(float(v))


def plot_bar(plt, labels, values, ylabel, title, out_path, ylim=None):
    plt.figure(figsize=(max(6, len(labels) * 1.1), 4))
    plt.bar(labels, values)
    plt.ylabel(ylabel)
    plt.title(title)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.xticks(rotation=25, ha="right")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser(description="Plot basic benchmark figures from results CSV.")
    parser.add_argument("--input", required=True, help="Input results CSV")
    parser.add_argument("--outdir", required=True, help="Output directory")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
    except Exception as e:  # pragma: no cover
        raise SystemExit(f"matplotlib is required: {e}")

    in_path = Path(args.input)
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    rows = load_rows(in_path)
    if not rows:
        raise SystemExit("No rows found in results CSV.")

    metrics_by_algo = defaultdict(lambda: defaultdict(list))
    success_by_algo = defaultdict(list)
    by_scene_algo_plan_time = defaultdict(list)

    for r in rows:
        algo = r.get("algo", "unknown")
        scene = r.get("scene_type", "unknown")
        succ = parse_int(r, "success", 0)
        pt = parse_float(r, "plan_time_ms", 0.0)
        exp_nodes = parse_float(r, "expanded_nodes", -1.0)
        min_clr = parse_float(r, "min_clearance_m", -1.0)
        mean_clr = parse_float(r, "mean_clearance_m", -1.0)
        path_len = parse_float(r, "path_length_m", -1.0)

        success_by_algo[algo].append(succ)
        metrics_by_algo[algo]["plan_time_ms"].append(pt)
        if exp_nodes >= 0:
            metrics_by_algo[algo]["expanded_nodes"].append(exp_nodes)
        if min_clr >= 0:
            metrics_by_algo[algo]["min_clearance_m"].append(min_clr)
        if mean_clr >= 0:
            metrics_by_algo[algo]["mean_clearance_m"].append(mean_clr)
        if path_len >= 0:
            metrics_by_algo[algo]["path_length_m"].append(path_len)
        by_scene_algo_plan_time[(scene, algo)].append(pt)

    algos = sorted(metrics_by_algo.keys())
    mean_pt = [mean(metrics_by_algo[a]["plan_time_ms"]) for a in algos]
    succ_rate = [100.0 * mean(success_by_algo[a]) for a in algos]
    plot_bar(
        plt,
        algos,
        mean_pt,
        "plan_time_ms",
        "Mean Planning Time by Algorithm",
        outdir / "fig_plan_time_bar.png",
    )
    plot_bar(
        plt,
        algos,
        succ_rate,
        "success_rate(%)",
        "Success Rate by Algorithm",
        outdir / "fig_success_rate_bar.png",
        ylim=(0, 100),
    )

    # Optional metrics, only draw if any value exists.
    opt_metrics = [
        ("expanded_nodes", "Mean Expanded Nodes", "fig_expanded_nodes_bar.png"),
        ("min_clearance_m", "Mean Min Clearance (m)", "fig_min_clearance_bar.png"),
        ("mean_clearance_m", "Mean Mean Clearance (m)", "fig_mean_clearance_bar.png"),
        ("path_length_m", "Mean Path Length (m)", "fig_path_length_bar.png"),
    ]
    for key, title, name in opt_metrics:
        vals = [mean(metrics_by_algo[a][key]) if metrics_by_algo[a][key] else 0.0 for a in algos]
        if any(metrics_by_algo[a][key] for a in algos):
            plot_bar(plt, algos, vals, key, title, outdir / name)

    scenes = sorted({k[0] for k in by_scene_algo_plan_time.keys()})
    x_labels = []
    y_vals = []
    for s in scenes:
        for a in algos:
            x_labels.append(f"{s}-{a}")
            y_vals.append(mean(by_scene_algo_plan_time.get((s, a), [])))
    if x_labels:
        plot_bar(
            plt,
            x_labels,
            y_vals,
            "plan_time_ms",
            "Mean Planning Time by Scene and Algorithm",
            outdir / "fig_plan_time_scene_algo.png",
        )

    print(f"Saved figures to: {outdir}")


if __name__ == "__main__":
    main()
