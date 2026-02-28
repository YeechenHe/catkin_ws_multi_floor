#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path


NUMERIC_KEYS = [
    "plan_time_ms",
    "expanded_nodes",
    "peak_open_list",
    "path_length_m",
    "sum_abs_turn_rad",
    "min_clearance_m",
    "mean_clearance_m",
]

LOWER_BETTER = {"plan_time_ms", "expanded_nodes", "peak_open_list", "path_length_m", "sum_abs_turn_rad"}
HIGHER_BETTER = {"success_rate", "min_clearance_m", "mean_clearance_m"}


def to_float(v):
    if v is None:
        return None
    s = str(v).strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def mean(values):
    return sum(values) / len(values) if values else 0.0


def sample_std(values):
    n = len(values)
    if n <= 1:
        return 0.0
    m = mean(values)
    return math.sqrt(sum((x - m) ** 2 for x in values) / (n - 1))


def pct_improvement(base, improved, metric):
    if metric in LOWER_BETTER:
        if base == 0:
            return 0.0
        return (base - improved) / abs(base) * 100.0
    if metric in HIGHER_BETTER:
        if base == 0:
            return 0.0
        return (improved - base) / abs(base) * 100.0
    return 0.0


def main():
    parser = argparse.ArgumentParser(description="Summarize baseline vs improved metrics for one point.")
    parser.add_argument("--input", required=True, help="results.csv path")
    parser.add_argument("--point-id", required=True, help="point_id filter")
    parser.add_argument("--out-csv", default="results/summary_case1.csv", help="summary output CSV")
    parser.add_argument("--out-md", default="results/summary_case1.md", help="summary output markdown")
    args = parser.parse_args()

    rows = []
    with Path(args.input).open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("point_id") == args.point_id and row.get("algo") in ("baseline", "improved"):
                rows.append(row)
    if not rows:
        raise SystemExit("No rows found for given point_id.")

    by_algo = {"baseline": [], "improved": []}
    for r in rows:
        by_algo[r["algo"]].append(r)

    if not by_algo["baseline"] or not by_algo["improved"]:
        raise SystemExit("Both baseline and improved records are required.")

    table = []
    n_base = len(by_algo["baseline"])
    n_impr = len(by_algo["improved"])
    succ_base = mean([to_float(r.get("success")) or 0.0 for r in by_algo["baseline"]])
    succ_impr = mean([to_float(r.get("success")) or 0.0 for r in by_algo["improved"]])
    table.append(
        {
            "metric": "success_rate",
            "baseline_mean": succ_base * 100.0,
            "baseline_std": sample_std([to_float(r.get("success")) or 0.0 for r in by_algo["baseline"]]) * 100.0,
            "improved_mean": succ_impr * 100.0,
            "improved_std": sample_std([to_float(r.get("success")) or 0.0 for r in by_algo["improved"]]) * 100.0,
            "improvement_pct": pct_improvement(succ_base * 100.0, succ_impr * 100.0, "success_rate"),
        }
    )

    for key in NUMERIC_KEYS:
        vb = [to_float(r.get(key)) for r in by_algo["baseline"]]
        vi = [to_float(r.get(key)) for r in by_algo["improved"]]
        vb = [x for x in vb if x is not None]
        vi = [x for x in vi if x is not None]
        if not vb or not vi:
            continue
        mb = mean(vb)
        mi = mean(vi)
        table.append(
            {
                "metric": key,
                "baseline_mean": mb,
                "baseline_std": sample_std(vb),
                "improved_mean": mi,
                "improved_std": sample_std(vi),
                "improvement_pct": pct_improvement(mb, mi, key),
            }
        )

    out_csv = Path(args.out_csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "point_id",
                "n_baseline",
                "n_improved",
                "metric",
                "baseline_mean",
                "baseline_std",
                "improved_mean",
                "improved_std",
                "improvement_pct",
            ],
        )
        writer.writeheader()
        for r in table:
            writer.writerow(
                {
                    "point_id": args.point_id,
                    "n_baseline": n_base,
                    "n_improved": n_impr,
                    **r,
                }
            )

    out_md = Path(args.out_md)
    lines = [
        f"# Summary for {args.point_id}",
        "",
        f"- baseline runs: {n_base}",
        f"- improved runs: {n_impr}",
        "",
        "| metric | baseline(mean±std) | improved(mean±std) | improvement(%) |",
        "|---|---:|---:|---:|",
    ]
    for r in table:
        lines.append(
            f"| {r['metric']} | {r['baseline_mean']:.4f} ± {r['baseline_std']:.4f} | "
            f"{r['improved_mean']:.4f} ± {r['improved_std']:.4f} | {r['improvement_pct']:.2f} |"
        )
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print(f"Saved: {out_csv}")
    print(f"Saved: {out_md}")


if __name__ == "__main__":
    main()
