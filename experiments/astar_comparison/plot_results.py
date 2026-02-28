#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


REQUIRED_COLUMNS = [
    "algo",
    "scene_type",
    "success",
    "plan_time_ms",
    "path_length_m",
    "min_clearance_m",
]


def _check_columns(df: pd.DataFrame):
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"results.csv 缺少列: {missing}")


def _normalize_algo_label(df: pd.DataFrame) -> pd.DataFrame:
    mapping = {
        "baseline": "Baseline A*",
        "improved": "Improved A*",
        "astar": "Baseline A*",
        "improved_astar": "Improved A*",
    }
    out = df.copy()
    out["algo_show"] = out["algo"].astype(str).str.lower().map(mapping).fillna(out["algo"])
    return out


def _save_fig(path: Path):
    plt.tight_layout()
    plt.savefig(path, dpi=220, bbox_inches="tight")
    plt.close()


def make_plots(df: pd.DataFrame, outdir: Path):
    sns.set_theme(style="whitegrid")
    outdir.mkdir(parents=True, exist_ok=True)

    # 1) 规划时间箱线图
    plt.figure(figsize=(9, 5))
    sns.boxplot(data=df, x="scene_type", y="plan_time_ms", hue="algo_show")
    plt.title("Planning Time Comparison")
    plt.xlabel("Scene Type")
    plt.ylabel("Planning Time (ms)")
    _save_fig(outdir / "fig_plan_time_boxplot.png")

    # 2) 路径长度柱状图（均值+标准差）
    plt.figure(figsize=(9, 5))
    sns.barplot(data=df, x="scene_type", y="path_length_m", hue="algo_show", ci="sd")
    plt.title("Path Length Comparison")
    plt.xlabel("Scene Type")
    plt.ylabel("Path Length (m)")
    _save_fig(outdir / "fig_path_length_bar.png")

    # 3) 最小障碍距离柱状图
    plt.figure(figsize=(9, 5))
    sns.barplot(data=df, x="scene_type", y="min_clearance_m", hue="algo_show", ci="sd")
    plt.title("Minimum Obstacle Clearance Comparison")
    plt.xlabel("Scene Type")
    plt.ylabel("Minimum Clearance (m)")
    _save_fig(outdir / "fig_min_clearance_bar.png")

    # 4) 成功率柱状图
    success_df = (
        df.groupby(["scene_type", "algo_show"], as_index=False)["success"]
        .mean()
        .rename(columns={"success": "success_rate"})
    )
    plt.figure(figsize=(9, 5))
    sns.barplot(data=success_df, x="scene_type", y="success_rate", hue="algo_show")
    plt.title("Success Rate Comparison")
    plt.xlabel("Scene Type")
    plt.ylabel("Success Rate")
    plt.ylim(0, 1.02)
    _save_fig(outdir / "fig_success_rate_bar.png")

    # 5) 效率-安全散点图（路径长度 vs 最小障碍距离）
    plt.figure(figsize=(8, 6))
    sns.scatterplot(
        data=df,
        x="path_length_m",
        y="min_clearance_m",
        hue="algo_show",
        style="scene_type",
        s=80,
    )
    plt.title("Efficiency-Safety Trade-off")
    plt.xlabel("Path Length (m) [Lower is better]")
    plt.ylabel("Minimum Clearance (m) [Higher is better]")
    _save_fig(outdir / "fig_efficiency_safety_scatter.png")

    # 6) 导出聚合统计表
    summary = (
        df.groupby(["scene_type", "algo_show"], as_index=False)
        .agg(
            n=("success", "count"),
            success_rate=("success", "mean"),
            plan_time_ms_mean=("plan_time_ms", "mean"),
            plan_time_ms_std=("plan_time_ms", "std"),
            path_length_m_mean=("path_length_m", "mean"),
            path_length_m_std=("path_length_m", "std"),
            min_clearance_m_mean=("min_clearance_m", "mean"),
            min_clearance_m_std=("min_clearance_m", "std"),
        )
    )
    summary.to_csv(outdir / "summary_stats.csv", index=False)


def main():
    parser = argparse.ArgumentParser(description="A* 对比实验自动出图")
    parser.add_argument("--input", required=True, help="结果 CSV 路径")
    parser.add_argument("--outdir", default="figures", help="图表输出目录")
    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"找不到输入文件: {input_path}")

    df = pd.read_csv(input_path)
    _check_columns(df)
    df = _normalize_algo_label(df)

    # 保证 success 为数值 0/1
    df["success"] = pd.to_numeric(df["success"], errors="coerce").fillna(0.0)

    outdir = Path(args.outdir)
    make_plots(df, outdir)
    print(f"[OK] 图表已输出到: {outdir.resolve()}")


if __name__ == "__main__":
    main()

