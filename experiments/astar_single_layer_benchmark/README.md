# 单层栅格 A* 对比实验框架

本目录用于在**单层栅格地图**上对比：

- 基线 A*（baseline）
- 改进 A*（improved）

目标是减少多楼层系统耦合，突出算法本身差异。

## 目录结构

- `maps/`：地图与地图说明
- `configs/`：实验模板（点位、结果、扩展轨迹、路径轨迹）
- `scripts/`：地图生成、批跑骨架、实验可视化脚本
- `results/`：实验结果 CSV（运行后生成）
- `figures/`：输出图像（运行后生成）
- `EXPERIMENT_PLAN.md`：实验 1~5 设计说明

## 当前支持的核心实验

> 当前推荐先做 **1 组跨区点位（repeat=20）** 验证可行性，再扩展到多组点位。

### 快速验证“绕障轨迹”（离线）

如果你暂时没有 rosbag，可先用离线搜索生成**不穿障**轨迹：

```bash
python3 scripts/generate_offline_traces.py \
  --map-yaml maps/contrast_case1.yaml \
  --points-csv configs/experiment_points_case1.csv \
  --point-id cross_zone_01 \
  --path-out results/path_trace.csv \
  --expansion-out results/expansion_trace.csv
```

再分别绘图：

```bash
python3 scripts/plot_expansion_comparison.py \
  --map-yaml maps/contrast_case1.yaml \
  --trace results/expansion_trace.csv \
  --point-id cross_zone_01 \
  --out figures/fig_expansion_overlay_cross_zone_01.png

python3 scripts/plot_path_comparison.py \
  --map-yaml maps/contrast_case1.yaml \
  --trace results/path_trace.csv \
  --point-id cross_zone_01 \
  --overlay-out figures/fig_path_overlay_cross_zone_01.png \
  --clearance-out figures/fig_clearance_profile_cross_zone_01.png
```

> 离线轨迹用于验证地图/可视化流程；正式对比结论请使用真实 planner 导出的路径与扩展数据。

### 实验 1：区域识别图（证明可识别）

使用 `plot_region_recognition.py` 从地图直接计算局部密度并划分 `open/normal/narrow`。

```bash
python3 scripts/plot_region_recognition.py \
  --map-yaml maps/contrast_case1.yaml \
  --outdir figures \
  --density-radius 2 \
  --inflation-lambda 0.5 \
  --threshold-open 0.20 \
  --threshold-narrow 0.45
```

输出：

- `figures/fig_density_heatmap.png`
- `figures/fig_region_labels.png`

### 实验 2：探索栅格对比（证明 6 邻域优势）

先准备扩展轨迹 CSV（参考 `configs/expansion_trace_template.csv`），再可视化：

```bash
python3 scripts/plot_expansion_comparison.py \
  --map-yaml maps/contrast_case1.yaml \
  --trace results/expansion_trace.csv \
  --point-id cross_zone_01 \
  --out figures/fig_expansion_overlay_cross_zone_01.png
```

输出：

- 扩展栅格叠加图
- 命令行统计：baseline/improved 唯一扩展格数量与重叠数量

### 实验 3：路径质量对比（证明避险能力）

先准备路径轨迹 CSV（参考 `configs/path_trace_template.csv`），再可视化。

> **重要**：路径叠加图必须使用**真实规划器**（baseline/improved A\*）导出的路径点。模板 CSV 仅为流程占位，若直接使用会出现轨迹穿过障碍物的假象；用真实 `nav_msgs/Path` 导出后再画图即可得到避障路径。

先从 rosbag 导出 `path_trace.csv`（导 baseline，再导 improved）：

```bash
# baseline
python3 scripts/export_path_trace_from_bag.py \
  --bag /path/to/case1_baseline.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --algo baseline \
  --point-id cross_zone_01 \
  --run-id 1 \
  --out results/path_trace.csv \
  --overwrite

# improved
python3 scripts/export_path_trace_from_bag.py \
  --bag /path/to/case1_improved.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --algo improved \
  --point-id cross_zone_01 \
  --run-id 2 \
  --out results/path_trace.csv
```

```bash
python3 scripts/plot_path_comparison.py \
  --map-yaml maps/contrast_case1.yaml \
  --trace results/path_trace.csv \
  --point-id cross_zone_01 \
  --overlay-out figures/fig_path_overlay_cross_zone_01.png \
  --clearance-out figures/fig_clearance_profile_cross_zone_01.png
```

输出：

- 路径叠加图
- 沿程 clearance 曲线
- 命令行统计：`path_length_m`、`min_clearance_m`、`mean_clearance_m`

## 快速开始

```bash
cd /home/eethanhe/catkin_ws/experiments/astar_single_layer_benchmark

# 1) 生成强对比地图（强制跨 open/normal/narrow）
python3 scripts/build_contrast_map.py \
  --out-map maps/contrast_case1

# 2) 使用单组点位（repeat=20）生成批跑命令骨架
python3 scripts/run_batch_placeholder.py \
  --points configs/experiment_points_case1.csv

# 3) 填入真实结果后做总览图
cp configs/results_template.csv results/results.csv
python3 scripts/plot_results.py \
  --input results/results.csv \
  --outdir figures

# 4) 对单组点位做统计汇总（均值/标准差/提升率）
python3 scripts/summarize_metrics.py \
  --input results/results.csv \
  --point-id cross_zone_01 \
  --out-csv results/summary_case1.csv \
  --out-md results/summary_case1.md
```

## 模板文件说明

- `configs/experiment_points_template.csv`：起终点定义
- `configs/experiment_points_case1.csv`：单组跨区点位（repeat=20）
- `configs/results_template.csv`：聚合指标结果
- `configs/expansion_trace_template.csv`：扩展轨迹模板（实验 2）
- `configs/path_trace_template.csv`：路径轨迹模板（实验 3）

## 场景类型约定

- `narrow`：狭窄通道/瓶颈
- `normal`：常规走廊与转角
- `open`：宽阔区域
