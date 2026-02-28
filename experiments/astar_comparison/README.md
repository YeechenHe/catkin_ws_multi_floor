# A* 对比实验脚手架

本目录用于对比：
- 传统 A*：`use_dijkstra=false`, `use_improved_astar=false`
- 改进 A*：`use_dijkstra=false`, `use_improved_astar=true`

## 文件说明

- `experiment_points.csv`：起终点测试集模板（你填写/扩充）
- `results_template.csv`：结果记录模板（每次实验填一行）
- `plot_results.py`：自动出图脚本（箱线图、柱状图、散点图）
- `plot_path_overlay.py`：路径叠加图脚本（地图上叠加 Baseline/Improved）
- `export_path_from_bag.py`：从 rosbag 的 `nav_msgs/Path` 自动导出 x,y CSV

## 建议实验流程

1. 准备起终点：
   - 按 `experiment_points.csv` 填好宽阔/常规/狭窄三类场景
   - 每类至少 10 组起终点

2. 分别运行两种算法：
   - Baseline：`use_improved_astar=false`
   - Ours：`use_improved_astar=true`
   - 其余参数保持一致，保证公平

3. 每个点记录指标到 `results.csv`（可由 `results_template.csv` 复制得到）：
   - `success`（0/1）
   - `plan_time_ms`
   - `path_length_m`
   - `min_clearance_m`
   - `mean_clearance_m`
   - `sum_abs_turn_rad`
   - `nav_time_s`（可选）
   - `replan_count`（可选）
   - `collision_count`（可选）

4. 自动出图：

```bash
cd /home/eethanhe/catkin_ws/experiments/astar_comparison
python3 plot_results.py --input results.csv --outdir figures
```

## 自动导出路径 CSV（推荐）

如果你录制了 rosbag，可以直接从 Path 话题导出 CSV，避免手工抄路径点。

### 1) 导出 Baseline 路径

```bash
cd /home/eethanhe/catkin_ws/experiments/astar_comparison
python3 export_path_from_bag.py \
  --bag case1_baseline.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --out baseline_case1.csv \
  --select last
```

### 2) 导出 Improved 路径

```bash
python3 export_path_from_bag.py \
  --bag case1_improved.bag \
  --topic /move_base/ImprovedAStarPlanner/plan \
  --out improved_case1.csv \
  --select last
```

### 3) 生成路径叠加图

```bash
python3 plot_path_overlay.py \
  --map-yaml /home/eethanhe/catkin_ws/src/robot-multi-floor-navigation/multi_floor_nav/maps/map_level0.yaml \
  --baseline baseline_case1.csv \
  --improved improved_case1.csv \
  --out figures/overlay_case1.png \
  --title "Level0 Case1 Overlay"
```

> `--select` 说明：
> - `last`：导出最后一条 Path（最常用）
> - `first`：导出第一条 Path
> - `all`：导出所有 Path（会多出 `path_idx,msg_time` 列）

## 快速参数切换（对应 nav.launch）

在 `multi_floor_nav/launch/nav.launch` 中：

- Baseline：
  - `use_dijkstra: false`
  - `use_improved_astar: false`

- Ours：
  - `use_dijkstra: false`
  - `use_improved_astar: true`

## 画图输出

脚本会生成：
- `fig_plan_time_boxplot.png`：规划时间箱线图
- `fig_path_length_bar.png`：路径长度柱状图（带标准差）
- `fig_min_clearance_bar.png`：最小障碍距离柱状图
- `fig_success_rate_bar.png`：成功率柱状图
- `fig_efficiency_safety_scatter.png`：效率-安全散点图

