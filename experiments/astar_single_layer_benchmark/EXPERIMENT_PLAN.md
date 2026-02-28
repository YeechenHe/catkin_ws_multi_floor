# A\* 与改进 A\* 实验设计（v1）

本计划围绕 3 个核心实验 + 2 个扩展实验展开。

当前执行策略：先做 `cross_zone_01` 单组点位（repeat=20）验证流程可行性，再扩展到多点位。

## 实验 1：区域识别能力验证

目标：证明改进 A\* 可基于局部障碍物密度识别 `open/normal/narrow` 区域。  
输入：地图 `.yaml + .pgm`。  
输出：

- 区域分类图（`fig_region_labels.png`）
- 障碍密度热力图（`fig_density_heatmap.png`）

关键设置：

- 与算法一致的局部窗口半径 `density_radius`
- 阈值 `threshold_open` 与 `threshold_narrow`

结论判据：

- 区域图与人工预期区域一致
- 狭窄区（瓶颈）被稳定识别为 `narrow`

## 实验 2：探索栅格对比（搜索效率）

目标：对比 Baseline A\* 与改进 A\*（6 邻域）的搜索扩展差异。  
输入：两算法扩展记录 CSV（同一起终点）。  
输出：

- 扩展栅格叠加图（`fig_expansion_overlay_*.png`）
- 扩展顺序热力图（可选）

关键指标：

- `expanded_nodes`
- `plan_time_ms`
- `peak_open_list`（可选）

结论判据：

- 改进 A\* 在同等成功率下扩展节点更少或更聚焦

## 实验 3：路径质量对比（安全与质量）

目标：证明改进 A\* 路径在危险障碍附近具有更好安全裕度。  
输入：Baseline/Improved 路径 CSV + 地图。  
输出：

- 路径叠加图（`fig_path_overlay_*.png`）
- 障碍距离沿程曲线（`fig_clearance_profile_*.png`）

关键指标：

- `min_clearance_m`
- `mean_clearance_m`
- `path_length_m`
- `sum_abs_turn_rad`

结论判据：

- 改进 A* 的 `min_clearance_m` 和 `mean_clearance_m` 提升
- 路径长度变化处于可接受范围

## 实验 4（建议）：鲁棒性

对障碍密度/瓶颈宽度进行多档变化，比较趋势一致性。

## 实验 5（建议）：参数敏感性

扫描关键参数（阈值、安全权重），评估性能稳定区间。

## 公平性与统计规范

- 同地图、同起终点、同重复次数
- 随机种子固定（若有随机过程）
- 每点重复次数建议 >= 20
- 报告均值、标准差、箱线图
- 明确失败判定：超时/无路/碰撞
