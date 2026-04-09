# 跨层重定位闭环实验报告（本轮）

## 1. 实验目的

- 检查地图切换后，机器人能否在目标楼层（L1）恢复有效定位。
- 检查重定位完成后，系统能否继续执行后续导航任务。
- 验证重定位结果是否足以支撑完整跨层任务闭环（L0 -> 电梯 -> L1 -> 目标到达）。

## 2. 实验环境与对象

- 工作空间：`/home/eethanhe/catkin_ws`
- 关键节点：`multi_floor_navigation_node`、`change_map_node`、`amcl`、`move_base`
- 关键日志源：`~/.ros/log/latest/rosout.log`
- 本轮检查时间：2026-04-08

## 3. 关键结论

1. **地图切换后，L1 重定位可成功恢复**。  
   证据：`Successfully loaded map for level: 1` 后出现 `RELOC_PASS`。

2. **重定位完成后，后续导航可继续执行并完成**。  
   证据：`Will send goal to x: 4, y: 5, at level: 1`，最终 `Robot arrived at Goal`。

3. **本轮已形成完整跨层闭环**（切图、重定位、发目标、到达全链路成功）。  
   证据：`move_base/status` 为 `status: 3`，`text: "Goal reached."`。

## 4. 关键日志时间线（仿真时钟）

- `2715.669`：`Successfully loaded map for level: 0`
- `2721.689`：L0 `RELOC_PASS`
- `2774.089`：`Exiting Lift at Level 1`
- `2786.668`：`Successfully loaded map for level: 1`
- `2786.690`：`Initializing Pose to x: 3.00, y: -0.50 at level: 1`
- `2792.692`：L1 `RELOC_PASS: linear_error=0.0005, angular_error=0.0283, ... C_t=1.0000`
- `2793.693`：`Will send goal to x: 4, y: 5, at level: 1`
- `2839.493`：`Robot arrived at Goal`

## 5. 量化指标（本轮）

- L1 切图成功到 L1 重定位通过耗时：  
  `2792.692 - 2786.668 = 6.024 s`

- L1 目标下发到到达耗时：  
  `2839.493 - 2793.693 = 45.800 s`

- L1 重定位通过时误差：
  - `linear_error = 0.0005`
  - `angular_error = 0.0283`
  - `tr_sigma = 0.0000`
  - `C_t = 1.0000`

- `move_base` 最终状态：
  - `status: 3`
  - `text: "Goal reached."`

## 6. 本轮关键参数（运行时读取）

### 6.1 ImprovedAStar 规划器参数

- `density_radius: 2`
- `inflation_lambda: 0.5`
- `lethal_cost_threshold: 253`
- `neutral_cost: 50.0`
- `threshold_open: 0.2`
- `threshold_narrow: 0.45`
- `w_obs_open: 0.15`
- `w_obs_normal: 0.45`
- `w_obs_narrow: 0.9`
- `w_dir_open: 0.35`
- `w_dir_normal: 0.25`
- `w_dir_narrow: 0.15`
- `alpha_open: 1.35`
- `alpha_normal: 1.15`
- `alpha_narrow: 1.0`

### 6.2 AMCL 初始位姿（运行时）

- `initial_pose_x: 3.905543907975116`
- `initial_pose_y: 4.819605301089716`
- `initial_pose_a: 0.9626441391293512`

> 注：该组为当前参数服务器中的 AMCL 初始位姿条目；跨层任务中，`multi_floor_navigation_node` 会在状态机中发布重定位初始化位姿（例如 L1 使用 `x=3.00, y=-0.50`），以实际日志为准。

## 7. 可参考图像与可视化参数

### 7.1 生成图像

- `experiment_logs/obstacle_density_floor0_static_map.png`
- `experiment_logs/obstacle_density_floor1_static_map.png`

### 7.2 主要可视化参数（当前脚本）

- `input_mode=static_map`
- `density_radius=10`
- `display_dilate_radius=3`
- `display_gamma=0.75`
- `imshow_vmax_mode=stretch`
- `auto_center_crop=true`
- `center_crop_margin_cells=45`
- `center_crop_threshold_ratio=0.05`
- 色条与主图同高（轴绑定）

## 8. 结果判定

- 地图切换后可恢复有效定位：**通过**
- 重定位后可继续执行导航：**通过**
- 能支撑完整跨层任务闭环：**通过**

本轮实验满足跨层闭环验证要求。

