# 重定位数据汇总（当前有效版本）

## 1. 说明

- 本文汇总当前多楼层导航实验中与“扰动”和“重定位”直接相关的数据。
- 数据来源：
  - `src/multi_floor_navigation.cpp`
  - `configs/spa_amcl_region_init.yaml`
  - `experiment_logs/CROSS_FLOOR_RELOC_CLOSURE_REPORT_2026-04-08.md`
- 口径说明：
  - 这里的“当前扰动”指 `initialpose` 发布前注入的 `init_pose_offset_*`。
  - 这里的“重定位指标”优先采用闭环报告中给出的时间线与数值。

## 2. 当前扰动（Perturbation）

`set_init_pose()` 中扰动参数读取与默认值如下（默认全为 0）：

- `init_pose_offset_x`：默认 `0.0`
- `init_pose_offset_y`：默认 `0.0`
- `init_pose_offset_theta`：默认 `0.0`

结论：当前实验按默认配置运行时，**初始位姿扰动为 0**（无额外扰动注入）。

## 3. 当前重定位算法形态

- 基础定位器：`AMCL`
- 跨层重定位策略：`SP-AMCL`（模块 A，区域初始化）
- 启动方式：`nav.launch` 中 `enable_module_a:=true` 时加载 `spa_amcl_region_init.yaml`

## 4. SP-AMCL 关键配置（当前）

来自 `configs/spa_amcl_region_init.yaml`：

### 4.1 区域初始化 sigma

- 全局 fallback：
  - `region_init_sigma_x = 0.15`
  - `region_init_sigma_y = 0.15`
  - `region_init_sigma_yaw = 0.10`

- 分楼层覆盖：
  - L0：`region_init_sigma_x_L0 = 0.0`，`region_init_sigma_y_L0 = 0.0`，`region_init_sigma_yaw_L0 = 0.0`
  - L1：`region_init_sigma_x_L1 = 0.20`，`region_init_sigma_y_L1 = 0.20`，`region_init_sigma_yaw_L1 = 0.12`

### 4.2 重定位判定相关阈值

- `max_linear_error = 0.20`
- `max_linear_error_L1 = 0.50`
- `max_angular_error = 0.15`

### 4.3 重定位等待与退避

- `reloc_min_check_delay_sec = 6.0`
- `reloc_min_check_delay_L0 = 6.0`
- `reloc_min_check_delay_L1 = 12.0`
- `region_init_backoff_factor = 1.3`
- `region_init_max_backoff = 2.0`

## 5. 本轮闭环报告中的重定位结果

来自 `experiment_logs/CROSS_FLOOR_RELOC_CLOSURE_REPORT_2026-04-08.md`：

### 5.1 L1 重定位关键时间点（仿真时钟）

- `2786.668`：成功切换到 L1 地图
- `2792.692`：L1 `RELOC_PASS`
- `2793.693`：下发 L1 导航目标
- `2839.493`：到达最终目标

### 5.2 量化指标

- L1 切图到 L1 重定位通过耗时：`6.024 s`
- L1 目标下发到目标到达耗时：`45.800 s`

### 5.3 L1 重定位通过时误差

- `linear_error = 0.0005`
- `angular_error = 0.0283`
- `tr_sigma = 0.0000`
- `C_t = 1.0000`

### 5.4 任务闭环结果

- `move_base status = 3`
- `text = "Goal reached."`
- 判定：跨层导航闭环通过

## 6. 当前阶段结论

1. 当前默认扰动为 0，实验可作为“无扰动基线”参考。  
2. 当前重定位方案为 AMCL 底座 + SP-AMCL（模块 A）区域初始化。  
3. 现有闭环报告已给出完整可复核重定位数据，且任务闭环成功。  

## 7. 后续建议记录项（用于多次复现实验）

建议后续每轮补充以下字段，便于横向比较：

- 扰动：`init_pose_offset_x/y/theta`
- L1 重定位通过耗时
- `linear_error / angular_error / tr_sigma / C_t`
- 电梯口行为标签（平稳进入 / 来回调整 / 安全停 / 擦碰）
- 最终闭环结果（成功/失败）与失败阶段

