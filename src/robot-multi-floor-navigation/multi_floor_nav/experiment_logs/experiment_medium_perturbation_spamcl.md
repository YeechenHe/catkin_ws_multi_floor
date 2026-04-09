# Medium 扰动 + SP-AMCL 全流程导航实验记录

本文档整理单次仿真实验的配置参数、过程指标与落盘数据，便于论文引用与复现。

## 1. 实验概述

- **场景**：多楼层仿真导航（Gazebo + ROS Noetic），启用 **模块 A（SP-AMCL 区域初始化）**。
- **扰动等级**：Medium —— 在 **L1 切图后** 对 `initialpose` 施加平面与航向偏移（L0 无扰动，避免起始层重定位卡死）。
- **论文/报告关注点**：**L1 重定位时序与曲线**。扰动 **仅施加在 L1**，L0 无 `initialpose` 偏移，**不作为 medium 扰动实验的主 KPI**；L0 相关 CSV/图仅随全流程自动落盘，正文可省略。
- **全局规划器**：Improved A*（`move_base` 插件）。
- **定位**：AMCL + SP-AMCL 分层参数（`spa_amcl_region_init.yaml`）。

### 1.1 本轮归档实验（2026-04-09，GUI 全流程）

- **启动方式**：`roslaunch ... enable_module_a:=true gui:=true headless:=false`，另终端 `rostopic pub -1 /start ...`。
- **L1 重定位 CSV（主数据）**：`experiment_logs/reloc_curve_1775700750_L1.csv`（41 个数据点，完整内容见 **§3.4**）。
- **主文曲线图（L1）**：`experiment_logs/reloc_curve_1775700750_L1_nom.png`（`plot_reloc_recovery_curve.py`，`linear_error_nom` / `angular_error_nom`，线门限 0.45 m、角门限 0.15 rad）。
- **L0 落盘（非主文）**：同次运行另生成 `reloc_curve_1775700681_L0.csv` 与 `_L0_nom.png`，**不纳入 medium 扰动结论**。
- **Improved A* 统计来源**：同次运行的 `roslaunch` 终端捕获（开发机路径示例：`~/.cursor/.../terminals/481032.txt`，仅作溯源；换机器后请用自己的日志重算）。

## 2. 实验设置（参数表）

### 2.1 SP-AMCL、扰动与电梯

| 类别 | 参数 | 数值 |
|------|------|------|
| 扰动注入 | `init_pose_offset_x` | 0.25 m |
| 扰动注入 | `init_pose_offset_y` | 0.25 m |
| 扰动注入 | `init_pose_offset_theta` | 0.15 rad |
| 重定位阈值 | `max_linear_error`（L0） | 0.20 m |
| 重定位阈值 | `max_linear_error_L1` | 0.45 m |
| 重定位阈值 | `max_angular_error` | 0.15 rad |
| 分层 sigma | L0 `σ_x, σ_y, σ_yaw` | 0.000, 0.000, 0.000 |
| 分层 sigma | L1 `σ_x, σ_y, σ_yaw` | 0.200, 0.200, 0.120 |
| 延时判据 | `reloc_min_check_delay_L0` | 6.0 s |
| 延时判据 | `reloc_min_check_delay_L1` | 12.0 s |
| 重试退避 | `region_init_backoff_factor` | 1.30 |
| 重试退避 | `region_init_max_backoff` | 2.00 |
| 电梯进入 | `lift_enter_linear_speed` | 0.25 m/s |
| 电梯进入 | `lift_enter_target_distance` | 3.00 m |
| 电梯进入 | `lift_front_safety_stop_distance` | 0.20 m |
| 电梯进入 | `lift_front_scan_half_angle_rad` | 0.25 rad |

### 2.2 Improved A* 全局规划器（算法参数）

实现为密度自适应、方向引导的代价模型 + A* 搜索；参数由 `move_base/ImprovedAStarPlanner` 命名空间加载。

| 参数 | 数值 | 含义（简述） |
|------|------|--------------|
| `density_radius` | 2 | 局部密度估计窗口半径（栅格） |
| `inflation_lambda` | 0.5 | 膨胀层在密度计算中的权重 |
| `threshold_open` | 0.20 | 开阔区域密度阈值 |
| `threshold_narrow` | 0.45 | 狭窄区域密度阈值 |
| `w_obs_open` / `normal` / `narrow` | 0.15 / 0.45 / 0.90 | 分区域障碍邻近代价权重 |
| `w_dir_open` / `normal` / `narrow` | 0.35 / 0.25 / 0.15 | 分区域方向惩罚权重 |
| `alpha_open` / `normal` / `narrow` | 1.35 / 1.15 / 1.00 | 分区域启发式膨胀系数 |
| `neutral_cost` | 50.0 | 基础代价尺度 |
| `lethal_cost_threshold` | 253 | 不可通行代价阈值（与 costmap 一致） |

**代码中固定常数（未通过 YAML 暴露，论文若讨论代价函数可引用）**

- `kBetaGoal = 0.45`、`kBetaTurn = 0.95`：目标偏向与转弯相关项系数（见 `improved_astar_planner.cpp`）。

**本轮启动时 ROS 日志回显（与上表一致）**

- `Initialized: density_radius=2, neutral_cost=50.0, threshold_open=0.20, threshold_narrow=0.45`
- `w_obs=(0.15,0.45,0.90), w_dir=(0.35,0.25,0.15), alpha=(1.35,1.15,1.00)`

**配置文件路径**

- `multi_floor_nav/launch/nav.launch`（扰动与电梯参数）
- `multi_floor_nav/configs/spa_amcl_region_init.yaml`（SP-AMCL 阈值、sigma、延时、退避）
- `multi_floor_nav/configs/improved_astar_params.yaml`（Improved A* 全部可调参数）

## 3. 本轮关键结果指标（过程与误差）

### 3.1 重定位与导航里程碑（归档运行 1775700681 / 1775700750）

以下时间戳来自仿真时钟（`roslaunch` 终端日志中的第二段时间戳，单位 s）。**与 medium 扰动相关的 KPI 以 L1 行为准**；表中 L0 行仅保留 **全流程时间线** 参考。

| 指标 | 数值 | 说明 |
|------|------:|------|
| L0 地图加载 → `RELOC_PASS` 历时 | 6.024 s | 仿真 1105.131 → 1111.155（L0 无扰动，**非主文重定位评价对象**） |
| L0 `RELOC_PASS` 名义误差 | 0.0004 m, 0.0190 rad | 日志；L0 无扰动，`linear_error_init` 与名义误差相同 |
| 请求 Lift L0 → 进入电梯 | 0.201 s | 1135.959 → 1136.160 |
| 请求 Lift L1 → 出电梯（L1 出口） | 7.000 s | 1153.360 → 1160.360 |
| 出电梯 → L1 地图加载完成 | 12.571 s | 1160.360 → 1172.931 |
| L1 自适应 sigma（首次） | (0.124, 0.124, 0.072) | `Adaptive sigma` 日志；本轮 **未** 出现 `Retry #1` |
| L1 重定位窗口（CSV） | 12.004 s | `elapsed_sec` 末行 `pass_now=1` |
| L1 通过时名义误差（CSV） | 0.3565 m, 0.0315 rad | 满足 `max_linear_error_L1=0.45`、`max_angular_error=0.15` |
| L1 首采样名义误差（CSV） | 0.3518 m, 0.1296 rad | `elapsed_sec≈1.0 s` |
| 动作层到达判定 | `status = 3` | `/move_base/status`，文本 `Goal reached.`（L1 终点导航） |

**说明**：状态机未保证打印 `DONE` 字符串时，以 **`move_base` 成功** 与 **CSV 中 `pass_now=1`** 作为全流程完成依据。

### 3.2 Improved A* 运行数据（同次运行，终端 481032 日志统计）

统计方式：对 `[ImprovedAStar] Plan found: N waypoints, M nodes expanded.` 正则匹配。

| 指标 | 数值 | 说明 |
|------|------:|------|
| 成功规划调用次数 | 1175 | 每次成功 `makePlan` 一条 |
| 路径点数（waypoints） | min / max / mean | 12 / 574 / 146.74 |
| 扩展节点数（nodes expanded） | min / max / mean | 11 / 9911 / 1282.49 |
| 规划失败次数 | 0 | 无 `Failed to find a plan` |

**高频路径形态 Top 5**

| waypoints | nodes expanded | 出现次数 |
|----------:|---------------:|---------:|
| 13 | 12 | 229 |
| 12 | 11 | 93 |
| 14 | 13 | 79 |
| 160 | 332 | 12 |
| 278 | 490 | 11 |

#### 3.2.2 规划统计使用边界说明

本节中的 `waypoints`、`expanded_nodes`、成功/失败次数等数据，仅用于说明本次 **SP-AMCL 扰动实验** 在全流程导航期间的规划运行背景，证明系统在该扰动工况下未出现规划失效。

若需开展 **Improved A* 平滑前后对比**，应使用独立实验文档 `experiment_improved_astar_smoothing_compare.md`，在固定地图、固定任务与固定定位前提下单独比较路径几何质量、安全性与时延指标，避免将定位扰动因素混入规划算法对照。

**论文表述提示**：`move_base` 周期性重规划导致调用次数远大于目标个数；若对比算法效率，建议在独立平滑对照实验中补充单次规划耗时或与基线规划器对照。

### 3.3 结果分析（论文可用表述，对齐本轮数据）

**主结论应写 L1**：在 medium 扰动下使用自适应 σ=(0.124, 0.124, 0.072)，**本轮无 sigma 退避重试**。**航向名义误差** 由约 **0.13 rad 降至约 0.032 rad**；**平面名义误差** 约在 **0.30–0.36 m** 波动，在 **`max_linear_error_L1=0.45 m`** 门限下于约 **12 s** 通过判据；可强调「分层门限下的可接受重定位」与 **角向收敛**；若需平面也压至厘米级可讨论收紧 L1 线门限的代价。L0 无扰动、仅服务起始层流程，**不作为本扰动实验的对照或主图**。Improved A* 全程 **1175** 次成功全局规划且无失败日志；`/move_base/status` 为 **`Goal reached`**，执行层到达 L1 终点。

### 3.4 L1 重定位时序数据（完整 CSV，Medium 扰动后）

**数据来源**：`experiment_logs/reloc_curve_1775700750_L1.csv`（切至 L1 地图后对 **名义真值** 的 `linear_error_nom` / `angular_error_nom` 等；**扰动仅作用于 L1 的 `initialpose` 发布**）。下列为与磁盘文件一致的全量内容（**1 行表头 + 41 个采样点**），可直接复制到表格软件或绑图脚本。

**作图常用列**：`elapsed_sec`、`linear_error_nom`、`angular_error_nom`；门限 **线 0.45 m、角 0.15 rad**；**`RELOC_PASS`** 对应末行 `elapsed_sec=12.0040`、`pass_now=1`。

```csv
elapsed_sec,linear_error_nom,angular_error_nom,linear_error_pub,angular_error_pub,tr_sigma,C_t,pose_ok,pass_now
1.0040,0.3518,0.1296,0.0124,0.0204,0.0339,0.9666,0,0
1.2050,0.3313,0.1121,0.0367,0.0379,0.0304,0.9701,0,0
1.6040,0.3190,0.0825,0.0650,0.0675,0.0256,0.9748,0,0
1.8040,0.3124,0.0743,0.0739,0.0757,0.0233,0.9770,0,0
2.0050,0.3041,0.0674,0.0858,0.0826,0.0212,0.9791,0,0
2.2050,0.3006,0.0605,0.0901,0.0895,0.0196,0.9806,0,0
2.6040,0.3079,0.0537,0.0835,0.0963,0.0165,0.9836,0,0
3.0040,0.3031,0.0511,0.0777,0.0989,0.0143,0.9858,0,0
3.2040,0.3091,0.0473,0.0799,0.1027,0.0132,0.9869,0,0
3.4050,0.3086,0.0463,0.0786,0.1037,0.0133,0.9868,0,0
3.6060,0.3139,0.0459,0.0751,0.1041,0.0121,0.9880,0,0
4.0060,0.3186,0.0443,0.0673,0.1057,0.0110,0.9890,0,0
4.4040,0.3208,0.0422,0.0681,0.1078,0.0105,0.9896,0,0
4.6050,0.3176,0.0431,0.0719,0.1069,0.0102,0.9898,0,0
4.8050,0.3269,0.0402,0.0715,0.1098,0.0093,0.9907,0,0
5.2040,0.3325,0.0414,0.0700,0.1086,0.0089,0.9911,0,0
5.4050,0.3342,0.0391,0.0705,0.1109,0.0086,0.9915,0,0
5.8050,0.3264,0.0391,0.0699,0.1109,0.0093,0.9907,0,0
6.2040,0.3353,0.0318,0.0739,0.1182,0.0076,0.9924,0,0
6.4070,0.3393,0.0322,0.0769,0.1178,0.0079,0.9921,0,0
6.8060,0.3396,0.0303,0.0769,0.1197,0.0076,0.9924,0,0
7.0060,0.3437,0.0302,0.0769,0.1198,0.0074,0.9926,0,0
7.4040,0.3412,0.0324,0.0781,0.1176,0.0077,0.9923,0,0
7.6050,0.3412,0.0319,0.0773,0.1181,0.0072,0.9928,0,0
8.0050,0.3421,0.0322,0.0781,0.1178,0.0067,0.9933,0,0
8.2060,0.3401,0.0326,0.0775,0.1174,0.0068,0.9932,0,0
8.6040,0.3406,0.0327,0.0761,0.1173,0.0068,0.9932,0,0
8.8050,0.3414,0.0330,0.0771,0.1170,0.0066,0.9934,0,0
9.2050,0.3430,0.0314,0.0784,0.1186,0.0061,0.9939,0,0
9.6040,0.3454,0.0316,0.0792,0.1184,0.0058,0.9942,0,0
9.8040,0.3483,0.0311,0.0797,0.1189,0.0054,0.9946,0,0
10.0050,0.3481,0.0312,0.0792,0.1188,0.0054,0.9947,0,0
10.2050,0.3525,0.0308,0.0805,0.1192,0.0051,0.9949,0,0
10.4050,0.3518,0.0308,0.0817,0.1192,0.0050,0.9950,0,0
10.6060,0.3530,0.0314,0.0821,0.1186,0.0049,0.9951,0,0
10.8060,0.3512,0.0306,0.0804,0.1194,0.0048,0.9952,0,0
11.2040,0.3538,0.0300,0.0823,0.1200,0.0045,0.9955,0,0
11.4040,0.3535,0.0304,0.0822,0.1196,0.0045,0.9956,0,0
11.6050,0.3549,0.0306,0.0846,0.1194,0.0045,0.9955,0,0
11.8060,0.3558,0.0310,0.0860,0.1190,0.0044,0.9956,0,0
12.0040,0.3565,0.0315,0.0868,0.1185,0.0044,0.9956,1,1
```

**L0**：同次运行虽有 `reloc_curve_1775700681_L0.csv` 落盘，**因无扰动、不反映 medium 设定，正文与主图可完全不引用**。

## 4. 重定位曲线落盘（CSV）与论文作图

**主文实验图约定（L1）**：**仅 L1** 的 **`linear_error_nom`、`angular_error_nom`** 随 `elapsed_sec`（相对名义真值）作为主图；线/角门限以虚线标在图中。**`linear_error_pub` / `angular_error_pub`** 不列入主文 KPI 图。**L1 平面名义误差** 在本轮中未出现单调下降至厘米级，图注应说明与 **`max_linear_error_L1`** 的关系。**L0 曲线不作为本 medium 扰动实验的主图**（扰动未加在 L0）。

### 4.1 指标定义（与 `RELOC_PASS` 日志一致）

- **`linear_error_nom` / `angular_error_nom`**：AMCL 位姿相对 **名义真值** `desired_init_pose`（地图标称点，不含扰动）。论文中描述「收敛到真实位姿」应使用这一对曲线；`RELOC_PASS` 中的 `linear_error` / `angular_error` 即该定义。
- **`linear_error_pub` / `angular_error_pub`**：AMCL 相对 **实际发布的 initialpose**（L1 为名义点 + medium 扰动；L0 与名义相同）。`RELOC_PASS` 中的 `linear_error_init` / `angular_error_init` 即通过瞬间该定义下的误差。
- **`elapsed_sec`**：自本次进入 `INIT_POSE` 起算（含 `reloc_min_check_delay` 等待段），便于画完整收敛过程。
- **`pose_ok`**：仅在与 `after_delay` 之后才可能为 1，表示已满足当前楼层的线/角门槛。
- **`pass_now`**：本采样行是否判定重定位通过（与 `check_robot_pose` 一致）。

### 4.2 文件命名与采样

- 每层重定位会生成独立文件：`experiment_logs/reloc_curve_<wall_time>_L0.csv`、`_L1.csv`（未设置 `~reloc_curve_csv_path` 时）。
- 默认每 **`reloc_curve_sample_period_sec`**（默认 **0.2 s**）写入一行；通过时刻 **保证额外写一行**（`pass_now=1`）。
- 若需更密曲线可在 launch 中设置：`<param name="reloc_curve_sample_period_sec" value="0.1" />`。

### 4.3 CSV 列格式（当前实现）

```text
elapsed_sec,linear_error_nom,angular_error_nom,linear_error_pub,angular_error_pub,tr_sigma,C_t,pose_ok,pass_now
```

### 4.4 生成论文用曲线图

默认脚本绘制 **名义误差**，兼容旧版仅含 `linear_error` 列的 CSV：

```bash
python3 "$(rospack find multi_floor_nav)/scripts/plot_reloc_recovery_curve.py" \
  --input_csv /path/to/reloc_curve_*_L1.csv \
  --output_png reloc_L1_nom.png \
  --linear_threshold 0.45 \
  --angular_threshold 0.15
```

论文绑图 **仅需 L1** 一行命令；L0 仅当需要调试起始层时再用 `--linear_threshold 0.20`。

### 4.5 本轮已生成图稿（仓库内路径）

| 用途 | 输入 CSV | 输出 PNG |
|------|----------|----------|
| **主文（L1，Medium 扰动）** | `experiment_logs/reloc_curve_1775700750_L1.csv` | `experiment_logs/reloc_curve_1775700750_L1_nom.png` |
| 可选（L0，无扰动，非 KPI） | `experiment_logs/reloc_curve_1775700681_L0.csv` | `experiment_logs/reloc_curve_1775700681_L0_nom.png` |

**主文复现（L1）**（在 catkin 工作空间已 `source devel/setup.bash` 时）：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775700750_L1.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775700750_L1_nom.png" \
  --linear_threshold 0.45 --angular_threshold 0.15
```

**可选（L0，非 medium 扰动结论）**：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775700681_L0.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775700681_L0_nom.png" \
  --linear_threshold 0.20 --angular_threshold 0.15
```

### 4.6 历史单次运行样本（旧 CSV 格式，仅一行）

以下为代码改动前、仅延迟结束后单次采样的示例（**无法**作时间序列）：

**文件**：`reloc_curve_1775660655_L0.csv`（旧列名）

| elapsed_sec | linear_error (m) | angular_error (rad) | tr_sigma | C_t | pose_ok | pass_now |
|------------:|-----------------:|--------------------:|---------:|----:|--------:|---------:|
| 12.0030 | 0.4284 | 0.1169 | 0.0043 | 0.9957 | 1 | 1 |

重新编译并跑通全流程后，应使用新生成的 `_L0.csv` / `_L1.csv` 绘制「随时间下降」曲线。

## 5. 复现命令（参考）

```bash
cd /path/to/catkin_ws
source devel/setup.bash
# 同时打开 Gazebo（gzclient）与 RViz：务必 gui:=true，且勿用 headless:=true
roslaunch multi_floor_nav nav.launch enable_module_a:=true gui:=true headless:=false
# 另终端
rostopic pub -1 /start std_msgs/Empty "{}"
```

## 6. 修订历史

| 日期 | 说明 |
|------|------|
| 2026-04-09 | 根据 medium 扰动单次运行日志与参数整理初版 |
| 2026-04-09 | 增补 Improved A* 参数表、启动日志回显及 `Plan found` 统计（终端 739400.txt） |
| 2026-04-09 | 重定位 CSV：等待期内周期采样、分列名义/发布误差、按楼层独立文件；更新 §4 作图说明 |
| 2026-04-09 | 约定主文实验图仅用 `linear_error_nom`、`angular_error_nom` |
| 2026-04-09 | 写入归档运行 1775700681/1775700750 指标、更新 Improved A* 统计（481032）、生成 L0/L1 `_nom.png` 并增加 §1.1 / §3.4 / §4.5 |
| 2026-04-09 | §3.4 嵌入 L1 完整 CSV；全文标明主 KPI 为 L1、L0 无扰动不纳入 medium 结论 |
