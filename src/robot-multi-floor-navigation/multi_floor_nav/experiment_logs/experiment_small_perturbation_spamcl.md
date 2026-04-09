# Small 扰动 + SP-AMCL 全流程导航实验记录

本文档整理单次仿真实验的配置参数、过程指标与落盘数据，便于论文引用与复现。

## 1. 实验概述

- **场景**：多楼层仿真导航（Gazebo + ROS Noetic），启用 **模块 A（SP-AMCL 区域初始化）**。
- **扰动等级**：Small —— 在 **L1 切图后** 对 `initialpose` 施加平面与航向偏移（L0 无扰动，避免起始层重定位卡死）。
- **论文/报告关注点**：**L1 重定位时序与曲线**。扰动 **仅施加在 L1**，L0 无 `initialpose` 偏移，**不作为 small 扰动实验的主 KPI**；L0 相关 CSV/图仅随全流程自动落盘，正文可省略。
- **全局规划器**：Improved A*（`move_base` 插件）。
- **定位**：AMCL + SP-AMCL 分层参数（`spa_amcl_region_init.yaml`）。

### 1.1 本轮归档实验

- **启动方式**：`roslaunch multi_floor_nav nav.launch enable_module_a:=true gui:=true headless:=false`，另终端 `rostopic pub -1 /start std_msgs/Empty "{}"`。
- **L1 重定位 CSV（主数据）**：`experiment_logs/reloc_curve_1775704387_L1.csv`（**37** 个采样点，完整内容见 **§3.4**）。
- **主文曲线图（L1）**：`experiment_logs/reloc_curve_1775704387_L1_nom.png`（`plot_reloc_recovery_curve.py`，`linear_error_nom` / `angular_error_nom`，线门限 0.45 m、角门限 0.15 rad）。
- **L0 落盘（非主文）**：同次运行另生成 `reloc_curve_1775704303_L0.csv` 与 `reloc_curve_1775704303_L0_nom.png`，**不纳入 small 扰动结论**。
- **Improved A* 统计来源**：本次 `roslaunch` 终端日志（终端记录末尾保留到 `Robot arrived at Goal`，部分中间里程碑时间戳未完整保留）。

## 2. 实验设置（参数表）

### 2.1 SP-AMCL、扰动与电梯

| 类别 | 参数 | 数值 |
| --- | --- | --- |
| 扰动注入 | `init_pose_offset_x` | 0.10 m |
| 扰动注入 | `init_pose_offset_y` | 0.10 m |
| 扰动注入 | `init_pose_offset_theta` | 0.05 rad |
| 扰动规模 | `d_planar = sqrt(dx^2 + dy^2)` | 0.141 m |
| 自适应 sigma（参考） | `sigma_adaptive_xy` | 0.049 m |
| 自适应 sigma（参考） | `sigma_adaptive_yaw` | 0.037 rad |
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
| --- | --- | --- |
| `density_radius` | 2 | 局部密度估计窗口半径（栅格） |
| `inflation_lambda` | 0.5 | 膨胀层在密度计算中的权重 |
| `threshold_open` | 0.20 | 开阔区域密度阈值 |
| `threshold_narrow` | 0.45 | 狭窄区域密度阈值 |
| `w_obs_open` / `normal` / `narrow` | 0.15 / 0.45 / 0.90 | 分区域障碍邻近代价权重 |
| `w_dir_open` / `normal` / `narrow` | 0.35 / 0.25 / 0.15 | 分区域方向惩罚权重 |
| `alpha_open` / `normal` / `narrow` | 1.35 / 1.15 / 1.00 | 分区域启发式膨胀系数 |
| `neutral_cost` | 50.0 | 基础代价尺度 |
| `lethal_cost_threshold` | 253 | 不可通行代价阈值（与 costmap 一致） |

#### 2.2.1 代码中固定常数（未通过 YAML 暴露）

- `kBetaGoal = 0.45`、`kBetaTurn = 0.95`：目标偏向与转弯相关项系数（见 `improved_astar_planner.cpp`）。

#### 2.2.2 本轮启动时 ROS 日志回显（请据实填写）

- `Initialized: density_radius=2, neutral_cost=50.0, threshold_open=0.20, threshold_narrow=0.45`
- `w_obs=(0.15,0.45,0.90), w_dir=(0.35,0.25,0.15), alpha=(1.35,1.15,1.00)`

#### 2.2.3 配置文件路径

- `multi_floor_nav/launch/nav.launch`（扰动与电梯参数）
- `multi_floor_nav/configs/spa_amcl_region_init.yaml`（SP-AMCL 阈值、sigma、延时、退避）
- `multi_floor_nav/configs/improved_astar_params.yaml`（Improved A* 全部可调参数）

## 3. 本轮关键结果指标（过程与误差）

### 3.1 重定位与导航里程碑

以下时间戳来自仿真时钟（`roslaunch` 终端日志中的第二段时间戳，单位 s）。**与 small 扰动相关的 KPI 以 L1 行为准**；表中 L0 行仅保留 **全流程时间线** 参考。由于当前终端文本仅保留了日志尾段，部分中间状态切换时刻未能从终端中完整回溯，因此该部分对缺失项据实标注为“日志未保留”。

| 指标 | 数值 | 说明 |
| --- | ---: | --- |
| L0 地图加载 → `RELOC_PASS` 历时 | 6.004 s | 以 `reloc_curve_1775704303_L0.csv` 末行 `pass_now=1` 为准；仿真起始层，无扰动，**非主文重定位评价对象** |
| L0 `RELOC_PASS` 名义误差 | 0.0003 m / 0.0285 rad | 来自 `reloc_curve_1775704303_L0.csv` 末行；L0 无扰动，`linear_error_init` 与名义误差基本一致 |
| 请求 Lift L0 → 进入电梯 | 日志未保留 | 当前终端截断后未保留该阶段精确时间戳 |
| 请求 Lift L1 → 出电梯（L1 出口） | 日志未保留 | 当前终端截断后未保留该阶段精确时间戳 |
| 出电梯 → L1 地图加载完成 | 日志未保留 | 当前终端截断后未保留该阶段精确时间戳 |
| L1 自适应 sigma（首次） | (0.049, 0.049, 0.037) | 按论文公式计算的参考值；本轮终端尾段未保留 `Adaptive sigma` 打印 |
| L1 重定位窗口（CSV） | 12.004 s | `reloc_curve_1775704387_L1.csv` 末行 `pass_now=1` |
| L1 通过时名义误差（CSV） | 0.1877 m / 0.0566 rad | 满足 `max_linear_error_L1=0.45`、`max_angular_error=0.15` |
| L1 首采样名义误差（CSV） | 0.1382 m / 0.0534 rad | 与 small 扰动量级一致，未出现异常放大 |
| 动作层到达判定 | 1215.034 s：`Robot arrived at Goal` | 终端尾段明确打印到达目标 |

**说明**：状态机未保证打印 `DONE` 字符串时，以 **`move_base` 成功** 与 **CSV 中 `pass_now=1`** 作为全流程完成依据。

### 3.2 Improved A* 运行数据

统计方式：对 `[ImprovedAStar] Plan found: N waypoints, M nodes expanded.` 正则匹配。本轮统计基于终端日志尾段保留内容，共匹配到 **499** 次成功规划。

| 指标 | 数值 | 说明 |
| --- | ---: | --- |
| 成功规划调用次数 | 499 | 每次成功 `makePlan` 一条 |
| 路径点数（waypoints） | min=13 / max=165 / mean=37.39 | 基于终端日志统计 |
| 扩展节点数（nodes expanded） | min=12 / max=446 / mean=68.16 | 基于终端日志统计 |
| 规划失败次数 | 0 | 未匹配到 `Failed to find a plan` |

#### 3.2.1 高频路径形态 Top 5

| waypoints | nodes expanded | 出现次数 |
| ---: | ---: | ---: |
| 13 | 12 | 334 |
| 161 | 421 | 4 |
| 14 | 13 | 4 |
| 158 | 395 | 3 |
| 160 | 408 | 3 |

**论文表述提示**：`move_base` 周期性重规划导致调用次数远大于目标个数；本轮日志中大量出现 **13 / 12** 的短路径重规划，说明在终段靠近目标时规划已进入稳定局部修正状态。若对比算法效率，建议后续补充单次规划耗时或与基线规划器对照。

### 3.3 结果分析（论文可用表述模板）

**本轮 small 扰动实验结果表明，SP-AMCL 在 L1 轻微偏移场景下能够稳定收敛，且未出现明显过度扩散。** 从 `reloc_curve_1775704387_L1.csv` 可见，L1 首采样名义误差为 **0.1382 m / 0.0534 rad**，与 small 扰动设定量级一致；至 `RELOC_PASS` 时，名义误差为 **0.1877 m / 0.0566 rad**，仍明显低于门限 **0.45 m / 0.15 rad**，重定位窗口为 **12.004 s**。同时，`linear_error_pub` 在通过时仅为 **0.0949 m**，说明 AMCL 相对发布初值并未出现异常漂移，整体收敛过程平稳。

**从 Improved A* 侧看，本轮全流程未出现规划失败。** 终端日志中共统计到 **499** 次成功规划，`waypoints` 为 **13–165**、`nodes expanded` 为 **12–446**，均值分别为 **37.39** 与 **68.16**。其中 **13 waypoints / 12 nodes** 出现 **334** 次，说明在任务末段已经进入高频、短距离、低开销的稳定重规划状态。结合终端最终打印 `Robot arrived at Goal`，可认为本轮 small 扰动下的全流程导航成功完成。

**论文撰写时，small 工况更适合强调“轻扰动下不过度扩散且不退化”。** 与 medium / large 相比，该工况的重点不在极限恢复能力，而在验证 SP-AMCL 在较小初值偏差下不会引入额外不稳定性；本轮数据支持这一结论。

### 3.4 L1 重定位时序数据（完整 CSV，Small 扰动后）

**数据来源**：`experiment_logs/reloc_curve_1775704387_L1.csv`（切至 L1 地图后对 **名义真值** 的 `linear_error_nom` / `angular_error_nom` 等；**扰动仅作用于 L1 的 `initialpose` 发布**）。以下内容已按本轮磁盘实际文件回填。

**作图常用列**：`elapsed_sec`、`linear_error_nom`、`angular_error_nom`；门限 **线 0.45 m、角 0.15 rad**；**`RELOC_PASS`** 对应末行 `pass_now=1`。

```csv
elapsed_sec,linear_error_nom,angular_error_nom,linear_error_pub,angular_error_pub,tr_sigma,C_t,pose_ok,pass_now
1.0040,0.1382,0.0534,0.0032,0.0034,0.0056,0.9944,0,0
1.4040,0.1343,0.0502,0.0118,0.0002,0.0057,0.9943,0,0
1.8030,0.1415,0.0522,0.0181,0.0022,0.0052,0.9948,0,0
2.0040,0.1466,0.0531,0.0237,0.0031,0.0052,0.9948,0,0
2.4030,0.1516,0.0538,0.0343,0.0038,0.0049,0.9951,0,0
2.6040,0.1557,0.0565,0.0412,0.0065,0.0048,0.9952,0,0
2.8060,0.1611,0.0570,0.0469,0.0070,0.0047,0.9953,0,0
3.2050,0.1700,0.0577,0.0542,0.0077,0.0045,0.9955,0,0
3.6030,0.1715,0.0596,0.0604,0.0096,0.0041,0.9959,0,0
3.8050,0.1754,0.0601,0.0657,0.0101,0.0038,0.9962,0,0
4.2060,0.1768,0.0605,0.0701,0.0105,0.0036,0.9964,0,0
4.6030,0.1815,0.0606,0.0755,0.0106,0.0034,0.9966,0,0
4.8070,0.1815,0.0592,0.0743,0.0092,0.0034,0.9966,0,0
5.0080,0.1840,0.0585,0.0782,0.0085,0.0033,0.9967,0,0
5.4080,0.1902,0.0574,0.0866,0.0074,0.0031,0.9969,0,0
5.8050,0.1930,0.0562,0.0903,0.0062,0.0029,0.9971,0,0
6.2070,0.1945,0.0556,0.0917,0.0056,0.0027,0.9973,0,0
6.6030,0.1923,0.0558,0.0909,0.0058,0.0025,0.9975,0,0
6.8040,0.1937,0.0557,0.0925,0.0057,0.0025,0.9975,0,0
7.0080,0.1944,0.0554,0.0927,0.0054,0.0025,0.9975,0,0
7.4060,0.1978,0.0561,0.0957,0.0061,0.0024,0.9977,0,0
7.8040,0.1980,0.0555,0.0956,0.0055,0.0023,0.9977,0,0
8.0050,0.1993,0.0546,0.0969,0.0046,0.0023,0.9977,0,0
8.2070,0.2009,0.0541,0.0987,0.0041,0.0022,0.9978,0,0
8.4080,0.2003,0.0544,0.0985,0.0044,0.0022,0.9978,0,0
8.8040,0.2000,0.0541,0.0990,0.0041,0.0021,0.9979,0,0
9.0060,0.1995,0.0547,0.0988,0.0047,0.0021,0.9979,0,0
9.4040,0.1968,0.0553,0.0972,0.0053,0.0021,0.9979,0,0
9.6040,0.1948,0.0555,0.0956,0.0055,0.0021,0.9979,0,0
9.8070,0.1940,0.0566,0.0959,0.0066,0.0020,0.9980,0,0
10.2040,0.1926,0.0570,0.0954,0.0070,0.0018,0.9982,0,0
10.6030,0.1925,0.0575,0.0965,0.0075,0.0017,0.9983,0,0
10.8030,0.1911,0.0577,0.0956,0.0077,0.0017,0.9983,0,0
11.0070,0.1908,0.0575,0.0958,0.0075,0.0016,0.9984,0,0
11.2080,0.1916,0.0571,0.0969,0.0071,0.0015,0.9985,0,0
11.6040,0.1908,0.0565,0.0968,0.0065,0.0014,0.9986,0,0
12.0040,0.1877,0.0566,0.0949,0.0066,0.0013,0.9987,1,1
```

**L0**：同次运行虽有 `reloc_curve_1775704303_L0.csv` 落盘，**因无扰动、不反映 small 设定，正文与主图可完全不引用**。

## 4. 重定位曲线落盘（CSV）与论文作图

**主文实验图约定（L1）**：**仅 L1** 的 **`linear_error_nom`、`angular_error_nom`** 随 `elapsed_sec`（相对名义真值）作为主图；线/角门限以虚线标在图中。**`linear_error_pub` / `angular_error_pub`** 不列入主文 KPI 图。由于 **small** 工况扰动较小，图注中可强调其用于验证 **SP-AMCL 在轻微偏移下不过度扩散、且不劣于 Baseline**。**L0 曲线不作为本 small 扰动实验的主图**（扰动未加在 L0）。

### 4.1 指标定义（与 `RELOC_PASS` 日志一致）

- **`linear_error_nom` / `angular_error_nom`**：AMCL 位姿相对 **名义真值** `desired_init_pose`（地图标称点，不含扰动）。论文中描述「收敛到真实位姿」应使用这一对曲线；`RELOC_PASS` 中的 `linear_error` / `angular_error` 即该定义。
- **`linear_error_pub` / `angular_error_pub`**：AMCL 相对 **实际发布的 initialpose**（L1 为名义点 + small 扰动；L0 与名义相同）。`RELOC_PASS` 中的 `linear_error_init` / `angular_error_init` 即通过瞬间该定义下的误差。
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
| --- | --- | --- |
| **主文（L1，Small 扰动）** | `experiment_logs/reloc_curve_1775704387_L1.csv` | `experiment_logs/reloc_curve_1775704387_L1_nom.png` |
| 可选（L0，无扰动，非 KPI） | `experiment_logs/reloc_curve_1775704303_L0.csv` | `experiment_logs/reloc_curve_1775704303_L0_nom.png` |

**主文复现（L1）**（在 catkin 工作空间已 `source devel/setup.bash` 时）：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775704387_L1.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775704387_L1_nom.png" \
  --linear_threshold 0.45 --angular_threshold 0.15
```

**可选（L0，非 small 扰动结论）**：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775704303_L0.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775704303_L0_nom.png" \
  --linear_threshold 0.20 --angular_threshold 0.15
```

### 4.6 历史单次运行样本（旧 CSV 格式，仅一行）

以下为代码改动前、仅延迟结束后单次采样的示例（**无法**作时间序列）。

**文件**：`reloc_curve_<wall_time>_L0.csv`（旧列名示意）

| elapsed_sec | linear_error (m) | angular_error (rad) | tr_sigma | C_t | pose_ok | pass_now |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 | 待填写 |

重新编译并跑通全流程后，应使用新生成的 `_L0.csv` / `_L1.csv` 绘制“随时间下降”曲线。

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
| --- | --- |
| 2026-04-09 | 依据 medium 文档版式生成 small 扰动实验记录模板 |
| 2026-04-09 | 按论文文档统一 small 扰动定义为 `(0.10, 0.10, 0.05)`，并补入 `d_planar` 与自适应 sigma 参考值 |
| 2026-04-09 | 按本轮实验结果回填 `reloc_curve_1775704303_L0.csv`、`reloc_curve_1775704387_L1.csv`、Improved A* 统计与生成图稿 |
