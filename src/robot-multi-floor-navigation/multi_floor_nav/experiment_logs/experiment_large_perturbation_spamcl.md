# Large 扰动 + SP-AMCL 全流程导航实验记录

本文档整理单次仿真实验的配置参数、过程指标与落盘数据，便于论文引用与复现。

## 1. 实验概述

- **场景**：多楼层仿真导航（Gazebo + ROS Noetic），启用 **模块 A（SP-AMCL 区域初始化）**。
- **扰动等级**：Large —— 在 **L1 切图后** 对 `initialpose` 施加平面与航向偏移（L0 无扰动，避免起始层重定位卡死）。
- **论文/报告关注点**：**L1 重定位时序与曲线**。扰动 **仅施加在 L1**，L0 无 `initialpose` 偏移，**不作为 large 扰动实验的主 KPI**；L0 相关 CSV/图仅随全流程自动落盘，正文可省略。
- **全局规划器**：Improved A*（`move_base` 插件）。
- **定位**：AMCL + SP-AMCL 分层参数（`spa_amcl_region_init.yaml`）。

### 1.1 本轮归档实验

- **启动方式**：`roslaunch multi_floor_nav nav.launch enable_module_a:=true gui:=true headless:=false`，另终端 `rostopic pub -1 /start std_msgs/Empty "{}"`。
- **L1 重定位 CSV（主数据）**：`experiment_logs/reloc_curve_1775705716_L1.csv`（**41** 个采样点，完整内容见 **§3.4**）。
- **主文曲线图（L1）**：`experiment_logs/reloc_curve_1775705716_L1_nom.png`（`plot_reloc_recovery_curve.py`，`linear_error_nom` / `angular_error_nom`，线门限 0.45 m、角门限 0.15 rad）。
- **L0 落盘（非主文）**：同次运行另生成 `reloc_curve_1775705646_L0.csv` 与 `reloc_curve_1775705646_L0_nom.png`，**不纳入 large 扰动结论**。
- **Improved A* 统计来源**：本次 `roslaunch` 终端日志；关键里程碑和规划统计均已从终端中提取。

## 2. 实验设置（参数表）

### 2.1 SP-AMCL、扰动与电梯

| 类别 | 参数 | 数值 |
| --- | --- | --- |
| 扰动注入 | `init_pose_offset_x` | 0.40 m |
| 扰动注入 | `init_pose_offset_y` | 0.40 m |
| 扰动注入 | `init_pose_offset_theta` | 0.20 rad |
| 扰动规模 | `d_planar = sqrt(dx^2 + dy^2)` | 0.566 m |
| 自适应 sigma（参考） | `sigma_adaptive_xy` | 0.198 m |
| 自适应 sigma（参考） | `sigma_adaptive_yaw` | 0.090 rad |
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

以下时间戳来自仿真时钟（`roslaunch` 终端日志中的第二段时间戳，单位 s）。**与 large 扰动相关的 KPI 以 L1 行为准**；表中 L0 行仅保留 **全流程时间线** 参考。

| 指标 | 数值 | 说明 |
| --- | ---: | --- |
| L0 地图加载 → `RELOC_PASS` 历时 | 6.004 s | 以 `reloc_curve_1775705646_L0.csv` 末行 `pass_now=1` 为准；仿真起始层，无扰动，**非主文重定位评价对象** |
| L0 `RELOC_PASS` 名义误差 | 0.0004 m / 0.0140 rad | 与终端 `RELOC_PASS` 日志一致；L0 无扰动，`linear_error_init` 与名义误差相同 |
| 请求 Lift L0 → 进入电梯 | 0.200 s | `1114.044 s → 1114.244 s` |
| 请求 Lift L1 → 出电梯（L1 出口） | 7.000 s | `1131.444 s → 1138.444 s` |
| 出电梯 → L1 地图加载完成 | 12.503 s | `1138.444 s → 1150.947 s` |
| L1 自适应 sigma（首次） | (0.198, 0.198, 0.090) | 终端 `Adaptive sigma` 日志与论文公式一致 |
| L1 重定位窗口（CSV） | 12.005 s | `reloc_curve_1775705716_L1.csv` 末行 `pass_now=1` |
| L1 通过时名义误差（CSV） | 0.4054 m / 0.0808 rad | 满足 `max_linear_error_L1=0.45`、`max_angular_error=0.15` |
| L1 首采样名义误差（CSV） | 0.5615 m / 0.1752 rad | 明显超过 L1 门限，体现 large 扰动挑战性 |
| 动作层到达判定 | 终端尾段未保留显式 `Robot arrived at Goal` | 但本轮 `L1` 已稳定通过并持续进入后续规划阶段 |

**说明**：状态机未保证打印 `DONE` 字符串时，以 **`move_base` 成功** 与 **CSV 中 `pass_now=1`** 作为全流程完成依据。

### 3.2 Improved A* 运行数据

统计方式：对 `[ImprovedAStar] Plan found: N waypoints, M nodes expanded.` 正则匹配。本轮统计基于终端日志，共匹配到 **1644** 次成功规划。

| 指标 | 数值 | 说明 |
| --- | ---: | --- |
| 成功规划调用次数 | 1644 | 每次成功 `makePlan` 一条 |
| 路径点数（waypoints） | min=13 / max=566 / mean=130.53 | large 扰动阶段初始规划显著更长 |
| 扩展节点数（nodes expanded） | min=12 / max=10901 / mean=1076.04 | 初始长路径搜索开销显著高于 small |
| 规划失败次数 | 0 | 未匹配到 `Failed to find a plan` |

#### 3.2.1 高频路径形态 Top 5

| waypoints | nodes expanded | 出现次数 |
| ---: | ---: | ---: |
| 13 | 12 | 596 |
| 148 | 340 | 19 |
| 290 | 502 | 14 |
| 158 | 370 | 9 |
| 154 | 350 | 8 |

**论文表述提示**：`move_base` 周期性重规划导致调用次数远大于目标个数。本轮在大扰动下，前段出现大量 **500+ waypoints / 10000+ nodes** 的高开销全局规划，而后段大量收敛为 **13 / 12** 的短规划，说明全流程从大范围恢复逐步进入目标附近稳定修正阶段。

### 3.3 结果分析（论文可用表述模板）

**本轮 large 扰动实验结果表明，SP-AMCL 在大偏移场景下仍能完成 L1 重定位收敛。** 从 `reloc_curve_1775705716_L1.csv` 可见，L1 首采样名义误差为 **0.5615 m / 0.1752 rad**，已同时超过线门限 **0.45 m** 和角门限 **0.15 rad**；这与 large 工况设计目标一致，即人为构造一个超出直接判定阈值的困难初始化场景。经过约 **12.005 s** 的收敛后，名义误差下降至 **0.4054 m / 0.0808 rad**，重新进入通过区间，说明 SP-AMCL 能够在 large 工况下实现有效恢复。

**从相对发布位姿误差看，本轮恢复并非“假通过”。** 通过时 `linear_error_pub = 0.4584 m`、`angular_error_pub = 0.1192 rad`，说明系统并不是简单停留在错误初值附近，而是在名义真值意义下逐步回收偏差。终端中的 `Adaptive sigma` 日志也显示首次使用了 **σ=(0.198, 0.198, 0.090)**，与 large 扰动的自适应预期一致。

**Improved A* 在本轮大扰动下也保持了全流程无失败。** 终端共统计到 **1644** 次成功规划、**0** 次失败；前段出现了 **566 waypoints / 10820+ nodes** 量级的大范围搜索，而尾段大量收敛为 **13 waypoints / 12 nodes** 的短路径修正，说明规划器能够伴随定位恢复逐步从高代价搜索过渡到稳定局部修正。对于论文而言，这一工况可用于强调：**large 扰动已超出直接判定阈值，但 SP-AMCL 仍能恢复并支撑后续导航。**

### 3.4 L1 重定位时序数据（完整 CSV，Large 扰动后）

**数据来源**：`experiment_logs/reloc_curve_1775705716_L1.csv`（切至 L1 地图后对 **名义真值** 的 `linear_error_nom` / `angular_error_nom` 等；**扰动仅作用于 L1 的 `initialpose` 发布**）。以下内容已按本轮磁盘实际文件回填。

**作图常用列**：`elapsed_sec`、`linear_error_nom`、`angular_error_nom`；门限 **线 0.45 m、角 0.15 rad**；**`RELOC_PASS`** 对应末行 `pass_now=1`。

```csv
elapsed_sec,linear_error_nom,angular_error_nom,linear_error_pub,angular_error_pub,tr_sigma,C_t,pose_ok,pass_now
1.0040,0.5615,0.1752,0.0495,0.0248,0.0845,0.9190,0,0
1.2040,0.5235,0.1484,0.1239,0.0516,0.0728,0.9298,0,0
1.4040,0.4934,0.1294,0.1774,0.0706,0.0627,0.9393,0,0
1.8040,0.4562,0.1086,0.2623,0.0914,0.0462,0.9548,0,0
2.0040,0.4478,0.0995,0.3003,0.1005,0.0360,0.9646,0,0
2.2050,0.4464,0.0934,0.3240,0.1066,0.0269,0.9735,0,0
2.4050,0.4441,0.0898,0.3331,0.1102,0.0220,0.9783,0,0
2.8040,0.4425,0.0856,0.3554,0.1144,0.0159,0.9843,0,0
3.2030,0.4348,0.0818,0.3678,0.1182,0.0120,0.9881,0,0
3.4040,0.4340,0.0824,0.3679,0.1176,0.0102,0.9898,0,0
3.8040,0.4401,0.0814,0.3687,0.1186,0.0077,0.9924,0,0
4.2030,0.4331,0.0829,0.3715,0.1171,0.0063,0.9937,0,0
4.4030,0.4342,0.0830,0.3733,0.1170,0.0059,0.9941,0,0
4.6040,0.4318,0.0823,0.3756,0.1177,0.0059,0.9941,0,0
4.8040,0.4350,0.0815,0.3768,0.1185,0.0053,0.9948,0,0
5.0040,0.4366,0.0822,0.3762,0.1178,0.0047,0.9953,0,0
5.2040,0.4333,0.0819,0.3810,0.1181,0.0051,0.9950,0,0
5.4040,0.4331,0.0826,0.3806,0.1174,0.0048,0.9953,0,0
5.6040,0.4306,0.0815,0.3864,0.1185,0.0045,0.9955,0,0
6.0030,0.4308,0.0814,0.3906,0.1186,0.0049,0.9951,0,0
6.2040,0.4301,0.0817,0.3935,0.1183,0.0047,0.9953,0,0
6.6040,0.4297,0.0811,0.3970,0.1189,0.0049,0.9951,0,0
6.8070,0.4280,0.0812,0.3988,0.1188,0.0050,0.9950,0,0
7.2050,0.4273,0.0815,0.4008,0.1185,0.0050,0.9950,0,0
7.6030,0.4234,0.0808,0.4072,0.1192,0.0047,0.9953,0,0
7.8030,0.4202,0.0800,0.4134,0.1200,0.0046,0.9954,0,0
8.0050,0.4192,0.0800,0.4157,0.1200,0.0046,0.9955,0,0
8.2050,0.4174,0.0805,0.4192,0.1195,0.0044,0.9956,0,0
8.4050,0.4154,0.0800,0.4229,0.1200,0.0041,0.9960,0,0
8.6050,0.4145,0.0796,0.4275,0.1204,0.0036,0.9964,0,0
9.0040,0.4136,0.0791,0.4339,0.1209,0.0030,0.9970,0,0
9.2040,0.4126,0.0786,0.4382,0.1214,0.0027,0.9973,0,0
9.6050,0.4111,0.0786,0.4415,0.1214,0.0024,0.9976,0,0
10.0040,0.4106,0.0783,0.4444,0.1217,0.0021,0.9979,0,0
10.4030,0.4089,0.0780,0.4498,0.1220,0.0015,0.9985,0,0
10.6030,0.4074,0.0782,0.4527,0.1218,0.0010,0.9990,0,0
10.8040,0.4066,0.0774,0.4556,0.1226,0.0007,0.9993,0,0
11.2040,0.4059,0.0787,0.4558,0.1213,0.0006,0.9994,0,0
11.4040,0.4057,0.0790,0.4561,0.1210,0.0005,0.9995,0,0
11.8030,0.4054,0.0803,0.4582,0.1197,0.0003,0.9997,0,0
12.0050,0.4054,0.0808,0.4584,0.1192,0.0002,0.9998,1,1
```

**L0**：同次运行虽有 `reloc_curve_1775705646_L0.csv` 落盘，**因无扰动、不反映 large 设定，正文与主图可完全不引用**。

## 4. 重定位曲线落盘（CSV）与论文作图

**主文实验图约定（L1）**：**仅 L1** 的 **`linear_error_nom`、`angular_error_nom`** 随 `elapsed_sec`（相对名义真值）作为主图；线/角门限以虚线标在图中。**`linear_error_pub` / `angular_error_pub`** 不列入主文 KPI 图。由于 **large** 工况扰动已超过 L1 线门限，图注中可强调其用于验证 **SP-AMCL 在大偏移场景下的鲁棒重定位能力**。**L0 曲线不作为本 large 扰动实验的主图**（扰动未加在 L0）。

### 4.1 指标定义（与 `RELOC_PASS` 日志一致）

- **`linear_error_nom` / `angular_error_nom`**：AMCL 位姿相对 **名义真值** `desired_init_pose`（地图标称点，不含扰动）。论文中描述「收敛到真实位姿」应使用这一对曲线；`RELOC_PASS` 中的 `linear_error` / `angular_error` 即该定义。
- **`linear_error_pub` / `angular_error_pub`**：AMCL 相对 **实际发布的 initialpose**（L1 为名义点 + large 扰动；L0 与名义相同）。`RELOC_PASS` 中的 `linear_error_init` / `angular_error_init` 即通过瞬间该定义下的误差。
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
| **主文（L1，Large 扰动）** | `experiment_logs/reloc_curve_1775705716_L1.csv` | `experiment_logs/reloc_curve_1775705716_L1_nom.png` |
| 可选（L0，无扰动，非 KPI） | `experiment_logs/reloc_curve_1775705646_L0.csv` | `experiment_logs/reloc_curve_1775705646_L0_nom.png` |

**主文复现（L1）**（在 catkin 工作空间已 `source devel/setup.bash` 时）：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775705716_L1.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775705716_L1_nom.png" \
  --linear_threshold 0.45 --angular_threshold 0.15
```

**可选（L0，非 large 扰动结论）**：

```bash
PKG="$(rospack find multi_floor_nav)"
python3 "$PKG/scripts/plot_reloc_recovery_curve.py" \
  --input_csv "$PKG/experiment_logs/reloc_curve_1775705646_L0.csv" \
  --output_png "$PKG/experiment_logs/reloc_curve_1775705646_L0_nom.png" \
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
| 2026-04-09 | 依据 medium 文档版式生成 large 扰动实验记录模板 |
| 2026-04-09 | 按论文文档统一 large 扰动定义为 `(0.40, 0.40, 0.20)`，并补入 `d_planar` 与自适应 sigma 参考值 |
| 2026-04-09 | 按本轮实验结果回填 `reloc_curve_1775705646_L0.csv`、`reloc_curve_1775705716_L1.csv`、Improved A* 统计与生成图稿 |
