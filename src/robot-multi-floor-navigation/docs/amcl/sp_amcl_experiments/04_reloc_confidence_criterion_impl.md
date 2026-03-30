# 模块 C：重定位完成判据优化 — 实现说明

## 1. 目的与动机

**Baseline 的缺陷**：单帧线性误差判据——某一帧满足阈值即放行导航，在 AMCL **尚未稳定或存在抖动**时会产生**误通过**：偶然一帧满足条件后估计又漂移，导航已在错误/波动位姿上规划，导致任务失败或重试。

**模块 C 的设计意义**：在**易出现单帧满足阈值但不稳定**的场景（如初值误差大、环境对称、粒子少或观测噪声大）下，通过**连续 K 帧 + 置信度 C_t > τ** 判据，只在估计稳定后才放行，**减少误通过、减少过早导航带来的任务失败**，从而在该类场景下**优于 Baseline**。

## 2. 判据公式与逻辑

- **置信度**：\( C_t = \exp(-\lambda \cdot \mathrm{tr}(\Sigma_t)) \)，其中 \( \Sigma_t \) 为 AMCL 当前位姿估计的协方差矩阵，\( \mathrm{tr}(\Sigma_t) \) 取 x、y、yaw 三个自由度的方差之和（即 `amcl_pose.pose.covariance[0] + [7] + [35]`）。
- **通过条件**（仅在启用模块 C 时）：
  - 线性误差仍满足 `linear_error < max_linear_error`（与原有判据一致）；
  - \( C_t > \tau \)；
  - 上述两条件**连续 K 帧**均满足，才输出"Robot at correct pose"并进入下一状态。
- **未启用模块 C**：保持原有行为，即单帧 `linear_error < max_linear_error` 即通过。

## 3. 实现要点

- **数据来源**：`/amcl_pose`（`geometry_msgs::PoseWithCovarianceStamped`）中的 `pose.covariance`（6×6，行优先），取索引 0、7、35 分别对应 x、y、yaw 的方差。
- **连续帧计数**：在 `CHECK_INITPOSE` 状态下每次 `check_robot_pose()` 若本帧满足（线性误差 + C_t > tau），则 `reloc_confidence_consecutive_count_++`；否则置 0。仅当 `reloc_confidence_consecutive_count_ >= K` 时返回 true。进入 `CHECK_INITPOSE` 时（从 `INIT_POSE` 转入）将计数清零。**重要**：未通过时**保持在 CHECK_INITPOSE**（不立即退回 INIT_POSE），以便下一控制周期继续检查、累积连续 K 帧；仅当超过 `reloc_check_timeout_sec`（默认 45 s）仍未通过时才退回 INIT_POSE 重试。
- **与模块 A/B 关系**：模块 C 仅改变"何时判定重定位完成"，与区域初始化（A）、切层窗口参数（B）独立，可任意组合使用。

## 4. 代码与配置

| 内容 | 路径 |
|------|------|
| 判据逻辑与计数 | `multi_floor_nav/src/multi_floor_navigation.cpp`（`check_robot_pose`、`INIT_POSE` 分支中重置计数） |
| 头文件成员与参数 | `multi_floor_nav/include/multi_floor_nav/multi_floor_navigation.h`（`use_covariance_reloc_`、`reloc_confidence_*`） |
| 参数默认与 yaml | `multi_floor_nav/configs/reloc_confidence_criterion.yaml` |
| Launch 开关 | `multi_floor_nav/launch/nav.launch`：`use_covariance_reloc`，为 true 时加载上述 yaml |

## 5. 参数说明

### 5.1 当前正式参数（经联合实验充分调优）

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `use_covariance_reloc` | true（实验时） | 为 true 时启用模块 C |
| `reloc_confidence_lambda` | **20.0** | 置信度公式中的 \( \lambda \)，越大对协方差越敏感 |
| `reloc_confidence_tau` | **0.85** | 置信度阈值，\( C_t > \tau \) 才计为一帧通过 |
| `reloc_confidence_K` | **3** | 连续 K 帧满足才判定重定位完成 |
| `reloc_min_check_delay_sec` | **6.0** | nomotion 最少等待时间（Baseline 和模块 C 一致） |
| `reloc_check_timeout_sec` | 45 | 全局 fallback timeout |
| `reloc_check_timeout_L0` | **25** | L0 层 timeout（缩短以增加 retry 机会） |
| `reloc_check_timeout_L1` | **45** | L1 层 timeout（跨层重定位保持充裕等待） |

### 5.2 参数调优历史

| 版本 | λ | τ | K | 问题 |
|------|---|---|---|------|
| v1（早期探索） | 1.0 | 0.5 | 2 | 约束过弱，tr(Σ)=0.1 时 C_t=0.905 远超 τ=0.5，几乎无过滤效果 |
| v2（联合调优后） | **20.0** | **0.85** | **3** | tr(Σ)=0.1 时 C_t=0.135，必须高度收敛才能通过；连续 3 帧保证时序稳定性 |
| v2.1（large优化） | 20.0 | 0.85 | **2**（large）/ 3（small/med） | large 工况下 tr(Σ) 抖动大，K=3 过严；per-floor timeout L0=25s/L1=45s |

### 5.3 λ=20.0 的物理含义

| tr(Σ) | C_t（λ=1.0 旧） | C_t（λ=20.0 新） | τ=0.85 判定 |
|--------|-----------------|------------------|-------------|
| 0.005 | 0.995 | 0.905 | **通过** |
| 0.008 | 0.992 | 0.852 | **通过**（刚过阈值） |
| 0.01 | 0.990 | 0.819 | **拦截** |
| 0.03 | 0.970 | 0.549 | **拦截** |
| 0.10 | 0.905 | 0.135 | **拦截** |

新参数在 tr(Σ)≈0.008 附近形成判定边界，确保只有粒子群高度聚拢时才宣告通过。

### 5.4 调参建议

- λ 控制灵敏度：增大 λ 使判据更严格（需更小的 tr(Σ)），减小则更宽松
- τ 控制通过门槛：0.85 要求 C_t 很高，适合需要高可靠性的场景
- K 控制时序稳定性：K=3 在可接受的时延增加下提供较强的抗抖动能力
- 若 AMCL 粒子数较少或环境噪声大，可适当降低 τ（如 0.80）或 K（如 2）

## 6. 使用方式

- **单独使用模块 C**：`roslaunch multi_floor_nav nav.launch use_covariance_reloc:=true`
- **与模块 A 联合使用**：`roslaunch multi_floor_nav nav.launch use_region_init:=true use_covariance_reloc:=true`
- **关闭模块 C（Baseline）**：`roslaunch multi_floor_nav nav.launch`（默认 `use_covariance_reloc:=false`）

实验脚本中对应的方法名：
- `relocc`：仅开启模块 C
- `sp_amcl_c`：模块 A + 模块 C 联合

## 7. 论文中的定位

模块 C 对应论文章节 3.3.3——"基于协方差迹与多帧时序检验的重定位判据"。

- 与模块 A（3.3.2 初始化优化）互补：模块 A 解决"粒子一开始如何分布"，模块 C 解决"何时才能认为真正收敛"
- 可单独评估有效性（Baseline vs 模块 C），也可与模块 A 联合评估（Baseline vs 模块 A+C）

## 8. 模块 C 对比实验

### 8.1 正式实验（v2，修正参数后）

- **参数**：λ=20.0, τ=0.85, K=3, reloc_min_check_delay=6.0s
- **对比**：Baseline（use_covariance_reloc:=false）vs 模块 C（use_covariance_reloc:=true），均不启用模块 A/B
- **工况**：small(0.1,0.1,0.05) / medium(0.25,0.25,0.15) / large(0.4,0.4,0.2)，各 n=5
- **脚本**：`scripts/amcl/run_batch_module_c.sh`
- **日志目录**：`multi_floor_nav/experiment_logs_module_c_v2/`
- **数据报告**：`docs/amcl/sp_amcl_experiments/paper_baseline_vs_reloc_criterion.md` §7

### 8.2 早期探索实验（v1，历史参考）

- **参数**：λ=1.0, τ=0.5, K=2（参数过弱，约束几乎无效）
- **工况**：none/small，各 n=2
- **结论**：任务成功率与 Baseline 相当（参数过弱导致无区分度）
- **数据报告**：`docs/amcl/sp_amcl_experiments/paper_baseline_vs_reloc_criterion.md` §1–§6
