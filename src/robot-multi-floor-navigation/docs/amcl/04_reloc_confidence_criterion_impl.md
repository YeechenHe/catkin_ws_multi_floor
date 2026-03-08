# 模块 C：重定位完成判据优化 — 实现说明

## 1. 目的与动机

**Baseline 的缺陷**：单帧线性误差判据——某一帧满足阈值即放行导航，在 AMCL **尚未稳定或存在抖动**时会产生**误通过**：偶然一帧满足条件后估计又漂移，导航已在错误/波动位姿上规划，导致任务失败或重试。

**模块 C 的设计意义**：在**易出现单帧满足阈值但不稳定**的场景（如初值误差大、环境对称、粒子少或观测噪声大）下，通过**连续 K 帧 + 置信度 C_t > τ** 判据，只在估计稳定后才放行，**减少误通过、减少过早导航带来的任务失败**，从而在该类场景下**优于 Baseline**（更高任务成功率或更少“通过重定位却任务失败”）。对应方案文档 `00_FA_AMCL_多楼层定位改进方案.md` 第 7 节。

## 2. 判据公式与逻辑

- **置信度**：\( C_t = \exp(-\lambda \cdot \mathrm{tr}(\Sigma_t)) \)，其中 \( \Sigma_t \) 为 AMCL 当前位姿估计的协方差矩阵，\( \mathrm{tr}(\Sigma_t) \) 取 x、y、yaw 三个自由度的方差之和（即 `amcl_pose.pose.covariance[0] + [7] + [35]`）。
- **通过条件**（仅在启用模块 C 时）：
  - 线性误差仍满足 `linear_error < max_linear_error`（与原有判据一致）；
  - \( C_t > \tau \)；
  - 上述两条件**连续 K 帧**均满足，才输出“Robot at correct pose”并进入下一状态。
- **未启用模块 C**：保持原有行为，即单帧 `linear_error < max_linear_error` 即通过。

## 3. 实现要点

- **数据来源**：`/amcl_pose`（`geometry_msgs::PoseWithCovarianceStamped`）中的 `pose.covariance`（6×6，行优先），取索引 0、7、35 分别对应 x、y、yaw 的方差。
- **连续帧计数**：在 `CHECK_INITPOSE` 状态下每次 `check_robot_pose()` 若本帧满足（线性误差 + C_t > tau），则 `reloc_confidence_consecutive_count_++`；否则置 0。仅当 `reloc_confidence_consecutive_count_ >= K` 时返回 true。进入 `CHECK_INITPOSE` 时（从 `INIT_POSE` 转入）将计数清零。**重要**：未通过时**保持在 CHECK_INITPOSE**（不立即退回 INIT_POSE），以便下一控制周期继续检查、累积连续 K 帧；仅当超过 `reloc_check_timeout_sec`（默认 45 s）仍未通过时才退回 INIT_POSE 重试。
- **与模块 A/B 关系**：模块 C 仅改变“何时判定重定位完成”，与区域初始化（A）、切层窗口参数（B）独立，可任意组合使用。

## 4. 代码与配置

| 内容 | 路径 |
|------|------|
| 判据逻辑与计数 | `multi_floor_nav/src/multi_floor_navigation.cpp`（`check_robot_pose`、`INIT_POSE` 分支中重置计数） |
| 头文件成员与参数 | `multi_floor_nav/include/multi_floor_nav/multi_floor_navigation.h`（`use_covariance_reloc_`、`reloc_confidence_*`） |
| 参数默认与 yaml | `multi_floor_nav/configs/reloc_confidence_criterion.yaml` |
| Launch 开关 | `multi_floor_nav/launch/nav.launch`：`use_covariance_reloc`，为 true 时加载上述 yaml |

## 5. 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_covariance_reloc` | false | 为 true 时启用模块 C（由 launch 的 use_covariance_reloc 通过 yaml 传入） |
| `reloc_confidence_lambda` | 1.0 | 置信度公式中的 \( \lambda \)，越大对协方差越敏感 |
| `reloc_confidence_tau` | 0.5 | 置信度阈值，\( C_t > \tau \) 才计为一帧通过（放宽以便判据可满足） |
| `reloc_confidence_K` | 2 | 连续 K 帧满足才判定重定位完成 |
| `reloc_check_timeout_sec` | 45 | 在 CHECK_INITPOSE 内超时未通过则退回 INIT_POSE 重试 |

**调参建议**：若 AMCL 收敛后 tr(Σ) 较小（如 &lt; 0.5），则 C_t 接近 1，易满足 tau=0.8；若 tr(Σ) 较大，可适当降低 `reloc_confidence_tau` 或 `reloc_confidence_lambda`。K 过大会延长判定时间，过小则抗抖动能力弱，建议 2～5。

## 6. 使用方式

- **关闭模块 C**（默认）：`roslaunch multi_floor_nav nav.launch` 或显式 `use_covariance_reloc:=false`
- **开启模块 C**：`roslaunch multi_floor_nav nav.launch use_covariance_reloc:=true`

可与模块 A/B 组合，例如：  
`roslaunch multi_floor_nav nav.launch use_ap_amcl:=true use_covariance_reloc:=true`

## 7. 论文中的定位

该模块适合作为**系统完善项或辅助改进**，不建议单独作为核心创新点；实现后可提升实验稳定性和可复现性，在方法部分可简要描述“采用基于协方差的置信度与连续 K 帧判据以确认重定位完成”。

## 8. 模块 C 对比实验与论文数据

- **对比设计**：Baseline 判据（单帧线性误差）vs 模块 C（C_t > τ 连续 K 帧 + 线性误差），同一 AMCL、无模块 A/B，pert=0 与 small 各 2 次 run。
- **运行**：`./scripts/amcl/run_one_experiment.sh baseline 0 0 0 1 200 multi_floor_nav/experiment_logs_module_c` 与 `relocc` 同理；解析：`python3 scripts/amcl/parse_experiment_logs.py multi_floor_nav/experiment_logs_module_c`。
- **论文用表与表述**：见 `docs/amcl/paper_baseline_vs_reloc_criterion.md`，解析后将 CSV 保存为 `docs/amcl/reloc_criterion_experiment_data.csv` 并填入该文档 §3、§4。
