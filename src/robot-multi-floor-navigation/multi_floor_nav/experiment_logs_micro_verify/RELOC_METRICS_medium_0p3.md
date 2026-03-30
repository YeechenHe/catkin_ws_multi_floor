# 重定位指标汇总 — 扰动 medium (0.3 m, 0.3 m, 0.15 rad)

数据来源：`experiment_logs_micro_verify/`，**baseline / relocc 各 8 次**（run1–8）。  
初始协方差：`sigma_xy=0.15 m`，`sigma_yaw=0.05 rad`（launch 传入）。  
**说明**：`false_accept` 列为「两层重定位均通过但导航任务未成功」，若论文只写重定位可忽略；**`tr(Σ)` 与 `e_pass`、通过后稳定性**与任务无关。

---

## 1. 总体对比（n=8）

| 指标 | Baseline | Relocc（模块 C） |
|------|----------|------------------|
| `reloc_ok = 2`（L0+L1 均判通过） | 7/8 | 4/8 |
| `retries` 合计 | 37 | 9 |
| `dt_L0` (s) mean±std | 3.125 ± 0.057 | **4.123 ± 0.238** |
| `dt_L1` (s) mean±std | 3.119 ± 0.053 | **3.419 ± 0.240** |

---

## 2. 通过时刻误差 `e_pass`（mean±std，全 8 次有日志则参与平均）

| 层 | Baseline | Relocc |
|----|----------|--------|
| L0 linear (m) | 0.269 ± 0.009 | 0.236 ± 0.015 |
| L0 yaw (rad) | 0.129 ± 0.009 | 0.118 ± 0.011 |
| L1 linear (m) | 0.406 ± 0.072 | 0.395 ± 0.007 |
| L1 yaw (rad) | 0.095 ± 0.045 | 0.050 ± 0.010 |

---

## 3. 通过时刻置信度 `tr(Σ)` 与 `C_t`

日志中的 `C_t` 按 `exp(-λ·tr(Σ))` 计算：**Baseline 未加载模块 C 时 `λ` 默认为 1**；**Relocc 使用 `λ=20`**，因此日志中两列 `C_t` **不宜直接横向对比**。

**可比量**：同一时刻 AMCL 发布的 **`tr(Σ)`**（对 `x,y,yaw` 方差求和）。

| 统计范围 | 方法 | L0 `tr(Σ)` mean | L1 `tr(Σ)` mean |
|----------|------|-----------------|-----------------|
| 全部 8 次（有 `RELOC_PASS` 则计入） | Baseline | 0.0212 | 0.0187 |
| 全部 8 次 | Relocc | 0.0122 | 0.0093 |
| **仅 `reloc_ok ≥ 2`（n=7 / n=4）** | Baseline | 0.0213 | 0.0187 |
| **仅 `reloc_ok ≥ 2`** | Relocc | 0.0129 | 0.0093 |

**统一尺度下的置信度**（论文作图建议用同一 `λ`，例如 `λ=20`）：  
\(C_t^{*} = \exp(-20 \cdot \mathrm{tr}(\Sigma))\)

| 统计范围 | 方法 | L0 \(C_t^{*}\) | L1 \(C_t^{*}\) |
|----------|------|----------------|----------------|
| `reloc_ok ≥ 2` 子集 | Baseline | ≈ 0.653 | ≈ 0.688 |
| `reloc_ok ≥ 2` 子集 | Relocc | ≈ 0.773 | ≈ 0.830 |

（由上表 `tr(Σ)` 近似计算：`exp(-20×0.0213)`、`exp(-20×0.0187)` 等。）

---

## 4. 通过后 2 s 窗口稳定性（`POST_PASS`）

| 统计 | 层 | Baseline mean±std | Relocc mean±std |
|------|----|-------------------|-----------------|
| `mean_linear_after` (m) | L0 | 0.363 ± 0.018 | 0.330 ± 0.014 |
| `max_linear_after` (m) | L0 | ~0.510 | ~0.498 |
| `mean_linear_after` (m) | L1 | 0.312 ± 0.072 | 0.289 ± 0.013 |
| `max_linear_after` (m) | L1 | ~0.408 | ~0.396 |

---

## 5. 逐次明细（CSV）

完整列见同目录 `results_reloc_metrics_full.txt`（首行为 CSV 表头，含 `e_tr_sigma_L0/L1`、`e_C_t_L0/L1` 及 `after_pass` 等）。

---

## 6. 论文作图建议（仅重定位）

1. **柱状图**：`reloc_ok=2` 比例、或 `retries`（体现判据严/松）。  
2. **箱线图**：`tr(Σ)` @ RELOC_PASS（L0/L1 各一张），或统一 `C_t^{*}=exp(-20·tr)`。  
3. **柱状或箱线图**：`mean_linear_after` / `max_linear_after`（L0/L1）。  
4. **柱状图**：`dt_L0`、`dt_L1`（代价）。  
5. **`e_pass`**：L0/L1 的 linear / yaw 对比。

---

*Generated from `scripts/amcl/parse_experiment_logs.py` output on this directory.*
