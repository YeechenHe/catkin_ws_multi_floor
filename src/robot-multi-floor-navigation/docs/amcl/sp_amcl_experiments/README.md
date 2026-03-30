# SP-AMCL 改进实验文档集

本目录集中存放**模块A（自适应区域初始化）、模块C（置信度重定位判据）及两者联合改进**的所有实验设计、数据与分析文档，便于论文写作时集中查阅。

---

## 推荐阅读顺序

```
算法实现说明
  ├── 02_region_init_impl.md          模块A实现原理
  └── 04_reloc_confidence_criterion_impl.md  模块C实现原理

实验设计方案
  ├── 03_region_init_experiment_plan.md      模块A实验方案（含扰动设计、指标定义）
  └── 3_3_3_reloc_criterion_experiment_plan_v3.md  模块C实验方案

实验数据与分析（论文用）
  ├── paper_sp_amcl_adaptive_sigma.md        ★ 模块A单独实验报告（n=5批量，核心文档）
  ├── paper_sp_amcl_plus_c.md                ★ 模块A+C联合改进实验报告（核心文档）
  ├── paper_baseline_vs_region_init.md       模块A早期实验数据（历史存档）
  ├── paper_baseline_vs_reloc_criterion.md   模块C单独实验报告
  └── region_init_3run_summary.md            模块A三次小结（早期）

原始数据（CSV）
  ├── baseline_vs_region_init_data.csv
  ├── reloc_criterion_experiment_data.csv
  └── reloc_criterion_validity_data.csv
```

---

## 核心文档速查

### 模块A：自适应区域初始化

| 文档 | 内容 |
|------|------|
| [`02_region_init_impl.md`](02_region_init_impl.md) | 实现说明：区域高斯初始化、per-floor sigma、retry 退避 |
| [`03_region_init_experiment_plan.md`](03_region_init_experiment_plan.md) | 实验方案：扰动工况设计、评价指标、执行指令 |
| [`paper_sp_amcl_adaptive_sigma.md`](paper_sp_amcl_adaptive_sigma.md) | **主报告**：自适应 sigma 算法推导、n=5 批量实验数据、出图建议 |

**核心结论（模块A，n=5）**：

| 工况 | Baseline 通过率 | SP-AMCL 通过率 | e_lin 对比 | e_ang 对比 | HLF |
|------|----------------|---------------|-----------|-----------|-----|
| Small | 5/5 | 5/5 | 持平（-0.4%）| 基本持平 | 0 vs 0 |
| Medium | 5/5 | 5/5 | 持平（-0.4%）| **-62.4%** | 1 vs 0 |
| Large | **0/5** | **4/5** | FAIL vs 0.2915 m | — | — |

---

### 模块C：置信度重定位判据

| 文档 | 内容 |
|------|------|
| [`04_reloc_confidence_criterion_impl.md`](04_reloc_confidence_criterion_impl.md) | 实现说明：C_t 公式、连续 K 帧判据、参数说明 |
| [`3_3_3_reloc_criterion_experiment_plan_v3.md`](3_3_3_reloc_criterion_experiment_plan_v3.md) | 实验方案 v3 |
| [`paper_baseline_vs_reloc_criterion.md`](paper_baseline_vs_reloc_criterion.md) | 模块C单独实验报告（n=8，medium 扰动） |

**核心结论（模块C单独，n=8）**：tr(Σ) 降低 42~50%，HLF 从 42.9% 降至 0%，但 L1 通过率从 87.5% 降至 50%（判据更严格导致部分超时）。

---

### 模块A+C：联合改进（最终方案）

| 文档 | 内容 |
|------|------|
| [`paper_sp_amcl_plus_c.md`](paper_sp_amcl_plus_c.md) | **主报告**：联合动机、参数配置、三方对比数据、分析结论 |

**模块C参数（联合场景调优后）**：tau=0.85，K=3（初始 tau=0.75, K=2 效果不佳，详见 §6）

**核心结论（模块A+C，单次验证）**：

| 工况 | vs Baseline e_lin | vs Baseline e_ang | HLF | 通过率 |
|------|------------------|------------------|-----|--------|
| Small | **-3.7%** | **-89.2%** | 否 | 1/1 |
| Medium | **-11.8%** | **-82.8%** | 否 | 1/1 |
| Large | Baseline 全部失败 | — | 否 | 1/1 |

---

## 关键参数文件位置

| 参数文件 | 路径 | 说明 |
|---------|------|------|
| 模块A | `multi_floor_nav/configs/spa_amcl_region_init.yaml` | sigma、等待时间、退避参数 |
| 模块C | `multi_floor_nav/configs/reloc_confidence_criterion.yaml` | lambda=20, tau=0.85, K=3 |

## 实验脚本

| 脚本 | 用法 |
|------|------|
| `scripts/amcl/run_one_experiment.sh` | 单次实验，method 可选 `baseline` / `sp_amcl` / `sp_amcl_c` |
| `scripts/amcl/run_batch_comparison.sh` | 批量实验（baseline + sp_amcl，各工况 n=5） |

> **注意**：本目录文件为 `docs/amcl/` 原文件的**副本**，原文件仍保留在 `docs/amcl/` 下。如需修改，请同步更新两处，或以 `docs/amcl/` 下的原文件为准。
