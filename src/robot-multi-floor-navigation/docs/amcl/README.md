# AMCL 相关文档

本目录集中存放**与 AMCL 定位、多楼层 SPA-AMCL 改进方案**相关的说明与实验流程文档。

## 文档索引

| 文件 | 说明 |
|------|------|
| [00_FA_AMCL_多楼层定位改进方案.md](00_FA_AMCL_多楼层定位改进方案.md) | 改进方案总览：SPA-AMCL 思路、模块说明、实现顺序、实验设计、论文建议 |
| [01_baseline_procedure.md](01_baseline_procedure.md) | 第 1 步：标准 AMCL 基线实验流程与数据采集方法 |
| [step1_run_guide.md](step1_run_guide.md) | 步骤一运行指南：按终端顺序跑通基线、录制脚本路径、5 次记录说明 |
| [baseline_5run_log.txt](baseline_5run_log.txt) | 5 次基线运行记录表（可复制填写） |
| [02_region_init_impl.md](02_region_init_impl.md) | 第 2 步：模块 A 最简版（区域初始化增强）实现说明与使用方式 |
| [03_ap_amcl_floor_window_impl.md](03_ap_amcl_floor_window_impl.md) | 模块 B：切层窗口参数切换（AP-AMCL）实现说明与对比实验方式 |
| [04_reloc_confidence_criterion_impl.md](04_reloc_confidence_criterion_impl.md) | 模块 C：重定位完成判据优化（C_t 置信度 + 连续 K 帧）实现说明 |
| [paper_baseline_vs_ap_amcl.md](paper_baseline_vs_ap_amcl.md) | **论文用**：Baseline vs AP-AMCL 实验设置与结果（模块 B） |
| [paper_baseline_vs_reloc_criterion.md](paper_baseline_vs_reloc_criterion.md) | **论文用**：Baseline 判据 vs 模块 C（重定位置信度判据）实验设置与结果 |
| [region_init_3run_summary.md](region_init_3run_summary.md) | use_region_init:=true 三轮运行小结（重定位收敛约 1 s、稳定性与任务结果） |
| [paper_baseline_vs_region_init.md](paper_baseline_vs_region_init.md) | **论文用**：改进前/后关键参数、逐次运行数据、汇总表与作图制表建议 |
| [baseline_vs_region_init_data.csv](baseline_vs_region_init_data.csv) | 上表原始数据（CSV，可导入 Excel/Python 作图） |

## 归类约定（后续新增 AMCL 文档请遵循）

- **所有与 AMCL 改进、实验流程、参数说明、对比实验设计相关的文档**，均放在本目录 `docs/amcl/` 下。
- 命名建议：`NN_简短描述.md`，如 `02_region_init_impl.md`、`03_reloc_criterion.md`，便于按顺序阅读。
- 与 AMCL 相关的**可执行脚本**放在仓库根目录下的 `scripts/amcl/`，参见 [scripts/amcl/README.md](../../scripts/amcl/README.md)。
- 配置文件仍保留在 `multi_floor_nav/configs/`（如 `amcl.yaml`），无需挪入本目录。

## 相关代码与配置位置

- AMCL 配置：`multi_floor_nav/configs/amcl.yaml`
- 模块 A 区域初始化：`multi_floor_nav/configs/spa_amcl_region_init.yaml`（use_region_init:=true 时加载）
- 模块 B 切层窗口参数：`multi_floor_nav/configs/ap_amcl_floor_window.yaml`（use_ap_amcl:=true 时加载）
- 模块 C 重定位置信度判据：`multi_floor_nav/configs/reloc_confidence_criterion.yaml`（use_covariance_reloc:=true 时加载）
- 启动 AMCL 的 launch：`multi_floor_nav/launch/nav.launch`
- 与 AMCL 交互的源码：`multi_floor_nav/src/multi_floor_navigation.cpp`（订阅 `amcl_pose`、发布 `initialpose`）
