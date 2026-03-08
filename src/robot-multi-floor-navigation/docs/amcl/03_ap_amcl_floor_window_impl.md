# 模块 B：切层窗口参数切换（AP-AMCL）实现说明

## 1. 目的

在**出电梯后的 L1 重定位阶段**临时提高 AMCL 的粒子数与恢复速度，以提升切层后重定位成功率和收敛速度；重定位完成后恢复原参数，控制计算开销。对应方案文档中的 **AP-AMCL**（仅粒子自适应、不加入语义先验）。

## 2. 实现要点

- **触发时机**：状态机在 `LOAD_MAP` 成功后、`desired_map_level == 1`（即刚切到 L1 地图）时，进入 `INIT_POSE` 之前，调用 `apply_amcl_floor_window_params(true)`，通过 dynamic_reconfigure 将 AMCL 的 `max_particles`、`recovery_alpha_fast` 设为窗口参数。
- **恢复时机**：在 `CHECK_INITPOSE` 通过（“Robot at correct pose”）且当前为 L1 时，调用 `apply_amcl_floor_window_params(false)`，恢复为切层前保存的 AMCL 配置。
- **不改 AMCL 源码**：仅通过 `dynamic_reconfigure::Client<amcl::AMCLConfig>` 读写 `/amcl` 的运行时参数。

## 3. 代码与配置

| 内容 | 路径 |
|------|------|
| 逻辑与 dynamic_reconfigure 调用 | `multi_floor_nav/src/multi_floor_navigation.cpp`（`apply_amcl_floor_window_params`、LOAD_MAP/CHECK_INITPOSE 分支） |
| 头文件 | `multi_floor_nav/include/multi_floor_nav/multi_floor_navigation.h`（Client、保存配置、窗口参数成员） |
| 窗口参数默认值 | `multi_floor_nav/configs/ap_amcl_floor_window.yaml` |
| Launch 开关 | `multi_floor_nav/launch/nav.launch`：`use_ap_amcl`，为 true 时加载上述 yaml 到 `multi_floor_navigation_node` |

## 4. 参数说明（可根据实验效果调整）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_floor_window_params` | false | 为 true 时启用切层窗口参数切换（由 launch 的 use_ap_amcl 通过 yaml 传入） |
| `amcl_window_max_particles` | 3500 | 窗口内 AMCL 最大粒子数（ baseline 常用 2000，适当增大有利于重定位） |
| `amcl_window_recovery_alpha_fast` | 0.2 | 窗口内快速恢复系数（baseline 常用 0.1，增大后更积极注入随机粒子） |

**调参建议**：若 L1 重定位仍易失败或收敛慢，可尝试将 `amcl_window_max_particles` 提高到 4000、`amcl_window_recovery_alpha_fast` 提高到 0.25；若 CPU 占用偏高，可适当降低窗口内粒子数或缩短窗口（当前窗口为“从切到 L1 到重定位通过”的整段）。

## 5. Baseline vs AP-AMCL 对比实验

- **Baseline-AMCL**：`roslaunch multi_floor_nav nav.launch use_region_init:=false use_ap_amcl:=false`
- **AP-AMCL**：`roslaunch multi_floor_nav nav.launch use_region_init:=false use_ap_amcl:=true`

两者均**不使用**区域初始化（模块 A），仅区分是否在 L1 切层窗口内切换 AMCL 参数。建议指标：

- L1 重定位成功率、L1 收敛时间；
- 任务成功率；
- 可选：平均粒子数或 CPU 占用（窗口内 vs 窗口外）。

可根据实验结果调整 `ap_amcl_floor_window.yaml` 中的 `amcl_window_max_particles`、`amcl_window_recovery_alpha_fast`，使 AP-AMCL 在重定位上明显优于 Baseline，同时控制计算开销在可接受范围。
