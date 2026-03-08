# 第 2 步：模块 A 最简版 — 区域初始化增强（实现说明）

本文档说明已实现的**电梯出口区域初始化**用法与对比方式，对应 SPA-AMCL 模块 A 最简版。

---

## 1. 实现方式

- **原逻辑**：`set_init_pose` 只发布一个位姿点，不设置协方差（相当于 0），AMCL 在单点附近初始化粒子。
- **现逻辑**：在发布 `initialpose` 时，若配置了 `region_init_sigma_x/y/yaw`，则写入 `PoseWithCovarianceStamped` 的协方差（方差 = sigma²），AMCL 会在电梯出口中心附近按高斯散布粒子，减轻对单点初值的敏感。

未配置或设为 0 时，行为与原来一致（基线）。

---

## 2. 如何启用区域初始化（SP-AMCL 第 2 步）

启动时加上参数 `use_region_init:=true`，并保证已加载区域初始化参数：

```bash
roslaunch multi_floor_nav nav.launch use_region_init:=true
```

此时会加载 `configs/spa_amcl_region_init.yaml` 到 `multi_floor_navigation_node`，默认：

- `region_init_sigma_x: 0.3`（米）
- `region_init_sigma_y: 0.3`（米）
- `region_init_sigma_yaw: 0.2`（弧度，约 11°）

---

## 3. 如何跑基线（不启用区域初始化）

不传 `use_region_init` 或显式设为 `false`：

```bash
roslaunch multi_floor_nav nav.launch
# 或
roslaunch multi_floor_nav nav.launch use_region_init:=false
```

其余流程与步骤一相同（发 `/start` 等）。

---

## 4. 修改与配置文件位置

| 内容 | 位置 |
|------|------|
| 区域协方差写入与参数读取 | `multi_floor_nav/src/multi_floor_navigation.cpp` 中 `set_init_pose()` |
| 区域初始化参数 | `multi_floor_nav/configs/spa_amcl_region_init.yaml` |
| Launch 开关 | `multi_floor_nav/launch/nav.launch` 中 `use_region_init` 及对应 `<rosparam>` |

---

## 5. 调参建议

- 出口位置不确定较大时，可适当增大 `region_init_sigma_x`、`region_init_sigma_y`（如 0.4～0.5）。
- 朝向不确定较大时，可增大 `region_init_sigma_yaw`（如 0.3～0.4 rad）。
- 做严格基线对比时，务必用 `use_region_init:=false`；做 SP-AMCL 对比时用 `use_region_init:=true`。

---

## 6. 与后续步骤的关系

- 本步仅增加**区域初始化**，未改切层窗口粒子参数（模块 B）和重定位完成判据（模块 C）。
- 完成本步后，可先对比 Baseline（`use_region_init:=false`）与 SP-AMCL 区域版（`use_region_init:=true`）的重定位时间与成功率，再进入第 3 步（模块 B + 模块 C）。
