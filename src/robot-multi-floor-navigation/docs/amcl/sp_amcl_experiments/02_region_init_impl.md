# 模块 A：区域初始化增强（SP-AMCL）— 实现说明

本文档说明已实现的**电梯出口区域初始化**及其全部改进特性，对应论文章节 3.3.2。

---

## 1. 核心改进思路

- **传统 AMCL（Baseline）**：`set_init_pose` 发布零协方差单点 `initialpose`，AMCL 所有粒子堆叠在同一点。当初值有偏差时，粒子群锁死在错误位置，依赖运动噪声扩散极慢。
- **SP-AMCL（模块 A）**：发布带**非零协方差**的 `initialpose`，粒子按高斯先验在电梯出口局部区域散布，真值更可能落入粒子覆盖范围，激光似然驱动重采样后自然向真值聚集。

---

## 2. 实现特性一览

| 特性 | 描述 | 对应缺陷修复 |
|------|------|-------------|
| 区域高斯初始化 | 发布 `initialpose` 时写入非零协方差，AMCL 按高斯散布粒子 | 基础功能 |
| **Per-floor sigma** | L0/L1 独立配置 σ 值（L1 走廊较规整，σ 可适当放宽） | 缺陷一 |
| **Per-floor 等待时间** | L0/L1 独立配置 `reloc_min_check_delay`（L1=12s，确保粒子收敛） | 缺陷二 |
| **Retry 退避策略** | 每次 retry 时 σ × backoff_factor，上限 max_backoff 倍初始值 | 缺陷五 |
| **自适应 sigma** | σ 根据偏移量动态缩放：σ = min(σ_floor, max(σ_min, α·d_planar)) | 缺陷六 |
| **零偏移退化处理** | 偏移量为零时 σ 自动归零，退化为与 Baseline 相同的单点初始化 | 缺陷七 |

---

## 3. 自适应 Sigma 策略（核心算法）

固定 σ 存在结构性矛盾：小偏移时过大的 σ 引入多余不确定性，大偏移时过小的 σ 无法覆盖真值。自适应策略根据偏移量动态调整：

$$\sigma_{\text{adaptive}} = \max\left(\sigma_{\min},\; \alpha \cdot d_{\text{planar}}\right)$$

$$\sigma_{x/y} = \min\left(\sigma_{x/y}^{\text{floor}},\; \sigma_{\text{adaptive}}\right)$$

其中：
- $\alpha = 0.35$：比例因子
- $\sigma_{\min} = 0.03$ m：最小 sigma
- $d_{\text{planar}} = \sqrt{\delta_x^2 + \delta_y^2}$：平面偏移量
- $\sigma^{\text{floor}}$：该楼层的 sigma 上限

**零偏移退化**：当 $d_{\text{planar}} < 10^{-6}$ 且 $|\Delta\theta| < 10^{-6}$ 时，sigma 直接归零，SP-AMCL 退化为与 Baseline 相同的单点初始化，避免在无扰动场景下引入不必要的粒子散布。

---

## 4. 如何启用区域初始化

```bash
roslaunch multi_floor_nav nav.launch use_region_init:=true
```

此时加载 `configs/spa_amcl_region_init.yaml` 到 `multi_floor_navigation_node`。

如何跑 Baseline（不启用）：

```bash
roslaunch multi_floor_nav nav.launch
# 或
roslaunch multi_floor_nav nav.launch use_region_init:=false
```

---

## 5. 参数配置

参数文件：`multi_floor_nav/configs/spa_amcl_region_init.yaml`

### 5.1 Per-floor Sigma

| 参数 | L0 | L1 | 含义 |
|------|----|----|------|
| `region_init_sigma_x_*` | 0.15 m | **0.20 m** | x 方向位置标准差（楼层基准上限） |
| `region_init_sigma_y_*` | 0.15 m | **0.20 m** | y 方向位置标准差 |
| `region_init_sigma_yaw_*` | 0.1 rad | **0.12 rad** | 朝向标准差 |

### 5.2 Per-floor 等待时间

| 参数 | 值 | 含义 |
|------|-----|------|
| `reloc_min_check_delay_sec`（全局） | 6.0 s | Baseline 也使用此值 |
| `reloc_min_check_delay_L0` | 6.0 s | L0 层等待时间 |
| `reloc_min_check_delay_L1` | **12.0 s** | L1 层粒子散布后需更多帧收敛 |

### 5.3 Retry 退避参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `region_init_backoff_factor` | 1.3 | 每次 retry σ 乘以此因子 |
| `region_init_max_backoff` | 2.0 | sigma 上限为初始值的 2 倍 |

### 5.4 自适应 Sigma 参数

| 参数 | 值 | 含义 |
|------|-----|------|
| `region_init_sigma_adaptive_alpha` | 0.35 | 比例因子 α |
| `region_init_sigma_min_scale` | 0.03 m | 最小 sigma |
| `region_init_sigma_min_yaw` | 0.02 rad | 最小偏航 sigma |

---

## 6. 修改与配置文件位置

| 内容 | 路径 |
|------|------|
| 核心逻辑（自适应sigma、退避策略） | `multi_floor_nav/src/multi_floor_navigation.cpp`（`set_init_pose()`） |
| 重定位通过判定 | `multi_floor_nav/src/multi_floor_navigation.cpp`（`check_robot_pose()`） |
| 头文件成员与参数声明 | `multi_floor_nav/include/multi_floor_nav/multi_floor_navigation.h` |
| 区域初始化参数 | `multi_floor_nav/configs/spa_amcl_region_init.yaml` |
| Launch 开关 | `multi_floor_nav/launch/nav.launch` 中 `use_region_init` |

---

## 7. 与其他模块的关系

- **模块 B（AP-AMCL）**：独立，可组合使用
- **模块 C（置信度判据）**：独立，可组合使用
- **模块 A + C 联合**：通过 `use_region_init:=true use_covariance_reloc:=true` 同时启用，在实验脚本中对应方法名 `sp_amcl_c`

---

## 8. 已修复缺陷列表

| 缺陷 | 描述 | 修复方案 | 难度 |
|------|------|---------|------|
| 缺陷一 | σ 为全局固定值，L0/L1 相同 | per-floor 参数 | 低 |
| 缺陷二 | SP-AMCL 与 Baseline 等待时间相同 | per-floor `reloc_min_check_delay` | 低 |
| 缺陷五 | retry 时每次重发相同 σ | 退避策略 σ×backoff_factor^n | 低 |
| 缺陷六 | 固定 σ 在小偏移时过大 | 自适应 sigma | 中 |
| 缺陷七 | 零偏移时仍有 σ_min 散布 | 零偏移退化处理 | 低 |

> 缺陷三（单帧通过判据）由模块 C 解决；缺陷四（高斯中心仍在偏差点）属高难度，暂不处理。
