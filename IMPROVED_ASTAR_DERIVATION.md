# Improved A* 推导文档（MathType 版）

本文档对应实现文件：
- `src/improved_global_planner/src/improved_astar.cpp`
- `src/improved_global_planner/src/planner_core.cpp`
- `src/robot-multi-floor-navigation/multi_floor_nav/launch/nav.launch`

目标：给出当前“6 邻域 + 密度自适应权重 + Weighted A*”的完整推导，所有公式可直接复制到 MathType（LaTeX 输入）中。

---

## 1. 变量与符号定义

### 1.1 栅格与节点
- 当前节点：`n`，栅格坐标为 `(x, y)`，线性索引 `i = x + y * nx`
- 邻居节点：`n'`，栅格坐标为 `(x', y')`，线性索引 `i'`
- 目标节点：`g`，坐标 `(x_g, y_g)`
- 地图宽度：`nx`，高度：`ny`

### 1.2 代价图参数
- `c_0`：基础步长代价（代码中的 `neutral_cost_`）
- `c(i')`：邻居节点 `i'` 的 costmap 代价（代码中的 `costs[next_i]`）
- `c_lethal`：致命障碍阈值（代码中的 `lethal_cost_`）

### 1.3 A* 代价函数
- `g(n')`：从起点到邻居节点的累计代价
- `h(n')`：邻居到目标的启发式估计代价
- `f(n')`：优先队列排序代价

MathType/LaTeX：
```tex
f(n') = g(n') + h(n')
```

---

## 2. 邻域改进：4 邻域到方向引导 6 邻域

### 2.1 基础 4 邻域
```tex
\mathcal{N}_4(n) = \{(x+1,y),(x-1,y),(x,y+1),(x,y-1)\}
```

### 2.2 方向引导的 2 个对角邻居
先计算目标相对位移：
```tex
\Delta x_g = x_g - x,\quad \Delta y_g = y_g - y
```

- 若 `|\Delta x_g| >= |\Delta y_g|`，主方向按 x 轴，补充 `(\pm1,+1)` 与 `(\pm1,-1)`
- 否则主方向按 y 轴，补充 `(+1,\pm1)` 与 `(-1,\pm1)`

最终：
```tex
\mathcal{N}_6(n)=\mathcal{N}_4(n)\cup\mathcal{N}_{diag,2}(n)
```

说明：该策略在保证分支数较低的同时，引导搜索朝目标主方向推进。

---

## 3. 基础步长代价与对角补偿

代码中 `p_calc_->calculatePotential(...)` 给出基础累计代价，记为 `g_calc(n')`。

定义对角步指示函数：
```tex
\delta_{diag}=
\begin{cases}
1, & |x'-x|=1\ \text{and}\ |y'-y|=1\\
0, & \text{otherwise}
\end{cases}
```

对角补偿项：
```tex
C_{diag} = \delta_{diag}\,(\sqrt{2}-1)\,c_0
```

基础 `g`：
```tex
g_{base}(n') = g_{calc}(n') + C_{diag}
```

---

## 4. 局部障碍物密度建模（三场景核心）

在 `n'` 周围取窗口半径 `r`（窗口大小 `(2r+1)\times(2r+1)`）：
- `N_total`：窗口内有效网格总数
- `N_occ`：障碍网格数（`c >= c_lethal`）
- `N_inflation`：膨胀区网格数（`c_0 < c < c_lethal`）

密度定义：
```tex
\rho = \frac{N_{occ} + \lambda N_{inflation}}{N_{total}},\quad \rho\in[0,1]
```

其中 `\lambda` 为膨胀区计数权重（`adaptive_inflation_lambda`）。

三模式划分：
```tex
\text{Open: } \rho < T_{open}
```
```tex
\text{Normal: } T_{open} \le \rho < T_{narrow}
```
```tex
\text{Narrow: } \rho \ge T_{narrow}
```

---

## 5. 障碍惩罚项（安全）

先归一化邻居代价：
```tex
c_{norm}(n') = \frac{\min(c(n'), c_{lethal}-1)}{c_{lethal}-1}
```

障碍惩罚：
```tex
C_{obs}(n') = w_{obs}(\rho)\,c_0\,[c_{norm}(n')]^2
```

说明：平方项使高代价区域惩罚增长更快，减少贴障路径。

---

## 6. 方向对齐惩罚（效率）

定义两向量：
```tex
\mathbf{v}=(x'-x,\ y'-y),\quad \mathbf{t}=(x_g-x,\ y_g-y)
```

夹角余弦：
```tex
\cos\theta = \frac{\mathbf{v}\cdot\mathbf{t}}{\|\mathbf{v}\|\,\|\mathbf{t}\|+\varepsilon}
```

方向惩罚：
```tex
C_{dir}(n') = w_{dir}(\rho)\,c_0\,(1-\cos\theta)
```

说明：越朝向目标（`cos(theta)` 越接近 1），惩罚越小。

---

## 7. 自适应累计代价 g

```tex
g(n') = g_{base}(n') + C_{obs}(n') + C_{dir}(n')
```

---

## 8. 启发函数 h 与 Weighted A*

采用 Octile 距离（适配含对角扩展）：
```tex
d_x = |x_g - x'|,\quad d_y = |y_g - y'|
```
```tex
h_{oct}(n') = \max(d_x,d_y) + (\sqrt{2}-1)\min(d_x,d_y)
```

加权启发：
```tex
f(n') = g(n') + \alpha(\rho)\,h_{oct}(n')\,c_0
```

其中 `\alpha(\rho)` 按三模式切换：
- Open：更大（更快）
- Normal：中等
- Narrow：接近 1（更稳）

---

## 9. 最终完整公式（可直接用于论文）

```tex
\begin{aligned}
f(n') =\;& g_{calc}(n') + \delta_{diag}(\sqrt{2}-1)c_0 \\
&+ w_{obs}(\rho)c_0\left(\frac{\min(c(n'),c_{lethal}-1)}{c_{lethal}-1}\right)^2 \\
&+ w_{dir}(\rho)c_0\left(1-\frac{\mathbf{v}\cdot\mathbf{t}}{\|\mathbf{v}\|\,\|\mathbf{t}\|+\varepsilon}\right) \\
&+ \alpha(\rho)\left[\max(d_x,d_y)+(\sqrt{2}-1)\min(d_x,d_y)\right]c_0
\end{aligned}
```

---

## 10. 与代码参数一一对应（当前默认值）

来自 `planner_core.cpp` 与 `nav.launch`：

- `adaptive_density_radius = 2`
- `adaptive_inflation_lambda = 0.5`
- `adaptive_threshold_open = 0.20`
- `adaptive_threshold_narrow = 0.45`
- `adaptive_w_obs_open = 0.15`
- `adaptive_w_obs_normal = 0.45`
- `adaptive_w_obs_narrow = 0.90`
- `adaptive_w_dir_open = 0.35`
- `adaptive_w_dir_normal = 0.25`
- `adaptive_w_dir_narrow = 0.15`
- `adaptive_alpha_open = 1.35`
- `adaptive_alpha_normal = 1.15`
- `adaptive_alpha_narrow = 1.00`

---

## 11. MathType 快速粘贴（仅公式行）

以下每行都可直接粘贴到 MathType（LaTeX 输入）：

```tex
\rho = \frac{N_{occ} + \lambda N_{inflation}}{N_{total}}
```
```tex
C_{diag} = \delta_{diag}\,(\sqrt{2}-1)\,c_0
```
```tex
c_{norm}(n') = \frac{\min(c(n'), c_{lethal}-1)}{c_{lethal}-1}
```
```tex
C_{obs}(n') = w_{obs}(\rho)\,c_0\,[c_{norm}(n')]^2
```
```tex
\cos\theta = \frac{\mathbf{v}\cdot\mathbf{t}}{\|\mathbf{v}\|\,\|\mathbf{t}\|+\varepsilon}
```
```tex
C_{dir}(n') = w_{dir}(\rho)\,c_0\,(1-\cos\theta)
```
```tex
h_{oct}(n') = \max(d_x,d_y) + (\sqrt{2}-1)\min(d_x,d_y)
```
```tex
f(n') = g(n') + \alpha(\rho)\,h_{oct}(n')\,c_0
```

