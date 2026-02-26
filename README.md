# Catkin Workspace — 多楼层导航与改进 A* 全局规划

ROS Noetic catkin 工作空间，包含：

- **improved_global_planner**：基于 global_planner 的改进 A* 全局规划器插件，支持原版 A* 与改进 A* 切换对比。
- **robot-multi-floor-navigation / multi_floor_nav**：多楼层导航（电梯跨层、地图切换、状态机）。
- **Jackal 相关包**：jackal_desktop、jackal_elevator、jackal_simulator 等（仿真与可视化）。

## 编译

```bash
cd /path/to/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 运行多楼层导航（使用改进 A* 插件）

```bash
roslaunch multi_floor_nav nav.launch
```

在 `nav.launch` 中通过参数切换全局规划算法：

- `use_dijkstra: false`、`use_improved_astar: false` → 原版 A*
- `use_dijkstra: false`、`use_improved_astar: true` → 改进 A*（ImprovedAStarExpansion）

## 推送到 GitHub（首次）

1. 在 [GitHub](https://github.com/new) 新建一个仓库（例如 `catkin_ws_multi_floor`），**不要**勾选 “Add a README”。
2. 在本机执行：

```bash
cd /path/to/catkin_ws
git remote add origin https://github.com/你的用户名/仓库名.git
git branch -M main
git push -u origin main
```

若使用 SSH：`git remote add origin git@github.com:你的用户名/仓库名.git`
