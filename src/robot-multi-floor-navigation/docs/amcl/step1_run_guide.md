# 步骤一运行指南：跑通基线并记录 5 次

按以下顺序在你本机**实际跑通**基线，并至少成功完成 5 次、做好记录。

---

## 1. 启动仿真（终端 1）

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch multi_floor_nav nav.launch
```

等待直到：

- Gazebo 中世界和 Jackal 机器人加载完成；
- RViz 中能看到地图（可能先空白，发 `/start` 后才有地图）；
- 终端里没有持续刷新的严重报错。

**说明**：刚启动时可能出现 `Timed out waiting for transform from base_link to map`，属正常（地图要等发 `/start` 后才由 change_map 发布）。

---

## 2. 可选：开始录制 rosbag（终端 2）

若要采集基线数据，在发 `/start` **之前**在本终端执行：

```bash
cd ~/catkin_ws/src/robot-multi-floor-navigation
./scripts/amcl/record_baseline.sh
```

任务结束后在本终端按 **Ctrl+C** 停止录制。bag 会保存在 `multi_floor_nav/bagfiles/baseline_YYYYMMDD_HHMMSS.bag`。

---

## 3. 启动状态机（终端 2 或 3）

确认 Gazebo 和 RViz 都已就绪后，**只执行一次**：

```bash
source ~/catkin_ws/devel/setup.bash
rostopic pub /start std_msgs/Empty "{}"
```

之后状态机会自动执行：加载 0 层地图 → 设 initialpose → 收敛后导航到电梯口 → 进电梯 → 到 1 层 → 出电梯 → 切 1 层图 → 再收敛 → 导航到终点。

---

## 4. 判定单次是否成功

- **成功**：终端 1 里出现 `[Multi Floor Nav] Robot arrived at Goal`，且机器人停在终点 (4.0, 5.0) 附近。
- **失败**：长时间卡在某一状态（如反复 “Setting initial pose failed”）、导航超时、或明显跑错楼层。

---

## 5. 重复 5 次并填写记录表

建议至少**成功跑通 5 次**完整流程，并填写下面的记录表（可复制到 `docs/amcl/baseline_5run_log.txt` 或自行建表）。

| 次数 | 日期/时间 | 结果（成功/失败） | 备注（若失败写卡在哪一步） |
|------|------------|-------------------|-----------------------------|
| 1    |            |                   |                             |
| 2    |            |                   |                             |
| 3    |            |                   |                             |
| 4    |            |                   |                             |
| 5    |            |                   |                             |

若某次失败，可简单记：例如 “CHECK_INITPOSE 反复重试”“NAV_TO_GOAL 超时”“出电梯后定位错”等。

---

## 6. 常见问题

- **nav.launch 报错找不到 jackal_elevator / jackal_navigation**  
  按项目主 README 安装依赖（如 `sudo apt-get install ros-noetic-jackal-*`），并确保 `jackal_elevator` 在 `catkin_ws/src` 下且已编译。

- **Gazebo 启动很慢或卡住**  
  第一次启动可能较慢；若本机无显示器可加：`roslaunch multi_floor_nav nav.launch gui:=false headless:=true`（无 GUI，仅验证节点是否起来）。

- **录制脚本路径**  
  录制基线请用：`./scripts/amcl/record_baseline.sh`（脚本已归类到 `scripts/amcl/`）。

---

## 7. 确认无报错、基线固定后

- 5 次记录表填好、至少 5 次成功；
- 需要时用 rosbag 事后分析收敛时间。

即可进入**第 2 步：实现模块 A 最简版（区域初始化增强）**。详细流程仍见 [01_baseline_procedure.md](01_baseline_procedure.md)。
