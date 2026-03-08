# 第 1 步：标准 AMCL 基线实验流程

本文档用于**固定实验流程**并**稳定采集 Baseline-AMCL 数据**，供后续与 SPA-AMCL 对比。

---

## 1. 环境与依赖

- ROS（Noetic 或 Melodic）
- 工作空间已编译：`catkin_make` 或 `catkin build`
- 依赖包已安装：`jackal_elevator`、`jackal_navigation`、`jackal_viz`、`map_server`、`amcl`、`move_base` 等（参见项目 README）

---

## 2. 固定实验流程（每次基线实验一致执行）

### 2.1 启动仿真与导航（终端 1）

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch multi_floor_nav nav.launch
```

等待直至：

- Gazebo 中世界与机器人加载完成；
- RViz 中显示地图与机器人；
- 终端中无持续报错（AMCL 可能尚未收到地图，属正常，见下）。

### 2.2 开始状态机（终端 2，仅执行一次）

在确认 Gazebo 与 RViz 均已就绪后，**只执行一次**：

```bash
source ~/catkin_ws/devel/setup.bash
rostopic pub /start std_msgs/Empty "{}"
```

此后状态机将依次：

1. 调用 `change_map(0)`，向 AMCL 发布 0 层地图；
2. 发布 `initialpose`（Level 0: x=4.0, y=-5.0, theta=0）；
3. 等待 AMCL 收敛（CHECK_INITPOSE）；
4. 发送导航目标（Level 0 → 电梯口 3.0, -0.5）；
5. 进电梯、请求电梯到 1 层、出电梯；
6. 切图到 Level 1，发布 `initialpose`（3.0, -0.5, π）；
7. 再次等待收敛后导航到终点（4.0, 5.0, 0）。

### 2.3 单次实验结束判定

- **成功**：状态机进入 `DONE`，终端出现 `[Multi Floor Nav] Robot arrived at Goal`。
- **失败**：长时间卡在某一状态、定位失败重试、导航超时或明显跑错楼层。

---

## 3. 基线数据采集（用于后续对比）

### 3.1 需要记录的话题（建议）

| 话题 | 用途 |
|------|------|
| `/amcl_pose` | 定位置信度、收敛时间、轨迹 |
| `/tf` | 位姿与时间戳 |
| `/odometry/filtered` | 里程计 |
| `/move_base/status` | 导航状态与成功/失败 |
| `/move_base_simple/goal` | 下发目标（可选） |

### 3.2 使用脚本录制（推荐）

在**另一终端**在发 `/start` 之前或同时启动录制（AMCL 相关脚本统一放在 `scripts/amcl/`）：

```bash
cd ~/catkin_ws/src/robot-multi-floor-navigation
# 在实验开始前或开始时执行，录制约 3–5 分钟或直至任务结束
./scripts/amcl/record_baseline.sh
```

录制结束后在 `multi_floor_nav/bagfiles/baseline_YYYYMMDD_HHMMSS.bag` 中保存数据，便于后续分析收敛时间、成功率与轨迹。

### 3.3 手动 rosbag 示例

若不用脚本，可手动执行：

```bash
mkdir -p ~/catkin_ws/src/robot-multi-floor-navigation/multi_floor_nav/bagfiles
cd ~/catkin_ws/src/robot-multi-floor-navigation/multi_floor_nav/bagfiles
rosbag record -O baseline_$(date +%Y%m%d_%H%M%S).bag \
  /amcl_pose /tf /odometry/filtered /move_base/status
```

---

## 4. 基线实验检查清单（每次跑基线前核对）

- [ ] 只开一个 `nav.launch`，无重复启动 amcl / move_base；
- [ ] `/start` 只发一次，且在 Gazebo/RViz 就绪后发；
- [ ] 不修改 `amcl.yaml` 与状态机逻辑（保持标准 AMCL 基线）；
- [ ] 若录制 rosbag：在发 `/start` 前或同时开始录制，任务结束或超时后停止；
- [ ] 记录当次结果：成功/失败、若失败记录卡住的状态或现象。

---

## 5. 建议重复次数

为得到稳定基线统计，建议：

- 至少成功跑通 **5 次** 完整流程（Level 0 → 电梯 → Level 1 → 终点）；
- 记录每次是否成功、从发 `initialpose` 到定位收敛的大致时间（可事后用 rosbag 精确算）；
- 若有失败，记录失败阶段（例如：CHECK_INITPOSE 反复重试、NAV_TO_GOAL 超时等）。

---

## 6. 与后续步骤的关系

- 本流程即 **Baseline-AMCL**：单点 `initialpose`、无区域先验、无切层窗口参数切换、无重定位判据。
- 第 2 步将在**不改变本流程其余部分**的前提下，仅把“单点 initialpose”改为“区域初始化增强”，形成 SP-AMCL 对比组。
