# use_region_init:=true 三轮运行小结

在 **headless** 下用 `roslaunch multi_floor_nav nav.launch use_region_init:=true gui:=false headless:=true` 跑 3 轮，发一次 `/start`，观察重定位与完整流程。

---

## 1. 配置

- **区域初始化**：`use_region_init:=true`（sigma_x=0.3, sigma_y=0.3, sigma_yaw=0.2）
- **全局规划器**：navfn（默认），无 NO PATH 刷屏
- **仿真**：Gazebo headless，无 GUI

---

## 2. 三轮结果概览

| 轮次 | Level 0 收敛 | Level 1 收敛 | 任务结果 |
|------|----------------|----------------|----------|
| Run 1 | ~1 s（1076.09→1077.09） | ~1 s（1136.30→1137.30） | 流程走完，终点目标已发（录制约 90 s 结束） |
| Run 2 | ~1 s（1076.39→1077.39） | ~1 s（1156.99→1157.99） | **Robot arrived at Goal** |
| Run 3 | ~1 s（1076.22→1077.22） | ~1 s（1154.43→1155.43） | 流程走完，终点目标已发（录制约 120 s 结束） |

时间均为仿真时间（sim time），单位秒。

---

## 3. 重定位是否更稳、收敛是否更快

- **收敛时间**：三层楼两次重定位（Level 0 初定位 + Level 1 出电梯后）均在约 **1 s** 内达到 “Robot at correct pose”，线性/角度误差小（如 Linear Error 0.07～0.10 m，Angular Error 0.00～0.03 rad）。
- **稳定性**：3 轮中两次重定位均一次通过，无 “Setting initial pose failed. Will Retry”，说明在 headless 下 **重定位稳定**。
- **完整流程**：Run 2 在约 120 s 内出现 “Robot arrived at Goal”；Run 1、Run 3 在截断前已发最终目标，流程均完整（L0→电梯→L1→发终点）。

结论：在本次 3 轮 headless 测试中，**use_region_init:=true 下重定位稳定、收敛快（约 1 s）**；与基线（单点 initialpose）的定量对比建议在本机带 GUI 下多跑几轮，用 rosbag 精确算收敛时间与成功率。

---

## 4. 日志关键字（便于自查）

- 区域初始化已用：`(region sigma: x=0.30, y=0.30, yaw=0.20)`
- Level 0 收敛：`Robot at correct pose`（紧随第一次 `Initializing Pose ... at level: 0`）
- Level 1 收敛：`Robot at correct pose`（紧随 `Initializing Pose ... at level: 1`）
- 任务成功：`Robot arrived at Goal`
