# AMCL 相关脚本

本目录存放与 **AMCL 定位、基线/SPA-AMCL 实验数据采集** 相关的可执行脚本。

## 脚本列表

| 脚本 | 说明 |
|------|------|
| [record_baseline.sh](record_baseline.sh) | 录制 Baseline-AMCL 实验的 rosbag（`/amcl_pose`、`/tf`、`/odometry/filtered`、`/move_base/status`），保存到 `multi_floor_nav/bagfiles/` |

## 使用方式

在仓库根目录下执行，例如：

```bash
cd ~/catkin_ws/src/robot-multi-floor-navigation
./scripts/amcl/record_baseline.sh
```

## 归类约定（后续新增 AMCL 相关脚本请遵循）

- **与 AMCL 实验、数据采集、参数测试相关的脚本**，均放在本目录 `scripts/amcl/` 下。
- 文档与实验流程说明请放在 `docs/amcl/`，参见 [docs/amcl/README.md](../../docs/amcl/README.md)。
