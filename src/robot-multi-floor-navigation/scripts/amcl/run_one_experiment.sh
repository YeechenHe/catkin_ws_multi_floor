#!/bin/bash
# 单次初值扰动实验：启动 nav（headless）、等待就绪、发送 /start、运行固定时长、保存日志
# 用法: run_one_experiment.sh <method> <offset_x> <offset_y> <offset_theta> <run_id> [duration_sec] [log_dir]
#  method: baseline | spamcl | apamcl（模块 B） | relocc（模块 C：重定位置信度判据）
# 示例: ./run_one_experiment.sh baseline 0 0 0 1 180 /tmp/exp_logs

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
# catkin_ws 为 robot-multi-floor-navigation 的上两级
source "$REPO_DIR/../../devel/setup.bash" 2>/dev/null || true
[ -n "$ROS_DISTRO" ] || { echo "Source catkin workspace first: source ~/catkin_ws/devel/setup.bash"; exit 1; }

METHOD="${1:-baseline}"
OX="${2:-0}"
OY="${3:-0}"
OT="${4:-0}"
RUN_ID="${5:-1}"
DURATION="${6:-180}"
LOG_DIR="${7:-$REPO_DIR/multi_floor_nav/experiment_logs}"

mkdir -p "$LOG_DIR"
PERT_NAME="pert_${OX}_${OY}_${OT}"
LOG_FILE="$LOG_DIR/${METHOD}_${PERT_NAME}_run${RUN_ID}.log"

USE_REGION="false"
USE_AP_AMCL="false"
USE_COVARIANCE_RELOC="false"
[ "$METHOD" = "spamcl" ] && USE_REGION="true"
[ "$METHOD" = "apamcl" ] && USE_AP_AMCL="true"
[ "$METHOD" = "relocc" ] && USE_COVARIANCE_RELOC="true"

echo "[run_one_experiment] method=$METHOD pert=($OX,$OY,$OT) run=$RUN_ID duration=${DURATION}s log=$LOG_FILE"

# 启动 roslaunch 在后台，最长 240s 让 Gazebo 起来（超时则终止）
(
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
  timeout --signal=INT 240 roslaunch multi_floor_nav nav.launch \
    use_region_init:="$USE_REGION" \
    use_ap_amcl:="$USE_AP_AMCL" \
    use_covariance_reloc:="$USE_COVARIANCE_RELOC" \
    init_pose_offset_x:="$OX" init_pose_offset_y:="$OY" init_pose_offset_theta:="$OT" \
    gui:=false headless:=true \
    2>&1 | tee "$LOG_FILE"
) &
LAUNCH_PID=$!

# 等待 /map_level_0 出现（表示地图与节点在运行）
echo "[run_one_experiment] Waiting for /map_level_0 (up to 90s)..."
for i in $(seq 1 90); do
  if rostopic list 2>/dev/null | grep -q map_level_0; then
    echo "[run_one_experiment] /map_level_0 ready at ${i}s"
    break
  fi
  sleep 1
  if [ $i -eq 90 ]; then
    echo "[run_one_experiment] Timeout waiting for /map_level_0"
    kill -INT $LAUNCH_PID 2>/dev/null || true
    wait $LAUNCH_PID 2>/dev/null || true
    exit 1
  fi
done
# 等 Gazebo/仿真稳定
echo "[run_one_experiment] Waiting 30s for Gazebo/sim to stabilize..."
sleep 30
# 必须等 change_map 服务可用再发 /start，否则会一直 "Failed to load map"
echo "[run_one_experiment] Waiting for /change_map service (up to 30s)..."
for i in $(seq 1 30); do
  if rosservice list 2>/dev/null | grep -q change_map; then
    echo "[run_one_experiment] /change_map ready at ${i}s"
    break
  fi
  sleep 1
done
sleep 3

# 发送 /start 启动状态机
echo "[run_one_experiment] Publishing /start"
rostopic pub -1 /start std_msgs/Empty "{}" || true

# 运行固定时长
echo "[run_one_experiment] Running for ${DURATION}s..."
sleep "$DURATION"

# 优雅结束 roslaunch
echo "[run_one_experiment] Stopping launch (SIGINT)..."
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 3
kill -9 $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true
echo "[run_one_experiment] Done. Log: $LOG_FILE"
