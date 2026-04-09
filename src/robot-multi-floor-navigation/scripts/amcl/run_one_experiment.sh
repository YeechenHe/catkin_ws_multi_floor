#!/bin/bash
# 单次初值扰动实验：启动 nav（headless）、等待就绪、发送 /start、运行固定时长、保存日志
# 用法: run_one_experiment.sh <method> <offset_x> <offset_y> <offset_theta> <run_id> [duration_sec] [log_dir]
#       [sigma_xy_L0] [sigma_yaw_L0] [sigma_xy_L1] [sigma_yaw_L1] [reloc_K_override] [max_linear_error_L1]
#  method: baseline | spamcl | apamcl（模块 B） | relocc（模块 C：重定位置信度判据）
#          sp_amcl_c（模块 A + 模块 C 联合）
# 示例: ./run_one_experiment.sh baseline 0 0 0 1 180 /tmp/exp_logs

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
# catkin_ws 为 robot-multi-floor-navigation 的上两级
source "$REPO_DIR/../../devel/setup.bash" 2>/dev/null || true
[ -n "$ROS_DISTRO" ] || { echo "Source catkin workspace first: source ~/catkin_ws/devel/setup.bash"; exit 1; }

# 每次实验开始前清理残留 ROS/Gazebo 进程，避免 spawn 冲突
cleanup_ros() {
    local pattern="gzserver|gzclient|rosmaster|roscore|multi_floor_navigation_node|move_base|map_server|change_map_node|rviz|robot_state_publisher|ekf_localization|twist_mux|twist_marker_server|urdf_spawner|controller_spawner|jackal_velocity"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    for pid in $pids; do
        # Avoid killing current script shell or parent shell.
        if [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ]; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    sleep 4
}
echo "[run_one_experiment] Cleaning up any residual ROS/Gazebo processes..."
cleanup_ros

METHOD="${1:-baseline}"
OX="${2:-0}"
OY="${3:-0}"
OT="${4:-0}"
RUN_ID="${5:-1}"
DURATION="${6:-180}"
LOG_DIR="${7:-$REPO_DIR/multi_floor_nav/experiment_logs}"
# 修复缺陷一：per-floor sigma 参数
# 参数 8/9 为 L0 的 sigma_xy/yaw（或全局 fallback）；参数 10/11 为 L1 的 sigma_xy/yaw
SIGMA_XY_L0="${8:-0.15}"
SIGMA_YAW_L0="${9:-0.1}"
SIGMA_XY_L1="${10:-$SIGMA_XY_L0}"    # 未指定时 L1 继承 L0
SIGMA_YAW_L1="${11:-$SIGMA_YAW_L0}"
RELOC_K_OVERRIDE="${12:-0}"          # 可选：覆盖 reloc_confidence_K（large 工况设 2，0=不覆盖）
MAX_LINEAR_ERROR_L1="${13:--1}"     # 可选：L1 线误差门限（m），默认 -1 与 L0 共用；>0 仅放宽 L1

mkdir -p "$LOG_DIR"
PERT_NAME="pert_${OX}_${OY}_${OT}"
LOG_FILE="$LOG_DIR/${METHOD}_${PERT_NAME}_run${RUN_ID}.log"

USE_REGION="false"
USE_AP_AMCL="false"
USE_COVARIANCE_RELOC="false"
if [ "$METHOD" = "spamcl" ] || [ "$METHOD" = "sp_amcl" ]; then USE_REGION="true"; fi
if [ "$METHOD" = "apamcl" ] || [ "$METHOD" = "ap_amcl" ]; then USE_AP_AMCL="true"; fi
if [ "$METHOD" = "relocc" ]; then USE_COVARIANCE_RELOC="true"; fi
if [ "$METHOD" = "sp_amcl_c" ]; then USE_REGION="true"; USE_COVARIANCE_RELOC="true"; fi

echo "[run_one_experiment] method=$METHOD pert=($OX,$OY,$OT) sigma_L0=($SIGMA_XY_L0,$SIGMA_YAW_L0) sigma_L1=($SIGMA_XY_L1,$SIGMA_YAW_L1) K_override=$RELOC_K_OVERRIDE max_linear_error_L1=$MAX_LINEAR_ERROR_L1 run=$RUN_ID duration=${DURATION}s log=$LOG_FILE"

# roslaunch 须在「等待 map + 稳定 + /start 后 sleep DURATION」整段期间保持存活。
# 旧版固定 timeout 240 会在 DURATION=240 时过早 SIGINT，导致日志截断、实验未完成。
LAUNCH_TIMEOUT_SEC=$(( DURATION + 240 ))
(
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
  timeout --signal=INT "$LAUNCH_TIMEOUT_SEC" roslaunch multi_floor_nav nav.launch \
    use_region_init:="$USE_REGION" \
    use_ap_amcl:="$USE_AP_AMCL" \
    use_covariance_reloc:="$USE_COVARIANCE_RELOC" \
    reloc_confidence_K_override:="$RELOC_K_OVERRIDE" \
    max_linear_error_L1:="$MAX_LINEAR_ERROR_L1" \
    init_pose_offset_x:="$OX" init_pose_offset_y:="$OY" init_pose_offset_theta:="$OT" \
    init_sigma_xy:="$SIGMA_XY_L0" init_sigma_yaw:="$SIGMA_YAW_L0" \
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

# 优雅结束 roslaunch，再强制清理所有残留进程
echo "[run_one_experiment] Stopping launch (SIGINT)..."
kill -INT $LAUNCH_PID 2>/dev/null || true
sleep 3
kill -9 $LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true
echo "[run_one_experiment] Cleaning up residual processes after run..."
cleanup_ros
echo "[run_one_experiment] Done. Log: $LOG_FILE"
