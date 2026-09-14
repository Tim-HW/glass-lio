#!/usr/bin/env bash
# Run glasslio on the 3-LiDAR ASDT1 rig (transfer-01a041b7 apartment bags), registering the
# fused /combined_point_cloud driven by the CENTER IMU. Uses config/3lidars.yaml.
#
#   ./scripts/run_3lidars.sh                     # 3lidars-apartment2 (the best: 1.18x), RViz
#   ./scripts/run_3lidars.sh 3lidars-apartment   # 1.46x
#   ./scripts/run_3lidars.sh 3lidars-apartment3 -n   # 3.06x, headless
#   ./scripts/run_3lidars.sh 3lidars-apartment -r 2  # 2x speed
#
# WHY the fused cloud: three 30-deg ToF FOVs stitched together see enough geometry that the
# in-plane translation one narrow sensor cannot observe becomes observable -- HALVING drift vs
# a single sensor. Loose coupling is optimal here (the IMU only adds noise; tight converges to
# loose without beating it). Still not metric SLAM, but the best ASDT1 result in the repo.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "${PKG_DIR}/../.." && pwd)"
CFG="${PKG_DIR}/config/3lidars.yaml"
RVIZ_CFG="${PKG_DIR}/rviz/asdt.rviz"          # Fixed Frame odom; shows /glasslio_node/local_map
DATA_DIR="${PKG_DIR}/data/transfer-01a041b7"

BAG_NAME="3lidars-apartment2"
RATE=1.0
USE_RVIZ=1
: "${ROS_DOMAIN_ID:=59}"
export ROS_DOMAIN_ID

while [[ $# -gt 0 ]]; do
  case "$1" in
    -r) RATE="$2"; shift 2 ;;
    -n) USE_RVIZ=0; shift ;;
    -d) export ROS_DOMAIN_ID="$2"; shift 2 ;;
    -h) sed -n '2,17p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*) echo "error: unknown flag '$1'" >&2; exit 1 ;;
    *)  BAG_NAME="$1"; shift ;;
  esac
done

BAG="${DATA_DIR}/${BAG_NAME}"
if [[ ! -d "$BAG" ]]; then
  echo "error: bag '${BAG_NAME}' not found under ${DATA_DIR}" >&2
  echo "       available:" >&2
  (ls -d "${DATA_DIR}"/*/ 2>/dev/null | sed 's,/$,,;s,.*/,         ,') >&2
  exit 1
fi

[[ -f "${WS}/install/setup.bash" ]] || {
  echo "error: ${WS}/install/setup.bash missing -- run 'colcon build' first" >&2; exit 1; }

set +u
# shellcheck disable=SC1090,SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090,SC1091
source "${WS}/install/setup.bash"
set -u

PIDS=()
start_bg() { setsid "$@" & PIDS+=($!); }
cleanup() {
  echo; echo "[run_3lidars] shutting down..."
  for p in "${PIDS[@]:-}"; do
    kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run_3lidars] domain=${ROS_DOMAIN_ID}  bag=${BAG}  rate=${RATE}"
echo "[run_3lidars] fused cloud /combined_point_cloud + center IMU (loose)."

start_bg ros2 run glasslio glasslio_node --ros-args --params-file "$CFG"

if [[ "$USE_RVIZ" -eq 1 ]]; then
  [[ -f "$RVIZ_CFG" ]] && start_bg rviz2 -d "$RVIZ_CFG" || start_bg rviz2
fi

sleep 3
# shellcheck disable=SC2086
start_bg ros2 bag play "$BAG" --rate "$RATE"
BAG_PID="${PIDS[-1]}"

echo "[run_3lidars] running. map on /glasslio_node/local_map (odom frame). Ctrl-C to stop."
wait "$BAG_PID"
echo "[run_3lidars] bag finished. Ctrl-C to close RViz."
[[ "$USE_RVIZ" -eq 1 ]] && wait
