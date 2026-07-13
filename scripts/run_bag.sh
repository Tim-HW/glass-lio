#!/usr/bin/env bash
# Launch the LIO node + rosbag + RViz2, on an isolated ROS domain.
#
# The domain matters: a nav2/slam_toolbox stack on domain 0 floods DDS discovery,
# wedges the ros2 CLI daemon, and makes this look like the node is hanging.
#
#   ./scripts/run_bag.sh              # default bag, rate 1.0, with RViz
#   ./scripts/run_bag.sh -r 0.5       # slow motion
#   ./scripts/run_bag.sh -l           # loop the bag
#   ./scripts/run_bag.sh -n           # no RViz (headless)
#   ./scripts/run_bag.sh -b /path/to/bag -d 7
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"    # .../src/lidar-odom
WS="$(cd "${PKG_DIR}/../.." && pwd)"                          # .../lidar_ws

BAG="${PKG_DIR}/data"
RVIZ_CFG="${PKG_DIR}/rviz/lio.rviz"
RATE=1.0
LOOP=""
USE_RVIZ=1
: "${ROS_DOMAIN_ID:=42}"
export ROS_DOMAIN_ID

while getopts "b:r:d:lnh" opt; do
  case "$opt" in
    b) BAG="$OPTARG" ;;
    r) RATE="$OPTARG" ;;
    d) export ROS_DOMAIN_ID="$OPTARG" ;;
    l) LOOP="--loop" ;;
    n) USE_RVIZ=0 ;;
    h) sed -n '2,11p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) exit 1 ;;
  esac
done

[[ -d "$BAG" ]] || { echo "error: bag not found: $BAG" >&2; exit 1; }

[[ -f "${WS}/install/setup.bash" ]] || {
  echo "error: ${WS}/install/setup.bash missing -- run 'colcon build' first" >&2
  exit 1
}

# ROS setup scripts reference unbound vars; -u must be off while sourcing them.
set +u
# shellcheck disable=SC1090,SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090,SC1091
source "${WS}/install/setup.bash"
set -u

PIDS=()

cleanup() {
  echo
  echo "[run_bag] shutting down..."
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
  # Only ever kill our own processes, never the user's other ROS stacks.
  pkill -9 -f 'lio_node' 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

pkill -9 -f 'lio_node' 2>/dev/null || true   # leftovers from a previous run
sleep 0.5

echo "[run_bag] domain=${ROS_DOMAIN_ID}  bag=${BAG}  rate=${RATE} ${LOOP}"

# STOPGAP: the node does not publish odom->livox_frame yet -- that TF is produced
# by registration (Phase 2, stage 3). Until then pose_ IS identity, so a static
# identity TF is truthful and lets RViz place the sensor-frame clouds against the
# world-frame map. DELETE THIS once the node broadcasts its own TF.
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
  --frame-id odom --child-frame-id livox_frame > /dev/null 2>&1 &
PIDS+=($!)

ros2 launch lidar-odom lio.launch.py &
PIDS+=($!)

if [[ "$USE_RVIZ" -eq 1 ]]; then
  if [[ -f "$RVIZ_CFG" ]]; then
    rviz2 -d "$RVIZ_CFG" > /dev/null 2>&1 &
  else
    rviz2 > /dev/null 2>&1 &
  fi
  PIDS+=($!)
fi

sleep 3   # let the node subscribe before the first scan lands

# shellcheck disable=SC2086
ros2 bag play "$BAG" --rate "$RATE" $LOOP &
BAG_PID=$!
PIDS+=("$BAG_PID")

echo "[run_bag] running. Ctrl-C to stop."
echo "[run_bag] topics: /lio_node/{deskewed,downsampled,local_map}"

wait "$BAG_PID"
echo "[run_bag] bag finished. Ctrl-C to close RViz."
[[ "$USE_RVIZ" -eq 1 ]] && wait   # keep RViz open to inspect the result
