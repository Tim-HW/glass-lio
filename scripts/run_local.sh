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

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"    # .../src/glasslio
WS="$(cd "${PKG_DIR}/../.." && pwd)"                          # .../lidar_ws

BAG="${PKG_DIR}/data"
RVIZ_CFG="${PKG_DIR}/rviz/glasslio.rviz"
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

# Start a child in ITS OWN PROCESS GROUP, so cleanup can kill the whole tree.
#
# `ros2 launch` forks the node as a grandchild, so killing the launch PID alone leaves the
# node running. The old fix was `pkill -f glasslio_node` -- which matches BY NAME and
# therefore kills EVERY glasslio_node on the machine, including one belonging to a
# concurrent run. Its comment claimed it "only ever kills our own processes". It did not,
# and it silently shot down other runs' nodes (they die with exit -9, which reads like a
# crash or an OOM and sends you hunting a bug that is not there).
#
# Process groups let us kill exactly our own tree and nothing else.
start_bg() {
  setsid "$@" &
  PIDS+=($!)
}

cleanup() {
  echo
  echo "[run_bag] shutting down..."
  for p in "${PIDS[@]:-}"; do
    kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true   # group, then PID
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Do NOT pkill leftovers by name -- that is the bug described above. Warn instead, and let
# the operator decide: a running node here is just as likely to be someone else's.
if pgrep -x glasslio_node >/dev/null 2>&1; then
  echo "[run_bag] warning: a glasslio_node is already running (pid $(pgrep -x glasslio_node | tr '\n' ' '))." >&2
  echo "[run_bag]          it is NOT killed automatically. Two publishers on one domain look" >&2
  echo "[run_bag]          exactly like 'time jumped backwards -- source restarted'." >&2
fi

echo "[run_bag] domain=${ROS_DOMAIN_ID}  bag=${BAG}  rate=${RATE} ${LOOP}"

# NOTE: the node broadcasts odom->livox_frame itself (stage [5], registration). The
# static_transform_publisher stopgap that used to live here was REMOVED -- two publishers
# of the same TF fight each other.

start_bg ros2 launch glasslio glasslio.launch.py

if [[ "$USE_RVIZ" -eq 1 ]]; then
  if [[ -f "$RVIZ_CFG" ]]; then
    start_bg rviz2 -d "$RVIZ_CFG"
  else
    start_bg rviz2
  fi
fi

sleep 3   # let the node subscribe before the first scan lands

# shellcheck disable=SC2086
start_bg ros2 bag play "$BAG" --rate "$RATE" $LOOP
BAG_PID="${PIDS[-1]}"

echo "[run_bag] running. Ctrl-C to stop."
echo "[run_bag] topics: /glasslio_node/{deskewed,downsampled,local_map}"

wait "$BAG_PID"
echo "[run_bag] bag finished. Ctrl-C to close RViz."
[[ "$USE_RVIZ" -eq 1 ]] && wait   # keep RViz open to inspect the result
