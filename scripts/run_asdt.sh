#!/usr/bin/env bash
# Run glasslio on the ASDT1 ToF bags (tim_house, tim_street, bedroom, kitchen, office...),
# with RViz, on an isolated ROS domain. Uses config/asdt1.yaml (loose coupling + the
# sparse-cloud tuning this sensor needs) and rviz/asdt.rviz.
#
#   ./scripts/run_asdt.sh                 # tim_house, rate 1, with RViz
#   ./scripts/run_asdt.sh tim_street      # a different bag
#   ./scripts/run_asdt.sh bedroom -n      # headless
#   ./scripts/run_asdt.sh tim_house -r 2  # 2x speed
#   ./scripts/run_asdt.sh tim_house -l    # loop
#   ./scripts/run_asdt.sh tim_house --tight   # tight coupling instead of loose -- watch it
#                                             # FREEZE (imu_prior_weight=1.0; see below)
#
# HONEST CAVEAT: this is a 30 deg, texture-less ToF -- in-plane translation is not
# observable, so the map DRIFTS (tim_house least, the others more). It runs and builds a
# recognisable-ish cloud; it does not do metric SLAM. See config/asdt1.yaml.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"    # .../src/glasslio
WS="$(cd "${PKG_DIR}/../.." && pwd)"                          # .../lidar_ws
CFG="${PKG_DIR}/config/asdt1.yaml"
RVIZ_CFG="${PKG_DIR}/rviz/asdt.rviz"

BAG_NAME="tim_house"
RATE=1.0
LOOP=""
USE_RVIZ=1
TIGHT_ARGS=()
: "${ROS_DOMAIN_ID:=43}"
export ROS_DOMAIN_ID

# First non-flag argument is the bag name; flags may come before or after it.
while [[ $# -gt 0 ]]; do
  case "$1" in
    -r) RATE="$2"; shift 2 ;;
    -n) USE_RVIZ=0; shift ;;
    -l) LOOP="--loop"; shift ;;
    --tight) TIGHT_ARGS=(-p registration.imu_prior_weight:=1.0); shift ;;
    -d) export ROS_DOMAIN_ID="$2"; shift 2 ;;
    -h) sed -n '2,17p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*) echo "error: unknown flag '$1'" >&2; exit 1 ;;
    *)  BAG_NAME="$1"; shift ;;
  esac
done

# Resolve the bag: data/<name>, or data/Lidar_Bags/<name>.
if [[ -d "${PKG_DIR}/data/${BAG_NAME}" ]]; then
  BAG="${PKG_DIR}/data/${BAG_NAME}"
elif [[ -d "${PKG_DIR}/data/Lidar_Bags/${BAG_NAME}" ]]; then
  BAG="${PKG_DIR}/data/Lidar_Bags/${BAG_NAME}"
else
  echo "error: bag '${BAG_NAME}' not found under data/ or data/Lidar_Bags/" >&2
  echo "       available:" >&2
  (cd "${PKG_DIR}/data" 2>/dev/null && ls -d */ Lidar_Bags/*/ 2>/dev/null | sed 's,/$,,;s,^,         ,') >&2
  exit 1
fi

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

# Same process-group discipline as run_local.sh: each child in its own group, so cleanup
# kills exactly our tree and nothing else (never pkill-by-name -- that shoots down other runs).
PIDS=()
start_bg() { setsid "$@" & PIDS+=($!); }
cleanup() {
  echo
  echo "[run_asdt] shutting down..."
  for p in "${PIDS[@]:-}"; do
    kill -9 -- "-${p}" 2>/dev/null || kill -9 "${p}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[run_asdt] domain=${ROS_DOMAIN_ID}  bag=${BAG}  rate=${RATE} ${LOOP}"

# The node reads config/asdt1.yaml directly (topics, m/s^2 accel, loose coupling, sparse
# tuning) -- glasslio.launch.py targets the Livox, so we run the node ourselves.
[[ ${#TIGHT_ARGS[@]} -gt 0 ]] && echo "[run_asdt] TIGHT coupling on -- expect the pose to FREEZE (see caveat)"
start_bg ros2 run glasslio glasslio_node --ros-args --params-file "$CFG" "${TIGHT_ARGS[@]}"

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

echo "[run_asdt] running. Ctrl-C to stop."
echo "[run_asdt] map on /glasslio_node/local_map (odom frame). Fixed Frame is odom in the rviz cfg."

wait "$BAG_PID"
echo "[run_asdt] bag finished. Ctrl-C to close RViz."
[[ "$USE_RVIZ" -eq 1 ]] && wait   # keep RViz open to inspect the result
