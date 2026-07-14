#!/usr/bin/env bash
#
# Build and enter the glasslio dev container, without VS Code.
#
#   ./docker/run.sh              build if needed, then drop into a shell
#   ./docker/run.sh --rebuild    force a rebuild of the image
#   ./docker/run.sh <cmd...>     run one command in the container and exit
#
# Examples:
#   ./docker/run.sh colcon build --packages-select glasslio
#   ./docker/run.sh colcon test  --packages-select glasslio
#   ./docker/run.sh ./src/glasslio/scripts/run_bag.sh -n
#
# The repo is mounted at /ws/src/glasslio, i.e. as the src/ of a colcon workspace at
# /ws -- so builds behave exactly as they do on a normal machine. Build artefacts land
# in /ws/build and /ws/install INSIDE the container, so they never collide with a host
# build of the same tree.
#
# This is a thin convenience wrapper. docker/Dockerfile is a plain image with no
# VS Code specifics: use it directly, or in your own compose file, if you prefer.

set -euo pipefail

readonly IMAGE="glasslio-dev"
readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

log() { printf '\033[1;34m[docker]\033[0m %s\n' "$*"; }

REBUILD=0
if [[ "${1:-}" == "--rebuild" ]]; then
  REBUILD=1
  shift
fi

if [[ "$REBUILD" -eq 1 ]] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  log "building $IMAGE (first run takes a few minutes)..."
  docker build -f "${REPO_DIR}/docker/Dockerfile" -t "$IMAGE" "$REPO_DIR"
fi

# GUI (RViz) on a Linux host. Harmless if there is no X server -- use run_bag.sh -n.
X11_ARGS=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  X11_ARGS=(--env "DISPLAY=${DISPLAY}" --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  # Let the container talk to the host X server. Narrow: local connections only.
  command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null 2>&1 || true
fi

# --net=host + --ipc=host: DDS discovery and rosbag2/RViz shared-memory transfers are
# painful otherwise. ROS_DOMAIN_ID keeps us off domain 0, where a stray nav2 stack floods
# discovery, wedges the ros2 CLI daemon, and presents as "the node is hung".
exec docker run --rm -it \
  --net=host \
  --ipc=host \
  --env ROS_DOMAIN_ID=42 \
  "${X11_ARGS[@]}" \
  --volume "${REPO_DIR}:/ws/src/glasslio:rw" \
  --workdir /ws \
  "$IMAGE" \
  "${@:-bash}"
