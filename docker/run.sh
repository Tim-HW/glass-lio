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

# --- GUI (RViz) ------------------------------------------------------------------------
# Harmless if there is no X server -- in that case use `run_bag.sh -n` (headless).
GUI_ARGS=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  GUI_ARGS+=(--env "DISPLAY=${DISPLAY}" --volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
  # Let the container talk to the host X server. Narrow: local connections only.
  command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null 2>&1 || true

  # THE GPU. Without /dev/dri the container has no hardware GL, and RViz dies with
  #
  #     MESA: error: Failed to query drm device.
  #     glx: failed to create dri3 screen
  #     failed to load driver: iris
  #
  # which reads like a broken install but is really just a missing device. Pass it
  # through when the host has one -- and add the device's own group, or the container
  # user cannot open it (the node is root:video, mode 0660).
  if [[ -d /dev/dri ]]; then
    GUI_ARGS+=(--device /dev/dri)
    for node in /dev/dri/*; do
      [[ -c "$node" ]] || continue
      gid="$(stat -c '%g' "$node")"
      GUI_ARGS+=(--group-add "$gid")
    done
    log "GPU: passing /dev/dri through (hardware GL)"
  else
    # No GPU visible: fall back to Mesa's software rasteriser. Slow, but RViz WORKS,
    # which beats a cryptic driver error. Requires libgl1-mesa-dri, which the image has.
    GUI_ARGS+=(--env LIBGL_ALWAYS_SOFTWARE=1)
    log "GPU: none visible -- using software GL (slower, but it works)"
  fi
fi

# --net=host + --ipc=host: DDS discovery and rosbag2/RViz shared-memory transfers are
# painful otherwise. ROS_DOMAIN_ID keeps us off domain 0, where a stray nav2 stack floods
# discovery, wedges the ros2 CLI daemon, and presents as "the node is hung".
exec docker run --rm -it \
  --net=host \
  --ipc=host \
  --env ROS_DOMAIN_ID=42 \
  "${GUI_ARGS[@]}" \
  --volume "${REPO_DIR}:/ws/src/glasslio:rw" \
  --workdir /ws \
  "$IMAGE" \
  "${@:-bash}"
