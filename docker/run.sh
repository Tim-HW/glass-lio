#!/usr/bin/env bash
#
# Build and enter the glasslio dev container. Thin wrapper over docker/docker-compose.yml:
# the compose file defines the container (bind-mounted repo, host networking, X11); this
# script just auto-builds the image on first use and forwards your command.
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
# The repo is bind-mounted at /ws/src/glasslio (the src/ of a colcon workspace at /ws), so
# builds behave as on a normal machine. build/ install/ log/ stay INSIDE the container.
# RViz uses Mesa's software GL (set in the compose file) -- no GPU passthrough.
#
# You run as the `ubuntu` user (uid 1000), not root -- so files created in the mounted repo
# are owned by you on the host. Passwordless sudo is available inside for the rare root need.

set -euo pipefail

readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly BASE="${REPO_DIR}/docker/docker-compose.yml"

log() { printf '\033[1;34m[docker]\033[0m %s\n' "$*"; }

REBUILD=0
if [[ "${1:-}" == "--rebuild" ]]; then
  REBUILD=1
  shift
fi

# Build the image if it is missing, or when forced.
if [[ "$REBUILD" -eq 1 ]] || ! docker image inspect glasslio-dev >/dev/null 2>&1; then
  log "building glasslio-dev (first run takes a few minutes)..."
  docker compose -f "$BASE" build
fi

export DISPLAY="${DISPLAY:-}"

# GUI (RViz): let the container reach the host X server. Harmless with no X server running
# -- use `run_bag.sh -n` (headless) in that case.
if [[ -n "$DISPLAY" && -d /tmp/.X11-unix ]]; then
  command -v xhost >/dev/null 2>&1 && xhost +local:docker >/dev/null 2>&1 || true
fi

# `run --rm` is a fresh, self-removing container per invocation. With no command, a shell.
exec docker compose -f "$BASE" run --rm glasslio "${@:-bash}"
