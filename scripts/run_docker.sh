#!/usr/bin/env bash
#
# One command, everything in Docker: build the image, fetch the test bag (once), build
# glasslio, then launch the node + bag [+ RViz] -- all inside the container. No local ROS
# install required.
#
#   ./scripts/run_docker.sh            # node + bag + RViz (Mesa software GL)
#   ./scripts/run_docker.sh -n         # headless (no RViz)
#   ./scripts/run_docker.sh -r 0.5     # every run_local.sh flag is forwarded (-r -l -d -b)
#
# The bag lands in data/ on your HOST -- data/ is bind-mounted, so it downloads only once
# (~1.4 GB) and survives across runs. The colcon build, by contrast, lives INSIDE the
# container and is ephemeral: it reruns each invocation (~40 s). That is exactly why
# download + build + run all happen in ONE container session below -- a separate
# `docker/run.sh colcon build` would build into a container that is then thrown away, and
# run_bag would find no overlay to source.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"    # .../src/glasslio

# Hand the whole flow to the docker wrapper as a single container command. run.sh builds
# the image if needed and wires host networking + X11; the script below runs at /ws.
# Args after `run_docker` become $@ inside, and are forwarded to run_local.sh.
exec "${PKG_DIR}/docker/run.sh" bash -lc '
  set -euo pipefail
  cd /ws
  ./src/glasslio/scripts/download_bag.sh                 # idempotent: skips if already here
  colcon build --packages-select glasslio
  exec ./src/glasslio/scripts/run_local.sh "$@"          # sources ROS + the overlay itself

' run_docker "$@"
