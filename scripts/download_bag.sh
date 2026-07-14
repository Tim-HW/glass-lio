#!/usr/bin/env bash
#
# Fetch the test bag glasslio is developed against, into data/.
#
#   ./scripts/download_bag.sh              download, unzip, verify, clean up
#   ./scripts/download_bag.sh --force      re-download even if the bag is here
#   ./scripts/download_bag.sh --keep-zip   keep the archive afterwards
#
# ~1.4 GB extracted, which is why it is not in git (data/.gitignore excludes it).
# Source: https://zenodo.org/records/14841855
#
# Safe to re-run: if the bag is already present and its checksum matches, this
# does nothing.

set -euo pipefail

readonly URL="https://zenodo.org/records/14841855/files/rosbag2_2024_04_16-14_17_01.zip?download=1"
readonly ZIP="rosbag2_2024_04_16-14_17_01.zip"
readonly DB3="rosbag2_2024_04_16-14_17_01_0.db3"
readonly META="metadata.yaml"

# Integrity check for the extracted bag. A truncated or corrupted download is by
# far the most likely failure here -- and it would surface as a baffling estimator
# bug rather than an obvious error. So we check, rather than hope.
readonly DB3_SHA256="3bbd390a97e57af47ad6699baa36eb4c5f39f61b35275505ecaf221c126354f5"

readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DATA_DIR="${REPO_DIR}/data"

FORCE=0
KEEP_ZIP=0
for arg in "$@"; do
  case "$arg" in
    --force)    FORCE=1 ;;
    --keep-zip) KEEP_ZIP=1 ;;
    -h|--help)  sed -n '3,9p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
  esac
done

log()  { printf '\033[1;34m[bag]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[bag]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[bag]\033[0m %s\n' "$*" >&2; exit 1; }

mkdir -p "$DATA_DIR"
cd "$DATA_DIR"

verify() {
  # True only if the bag is present AND intact.
  [[ -f "$DB3" && -f "$META" ]] || return 1
  log "verifying checksum (1.4 GB, takes a few seconds)..."
  echo "${DB3_SHA256}  ${DB3}" | sha256sum --check --status
}

# --- already have it? ------------------------------------------------------
if [[ "$FORCE" -eq 0 ]] && verify; then
  log "bag already present and verified. Nothing to do."
  log "run:  ./scripts/run_bag.sh"
  exit 0
fi

if [[ -f "$DB3" && "$FORCE" -eq 0 ]]; then
  warn "a bag is present but FAILED its checksum -- corrupt or truncated. Re-downloading."
fi

# --- fetch -----------------------------------------------------------------
# curl or wget, whichever exists. Both RESUME a partial transfer, which matters a
# great deal for 1.4 GB on a flaky connection: an interrupted run is fixed by
# simply re-running this script, not by starting over.
command -v unzip >/dev/null 2>&1 || die "need 'unzip' (sudo apt install unzip)."

if command -v curl >/dev/null 2>&1; then
  log "downloading ~1.4 GB from Zenodo (resumable -- re-run if interrupted)..."
  curl -L --fail --progress-bar -C - -o "$ZIP" "$URL" \
    || die "download failed. Re-run to resume from where it stopped."
elif command -v wget >/dev/null 2>&1; then
  log "downloading ~1.4 GB from Zenodo (resumable -- re-run if interrupted)..."
  wget --continue --show-progress -O "$ZIP" "$URL" \
    || die "download failed. Re-run to resume from where it stopped."
else
  die "need either curl or wget."
fi

# --- extract ---------------------------------------------------------------
log "extracting..."
unzip -o -q "$ZIP" -d .

# The archive may or may not wrap the files in a directory. Handle both, so the
# bag always lands flat in data/, which is where run_bag.sh looks for it.
if [[ ! -f "$DB3" ]]; then
  nested="$(find . -mindepth 2 -name "$DB3" -print -quit)"
  [[ -n "$nested" ]] || die "archive does not contain $DB3"
  nested_dir="$(dirname "$nested")"
  log "flattening ${nested_dir#./}/ into data/"
  mv -f "$nested_dir"/* .
  rmdir "$nested_dir" 2>/dev/null || true
fi

# --- verify ----------------------------------------------------------------
verify || die "checksum MISMATCH after extraction -- the download is corrupt. Re-run with --force."

if [[ "$KEEP_ZIP" -eq 0 ]]; then
  rm -f "$ZIP"
  log "removed the archive (keep it with --keep-zip)"
fi

log "done. $(du -h "$DB3" | cut -f1) bag, verified."
log "run:  ./scripts/run_bag.sh"
