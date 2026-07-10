#!/bin/bash
# Fair A/B latency benchmark: MQ baseline vs eventfd (#1432), using the
# self-contained agnocast_bench probe.
#
# Both measured branches share base #1426; the ONLY functional difference is the
# publish-notification mechanism (MQ mq_send x N  vs  one in-ioctl eventfd signal).
# For each branch this clean-builds agnocast, builds the probe into that branch's
# install via --paths (so the probe is never committed to the measured branches
# and nothing vanishes on `git switch`), loads the matching kmod, then sweeps the
# subscriber count and records publish/e2e latency.
#
# Run it from anywhere the package is checked out, e.g.:
#   bash ~/agnocast/src/agnocast_bench/run_ab.sh
set -uo pipefail

# This script drives `git switch`, so it must not execute from inside the repo's
# package dir (which disappears on switch). Relocate a copy to a temp dir and
# re-exec from there; that copy is also the --paths source for the probe build.
if [[ "${AGNO_BENCH_RELOCATED:-}" != "1" ]]; then
  SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  TMP="$(mktemp -d)/agnocast_bench"
  cp -r "$SELF_DIR" "$TMP"
  export AGNO_BENCH_RELOCATED=1 AGNO_BENCH_PKG="$TMP"
  exec bash "$TMP/run_ab.sh" "$@"
fi
PKG="$AGNO_BENCH_PKG"

AGNO=~/agnocast
RATE=100; WARMUP=2; MEASURE=10; QOS=10
SUBS=(1 2 4 8 16 32)          # fan-out sweep (32 = nproc here)
STAMP=$(date +%Y%m%d_%H%M%S)
RESULTS=~/eventfd_bench_${STAMP}
BASE_BRANCH="bench/base"
EVENTFD_BRANCH="feat/eventfd-publish-notification-v2"

sudo -v || { echo "sudo required"; exit 1; }
( while true; do sudo -n true; sleep 60; done ) & KA=$!
trap 'kill $KA 2>/dev/null' EXIT
sudo sysctl -w fs.mqueue.queues_max=4096 >/dev/null   # MQ baseline: 1 MQ / subscriber

run_branch() {
  local branch="$1" tag="$2"
  echo "==================== $branch ($tag) ===================="
  cd "$AGNO" || exit 1
  if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "ERROR: $AGNO has tracked changes; commit/stash first."; exit 1
  fi
  git switch "$branch" || exit 1

  rm -rf build install
  bash scripts/dev/build_all.bash > "/tmp/ba_${tag}.log" 2>&1
  if grep -qE "^Failed +<<<|package(s)? (failed|aborted)" "/tmp/ba_${tag}.log"; then
    echo "ERROR: agnocast build failed on $branch (see /tmp/ba_${tag}.log)"; exit 1
  fi

  sudo rmmod agnocast 2>/dev/null || true
  sudo insmod agnocast_kmod/agnocast.ko || { echo "insmod failed"; exit 1; }

  # ROS setup scripts reference unbound vars, so relax nounset around all sourcing.
  set +u
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  source "$AGNO/install/setup.bash" >/dev/null 2>&1
  set -u
  # Build the probe against THIS branch's agnocastlib, from the relocated copy.
  colcon build --symlink-install --packages-select agnocast_bench --paths "$PKG" \
    > "/tmp/probe_${tag}.log" 2>&1 || { echo "probe build failed (see /tmp/probe_${tag}.log)"; exit 1; }
  set +u; source "$AGNO/install/setup.bash" >/dev/null 2>&1; set -u

  local HH="$AGNO/install/agnocastlib/lib/libagnocast_heaphook.so"
  local PUB="$AGNO/install/agnocast_bench/lib/agnocast_bench/bench_pub"
  local SUB="$AGNO/install/agnocast_bench/lib/agnocast_bench/bench_sub"

  for N in "${SUBS[@]}"; do
    local od="${RESULTS}/${tag}/N${N}"; mkdir -p "$od"
    echo "--- $tag  N=$N ---"
    for s in $(seq 0 $((N - 1))); do
      LD_PRELOAD="$HH" "$SUB" --ros-args \
        -p rate_hz:=$RATE -p warmup_sec:=$WARMUP -p qos_depth:=$QOS \
        -p output:="$od/sub_$s.csv" > "$od/sub_$s.log" 2>&1 &
    done
    sleep 3
    LD_PRELOAD="$HH" timeout 180 "$PUB" --ros-args \
      -p rate_hz:=$RATE -p warmup_sec:=$WARMUP -p measure_sec:=$MEASURE \
      -p num_subs:=$N -p qos_depth:=$QOS -p output:="$od/pub.csv" \
      > "$od/pub.log" 2>&1
    sleep 3
    pkill -9 -x bench_sub 2>/dev/null || true
    pkill -9 -x bench_pub 2>/dev/null || true
    sleep 1
    rm -f /dev/shm/agnocast@* /dev/mqueue/agnocast* 2>/dev/null || true
    sleep 1
  done
}

mkdir -p "$RESULTS"
run_branch "$BASE_BRANCH"    "mq_base"
run_branch "$EVENTFD_BRANCH" "eventfd"

echo ""
echo "==================== DONE ===================="
echo "Results: $RESULTS"
python3 "$PKG/analyze.py" "$RESULTS"
echo "NOTE: both branches carry the TEMP instrumentation commit; drop it before pushing #1432."
