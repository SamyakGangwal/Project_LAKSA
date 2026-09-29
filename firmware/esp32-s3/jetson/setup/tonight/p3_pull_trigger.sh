#!/usr/bin/env bash
# P3 stop tests c/d: spin the wheel with the traction bench, then tell the
# operator to pull a cable.  The operator runs this at the car, filming.
#
#   p3_pull_trigger.sh c     # then pull the ESP32 USB-C on PULL NOW
#   p3_pull_trigger.sh d     # then pull the XT60 on PULL NOW
#
# Marks the PULL NOW time in the running vesc_logger CSV
# (~/laksa_run/p3_stops.current).  The actual pull is timed from video.
set -uo pipefail

TEST="${1:-}"
case "${TEST}" in
    c) WHAT="ESP32 USB-C cable" ;;
    d) WHAT="XT60 battery connector" ;;
    *) echo "usage: $0 c|d" >&2; exit 64 ;;
esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$(cat "${HOME}/laksa_run/p3_stops.current" 2>/dev/null)"
[[ -n "${LOG}" && -f "${LOG}" ]] || { echo "no running P3 logger CSV" >&2; exit 1; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
[[ -f "${HOME}/zed_ws/install/setup.bash" ]] && source "${HOME}/zed_ws/install/setup.bash"
source "${HOME}/third_party/third_party_ws/install/setup.bash"
source "${HOME}/laksa_ws/install/setup.bash"
set -u

pubs="$(python3 "${HERE}/topic_publishers.py" /laksa/command /laksa/brake)" || { echo "publisher check failed" >&2; exit 1; }
if [[ -z "${pubs}" ]] || echo "${pubs}" | awk '$2 != 0 {bad=1} END {exit !bad}'; then
    echo "REFUSING: another command publisher is present:" >&2; echo "${pubs}" >&2; exit 1
fi

echo "Test ${TEST}: get ready to pull the ${WHAT}. Starting the bench (wheel spins in ~3 s, for 8 s)..."
python3 "${HERE}/../traction_bench.py" --steps 0.22 --hold 8 > "${HOME}/laksa_run/p3_test_${TEST}_bench.log" 2>&1 &
BENCH=$!
if python3 "${HERE}/wait_spin.py" --target 911 --timeout 8; then
    T0="$(date +%s.%N)"
    printf '\a\n\n    >>>>>>>>>>  PULL NOW: %s  <<<<<<<<<<\n\n\a' "${WHAT}"
    python3 -c "import sys; sys.path.insert(0, '${HERE}'); import vesc_logger as v; \
v._append('${LOG}', {'t': '${T0}', 'source': 'mark', 'text': 'test ${TEST}: PULL NOW prompt'})"
    echo "PULL NOW shown at $(date -d "@${T0}" +%H:%M:%S.%3N)"
else
    echo "Wheel did not reach speed: DO NOT PULL. The bench will brake on its own."
fi
wait "${BENCH}"; echo "bench exited ($?); log: ~/laksa_run/p3_test_${TEST}_bench.log"
