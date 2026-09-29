#!/usr/bin/env bash
# Restart only drive_supervisor with the arguments the launcher used
# (supervisor_args= in ~/laksa_run/latest/session.txt), with
# actuation_enabled set to $1.  Everything else keeps running.
#
#   restart_supervisor.sh false|true [--dry-run]
#
# --dry-run prints the command it would run and changes nothing.
#
# Refuses if anything other than the supervisor publishes /laksa/command or
# /laksa/brake, before stopping it or after it has stopped.
set -uo pipefail

ACTUATION="${1:-}"
DRY_RUN="${2:-}"
if [[ "${ACTUATION}" != "true" && "${ACTUATION}" != "false" ]] || [[ -n "${DRY_RUN}" && "${DRY_RUN}" != "--dry-run" ]]; then
    echo "usage: $0 false|true [--dry-run]" >&2; exit 64
fi
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${HOME}/laksa_run"
PID_FILE="${RUN_DIR}/supervisor.pid"
SESSION="$(readlink -f "${RUN_DIR}/latest" || true)"
SESSION_FILE="${SESSION}/session.txt"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
source /opt/ros/humble/setup.bash
[[ -f "${HOME}/zed_ws/install/setup.bash" ]] && source "${HOME}/zed_ws/install/setup.bash"
source "${HOME}/third_party/third_party_ws/install/setup.bash"
source "${HOME}/laksa_ws/install/setup.bash"
set -u

die() { echo "restart_supervisor: $*" >&2; exit 1; }

[[ -f "${PID_FILE}" ]] || die "no ${PID_FILE}: is the launcher running?"
[[ -f "${SESSION_FILE}" ]] || die "no ${SESSION_FILE}"
line="$(grep '^supervisor_args=' "${SESSION_FILE}" | tail -1)"
[[ -n "${line}" ]] || die "${SESSION_FILE} has no supervisor_args= line (session started by an older launcher?)"
eval "ARGS=(${line#supervisor_args=})"
[[ ${#ARGS[@]} -gt 0 ]] || die "empty supervisor_args"

# Same arguments, actuation_enabled replaced in place.
found=0
for i in "${!ARGS[@]}"; do
    if [[ "${ARGS[$i]}" == actuation_enabled:=* ]]; then ARGS[$i]="actuation_enabled:=${ACTUATION}"; found=1; fi
done
[[ ${found} -eq 1 ]] || ARGS+=(-p "actuation_enabled:=${ACTUATION}")
if [[ -n "${DRY_RUN}" ]]; then
    echo "dry run: would restart pgid $(cat "${PID_FILE}") as:"
    echo "  ros2 run laksa_bringup drive_supervisor_node.py --ros-args $(printf '%q ' "${ARGS[@]}")"
    exit 0
fi

publishers() {   # prints "<topic> <count> <nodes>" per topic
    python3 "${HERE}/topic_publishers.py" /laksa/command /laksa/brake
}

echo "before:"; before="$(publishers)"; echo "${before}" | sed 's/^/  /'
while read -r topic count nodes; do
    if [[ "${count}" -gt 1 || ( "${count}" -eq 1 && "${nodes}" != "drive_supervisor" ) ]]; then
        die "refusing: ${topic} has publisher(s) other than drive_supervisor: ${nodes}"
    fi
done <<< "${before}"

pid="$(cat "${PID_FILE}")"
if kill -0 -- "-${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 -- "-${pid}" 2>/dev/null || break; sleep 0.5; done
    if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null
        for _ in $(seq 1 6); do kill -0 -- "-${pid}" 2>/dev/null || break; sleep 0.5; done
    fi
    kill -0 -- "-${pid}" 2>/dev/null && die "supervisor process group ${pid} did not exit; not starting a second one"
    echo "stopped supervisor (pgid ${pid})"
else
    echo "supervisor (pgid ${pid}) was not running"
fi
rm -f "${PID_FILE}"

echo "after stop:"; after="$(publishers)"; echo "${after}" | sed 's/^/  /'
while read -r topic count nodes; do
    [[ "${count}" -eq 0 ]] || die "refusing to start: ${topic} still has publisher(s): ${nodes}"
done <<< "${after}"

echo "=== restart $(date -Is) actuation_enabled:=${ACTUATION}" >> "${SESSION}/supervisor.log"
nohup setsid ros2 run laksa_bringup drive_supervisor_node.py --ros-args "${ARGS[@]}" \
    >> "${SESSION}/supervisor.log" 2>&1 < /dev/null &
echo $! > "${PID_FILE}"
echo "started supervisor (pid $(cat "${PID_FILE}")): ros2 run laksa_bringup drive_supervisor_node.py --ros-args ${ARGS[*]}"

for _ in $(seq 1 20); do
    ros2 param get /drive_supervisor actuation_enabled >/dev/null 2>&1 && break
    sleep 0.5
done
echo "actuation_enabled:   $(ros2 param get /drive_supervisor actuation_enabled 2>&1)"
echo "navigation_max_erpm: $(ros2 param get /drive_supervisor navigation_max_erpm 2>&1)"
echo "now:"; publishers | sed 's/^/  /'
