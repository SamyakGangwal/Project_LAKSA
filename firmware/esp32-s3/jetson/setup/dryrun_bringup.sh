#!/usr/bin/env bash
# LAKSA bench/field bring-up: LiDAR, ZED (depth, VIO, object detection),
# perception, EKF odometry, RTAB-Map mapping, drive_supervisor, learned driver
# and the LAKSA Console.
#
# It does not start a micro-ROS Agent (the one already bridging the ESP32 is
# reused).  Nothing moves until an operator explicitly starts driving from the
# console or laksa_operator: without an operator the supervisor holds brake.
#
#   dryrun_bringup.sh start    everything, supervisor actuation DISABLED
#   dryrun_bringup.sh trial    everything, actuation ENABLED, explore 0.6 m/s (console, up to 1.0)
#   dryrun_bringup.sh race     actuation ENABLED, no operator needed: ARM a mode in the
#                              console, the car starts on a green signal and stops on red
#                              (speed per mode, up to the model's 3 m/s)
#   dryrun_bringup.sh auto     the mode chosen on the console (trial or race; default trial),
#                              saved in ~/.config/laksa/car_mode; used by laksa-car.service
#   dryrun_bringup.sh console  restart only the console (host follows network)
#   dryrun_bringup.sh stop | status
#
# Optional overrides for start|trial|race (unset = the behaviour described above;
# from KarSha's route tooling, commit 995ace9):
#   LAKSA_NAV_ERPM       supervisor navigation_max_erpm
#   LAKSA_CRUISE_ERPM    supervisor exploration_max_erpm (default: per mode)
#   LAKSA_DRIVER_CAP     learned driver speed_cap_mps (default: per mode)
#   LAKSA_BT_XML         NavigateToPose behaviour tree (default: navigate_ackermann.xml)
#   LAKSA_NAV_EXTRA      extra --params-file for controller_server, after the field overrides
#   LAKSA_NAV_CMD_TOPIC  cmd_vel remap for controller_server and behavior_server
#                        (default /laksa/nav_cmd_vel, the supervisor's navigation input)
set -uo pipefail

RUN_DIR="${HOME}/laksa_run"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
JETSON="${REPO}/firmware/esp32-s3/jetson"
SUPERVISOR_CONFIG="${JETSON}/laksa_bringup/config/drive_supervisor.yaml"
ZED_PROFILE="${JETSON}/laksa_learned_driver/config/zed_perception.yaml"
EKF_CONFIG="${JETSON}/laksa_learned_driver/config/ekf_field.yaml"
RTAB_OVERRIDES="${JETSON}/laksa_learned_driver/config/rtabmap_field_overrides.yaml"
# Every run's logs, decisions and recordings live under one folder on the car
# (laksa_deploy.sh moves old ~/laksa_sessions there and prunes the oldest).
LOG_ROOT="${LAKSA_LOG_ROOT:-${HOME}/laksa_logs}"
SESSIONS_DIR="${LOG_ROOT}/sessions"
# A packaged release (~/laksa/releases/<version>/src) builds its own workspace next
# to its source; a plain checkout uses ~/laksa_ws as before.
if [[ -f "${REPO}/../ws/install/setup.bash" ]]; then
    LAKSA_WS="$(cd "${REPO}/../ws" && pwd)"
else
    LAKSA_WS="${HOME}/laksa_ws"
fi
# Small, replayable record of each run (no full camera video).
BAG_TOPICS=(/laksa/lidar/scan_validated /laksa/command /laksa/brake /laksa/lidar_cruise_cmd_vel
    /laksa/exploration_status /laksa/mission_state /laksa/autonomy_health /laksa/emergency_stop
    /laksa/emergency_stop_reason /laksa/perception/detections /laksa/perception/person
    /laksa/perception/ground /laksa/odometry/fused /zed/zed_node/odom
    /laksa/vesc/state /laksa/state /joy /tf /tf_static /map)
RTAB_CONFIG="${JETSON}/laksa_mapping/config/rtabmap_fused.yaml"
DRIVER_CONFIG="${JETSON}/laksa_learned_driver/config/learned_driver.yaml"
NAV_CONFIG="${JETSON}/laksa_bringup/config/nav2_ackermann.yaml"
NAV_OVERRIDES="${JETSON}/laksa_learned_driver/config/nav2_field_overrides.yaml"
BT_XML="${JETSON}/laksa_bringup/config/navigate_ackermann.xml"
BT_THROUGH_XML="${JETSON}/laksa_bringup/config/navigate_through_poses_ackermann.xml"
# Nav2 (route planning and following to a console goal); LAKSA_NAV=0 skips it.
LAKSA_NAV="${LAKSA_NAV:-1}"
TOKEN_FILE="${HOME}/.config/laksa/console_token"
HOTSPOT_IP="10.42.0.1"

# Match the ROS network of the agent already bridging the ESP32.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI

source_ros() {
    set +u
    source /opt/ros/humble/setup.bash
    [[ -f "${HOME}/zed_ws/install/setup.bash" ]] && source "${HOME}/zed_ws/install/setup.bash"
    source "${HOME}/third_party/third_party_ws/install/setup.bash"
    source "${LAKSA_WS}/install/setup.bash"
    set -u
}

LOG_DIR="${RUN_DIR}"
start_one() {
    local name="$1"; shift
    nohup setsid "$@" > "${LOG_DIR}/${name}.log" 2>&1 < /dev/null &
    echo $! > "${RUN_DIR}/${name}.pid"
    echo "started ${name} (pid $(cat "${RUN_DIR}/${name}.pid"))"
}

stop_one() {
    local pid_file="$1" pid
    pid="$(cat "${pid_file}")"
    kill -INT -- "-${pid}" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 -- "-${pid}" 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- "-${pid}" 2>/dev/null
    rm -f "${pid_file}"
    echo "stopped $(basename "${pid_file}" .pid)"
}

console_host() {
    # The car's own hotspot when it is up (only devices that joined LAKSA-CAR
    # can reach it); otherwise localhost, for SSH tunnels.
    if ip -4 -o addr show 2>/dev/null | grep -q " ${HOTSPOT_IP}/"; then echo "${HOTSPOT_IP}"; else echo 127.0.0.1; fi
}

start_console() {
    # Fixed console token, generated once on this Jetson and never committed.
    if [[ ! -s "${TOKEN_FILE}" ]]; then
        mkdir -p "$(dirname "${TOKEN_FILE}")"
        ( umask 077; python3 -c "import secrets; print(secrets.token_urlsafe(9))" > "${TOKEN_FILE}" )
    fi
    chmod 600 "${TOKEN_FILE}"
    local host
    host="$(console_host)"
    [[ -f "${RUN_DIR}/console.pid" ]] && stop_one "${RUN_DIR}/console.pid" >/dev/null
    start_one console ros2 run laksa_learned_driver laksa_console --ros-args \
        -p host:="${host}" -p token_file:="${TOKEN_FILE}"
    echo "${host}" > "${RUN_DIR}/console_host.txt"
    ( umask 077; echo "http://${host}:8095/?token=$(cat "${TOKEN_FILE}")" > "${RUN_DIR}/console_url.txt" )
    echo "console: $(cat "${RUN_DIR}/console_url.txt")"
}

MODE="${1:-status}"
CAR_MODE_FILE="${HOME}/.config/laksa/car_mode"
if [[ "${MODE}" == "auto" ]]; then
    MODE="$(cat "${CAR_MODE_FILE}" 2>/dev/null || true)"
    [[ "${MODE}" == "race" ]] || MODE=trial
fi
ACTUATION=false
CRUISE_ERPM=1000.0
DRIVER_CAP=0.24
NAV_ERPM=""
SUPERVISOR_EXTRA=()
DRIVER_EXTRA=()
if [[ "${MODE}" == "trial" ]]; then
    # Trial & explore: an operator holds the run.  The drive stalls below ~0.2 m/s
    # (bench 2026-09-28), so explore runs at 0.6 m/s by default (console slider) with a
    # 1.0 m/s ceiling, and Nav2 routes at 0.22 m/s (KarSha's measured reliable start).
    # Capped at 0.6 m/s (2,485 eRPM): the ESP32 firmware on the car rejects drive
    # commands above ~0.6-0.8 m/s and holds the brake (field run 2026-10-01: 0.6
    # accepted 100%, 0.8 accepted 6%). Clamping here keeps the car driving.
    ACTUATION=true; CRUISE_ERPM=2485.0; DRIVER_CAP=0.6; NAV_ERPM=1000.0
fi
if [[ "${MODE}" == "race" ]]; then
    # No speed cap below the model's trained 3 m/s (12,430 eRPM at 4,142 eRPM per m/s);
    # the race manager sets the speed per mode.  Operator-free, odometry sanity
    # check allows 4 m/s, and the clearance check looks 8 m ahead.
    ACTUATION=true; CRUISE_ERPM=12500.0; DRIVER_CAP=2.5
    SUPERVISOR_EXTRA=(-p require_operator:=false -p max_odom_linear_speed_mps:=4.0)
    DRIVER_EXTRA=(-p governor_horizon_m:=8.0)
    NAV_ERPM=1000.0
fi
CRUISE_ERPM="${LAKSA_CRUISE_ERPM:-${CRUISE_ERPM}}"
DRIVER_CAP="${LAKSA_DRIVER_CAP:-${DRIVER_CAP}}"
NAV_ERPM="${LAKSA_NAV_ERPM:-${NAV_ERPM}}"
BT_XML="${LAKSA_BT_XML:-${BT_XML}}"
NAV_EXTRA="${LAKSA_NAV_EXTRA:-}"
NAV_CMD_TOPIC="${LAKSA_NAV_CMD_TOPIC:-/laksa/nav_cmd_vel}"

check_overrides() {
    # Only overrides that are set are checked, so the defaults behave as before.
    local name value
    for name in LAKSA_CRUISE_ERPM LAKSA_DRIVER_CAP LAKSA_NAV_ERPM; do
        value="${!name:-}"
        if [[ -n "${value}" && ! "${value}" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
            echo "refusing to start: ${name}='${value}' is not a non-negative number" >&2; exit 64
        fi
    done
    for name in LAKSA_BT_XML LAKSA_NAV_EXTRA; do
        value="${!name:-}"
        if [[ -n "${value}" && ! -f "${value}" ]]; then
            echo "refusing to start: ${name} file '${value}' does not exist" >&2; exit 64
        fi
    done
    value="${LAKSA_NAV_CMD_TOPIC:-}"
    if [[ -n "${value}" && ! "${value}" =~ ^/[A-Za-z0-9_/]+$ ]]; then
        echo "refusing to start: LAKSA_NAV_CMD_TOPIC='${value}' is not an absolute topic name" >&2; exit 64
    fi
}

# rgbd_sync pairs the ZED colour and depth images for RTAB-Map.
RGBD_SYNC_CMD=(ros2 run rtabmap_sync rgbd_sync --ros-args -r __ns:=/laksa/fused_mapping
    -r __node:=rgbd_sync -p approx_sync:=true -p approx_sync_max_interval:=0.05 -p queue_size:=10 -p qos:=2
    -r rgb/image:=/zed/zed_node/rgb/color/rect/image -r rgb/camera_info:=/zed/zed_node/rgb/color/rect/camera_info
    -r depth/image:=/zed/zed_node/depth/depth_registered)

case "${MODE}" in
start|trial|race)
    check_overrides
    # The supervisor declares these as doubles: a bare "1000" is an integer and
    # makes it exit with InvalidParameterTypeException.
    [[ "${CRUISE_ERPM}" == *.* ]] || CRUISE_ERPM="${CRUISE_ERPM}.0"
    [[ -z "${NAV_ERPM}" || "${NAV_ERPM}" == *.* ]] || NAV_ERPM="${NAV_ERPM}.0"
    SUPERVISOR_ARGS=(--params-file "${SUPERVISOR_CONFIG}" -p actuation_enabled:=${ACTUATION} -p autonomy_enabled:=true
        -p exploration_max_erpm:=${CRUISE_ERPM} "${SUPERVISOR_EXTRA[@]}")
    [[ -n "${NAV_ERPM}" ]] && SUPERVISOR_ARGS+=(-p navigation_max_erpm:=${NAV_ERPM})
    NAV_EXTRA_ARGS=()
    [[ -n "${NAV_EXTRA}" ]] && NAV_EXTRA_ARGS=(--params-file "${NAV_EXTRA}")
    mkdir -p "${RUN_DIR}"
    # The Jetson has no RTC battery: every boot starts at the same saved clock time
    # until NTP syncs (and never syncs in the field), so a timestamp alone reused one
    # folder across boots and overwrote its logs.  The boot id keeps boots apart.
    SESSION="${SESSIONS_DIR}/$(date +%Y%m%dT%H%M%S)_boot$(cut -c1-6 /proc/sys/kernel/random/boot_id 2>/dev/null)"
    mkdir -p "${SESSION}"
    ln -sfn "${SESSION}" "${RUN_DIR}/latest"
    LOG_DIR="${SESSION}"
    echo "session ${SESSION}"
    source_ros
    start_one description ros2 launch laksa_description description.launch.py
    start_one lidar ros2 launch laksa_bringup lidar_mapping.launch.py \
        serial_port:=/dev/laksa_lidar enable_mapping:=false
    start_one lidar_guard ros2 launch laksa_lidar lidar_guard.launch.py
    if [[ -f "${HOME}/zed_ws/install/setup.bash" ]]; then
        # ZED odometry -> base pose -> EKF (+ measured VESC speed) = /laksa/odometry/fused
        # IMU TF off: at 200 Hz it was ~75% of /tf and no node uses the ZED IMU frame
        # (the ZED SDK fuses its IMU internally; the EKF fuses ZED pose + VESC speed).
        start_one zed ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i camera_name:=zed \
            publish_urdf:=true publish_tf:=false publish_map_tf:=false publish_imu_tf:=false \
            ros_params_override_path:="${ZED_PROFILE}"
        start_one zed_base_pose ros2 run laksa_mapping zed_base_pose_adapter
        start_one state_measurements ros2 run laksa_bringup state_measurements_node.py --ros-args \
            -p odom_frame:=odom -p base_frame:=base_footprint
        start_one ekf ros2 run robot_localization ekf_node --ros-args --params-file "${EKF_CONFIG}" \
            -r odometry/filtered:=/laksa/odometry/fused
    fi
    start_one supervisor ros2 run laksa_bringup drive_supervisor_node.py --ros-args "${SUPERVISOR_ARGS[@]}"
    start_one learned_driver ros2 run laksa_learned_driver learned_driver_node --ros-args \
        --params-file "${DRIVER_CONFIG}" -p speed_cap_mps:=${DRIVER_CAP} -p decision_log:="${SESSION}/decisions.csv" "${DRIVER_EXTRA[@]}"
    # Bluetooth/USB gamepad, if one is paired: B = emergency stop, sticks take over.
    start_one joy ros2 run joy game_controller_node --ros-args -r __node:=joy_node --params-file "${SUPERVISOR_CONFIG}"
    if [[ "${MODE}" == "race" ]]; then
        start_one race_manager ros2 run laksa_learned_driver race_manager
    fi
    if [[ -f "${HOME}/zed_ws/install/setup.bash" ]]; then
        start_one zed_perception ros2 run laksa_learned_driver zed_perception
        start_one rgbd_sync "${RGBD_SYNC_CMD[@]}"
        # rgbd_sync sometimes stops receiving a few seconds after start while the ZED
        # keeps publishing (2026-10-01): the map freezes and routes fail. Restart it.
        start_one mapping_watchdog bash "${BASH_SOURCE[0]}" watchdog
        start_one rtabmap ros2 run rtabmap_slam rtabmap -d --ros-args -r __ns:=/laksa/fused_mapping -r __node:=rtabmap \
            --params-file "${RTAB_CONFIG}" --params-file "${RTAB_OVERRIDES}" -p database_path:="${SESSION}/rtabmap.db" \
            -r rgbd_image:=/laksa/fused_mapping/rgbd_image -r scan:=/laksa/lidar/scan_validated \
            -r odom:=/laksa/odometry/fused -r map:=/map
        if [[ "${LAKSA_NAV}" == "1" ]]; then
            # Controller and BackUp output go to the supervisor's navigation
            # input, never to the ESP32: the supervisor gates and caps them.
            start_one nav_controller ros2 run nav2_controller controller_server --ros-args \
                --params-file "${NAV_CONFIG}" --params-file "${NAV_OVERRIDES}" "${NAV_EXTRA_ARGS[@]}" \
                -r cmd_vel:="${NAV_CMD_TOPIC}"
            start_one nav_planner ros2 run nav2_planner planner_server --ros-args \
                --params-file "${NAV_CONFIG}" --params-file "${NAV_OVERRIDES}"
            start_one nav_behavior ros2 run nav2_behaviors behavior_server --ros-args \
                --params-file "${NAV_CONFIG}" -r cmd_vel:="${NAV_CMD_TOPIC}"
            start_one nav_bt ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file "${NAV_CONFIG}" \
                -p default_nav_to_pose_bt_xml:="${BT_XML}" -p default_nav_through_poses_bt_xml:="${BT_THROUGH_XML}"
            start_one nav_lifecycle ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
                -r __node:=lifecycle_manager_navigation --params-file "${NAV_CONFIG}"
        fi
    fi
    # The car is often switched off by pulling the battery: write 60 s chunks with
    # the crash-safe SQLite preset so a power cut loses at most the open chunk
    # (closed chunks stay readable; "ros2 bag reindex" rebuilds metadata.yaml).
    start_one recorder ros2 bag record -o "${SESSION}/bag" --storage-preset-profile resilient         --max-bag-duration 60 "${BAG_TOPICS[@]}"
    start_console
    {
        echo "mode=${MODE} actuation=${ACTUATION} cruise_erpm=${CRUISE_ERPM} driver_cap=${DRIVER_CAP} started=$(date -Is)"
        echo "nav_erpm=${NAV_ERPM:-yaml}"
        echo "bt_xml=${BT_XML}"
        echo "nav_extra=${NAV_EXTRA:-none}"
        echo "nav_cmd_topic=${NAV_CMD_TOPIC}"
        # Read back by KarSha's setup/tonight/restart_supervisor.sh.
        echo "supervisor_args=$(printf '%q ' "${SUPERVISOR_ARGS[@]}")"
    } > "${SESSION}/session.txt"
    echo "${MODE}" > "${RUN_DIR}/car_mode_active"
    echo "mode ${MODE}: supervisor actuation_enabled=${ACTUATION}, cruise cap ${CRUISE_ERPM} eRPM, driver cap ${DRIVER_CAP} m/s"
    ;;
console)
    mkdir -p "${RUN_DIR}"
    source_ros
    start_console
    ;;
watchdog)
    # Internal (started by start|trial|race): restart rgbd_sync when its log shows
    # it starved for ~15 s while the ZED process is alive.  Reads logs only.
    source_ros
    session="$(readlink -f "${RUN_DIR}/latest")"
    last_restart=0
    while sleep 10; do
        log="${session}/rgbd_sync.log"
        zed_pid="$(cat "${RUN_DIR}/zed.pid" 2>/dev/null || true)"
        [[ -n "${zed_pid}" && -f "${log}" ]] && kill -0 "${zed_pid}" 2>/dev/null || continue
        starved="$(grep -a -v ddsi "${log}" | tail -n 3 | grep -c "rgbd_sync: Did not receive data")"
        now="$(date +%s)"
        if (( starved == 3 && now - $(stat -c %Y "${log}") < 10 && now - last_restart > 30 )); then
            echo "$(date -Is) rgbd_sync starved while the ZED runs; restarting it"
            [[ -f "${RUN_DIR}/rgbd_sync.pid" ]] && stop_one "${RUN_DIR}/rgbd_sync.pid"
            echo "--- restarted by the mapping watchdog $(date -Is)" >> "${log}"
            nohup setsid "${RGBD_SYNC_CMD[@]}" >> "${log}" 2>&1 < /dev/null &
            echo $! > "${RUN_DIR}/rgbd_sync.pid"
            last_restart="${now}"
        fi
    done
    ;;
stop)
    # The watchdog first, so it cannot restart rgbd_sync while the rest stops.
    [[ -f "${RUN_DIR}/mapping_watchdog.pid" ]] && stop_one "${RUN_DIR}/mapping_watchdog.pid"
    for pid_file in "${RUN_DIR}"/*.pid; do
        [[ -f "${pid_file}" ]] || continue
        stop_one "${pid_file}"
    done
    ;;
status)
    for pid_file in "${RUN_DIR}"/*.pid; do
        [[ -f "${pid_file}" ]] || continue
        pid="$(cat "${pid_file}")"
        if kill -0 "${pid}" 2>/dev/null; then state=running; else state=EXITED; fi
        echo "$(basename "${pid_file}" .pid): ${state}"
    done
    ;;
*)
    echo "usage: $0 start|trial|race|auto|console|stop|status" >&2
    exit 64
    ;;
esac
