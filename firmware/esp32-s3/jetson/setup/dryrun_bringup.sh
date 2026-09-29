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
#   dryrun_bringup.sh trial    everything, actuation ENABLED, cruise ~0.15 m/s
#   dryrun_bringup.sh console  restart only the console (host follows network)
#   dryrun_bringup.sh stop | status
set -uo pipefail

RUN_DIR="${HOME}/laksa_run"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
JETSON="${REPO}/firmware/esp32-s3/jetson"
SUPERVISOR_CONFIG="${JETSON}/laksa_bringup/config/drive_supervisor.yaml"
ZED_PROFILE="${JETSON}/laksa_learned_driver/config/zed_perception.yaml"
EKF_CONFIG="${JETSON}/laksa_learned_driver/config/ekf_field.yaml"
RTAB_OVERRIDES="${JETSON}/laksa_learned_driver/config/rtabmap_field_overrides.yaml"
SESSIONS_DIR="${HOME}/laksa_sessions"
# Small, replayable record of each run (no full camera video).
BAG_TOPICS=(/laksa/lidar/scan_validated /laksa/command /laksa/brake /laksa/lidar_cruise_cmd_vel
    /laksa/exploration_status /laksa/mission_state /laksa/autonomy_health /laksa/emergency_stop
    /laksa/emergency_stop_reason /laksa/perception/detections /laksa/perception/person
    /laksa/perception/ground /laksa/odometry/fused /zed/zed_node/odom
    /laksa/vesc/state /laksa/state /joy /tf /tf_static /map)
RTAB_CONFIG="${JETSON}/laksa_mapping/config/rtabmap_fused.yaml"
DRIVER_CONFIG="${JETSON}/laksa_learned_driver/config/learned_driver.yaml"
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
    source "${HOME}/laksa_ws/install/setup.bash"
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
ACTUATION=false
CRUISE_ERPM=1000.0
DRIVER_CAP=0.24
if [[ "${MODE}" == "trial" ]]; then ACTUATION=true; CRUISE_ERPM=620.0; DRIVER_CAP=0.15; fi

case "${MODE}" in
start|trial)
    mkdir -p "${RUN_DIR}"
    SESSION="${SESSIONS_DIR}/$(date +%Y%m%dT%H%M%S)"
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
    start_one supervisor ros2 run laksa_bringup drive_supervisor_node.py --ros-args \
        --params-file "${SUPERVISOR_CONFIG}" -p actuation_enabled:=${ACTUATION} -p autonomy_enabled:=true \
        -p exploration_max_erpm:=${CRUISE_ERPM}
    start_one learned_driver ros2 run laksa_learned_driver learned_driver_node --ros-args \
        --params-file "${DRIVER_CONFIG}" -p speed_cap_mps:=${DRIVER_CAP} -p decision_log:="${SESSION}/decisions.csv"
    if [[ -f "${HOME}/zed_ws/install/setup.bash" ]]; then
        start_one zed_perception ros2 run laksa_learned_driver zed_perception
        start_one rgbd_sync ros2 run rtabmap_sync rgbd_sync --ros-args -r __ns:=/laksa/fused_mapping \
            -r __node:=rgbd_sync -p approx_sync:=true -p approx_sync_max_interval:=0.05 -p queue_size:=10 -p qos:=2 \
            -r rgb/image:=/zed/zed_node/rgb/color/rect/image -r rgb/camera_info:=/zed/zed_node/rgb/color/rect/camera_info \
            -r depth/image:=/zed/zed_node/depth/depth_registered
        start_one rtabmap ros2 run rtabmap_slam rtabmap -d --ros-args -r __ns:=/laksa/fused_mapping -r __node:=rtabmap \
            --params-file "${RTAB_CONFIG}" --params-file "${RTAB_OVERRIDES}" -p database_path:="${SESSION}/rtabmap.db" \
            -r rgbd_image:=/laksa/fused_mapping/rgbd_image -r scan:=/laksa/lidar/scan_validated \
            -r odom:=/laksa/odometry/fused -r map:=/map
    fi
    start_one recorder ros2 bag record -o "${SESSION}/bag" "${BAG_TOPICS[@]}"
    start_console
    echo "mode=${MODE} actuation=${ACTUATION} cruise_erpm=${CRUISE_ERPM} driver_cap=${DRIVER_CAP} started=$(date -Is)" > "${SESSION}/session.txt"
    echo "mode ${MODE}: supervisor actuation_enabled=${ACTUATION}, cruise cap ${CRUISE_ERPM} eRPM, driver cap ${DRIVER_CAP} m/s"
    ;;
console)
    mkdir -p "${RUN_DIR}"
    source_ros
    start_console
    ;;
stop)
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
    echo "usage: $0 start|trial|console|stop|status" >&2
    exit 64
    ;;
esac
