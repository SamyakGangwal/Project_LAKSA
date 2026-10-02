#!/usr/bin/env bash
# Drive the AutoDRIVE RoboRacer digital twin with the LAKSA car software (WSL / Ubuntu 22.04, ROS 2 Humble).
#
#   run_sim.sh setup     once: pip deps for the AutoDRIVE devkit, build the sim workspace
#   run_sim.sh start     devkit bridge + adapter (fake ESP32) + supervisor + learned driver + console
#   run_sim.sh course    the same stack on the 2026 competition course replica (2-D, no AutoDRIVE app);
#                        LAKSA_SIM_COURSE=obstacle|obstacle_plain, LAKSA_SIM_SEED=<n> for other buckets
#   run_sim.sh stop | status
#
# Then start the AutoDRIVE simulator on Windows, connect it to 127.0.0.1:4567, and open
# http://localhost:8096/?token=<printed token>  (8096: the car's console tunnel uses 8095).
# Options (environment): LAKSA_SIM_SPEED_CAP (0.6), LAKSA_SIM_FIRMWARE_LIMIT (0 = off; e.g. 0.7
# copies the car's ESP32 speed limit), AUTODRIVE_DEVKIT (folder with the unzipped devkit).
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON="$(cd "${HERE}/../.." && pwd)"
INTERFACES="$(cd "${JETSON}/../extra_ros_packages/laksa_interfaces" && pwd)"
DEVKIT="${AUTODRIVE_DEVKIT:-/mnt/d/projects/autodrive/autodrive_devkit}"
WS="${HOME}/laksa_sim_ws"
RUN="${HOME}/laksa_sim_run"
SPEED_CAP="${LAKSA_SIM_SPEED_CAP:-0.6}"
FIRMWARE_LIMIT="${LAKSA_SIM_FIRMWARE_LIMIT:-0.0}"
CRUISE_ERPM="$(python3 -c "print(round(${SPEED_CAP} * 4142.0, 1))")"

source_ros() {
    set +u
    source /opt/ros/humble/setup.bash
    [[ -f "${WS}/install/setup.bash" ]] && source "${WS}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"          # never mix with a real car on the network
    export ROS_LOCALHOST_ONLY=1
}

start_one() {
    local name="$1"; shift
    nohup setsid "$@" > "${SESSION}/${name}.log" 2>&1 < /dev/null &
    echo $! > "${RUN}/${name}.pid"
    echo "started ${name}"
}

GYM_V1="${F1TENTH_GYM_V1:-/mnt/d/projects/Project_LAKSA/scratch/f1tenth_gym_v1}"

case "${1:-status}" in
setup)
    python3 -m pip install --user -q -r "${DEVKIT}/requirements_python_3.10.txt"
    mkdir -p "${WS}/src"
    ln -sfn "${INTERFACES}" "${WS}/src/laksa_interfaces"
    ln -sfn "${JETSON}/laksa_bringup" "${WS}/src/laksa_bringup"
    ln -sfn "${JETSON}/laksa_learned_driver" "${WS}/src/laksa_learned_driver"
    ln -sfn "${DEVKIT}" "${WS}/src/autodrive_roboracer"
    chmod +x "${DEVKIT}"/autodrive_roboracer/*.py 2>/dev/null || true
    set +u; source /opt/ros/humble/setup.bash; set -u
    cd "${WS}" && colcon build --symlink-install \
        --packages-select laksa_interfaces laksa_bringup laksa_learned_driver autodrive_roboracer
    ;;
start|course)
    BACKEND="$1"
    mkdir -p "${RUN}"
    SESSION="${HOME}/laksa_sim_logs/$(date +%Y%m%dT%H%M%S)"
    mkdir -p "${SESSION}"; ln -sfn "${SESSION}" "${RUN}/latest"
    source_ros
    # The console enables Trial & explore only when it knows the car runs TRIAL.
    mkdir -p "${HOME}/laksa_run" "${HOME}/.config/laksa"
    echo trial > "${HOME}/laksa_run/car_mode_active"; echo trial > "${HOME}/.config/laksa/car_mode"
    TOKEN_FILE="${HOME}/.config/laksa/sim_console_token"
    [[ -s "${TOKEN_FILE}" ]] || { mkdir -p "$(dirname "${TOKEN_FILE}")"; python3 -c "import secrets; print(secrets.token_urlsafe(9))" > "${TOKEN_FILE}"; }
    if [[ "${BACKEND}" == "course" ]]; then
        # 2-D course replica (training simulator); the LAKSA LiDAR position, facing forward.
        LIDAR_X=0.31542
        start_one course_sim env PYTHONPATH="${GYM_V1}:${PYTHONPATH:-}" python3 "${JETSON}/sim/course/laksa_course_sim.py" \
            --ros-args -p course:="${LAKSA_SIM_COURSE:-obstacle}" -p seed:="${LAKSA_SIM_SEED:-1}"
    else
        LIDAR_X=0.2733                    # the AutoDRIVE RoboRacer LiDAR
        start_one bridge ros2 launch autodrive_roboracer bringup_headless.launch.py
    fi
    start_one adapter python3 "${HERE}/laksa_autodrive_adapter.py" --ros-args \
        -p reject_above_mps:="${FIRMWARE_LIMIT}" -p lidar_x_m:="${LIDAR_X}"
    start_one supervisor ros2 run laksa_bringup drive_supervisor_node.py --ros-args \
        --params-file "${JETSON}/laksa_bringup/config/drive_supervisor.yaml" \
        -p actuation_enabled:=true -p autonomy_enabled:=true -p exploration_max_erpm:="${CRUISE_ERPM}"
    # The sim LiDAR faces forward at x = 0.2733 m (the car's faces backwards at 0.315 m);
    # no ZED in the sim, so the camera obstacle layer is off.
    start_one learned_driver ros2 run laksa_learned_driver learned_driver_node --ros-args \
        --params-file "${JETSON}/laksa_learned_driver/config/learned_driver.yaml" \
        -p lidar_x_m:="${LIDAR_X}" -p lidar_yaw_rad:=0.0 -p speed_cap_mps:="${SPEED_CAP}" -p use_camera:=false \
        -p decision_log:="${SESSION}/decisions.csv"
    start_one console ros2 run laksa_learned_driver laksa_console --ros-args \
        -p host:=127.0.0.1 -p port:=8096 -p token_file:="${TOKEN_FILE}"
    echo "speed cap ${SPEED_CAP} m/s (${CRUISE_ERPM} eRPM); firmware limit ${FIRMWARE_LIMIT}; logs ${SESSION}"
    echo "console: http://localhost:8096/?token=$(cat "${TOKEN_FILE}")"
    [[ "${BACKEND}" == "start" ]] && echo "now start the AutoDRIVE simulator on Windows and connect it to localhost:4567"
    ;;
stop)
    for pid_file in "${RUN}"/*.pid; do
        [[ -f "${pid_file}" ]] || continue
        pid="$(cat "${pid_file}")"
        kill -INT -- "-${pid}" 2>/dev/null; sleep 1; kill -TERM -- "-${pid}" 2>/dev/null
        rm -f "${pid_file}"; echo "stopped $(basename "${pid_file}" .pid)"
    done
    ;;
status)
    for pid_file in "${RUN}"/*.pid; do
        [[ -f "${pid_file}" ]] || continue
        kill -0 "$(cat "${pid_file}")" 2>/dev/null && s=running || s=EXITED
        echo "$(basename "${pid_file}" .pid): ${s}"
    done
    ;;
*) echo "usage: $0 setup|start|course|stop|status" >&2; exit 64 ;;
esac
