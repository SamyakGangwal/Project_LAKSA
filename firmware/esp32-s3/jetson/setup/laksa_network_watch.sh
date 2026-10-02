#!/usr/bin/env bash
# LAKSA network watcher (runs as root via laksa-network-watch.service).
#
#  * Away from known Wi-Fi: make sure the car's own hotspot (LAKSA-CAR,
#    10.42.0.1) is up, so operators can join it and open the console link.
#  * Back near a known Wi-Fi: drop the hotspot so the Jetson rejoins it.
#  * Keep the console bound to the right address (hotspot or localhost).
set -uo pipefail

HOTSPOT_CONN="laksa-hotspot"
CAR_USER="${LAKSA_CAR_USER:-samyak}"
CAR_HOME="$(getent passwd "${CAR_USER}" | cut -d: -f6)"
# The installed package (laksa_deploy.sh) if there is one, else the old checkout.
LAUNCHER="${CAR_HOME}/laksa/current/src/firmware/esp32-s3/jetson/setup/dryrun_bringup.sh"
[[ -f "${LAUNCHER}" ]] || LAUNCHER="${CAR_HOME}/src/Project_LAKSA/firmware/esp32-s3/jetson/setup/dryrun_bringup.sh"
RUN_DIR="${CAR_HOME}/laksa_run"
PERIOD_S=5
NO_WIFI_BEFORE_HOTSPOT_S=45
SCAN_EVERY_S=60

log() { echo "[laksa-network-watch] $*"; }

wifi_device() { nmcli -t -f DEVICE,TYPE device | awk -F: '$2 == "wifi" {print $1; exit}'; }
active_conn() { nmcli -t -f NAME,DEVICE connection show --active | awk -F: -v d="$1" '$2 == d {print $1; exit}'; }
known_ssids() {
    nmcli -t -f NAME,TYPE connection show | awk -F: '$2 == "802-11-wireless" {print $1}' | while read -r name; do
        [[ "${name}" == "${HOTSPOT_CONN}" ]] && continue
        nmcli -g 802-11-wireless.ssid connection show "${name}" 2>/dev/null
    done
}

no_wifi_since=0
last_scan=0
while true; do
    now=$(date +%s)
    dev="$(wifi_device)"
    current="$( [[ -n "${dev}" ]] && active_conn "${dev}" )"

    if [[ "${current}" == "${HOTSPOT_CONN}" ]]; then
        no_wifi_since=0
        if (( now - last_scan >= SCAN_EVERY_S )); then
            last_scan=${now}
            visible="$(nmcli -t -f SSID device wifi list --rescan yes 2>/dev/null)"
            while read -r ssid; do
                [[ -n "${ssid}" ]] || continue
                if grep -Fxq "${ssid}" <<< "${visible}"; then
                    log "known Wi-Fi '${ssid}' visible; dropping the hotspot"
                    nmcli connection down "${HOTSPOT_CONN}" >/dev/null 2>&1
                    break
                fi
            done < <(known_ssids)
        fi
    elif [[ -z "${current}" ]]; then
        (( no_wifi_since == 0 )) && no_wifi_since=${now}
        if (( now - no_wifi_since >= NO_WIFI_BEFORE_HOTSPOT_S )); then
            log "no Wi-Fi for ${NO_WIFI_BEFORE_HOTSPOT_S}s; starting hotspot ${HOTSPOT_CONN}"
            nmcli connection up "${HOTSPOT_CONN}" >/dev/null 2>&1 || log "hotspot start failed"
            no_wifi_since=0
        fi
    else
        no_wifi_since=0
    fi

    # Rebind the console when the network changed under it.
    if [[ -f "${RUN_DIR}/console.pid" ]] && kill -0 "$(cat "${RUN_DIR}/console.pid")" 2>/dev/null; then
        want=127.0.0.1
        ip -4 -o addr show 2>/dev/null | grep -q " 10\.42\.0\.1/" && want=10.42.0.1
        have="$(cat "${RUN_DIR}/console_host.txt" 2>/dev/null)"
        if [[ "${want}" != "${have}" ]]; then
            # Restart the whole stack, not just the console: ROS processes that
            # started on the previous network did not rediscover a console that
            # restarted on the new one (field test 2026-09-28).
            log "console bound to '${have}', network now wants '${want}'; restarting laksa-car"
            systemctl restart laksa-car.service
            sleep 60
        fi
    fi
    # The console asks for a restart after the car mode is changed (trial/race);
    # this service already restarts laksa-car, so the web page needs no sudo.
    # SHUT DOWN on the console: stop the car stack cleanly (recording closed,
    # logs flushed), then power the Jetson off so the battery can be pulled.
    if [[ -f "${RUN_DIR}/shutdown_request" ]]; then
        rm -f "${RUN_DIR}/shutdown_request"
        log "shutdown requested from the console; stopping laksa-car and powering off"
        systemctl stop laksa-car.service
        sync
        systemctl poweroff
        exit 0
    fi
    if [[ -f "${RUN_DIR}/restart_request" ]]; then
        rm -f "${RUN_DIR}/restart_request"
        log "car mode change requested from the console; restarting laksa-car"
        systemctl restart laksa-car.service
    fi
    sleep "${PERIOD_S}"
done
