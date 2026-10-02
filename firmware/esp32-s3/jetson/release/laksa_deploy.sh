#!/usr/bin/env bash
# LAKSA package deployer on the Jetson (installed as ~/laksa/bin/laksa_deploy.sh).
#
#   laksa_deploy.sh boot        run by laksa-deploy.service at every boot, before
#                               laksa-car.service: log folder, log pruning, and
#                               install the newest package in ~/laksa/incoming
#   laksa_deploy.sh status      current / previous release, waiting packages
#   laksa_deploy.sh rollback    switch back to the previous release and restart the car
#   laksa_deploy.sh bootstrap <unpacked package>
#                               first install on a Jetson: this script + laksa-deploy.service
#
# Layout:
#   ~/laksa/incoming/laksa-car-<version>.tar.gz (+ .sha256)   copied here by laksa_release.py
#   ~/laksa/releases/laksa-car-<version>/{src,ws,install.sh,MANIFEST.json}
#   ~/laksa/current -> releases/<version>    what laksa-car.service runs
#   ~/laksa/previous -> releases/<version>   what rollback returns to
#   ~/laksa_logs/sessions/<time>_boot<id>/   one folder per car run (launcher)
#   ~/laksa_logs/deploy/                     this script's own logs
#
# A package is switched on only after its install.sh builds and tests it; a
# failed package goes to ~/laksa/incoming/failed and the car keeps running the
# current release.  Nothing here touches ~/src/Project_LAKSA (KarSha's checkout).
set -uo pipefail

ROOT="${LAKSA_ROOT:-${HOME}/laksa}"
LOGS="${LAKSA_LOG_ROOT:-${HOME}/laksa_logs}"
INCOMING="${ROOT}/incoming"
RELEASES="${ROOT}/releases"
CURRENT="${ROOT}/current"
PREVIOUS="${ROOT}/previous"
KEEP_RELEASES=4                       # besides current and previous
SESSIONS_MAX_GB="${LAKSA_SESSIONS_MAX_GB:-100}"
DISK_MIN_FREE_GB="${LAKSA_DISK_MIN_FREE_GB:-50}"
BUILD_TIMEOUT="${LAKSA_BUILD_TIMEOUT:-45m}"
UNITS=(laksa-deploy.service laksa-car.service laksa-network-watch.service)

log() { echo "[laksa-deploy $(date '+%F %T')] $*"; }
boot_id() { cut -c1-6 /proc/sys/kernel/random/boot_id 2>/dev/null || echo unknown; }
release_of() { [[ -L "$1" ]] && basename "$(readlink -f "$1")" || echo none; }

layout() {
    mkdir -p "${INCOMING}/installed" "${INCOMING}/failed" "${RELEASES}" "${ROOT}/bin" \
        "${LOGS}/sessions" "${LOGS}/deploy"
}

migrate_old_sessions() {
    # Runs before this change lived in ~/laksa_sessions; move them, never delete.
    local old="${HOME}/laksa_sessions" entry
    [[ "${LAKSA_NO_SYSTEM:-0}" == "1" ]] && return 0
    [[ -d "${old}" && ! -L "${old}" ]] || return 0
    for entry in "${old}"/*; do
        [[ -e "${entry}" ]] || continue
        mv -n "${entry}" "${LOGS}/sessions/" 2>/dev/null || log "could not move ${entry}"
    done
    rmdir "${old}" 2>/dev/null && ln -s "${LOGS}/sessions" "${old}" && log "moved ~/laksa_sessions into ${LOGS}/sessions"
}

prune_logs() {
    # Oldest sessions go first while the folder is over its cap or the disk is short.
    local sessions="${LOGS}/sessions" newest size_gb free_gb oldest
    newest="$(readlink -f "${HOME}/laksa_run/latest" 2>/dev/null || true)"
    while true; do
        size_gb=$(( $(du -s --block-size=1G "${sessions}" 2>/dev/null | cut -f1) + 0 ))
        free_gb=$(( $(df --output=avail --block-size=1G "${LOGS}" | tail -1) + 0 ))
        (( size_gb > SESSIONS_MAX_GB || free_gb < DISK_MIN_FREE_GB )) || break
        oldest="$(ls -1tr "${sessions}" | head -1)"
        [[ -n "${oldest}" && "${sessions}/${oldest}" != "${newest}" ]] || break
        log "pruning session ${oldest} (sessions ${size_gb} GB, free ${free_gb} GB)"
        rm -rf -- "${sessions:?}/${oldest}"
    done
    ls -1t "${LOGS}/deploy" 2>/dev/null | tail -n +51 | while read -r name; do rm -f -- "${LOGS}/deploy/${name}"; done
}

valid_archive() {
    # One top-level folder named like the package, no absolute or parent paths.
    local package="$1" name="$2"
    tar -tzf "${package}" > "${ROOT}/.listing" || return 1
    if grep -q -E '(^/|(^|/)\.\.(/|$))' "${ROOT}/.listing"; then log "unsafe path in ${package}"; return 1; fi
    if grep -v -q -E "^${name}(/|$)" "${ROOT}/.listing"; then log "files outside ${name}/ in ${package}"; return 1; fi
    grep -q -x "${name}/install.sh" "${ROOT}/.listing" && grep -q -x "${name}/MANIFEST.json" "${ROOT}/.listing"
}

sync_system_files() {
    # This script and the systemd units follow the active release.
    local jetson="${CURRENT}/src/firmware/esp32-s3/jetson" unit changed=0
    # LAKSA_NO_SYSTEM=1: trial install in a scratch LAKSA_ROOT, leave the car alone.
    [[ "${LAKSA_NO_SYSTEM:-0}" == "1" ]] && { log "LAKSA_NO_SYSTEM=1: not updating ${ROOT}/bin or systemd"; return 0; }
    install -m 0755 "${jetson}/release/laksa_deploy.sh" "${ROOT}/bin/laksa_deploy.sh"
    for unit in "${UNITS[@]}"; do
        local source="${jetson}/systemd/${unit}"
        [[ -f "${source}" ]] || continue
        if ! cmp -s "${source}" "/etc/systemd/system/${unit}"; then
            if sudo -n install -m 0644 "${source}" "/etc/systemd/system/${unit}"; then
                log "updated ${unit}"; changed=1
            else
                log "WARNING: cannot update ${unit} (needs passwordless sudo)"
            fi
        fi
    done
    (( changed )) && sudo -n systemctl daemon-reload && sudo -n systemctl enable laksa-deploy.service laksa-car.service >/dev/null 2>&1
    return 0
}

activate() {
    local target="$1" was=""
    # readlink -f of a missing link still prints a path, so check the link first.
    [[ -L "${CURRENT}" ]] && was="$(readlink -f "${CURRENT}")"
    ln -sfn "${target}" "${ROOT}/.current.new" && mv -Tf "${ROOT}/.current.new" "${CURRENT}"
    if [[ -n "${was}" && "${was}" != "$(readlink -f "${target}")" ]]; then
        ln -sfn "${was}" "${ROOT}/.previous.new" && mv -Tf "${ROOT}/.previous.new" "${PREVIOUS}"
    fi
    log "current release: $(release_of "${CURRENT}") (previous: $(release_of "${PREVIOUS}"))"
    sync_system_files
}

prune_releases() {
    local keep_current keep_previous name
    keep_current="$(release_of "${CURRENT}")"; keep_previous="$(release_of "${PREVIOUS}")"
    ls -1t "${RELEASES}" | grep -v '^\.' | grep -v -x -e "${keep_current}" -e "${keep_previous}" \
        | tail -n +$((KEEP_RELEASES + 1)) | while read -r name; do
        log "removing old release ${name}"; rm -rf -- "${RELEASES:?}/${name}"
    done
    rm -rf -- "${RELEASES}"/.staging-*
}

install_package() {
    local package="$1" name staging
    name="$(basename "${package}" .tar.gz)"
    log "installing ${name}"
    if [[ ! -f "${package}.sha256" ]] || ! (cd "${INCOMING}" && sha256sum --quiet -c "$(basename "${package}").sha256"); then
        log "checksum missing or wrong for ${name}"; return 1
    fi
    valid_archive "${package}" "${name}" || { log "${name} is not a valid LAKSA package"; return 1; }
    if [[ -d "${RELEASES}/${name}" ]]; then
        log "${name} is already installed; activating it"
        activate "${RELEASES}/${name}"; return 0
    fi
    staging="${RELEASES}/.staging-${name}"
    rm -rf -- "${staging}"; mkdir -p "${staging}"
    tar -xzf "${package}" -C "${staging}" --strip-components=1 --no-same-owner || return 1
    # The workspace records absolute paths, so build where the release will live.
    mv -T "${staging}" "${RELEASES}/${name}" || return 1
    if ! timeout "${BUILD_TIMEOUT}" bash "${RELEASES}/${name}/install.sh"; then
        log "build or checks failed for ${name}"
        rm -rf -- "${RELEASES:?}/${name}"
        return 1
    fi
    activate "${RELEASES}/${name}"
}

cmd_boot() {
    layout
    exec > >(tee -a "${LOGS}/deploy/$(date +%Y%m%dT%H%M%S)_boot$(boot_id).log") 2>&1
    log "boot $(boot_id); current release $(release_of "${CURRENT}")"
    migrate_old_sessions
    prune_logs
    local package
    package="$(ls -1t "${INCOMING}"/laksa-car-*.tar.gz 2>/dev/null | head -1)"
    if [[ -z "${package}" ]]; then
        log "no new package"
    elif install_package "${package}"; then
        mv -f "${package}" "${package}.sha256" "${INCOMING}/installed/" 2>/dev/null
        # Older packages that were never installed are superseded by this one.
        for stale in "${INCOMING}"/laksa-car-*.tar.gz; do
            [[ -e "${stale}" ]] && mv -f "${stale}" "${stale}.sha256" "${INCOMING}/installed/" 2>/dev/null
        done
    else
        mv -f "${package}" "${package}.sha256" "${INCOMING}/failed/" 2>/dev/null
        log "kept release $(release_of "${CURRENT}"); failed package moved to ${INCOMING}/failed"
    fi
    prune_releases
    [[ -L "${CURRENT}" ]] || log "WARNING: no release installed yet"
    return 0
}

cmd_status() {
    echo "current:  $(release_of "${CURRENT}")"
    echo "previous: $(release_of "${PREVIOUS}")"
    echo "releases: $(ls -1t "${RELEASES}" 2>/dev/null | grep -v '^\.' | tr '\n' ' ')"
    echo "waiting:  $(ls -1 "${INCOMING}"/*.tar.gz 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
    echo "failed:   $(ls -1 "${INCOMING}"/failed/*.tar.gz 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
    echo "logs:     ${LOGS} ($(du -sh "${LOGS}" 2>/dev/null | cut -f1)); last deploy log: $(ls -1t "${LOGS}/deploy" 2>/dev/null | head -1)"
    [[ -f "${CURRENT}/MANIFEST.json" ]] && cat "${CURRENT}/MANIFEST.json"
    return 0
}

cmd_rollback() {
    layout
    [[ -L "${PREVIOUS}" && -d "$(readlink -f "${PREVIOUS}")" ]] || { echo "no previous release" >&2; return 1; }
    activate "$(readlink -f "${PREVIOUS}")"
    [[ "${LAKSA_NO_SYSTEM:-0}" == "1" ]] && return 0
    sudo -n systemctl restart laksa-car.service && log "laksa-car restarted on $(release_of "${CURRENT}")"
}

cmd_bootstrap() {
    # First install: copy this script and laksa-deploy.service from an unpacked
    # package.  laksa-car.service is switched to ~/laksa/current only after the
    # first release has built (activate -> sync_system_files).
    local unpacked="$1" jetson
    jetson="${unpacked}/src/firmware/esp32-s3/jetson"
    layout
    install -m 0755 "${jetson}/release/laksa_deploy.sh" "${ROOT}/bin/laksa_deploy.sh"
    sudo -n install -m 0644 "${jetson}/systemd/laksa-deploy.service" /etc/systemd/system/laksa-deploy.service
    sudo -n systemctl daemon-reload
    sudo -n systemctl enable laksa-deploy.service
    log "bootstrap done: ${ROOT}/bin/laksa_deploy.sh and laksa-deploy.service installed"
}

case "${1:-status}" in
    boot) cmd_boot ;;
    status) cmd_status ;;
    rollback) cmd_rollback ;;
    bootstrap) cmd_bootstrap "${2:?unpacked package folder}" ;;
    *) echo "usage: $0 boot|status|rollback|bootstrap <dir>" >&2; exit 64 ;;
esac
