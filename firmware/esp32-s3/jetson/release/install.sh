#!/usr/bin/env bash
# Build one unpacked LAKSA release in place; run by laksa_deploy.sh, never by hand
# on a release that is already current.
#
#   <release>/src   the package's source (firmware/esp32-s3/jetson, laksa_interfaces)
#   <release>/ws    colcon workspace built here from <release>/src
#
# Uses the prebuilt shared workspaces (~/third_party/third_party_ws, ~/zed_ws) and
# the apt packages from setup/01_system_setup.sh; it installs nothing system-wide.
# A non-zero exit leaves the running release untouched.
set -euo pipefail

RELEASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${RELEASE}/src"
WS="${RELEASE}/ws"
JETSON="${SRC}/firmware/esp32-s3/jetson"
THIRD_PARTY_WS="${HOME}/third_party/third_party_ws"
JOBS="${LAKSA_BUILD_JOBS:-3}"   # 7.4 GB RAM Orin Nano: keep C++ builds bounded

[[ -f "${RELEASE}/MANIFEST.json" ]] || { echo "not a LAKSA release: ${RELEASE}" >&2; exit 2; }
if [[ ! -f "${THIRD_PARTY_WS}/install/setup.bash" ]]; then
    echo "missing ${THIRD_PARTY_WS}: run setup/02_build_workspaces.sh once on this Jetson" >&2
    exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${THIRD_PARTY_WS}/install/setup.bash"
set -u

echo "== workspace ${WS}"
mkdir -p "${WS}/src"
ln -sfn "${SRC}/firmware/esp32-s3/extra_ros_packages/laksa_interfaces" "${WS}/src/laksa_interfaces"
for package in "${JETSON}"/laksa_*; do
    [[ -f "${package}/package.xml" ]] && ln -sfn "${package}" "${WS}/src/$(basename "${package}")"
done
# Same OpenCV link workaround as setup/02_build_workspaces.sh (JetPack OpenCV 4.8
# vs the Ubuntu 4.5 runtime that RTAB-Map links against).
compat="${WS}/opencv_link_compat"
mkdir -p "${compat}"
aruco="$(ls /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.5d 2>/dev/null || true)"
[[ -n "${aruco}" ]] && ln -sfn "${aruco}" "${compat}/libopencv_aruco.so"
cd "${WS}"
MAKEFLAGS="-j${JOBS}" nice colcon build --symlink-install --parallel-workers "${JOBS}" \
    --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_EXE_LINKER_FLAGS=-L${compat}"

echo "== checks"
set +u
source "${WS}/install/setup.bash"
set -u
for package in laksa_interfaces laksa_bringup laksa_learned_driver laksa_lidar laksa_mapping; do
    ros2 pkg prefix "${package}" >/dev/null || { echo "package ${package} not installed" >&2; exit 1; }
done
compgen -G "${JETSON}/laksa_learned_driver/models/*.npz" >/dev/null \
    || { echo "no driver model in the package" >&2; exit 1; }
bash -n "${JETSON}/setup/dryrun_bringup.sh"
cd "${JETSON}/laksa_learned_driver"
for test in test/test_learned_driver.py test/test_hold_latch.py test/test_uturn_sim.py; do
    PYTHONPATH=. python3 "${test}" 2>&1 | tail -3
done
echo "release $(basename "${RELEASE}") built"
