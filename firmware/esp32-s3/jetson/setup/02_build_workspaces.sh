#!/usr/bin/env bash
# Build the LAKSA ROS 2 workspaces as the normal user (no sudo).
#
#   ~/third_party/third_party_ws : sllidar_ros2 (Slamtec), rf2o_laser_odometry
#                                  pinned to the car's commit + LAKSA patch
#   ~/laksa_ws                   : laksa_interfaces and the Jetson packages,
#                                  symlinked from this repository checkout
#
# Build only: nothing is launched, no device is opened.  zed_wrapper is not
# built here (it needs the proprietary ZED SDK).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
JETSON="${REPO}/firmware/esp32-s3/jetson"
THIRD_PARTY_WS="${HOME}/third_party/third_party_ws"
LAKSA_WS="${HOME}/laksa_ws"
RF2O_COMMIT="b38c68e46387b98845ecbfeb6660292f967a00d3"
JOBS="${LAKSA_BUILD_JOBS:-3}"   # 7.4 GB RAM Orin Nano: keep C++ builds bounded

set +u
source /opt/ros/humble/setup.bash
set -u

rosdep update --rosdistro humble >/dev/null

echo "== third-party workspace"
mkdir -p "${THIRD_PARTY_WS}/src"
cd "${THIRD_PARTY_WS}/src"
[[ -d sllidar_ros2 ]] || git clone --quiet https://github.com/Slamtec/sllidar_ros2.git
if [[ ! -d rf2o_laser_odometry ]]; then
    git clone --quiet https://github.com/MAPIRlab/rf2o_laser_odometry.git
fi
git -C rf2o_laser_odometry fetch --quiet --depth 1 origin "${RF2O_COMMIT}"
git -C rf2o_laser_odometry checkout --quiet --detach "${RF2O_COMMIT}"
patch_file="${JETSON}/patches/rf2o-laser-odometry-laksa.patch"
if ! git -C rf2o_laser_odometry apply --reverse --check "${patch_file}" 2>/dev/null; then
    git -C rf2o_laser_odometry apply "${patch_file}"
fi
echo "sllidar_ros2 $(git -C sllidar_ros2 rev-parse --short HEAD); rf2o $(git -C rf2o_laser_odometry rev-parse --short HEAD) + LAKSA patch"
cd "${THIRD_PARTY_WS}"
rosdep check --from-paths src --ignore-src --rosdistro humble || true
MAKEFLAGS="-j${JOBS}" colcon build --symlink-install --parallel-workers "${JOBS}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "== LAKSA workspace"
mkdir -p "${LAKSA_WS}/src"
ln -sfn "${REPO}/firmware/esp32-s3/extra_ros_packages/laksa_interfaces" "${LAKSA_WS}/src/laksa_interfaces"
for package in "${JETSON}"/laksa_*; do
    [[ -f "${package}/package.xml" ]] && ln -sfn "${package}" "${LAKSA_WS}/src/$(basename "${package}")"
done
set +u
source "${THIRD_PARTY_WS}/install/setup.bash"
set -u
cd "${LAKSA_WS}"
rosdep check --from-paths src --ignore-src --rosdistro humble \
    --skip-keys "zed_wrapper laksa_lab" || true
# JetPack installs NVIDIA's libopencv-dev 4.8 while ROS/RTAB-Map use Ubuntu's
# OpenCV 4.5 runtime.  RTAB-Map's CMake config asks the linker for
# -lopencv_aruco, whose unversioned dev symlink only Ubuntu's (conflicting)
# libopencv-contrib-dev provides.  Point the linker at the installed 4.5
# runtime library instead of replacing JetPack's OpenCV.
compat="${LAKSA_WS}/opencv_link_compat"
mkdir -p "${compat}"
aruco="$(ls /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.5d 2>/dev/null || true)"
[[ -n "${aruco}" ]] && ln -sfn "${aruco}" "${compat}/libopencv_aruco.so"
MAKEFLAGS="-j${JOBS}" colcon build --symlink-install --parallel-workers "${JOBS}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_EXE_LINKER_FLAGS=-L${compat}"

echo
echo "Workspaces built. Source them with:"
echo "  source ${THIRD_PARTY_WS}/install/setup.bash && source ${LAKSA_WS}/install/setup.bash"
