#!/usr/bin/env bash
# LAKSA Jetson system setup (run once with sudo on Ubuntu 22.04 / JetPack 6).
#
# Installs ROS 2 Humble and the apt packages the LAKSA stack uses, and writes
# the canonical ROS middleware environment to /etc/laksa.  It deliberately does
# NOT: run apt upgrade, install udev rules, install or enable systemd services,
# change group membership, or open any serial device.  Hardware ownership is a
# separate, explicit step.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
    echo "Run with sudo: sudo bash $0" >&2
    exit 1
fi

REPO_JETSON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export DEBIAN_FRONTEND=noninteractive

echo "== ROS 2 Humble apt source"
apt-get update
apt-get install -y --no-install-recommends curl gnupg lsb-release software-properties-common ca-certificates
add-apt-repository -y universe
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
apt-get update

echo "== ROS 2 Humble and LAKSA runtime dependencies"
apt-get install -y --no-install-recommends \
    ros-humble-ros-base ros-dev-tools \
    python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-robot-state-publisher ros-humble-xacro ros-humble-tf2-ros ros-humble-tf2-tools \
    ros-humble-joy ros-humble-joy-teleop ros-humble-teleop-twist-joy ros-humble-twist-mux \
    ros-humble-laser-filters ros-humble-slam-toolbox ros-humble-robot-localization \
    ros-humble-rtabmap-ros \
    ros-humble-nav2-planner ros-humble-nav2-controller ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator ros-humble-nav2-lifecycle-manager ros-humble-nav2-map-server \
    ros-humble-nav2-amcl ros-humble-nav2-smac-planner ros-humble-nav2-mppi-controller \
    ros-humble-nav2-costmap-2d ros-humble-nav2-msgs \
    ros-humble-web-video-server ros-humble-sensor-msgs-py ros-humble-diagnostic-msgs \
    ros-humble-ament-cmake-pytest \
    python3-aiohttp python3-psutil python3-yaml python3-numpy python3-scipy python3-pytest

echo "== rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi

echo "== /etc/laksa ROS middleware environment"
install -d -m 0755 /etc/laksa
install -m 0644 "${REPO_JETSON_DIR}/systemd/ros-runtime.env" /etc/laksa/ros-runtime.env
install -m 0644 "${REPO_JETSON_DIR}/systemd/cyclonedds.xml" /etc/laksa/cyclonedds.xml

echo
echo "LAKSA system setup complete. Nothing was started and no serial device was opened."
