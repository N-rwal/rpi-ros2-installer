#!/usr/bin/env bash
set -euo pipefail

if [ "$EUID" -eq 0 ]; then
  echo "Do not run as root. The script will use sudo where required."
  exit 1
fi

ARCH=$(uname -m)
if [[ "$ARCH" != "aarch64" && "$ARCH" != "x86_64" ]]; then
  echo "Unsupported architecture: $ARCH"
  exit 1
fi

source /etc/os-release
if [[ "$VERSION_ID" != "24.04" ]]; then
  echo "This installer is intended for Ubuntu 24.04"
  exit 1
fi

ROS_DISTRO=jazzy
WS=~/ros2_ws

echo "Updating system..."
sudo apt update
sudo apt install -y software-properties-common curl git

echo "Enabling universe repo..."
sudo add-apt-repository -y universe

sudo apt update

echo "Adding ROS 2 apt source..."
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')

UBUNTU_CODENAME=$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})

curl -L -o /tmp/ros2-apt-source.deb \
https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb

sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update

echo "Installing ROS 2 base + tools..."
sudo apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-${ROS_DISTRO}-image-tools \
  ros-${ROS_DISTRO}-vision-msgs \
  python3-pip \
  clang \
  portaudio19-dev

echo "Sourcing ROS automatically..."
if ! grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi

source /opt/ros/${ROS_DISTRO}/setup.bash

echo "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

echo "Creating workspace..."
mkdir -p ${WS}/src
cd ${WS}/src

echo "Cloning repositories..."
if [ ! -d "go2_ros2_light" ]; then
  git clone https://github.com/N-rwal/go2_ros2_light.git
fi

if [ ! -d "sllidar_ros2" ]; then
  git clone https://github.com/N-rwal/sllidar_ros2.git
fi

echo "Initializing submodules..."
cd go2_ros2_light
git submodule update --init --recursive

echo "Installing Python requirements..."
pip install -r requirements.txt

echo "Installing ROS dependencies..."
cd ${WS}
rosdep install --from-paths src --ignore-src -r -y

echo "Creating udev rules..."
cd src/sllidar_ros2
sudo bash scripts/create_udev_rules.sh

echo "Building workspace..."
cd ${WS}
colcon build

echo "Setup complete!"
echo "Run: source ${WS}/install/setup.bash"
