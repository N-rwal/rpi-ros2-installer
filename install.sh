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

# Track completed steps
STATE_FILE="/tmp/ros2_install_state"
touch "$STATE_FILE"

step_done() {
  grep -q "^$1$" "$STATE_FILE"
}

mark_step_done() {
  echo "$1" >> "$STATE_FILE"
}

echo "Updating system..."
if ! step_done "system_update"; then
  sudo apt update
  sudo apt install -y software-properties-common curl git
  mark_step_done "system_update"
fi

echo "Enabling universe repo..."
if ! step_done "universe_repo"; then
  sudo add-apt-repository -y universe
  sudo apt update
  mark_step_done "universe_repo"
fi

echo "Adding ROS 2 apt source..."
if ! step_done "ros_apt_source"; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F "tag_name" | awk -F\" '{print $4}')

  UBUNTU_CODENAME=$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})

  curl -L -o /tmp/ros2-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb

  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
  mark_step_done "ros_apt_source"
fi

echo "Installing ROS 2 base + tools..."
if ! step_done "ros_packages"; then
  sudo apt install -y \
    ros-${ROS_DISTRO}-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-image-tools \
    ros-${ROS_DISTRO}-vision-msgs \
    python3-pip \
    clang \
    portaudio19-dev
  mark_step_done "ros_packages"
fi

echo "Sourcing ROS automatically..."
if ! grep -q "/opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi

set +u
if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Error: ROS_DISTRO is not set"
    exit 1
fi
if [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "Error: ROS setup file not found at /opt/ros/${ROS_DISTRO}/setup.bash"
    exit 1
fi
echo "Sourcing ROS setup (with unset checking disabled)..."
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

echo "Initializing rosdep..."
if ! step_done "rosdep_init"; then
  sudo rosdep init 2>/dev/null || true
  rosdep update
  mark_step_done "rosdep_init"
fi

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
if [ -d ".git" ]; then
  git submodule update --init --recursive
fi

echo "Installing Python requirements..."
if [ -f "requirements.txt" ]; then
  pip install --user -r requirements.txt --break-system-packages
fi

echo "Installing ROS dependencies..."
cd ${WS}
rosdep install --from-paths src --ignore-src -r -y

echo "Creating udev rules..."
if [ -f "src/sllidar_ros2/scripts/create_udev_rules.sh" ]; then
  cd src/sllidar_ros2
  sudo bash scripts/create_udev_rules.sh
  cd ${WS}
fi

echo "Building workspace..."
if ! step_done "colcon_build"; then
  cd ${WS}
  if colcon build; then
    mark_step_done "colcon_build"
  else
    echo "Build failed. You can fix the issue and run the script again."
    exit 1
  fi
fi

echo "Setup complete!"
echo "Run: source ${WS}/install/setup.bash"
