#!/usr/bin/env bash
# Based on: https://github.com/Tiryoh/ros2_setup_scripts_ubuntu

set -eu

TARGET_OS=noble

# Check OS version
if ! which lsb_release > /dev/null ; then
	apt-get update
	apt-get install -y curl lsb-release
fi

if [[ "$(lsb_release -sc)" == "$TARGET_OS" ]]; then
	echo "OS Check Passed"
else
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This OS (version: $(lsb_release -sc)) is not supported"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

if ! dpkg --print-architecture | grep -q 64; then
	printf '\033[33m%s\033[m\n' "=================================================="
	printf '\033[33m%s\033[m\n' "ERROR: This architecture ($(dpkg --print-architecture)) is not supported"
	printf '\033[33m%s\033[m\n' "See https://www.ros.org/reps/rep-2000.html"
	printf '\033[33m%s\033[m\n' "=================================================="
	exit 1
fi

apt-get update && apt upgrade -y

# Install ROS
apt-get update
apt-get install -y software-properties-common
add-apt-repository -y universe
apt-get install -y curl gnupg2 lsb-release build-essential wget

# Configure the ROS apt repository via the ros2-apt-source package (keeps the signing key up to date)
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb"
dpkg -i /tmp/ros2-apt-source.deb
rm -f /tmp/ros2-apt-source.deb

apt-get update
apt-get install -y ros-jazzy-desktop \
                   python3-argcomplete python3-colcon-clean \
                   python3-colcon-common-extensions \
                   python3-rosdep


set +u

[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update

# Add ROS entries to ~/.bashrc
grep -F "source /opt/ros/jazzy/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc \
    || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
grep -F "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" ~/.bashrc \
    || echo "# export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
