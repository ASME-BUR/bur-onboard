#!/usr/bin/env bash
# Based on: https://github.com/Tiryoh/ros2_setup_scripts_ubuntu

set -eu

TARGET_OS=jammy

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

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update
apt-get install -y ros-humble-desktop \
                   python3-argcomplete python3-colcon-clean \
                   python3-colcon-common-extensions \
                   python3-rosdep


set +u

[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update

# Add ROS entries to ~/.bashrc
grep -F "source /opt/ros/humble/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc \
    || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
grep -F "export ROS_LOCALHOST_ONLY=1" ~/.bashrc \
    || echo "# export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

source /opt/ros/humble/setup.bash
