let DEBIAN_FRONTEND="noninteractive"

 apt update 
 apt install locales 
locale-gen en_US en_US.UTF-8 
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
export LANG=en_US.UTF-8

 apt install -y software-properties-common && add-apt-repository universe

 apt update
 apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
dpkg -i /tmp/ros2-apt-source.deb

 apt-get update
 apt upgrade -y
 apt-get install -y ros-humble-desktop \
                        python3-colcon-common-extensions \
                        python3-rosdep \
                        python3-pip \
                        unzip

 apt-get update
 apt-get install -y ros-${ROS_DISTRO}-behaviortree-cpp \
                        ros-${ROS_DISTRO}-control-toolbox \
                        ros-${ROS_DISTRO}-image-common \
                        ros-${ROS_DISTRO}-mavros \
                        ros-${ROS_DISTRO}-navigation2 \
                        ros-${ROS_DISTRO}-robot-localization \
                        libeigen3-dev

rosdep update
