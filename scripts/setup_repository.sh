# Add ROS entries to ~/.bashrc
grep -F "source /opt/ros/humble/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc \
    || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
grep -F "export ROS_LOCALHOST_ONLY=1" ~/.bashrc \
    || echo "# export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# Install apt dependencies
apt-get update
apt-get upgrade -y
apt-get install -y ros-${ROS_DISTRO}-control-toolbox \
                   ros-${ROS_DISTRO}-robot-localization \
                   ros-${ROS_DISTRO}-mavros \
                   ros-${ROS_DISTRO}-navigation2 \
                   ros-${ROS_DISTRO}-behaviortree-cpp \
                   ros-${ROS_DISTRO}-image-common \
                   python3-pip \
                   python3-rosdep \
                   python3-vcs2l \
                   libeigen3-dev \
                   unzip \
                   wget

# Install pigpio
wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
     make install && \
    cd .. && \
    rm -rf master.zip pigpio-master

[ -e /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
rosdep update
