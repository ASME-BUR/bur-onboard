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
                        python3-vcstool \
                        libeigen3-dev \
                        unzip

python3 -m pip install vcs2l

wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
     make install && \
    cd .. && \
    rm -rf master.zip pigpio-master

rosdep update

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/2024-2025/ros2_ws/install/setup.bash" >> ~/.bashrc
